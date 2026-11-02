#!/usr/bin/env python3
"""probe_limits.py: odskusa limity AI sluzieb (Gemini, Mistral, Groq, OpenRouter) opakovanym posielanim malych poziadaviek.

Bezi u teba (kluce citam z ~/.chappie_keys cez llm_backends.load_keys, nikdy ich nevypisuje).
Pouzitie:
  python3 tools/probe_limits.py                    # rychly prehlad: 1 poziadavka na sluzbu + hlavicky s limitmi
  python3 tools/probe_limits.py --rpm              # zmeria limit za minutu (burst tak dlho, kym pride 429)
  python3 tools/probe_limits.py --size             # zvacsuje prompt (1k, 4k, 16k, 64k tokenov), kym sluzba neodmietne
  python3 tools/probe_limits.py --images           # zvacsuje pocet obrazkov v jednej poziadavke (1,2,4,8,16,32)
  python3 tools/probe_limits.py --daily 300 --yes  # POZOR: spotrebuje denny limit sluzby (az 300 poziadaviek)
  --service gemini|mistral|groq|openrouter (viackrat)  obmedzi test na vybrane sluzby;  --all = --rpm --size --images
Vysledky sa ulozia do ~/.chappie_limits.json a na konci sa vypisu odporucane riadky pre ~/.chappie_keys
(napr. GEMINI_DAILY_LIMIT=250), ktore pouziva retaz AI na vypis 'zostava'.
Testy su lacne (poziadavky maju ~10 tokenov, --size pouzije najviac --max-tokens, predvolene 64 000).
Free tier ma limity za minutu aj za den: po --rpm moze sluzba ~1 minutu odpovedat 429, retaz AI ju vtedy preskoci.
"""
import argparse
import json
import os
import re
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser(os.environ.get("CHAPPIE_WS", "~/agent-based-humanoid-control"))
for p in (os.path.join(WS, "src/humanoid_agent/humanoid_agent"), os.path.join(HERE, "../src/humanoid_agent/humanoid_agent")):
    if os.path.isdir(p):
        sys.path.insert(0, p)
import llm_backends as L  # noqa: E402

OUT = os.path.expanduser("~/.chappie_limits.json")
UA = "chappie-agent/1.0"        # bez User-Agent Cloudflare (Groq) vrati 403 error code 1010
LIMIT_HDR = re.compile(r"ratelimit|retry-after|x-.*remaining|x-.*limit", re.I)


def tokens_in(js):
    """vstupne tokeny z odpovede Gemini alebo OpenAI-kompatibilnej sluzby"""
    try:
        if "usageMetadata" in js:
            return int(js["usageMetadata"].get("promptTokenCount", 0))
        return int(js.get("usage", {}).get("prompt_tokens", 0))
    except (AttributeError, TypeError, ValueError):
        return 0


class Svc:
    """jedna sluzba: adresa, hlavicky, telo poziadavky (Gemini alebo OpenAI-kompatibilne)"""

    def __init__(self, name, llm):
        self.name, self.llm = name, llm
        self.gemini = name == "gemini"
        if self.gemini:
            self.model = os.environ.get("GEMINI_MODEL") or llm.model
            base = os.environ.get("CHAPPIE_PROBE_GEMINI_URL", "https://generativelanguage.googleapis.com/v1beta/models/%s:generateContent")
            self.url = base % self.model if "%s" in base else base
            self.headers = {"Content-Type": "application/json", "x-goog-api-key": llm.api_key, "User-Agent": UA}
        else:
            self.model = llm.models[0]
            self.url = llm.url
            self.headers = {"Content-Type": "application/json", "Authorization": "Bearer " + llm.api_key, "User-Agent": UA}

    def body(self, text, images=0):
        png = L._test_png() if images else None
        import base64
        if self.gemini:
            parts = [{"text": text}] + [{"inline_data": {"mime_type": "image/png", "data": base64.b64encode(png).decode()}}
                                         for _ in range(images)]
            return {"contents": [{"role": "user", "parts": parts}], "generationConfig": {"maxOutputTokens": 16}}
        content = text
        if images:
            content = [{"type": "text", "text": text}]
            for _ in range(images):
                u = "data:image/png;base64," + base64.b64encode(png).decode()
                content.append({"type": "image_url", "image_url": {"url": u} if self.llm.img_object else u})
        return {"model": self.model, "max_tokens": 8, "messages": [{"role": "user", "content": content}]}

    def call(self, text, images=0, timeout=90):
        """-> dict(status, secs, tokens_in, headers(len limity), msg)"""
        req = urllib.request.Request(self.url, data=json.dumps(self.body(text, images)).encode(), headers=self.headers, method="POST")
        t0 = time.time()
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                js = json.loads(r.read().decode("utf-8", "replace"))
                hdr = {k.lower(): v for k, v in r.headers.items() if LIMIT_HDR.search(k)}
                tin = tokens_in(js)
                return {"status": r.status, "secs": time.time() - t0, "tokens_in": tin, "headers": hdr, "msg": ""}
        except urllib.error.HTTPError as e:
            raw = e.read().decode("utf-8", "replace")
            msg = raw[:300]
            try:
                err = json.loads(raw)["error"]
                msg = str(err.get("message", msg)) if isinstance(err, dict) else str(err)
            except (ValueError, KeyError, TypeError):
                pass
            hdr = {k.lower(): v for k, v in e.headers.items() if LIMIT_HDR.search(k)}
            return {"status": e.code, "secs": time.time() - t0, "tokens_in": 0, "headers": hdr, "msg": " ".join(msg.split())[:300]}
        except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
            return {"status": 0, "secs": time.time() - t0, "tokens_in": 0, "headers": {}, "msg": "siet: %s" % e}


def scope_of(msg):
    low = msg.lower().replace("-", " ").replace("_", " ")      # OpenRouter: free-models-per-day
    if "per day" in low or "perday" in low.replace(" ", "") or "daily" in low:
        return "den"
    if "per minute" in low or "perminute" in low.replace(" ", "") or "tpm" in low or "rpm" in low:
        return "minutu"
    return "?"


def test_info(s, res):
    r = s.call("Reply with the single word OK.")
    for m in [] if s.gemini else s.llm.models[1:]:
        if r["status"] not in (400, 402, 403, 404, 429):
            break
        print("  model %s -> HTTP %s, skusam dalsi model" % (s.model, r["status"]))
        s.model = m
        r = s.call("Reply with the single word OK.")
    res["info"] = {"status": r["status"], "model": s.model, "headers": r["headers"], "msg": r["msg"]}
    print("  model %s -> HTTP %s (%.1f s)" % (s.model, r["status"], r["secs"]))
    for k, v in sorted(r["headers"].items()):
        print("    %s: %s" % (k, v))
    if not r["headers"]:
        print("    (sluzba neposiela hlavicky s limitmi; zmeria ich --rpm)")
    if r["status"] != 200:
        print("    chyba: %s" % r["msg"])
        if r["status"] == 403 and "1010" in r["msg"]:
            print("    -> Cloudflare zablokoval poziadavku (chyba 1010): chyba User-Agent; pre agenta pusti fix_llm_useragent.py")
        elif r["status"] == 429:
            print("    -> kvota/rate limit uz pri prvej poziadavke: skontroluj plan a limity v konzole sluzby (nie je to chyba skriptu)")
        elif r["status"] in (401, 403):
            print("    -> kluc je neplatny alebo nema pristup k modelu")
    return r["status"] == 200


def test_rpm(s, res, parallel, max_req, max_secs):
    lock, stop = threading.Lock(), threading.Event()
    ok, first429, t0 = [0], [None], time.time()

    def worker():
        while not stop.is_set():
            with lock:
                if ok[0] + (1 if first429[0] else 0) >= max_req or time.time() - t0 > max_secs:
                    stop.set()
                    return
                sent = ok[0]
            r = s.call("Reply with the single word OK.")
            with lock:
                if r["status"] == 200:
                    ok[0] += 1
                elif r["status"] == 429 and first429[0] is None:
                    first429[0] = {"after": ok[0], "at": time.time() - t0, "msg": r["msg"], "headers": r["headers"]}
                    stop.set()
                elif r["status"] != 429:
                    first429[0] = first429[0] or {"after": ok[0], "at": time.time() - t0, "msg": "HTTP %s %s" % (r["status"], r["msg"]), "headers": {}}
                    stop.set()
    ths = [threading.Thread(target=worker) for _ in range(parallel)]
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    dur = time.time() - t0
    f = first429[0]
    out = {"ok_before_limit": ok[0], "seconds": round(dur, 1), "limited": bool(f)}
    if f:
        out.update(scope=scope_of(f["msg"]), message=f["msg"], retry_headers=f["headers"])
        print("  %d poziadaviek prislo za %.0f s, potom limit (%s). Sprava: %s" % (
            ok[0], dur, ("za " + out["scope"]) if out["scope"] != "?" else "dovod neuvedeny, pravdepodobne za minutu", f["msg"][:160]))
        if out["scope"] in ("minutu", "?") and dur < 70 and ok[0] > 0:
            out["rpm_estimate"] = ok[0] + 1          # +1: uvodna poziadavka testu info
            print("  odhad limitu za minutu: ~%d poziadaviek" % (ok[0] + 1))
    else:
        print("  %d poziadaviek za %.0f s bez limitu (limit za minutu je vyssi ako %d alebo ho test nedosiahol)" % (ok[0], dur, ok[0]))
    res["rpm"] = out


def test_size(s, res, max_tokens):
    out = []
    for target in (1000, 4000, 16000, 64000, 128000):
        if target > max_tokens:
            break
        text = "Count the words below and reply only OK. " + ("lorem ipsum dolor sit amet " * (target // 5))
        r = s.call(text, timeout=180)
        out.append({"target": target, "status": r["status"], "tokens_in": r["tokens_in"], "message": r["msg"]})
        print("  ~%6d tokenov -> HTTP %s%s" % (target, r["status"], (" (%d tokenov podla sluzby)" % r["tokens_in"]) if r["tokens_in"] else (" " + r["msg"][:140])))
        if r["status"] != 200:
            break
    good = [o["tokens_in"] or o["target"] for o in out if o["status"] == 200]
    res["size"] = {"steps": out, "max_ok_tokens": max(good) if good else 0}


def test_images(s, res):
    out = []
    for n in (1, 2, 4, 8, 16, 32):
        r = s.call("Reply only OK.", images=n)
        out.append({"images": n, "status": r["status"], "message": r["msg"]})
        print("  %2d obrazkov -> HTTP %s %s" % (n, r["status"], r["msg"][:120] if r["status"] != 200 else ""))
        if r["status"] != 200:
            break
    good = [o["images"] for o in out if o["status"] == 200]
    res["images"] = {"steps": out, "max_ok_images": max(good) if good else 0}


def test_daily(s, res, n, cooldown):
    """posiela poziadavky; 429 s nejasnym dovodom -> pocka `cooldown` s: ak sa uvolni, bol to limit za minutu"""
    ok, msg, t0, waits = 0, "", time.time(), 0
    for i in range(n):
        r = s.call("Reply with the single word OK.")
        if r["status"] == 200:
            ok += 1
            if ok % 25 == 0:
                print("  ... %d poziadaviek" % ok)
            continue
        if r["status"] == 429 and scope_of(r["msg"]) != "den" and waits < 400:
            w = max(cooldown, (L._retry_seconds(r["msg"]) or 0) + 1)
            print("  limit po %d poziadavkach (asi za minutu), cakam %.0f s" % (ok, w))
            time.sleep(w)
            waits += 1
            r2 = s.call("Reply with the single word OK.")
            if r2["status"] == 200:
                ok += 1
                continue
            r = r2
        msg = "HTTP %s %s" % (r["status"], r["msg"])
        break
    reached = bool(msg)
    res["daily"] = {"requests_ok": ok, "reached_limit": reached and ok > 0, "message": msg, "seconds": round(time.time() - t0)}
    if reached and ok == 0:
        print("  uz prva poziadavka dostala limit: %s" % msg[:160])
    else:
        print("  %d poziadaviek %s" % (ok, ("potom: " + msg[:160]) if reached else "bez dennej zarazky (limit je vyssi ako %d)" % n))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--service", action="append", choices=["gemini", "mistral", "groq", "openrouter"])
    ap.add_argument("--rpm", action="store_true")
    ap.add_argument("--size", action="store_true")
    ap.add_argument("--images", action="store_true")
    ap.add_argument("--all", action="store_true", help="--rpm --size --images")
    ap.add_argument("--daily", type=int, default=0, metavar="N", help="najviac N poziadaviek na zistenie dennej zarazky")
    ap.add_argument("--yes", action="store_true", help="potvrdi --daily")
    ap.add_argument("--cooldown", type=int, default=65, help="cakanie [s] po 429 pred dalsim testom (limit za minutu sa uvolni)")
    ap.add_argument("--parallel", type=int, default=4)
    ap.add_argument("--max-requests", type=int, default=80)
    ap.add_argument("--max-seconds", type=int, default=75)
    ap.add_argument("--max-tokens", type=int, default=64000)
    a = ap.parse_args()
    if a.all:
        a.rpm = a.size = a.images = True
    if a.daily and not a.yes:
        sys.exit("--daily spotrebuje denny limit sluzby (az %d poziadaviek). Ak to chces, pridaj --yes." % a.daily)
    L.load_keys()
    builders = {"gemini": L._gemini, "mistral": L._mistral, "groq": L._groq, "openrouter": L._openrouter}
    try:
        old = json.load(open(OUT))
    except (OSError, ValueError):
        old = {}
    for name in a.service or list(builders):
        try:
            llm = builders[name]()
        except Exception as e:
            llm = None
            print("== %s: %s" % (name, e))
        if llm is None:
            print("== %s: bez kluca, preskakujem" % name)
            continue
        s = Svc(name, llm)
        print("== %s" % name)
        res = {"time": time.strftime("%Y-%m-%d %H:%M:%S")}
        if test_info(s, res):
            if a.rpm:
                print(" -- limit za minutu (%d paralelne, max %d poziadaviek / %d s)" % (a.parallel, a.max_requests, a.max_seconds))
                test_rpm(s, res, a.parallel, a.max_requests, a.max_seconds)
                if res["rpm"]["limited"] and (a.size or a.images or a.daily):
                    print("  cakam %d s, aby sa uvolnil limit za minutu..." % a.cooldown)
                    time.sleep(a.cooldown)
            if a.size:
                print(" -- velkost promptu")
                test_size(s, res, a.max_tokens)
            if a.images:
                print(" -- pocet obrazkov v poziadavke")
                test_images(s, res)
            if a.daily:
                print(" -- denna zarazka (najviac %d poziadaviek)" % a.daily)
                test_daily(s, res, a.daily, a.cooldown)
        old[name] = res
        json.dump(old, open(OUT, "w"), indent=1, ensure_ascii=False)      # uloz po kazdej sluzbe (aj pri Ctrl+C)
    json.dump(old, open(OUT, "w"), indent=1, ensure_ascii=False)
    print("\nUlozene do %s" % OUT)
    tips = []
    for name, r in old.items():
        d = r.get("daily") or {}
        if d.get("reached_limit") and d.get("requests_ok", 0) > 0:
            tips.append("%s_DAILY_LIMIT=%d" % (name.upper(), d["requests_ok"]))
        im = (r.get("images") or {}).get("max_ok_images")
        if im:
            tips.append("%s_MAX_IMAGES=%d    # najviac obrazkov, ktore sluzba prijala v tomto teste" % (name.upper(), im))
    if tips:
        print("Odporucane riadky do ~/.chappie_keys:")
        print("  " + "\n  ".join(tips))


if __name__ == "__main__":
    main()
