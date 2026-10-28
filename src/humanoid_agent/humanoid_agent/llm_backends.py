"""llm_backends: volanie AI cez HTTPS REST (bez externych kniznic, iba stdlib).

Podporene: Gemini (Google), Claude (Anthropic).  Kluce sa citaju z premennych prostredia:
  GEMINI_API_KEY  alebo  ANTHROPIC_API_KEY
Gemini: GEMINI_MODEL=<model> (prebije predvoleny), GEMINI_THINKING=off|minimal|low|medium|high (predvolene low:
  menej "premyslania" = rychlejsie odpovede). Pri vypadku/limite modelu sa automaticky skusia dalsie dostupne modely.
ask(system, user_text, images): images = zoznam (bytes, mime, popis) alebo jeden bytes obrazok (+ mime).
"""
import base64
import json
import os
import re
import time
import urllib.error
import urllib.request

TEMP_CODES = (429, 500, 502, 503, 504)


def _post_once(url, headers, body, timeout):
    req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"), headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            out = json.loads(r.read().decode("utf-8"))
            _RESP[0], _RESP[1] = dict((k.lower(), v) for k, v in r.headers.items()), out
            return out
    except urllib.error.HTTPError as e:
        raw = e.read().decode("utf-8", "replace")
        detail = raw[:500]
        try:                                   # kratka sprava namiesto celeho JSON
            err = json.loads(raw)["error"]
            detail = str(err.get("message", detail)).strip()[:400]
        except (ValueError, KeyError, TypeError, AttributeError):
            pass
        raise RuntimeError("HTTP %d: %s" % (e.code, detail))


def _retry_seconds(msg):
    """z hlasenia o kvote ('Please retry in 6h14m19.5s') vrati pocet sekund, inak None"""
    m = re.search(r"(?:retry|try again) in (?:(\d+)h)?(?:(\d+)m)?(?:([0-9.]+)s)?", msg)
    if not m or not any(m.groups()):
        return None
    h, mi, s = (float(x) if x else 0.0 for x in m.groups())
    return h * 3600 + mi * 60 + s


def _post(url, headers, body, timeout, tries=5):
    """POST s opakovanim pri docasnych chybach (503/429/500/502/504, siet).
    Pri vycerpanej kvote (429 s dlhym cakanim) sa neopakuje - vyssia vrstva skusi iny model."""
    delay = 2.0
    for k in range(tries):
        try:
            return _post_once(url, headers, body, timeout)
        except RuntimeError as e:
            msg = str(e)
            if not any(("HTTP %d" % c) in msg for c in TEMP_CODES) or k == tries - 1:
                raise
            if "HTTP 429" in msg:
                w = _retry_seconds(msg)
                if (w is not None and w > 20) or (w is None and ("per day" in msg.lower() or "quota" in msg.lower())):
                    raise                      # denny limit: cakat nema zmysel
                if w is not None:
                    delay = max(delay, w + 1)
        except (urllib.error.URLError, TimeoutError, ConnectionError):
            if k == tries - 1:
                raise RuntimeError("Siet: nedostupna sluzba AI")
        print("[AI] docasna chyba, dalsi pokus %d/%d o %.0f s" % (k + 2, tries, delay), flush=True)
        time.sleep(delay)
        delay = min(delay * 2, 20.0)


def _images(image, mime):
    """-> [(bytes, mime, popis)]  (image: None | bytes | zoznam (bytes, mime, popis))"""
    if not image:
        return []
    if isinstance(image, (bytes, bytearray)):
        return [(bytes(image), mime or "image/png", "")]
    return [(b, m or "image/png", l or "") for b, m, l in image]


class GeminiLLM:
    def __init__(self, model="gemini-3.6-flash", api_key=None, timeout=60, temperature=0.2, thinking=None):
        self.model, self.timeout, self.temperature = model, timeout, temperature
        self.api_key = api_key or os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
        if not self.api_key:
            raise RuntimeError("Chyba GEMINI_API_KEY (export GEMINI_API_KEY=...)")
        self.thinking = (thinking if thinking is not None else os.environ.get("GEMINI_THINKING", "low")).lower()
        self.no_think = self.thinking in ("", "off", "none")

    def build(self, system, user_text, image, mime, think=True):
        parts = [{"text": user_text}]
        for data, m, label in _images(image, mime):
            if label:
                parts.append({"text": label})
            parts.append({"inline_data": {"mime_type": m, "data": base64.b64encode(data).decode("ascii")}})
        cfg = {"temperature": self.temperature, "responseMimeType": "application/json"}
        if think and not self.no_think:
            cfg["thinkingConfig"] = {"thinkingLevel": self.thinking}
        return {
            "systemInstruction": {"parts": [{"text": system}]},
            "contents": [{"role": "user", "parts": parts}],
            "generationConfig": cfg,
        }

    def _candidates(self):
        """dostupne modely Gemini (flash, flash-lite, pro): najprv najnovsie flash"""
        req = urllib.request.Request("https://generativelanguage.googleapis.com/v1beta/models?pageSize=200",
                                     headers={"x-goog-api-key": self.api_key})
        with urllib.request.urlopen(req, timeout=20) as r:
            data = json.loads(r.read().decode("utf-8"))
        out = []
        for mm in data.get("models", []):
            name = mm.get("name", "").replace("models/", "")
            g = re.match(r"gemini-([0-9]+(?:[.][0-9]+)?)-(flash-lite|flash|pro)(-preview)?$", name)
            if not g or "generateContent" not in mm.get("supportedGenerationMethods", []):
                continue
            rank = {"flash": 0, "flash-lite": 1, "pro": 2}[g.group(2)]
            out.append((rank, -float(g.group(1)), 1 if g.group(3) else 0, name))
        out.sort()
        return [x[3] for x in out][:5]

    def ask(self, system, user_text, image=None, mime=None):
        first = os.environ.get("GEMINI_MODEL") or self.model
        try:
            return self._ask_model(first, system, user_text, image, mime, 5)
        except RuntimeError as e:
            if not any(("HTTP %d" % c) in str(e) for c in (404,) + TEMP_CODES):
                raise
            err = e
        try:
            cands = [c for c in self._candidates() if c != first]
        except Exception as e2:
            raise RuntimeError("%s (zoznam modelov sa nepodarilo ziskat: %s)" % (err, e2))
        for c in cands:
            try:
                out = self._ask_model(c, system, user_text, image, mime, 2)
                print("[gemini] model %s nedostupny, pouzity %s (nastav: export GEMINI_MODEL=%s)" % (first, c, c),
                      flush=True)
                self.model = c
                return out
            except RuntimeError as e3:
                err = e3
        raise err

    def _ask_model(self, model, system, user_text, image, mime, tries):
        url = "https://generativelanguage.googleapis.com/v1beta/models/%s:generateContent" % model
        headers = {"Content-Type": "application/json", "x-goog-api-key": self.api_key}
        t0 = time.time()
        try:
            out = _post(url, headers, self.build(system, user_text, image, mime, True), self.timeout, tries)
            print("[gemini] %s odpovedal za %.1f s (obrazky: %d, thinking: %s)" % (
                model, time.time() - t0, len(_images(image, mime)), "off" if self.no_think else self.thinking), flush=True)
        except RuntimeError as e:
            # niektore modely nepoznaju thinkingLevel -> zopakuj bez neho (a uz ho nepouzivaj)
            if "HTTP 400" in str(e) and "think" in str(e).lower() and not self.no_think:
                self.no_think = True
                out = _post(url, headers, self.build(system, user_text, image, mime, False), self.timeout, tries)
            else:
                raise
        try:
            return "".join(p.get("text", "") for p in out["candidates"][0]["content"]["parts"])
        except (KeyError, IndexError):
            raise RuntimeError("Neocakavana odpoved Gemini: %s" % json.dumps(out)[:300])


class ClaudeLLM:
    def __init__(self, model="claude-sonnet-5-5", api_key=None, timeout=60, max_tokens=1024):
        self.model, self.timeout, self.max_tokens = model, timeout, max_tokens
        self.api_key = api_key or os.environ.get("ANTHROPIC_API_KEY")
        if not self.api_key:
            raise RuntimeError("Chyba ANTHROPIC_API_KEY (export ANTHROPIC_API_KEY=...)")

    def build(self, system, user_text, image, mime):
        content = []
        for data, m, label in _images(image, mime):
            if label:
                content.append({"type": "text", "text": label})
            content.append({"type": "image", "source": {
                "type": "base64", "media_type": m, "data": base64.b64encode(data).decode("ascii")}})
        content.append({"type": "text", "text": user_text})
        return {"model": self.model, "max_tokens": self.max_tokens, "system": system,
                "messages": [{"role": "user", "content": content}]}

    def ask(self, system, user_text, image=None, mime=None):
        out = _post("https://api.anthropic.com/v1/messages",
                    {"Content-Type": "application/json", "x-api-key": self.api_key,
                     "anthropic-version": "2023-06-01"},
                    self.build(system, user_text, image, mime), self.timeout)
        try:
            return "".join(b.get("text", "") for b in out["content"] if b.get("type") == "text")
        except (KeyError, TypeError):
            raise RuntimeError("Neocakavana odpoved Claude: %s" % json.dumps(out)[:300])


# ===== Retaz AI sluzieb: Gemini -> Mistral -> Groq  (pridal add_llm_chain.py) =====
KEYS_FILE = os.path.expanduser("~/.chappie_keys")
_DEAD = {}        # nazov sluzby -> cas, do ktoreho ju preskakujeme (zdielane medzi ulohami)
_LAST = [None]    # ktora sluzba odpovedala naposledy
_RESP = [None, None]   # hlavicky a JSON posledne prijatej odpovede (plni _post_once)
USAGE_FILE = os.path.expanduser("~/.chappie_usage.json")


def _day_key(name):
    t = time.time()
    if name == "gemini":
        t -= 8 * 3600                  # kvota Google sa nuluje o polnoci pacifickeho casu
    elif name == "mistral":
        return time.strftime("%Y-%m", time.gmtime(t))        # Mistral: mesacny limit
    return time.strftime("%Y-%m-%d", time.gmtime(t))


def _usage_update(name, tokens):
    """pripocita poziadavku do ~/.chappie_usage.json -> (poziadavky, tokeny) za aktualne obdobie"""
    try:
        with open(USAGE_FILE) as f:
            d = json.load(f)
    except (OSError, ValueError):
        d = {}
    key = _day_key(name)
    u = d.get(name) or {}
    if u.get("day") != key:
        u = {"day": key, "requests": 0, "tokens": 0}
    u["requests"] += 1
    u["tokens"] += int(tokens)
    d[name] = u
    try:
        with open(USAGE_FILE, "w") as f:
            json.dump(d, f)
    except OSError:
        pass
    return u["requests"], u["tokens"]


def _tokens(js):
    try:
        if "usageMetadata" in js:
            return int(js["usageMetadata"].get("totalTokenCount", 0))
        u = js.get("usage", {})
        return int(u.get("total_tokens") or (u.get("input_tokens", 0) + u.get("output_tokens", 0)))
    except (AttributeError, TypeError, ValueError):
        return 0


COST_FILE = os.path.expanduser("~/.chappie_cost.json")


def _tokens_split(js):
    """-> (vstupne, vystupne) tokeny z odpovede Gemini / OpenAI-kompatibilnej / Claude"""
    try:
        if "usageMetadata" in js:
            u = js["usageMetadata"]
            return (int(u.get("promptTokenCount", 0)),
                    int(u.get("candidatesTokenCount", 0)) + int(u.get("thoughtsTokenCount", 0)))
        u = js.get("usage", {})
        return (int(u.get("prompt_tokens") or u.get("input_tokens") or 0),
                int(u.get("completion_tokens") or u.get("output_tokens") or 0))
    except (AttributeError, TypeError, ValueError):
        return 0, 0


def _price(name):
    """(cena vstup, cena vystup) v USD za 1 mil. tokenov z <SLUZBA>_PRICE_IN / _PRICE_OUT (predvolene 0 = free tier)"""
    try:
        return (float(os.environ.get(name.upper() + "_PRICE_IN", "0") or 0),
                float(os.environ.get(name.upper() + "_PRICE_OUT", "0") or 0))
    except ValueError:
        return 0.0, 0.0


def _cost_log(name, model, tin, tout):
    """zapise poziadavku do ~/.chappie_cost.json: {"days": {den: {sluzba: {model: {req, in, out}}}}} -> cena v USD"""
    try:
        with open(COST_FILE) as f:
            d = json.load(f)
    except (OSError, ValueError):
        d = {}
    day = time.strftime("%Y-%m-%d")
    m = d.setdefault("days", {}).setdefault(day, {}).setdefault(name, {}).setdefault(model or "?", {"req": 0, "in": 0, "out": 0})
    m["req"] += 1
    m["in"] += int(tin)
    m["out"] += int(tout)
    try:
        with open(COST_FILE, "w") as f:
            json.dump(d, f)
    except OSError:
        pass
    pin, pout = _price(name)
    return (tin * pin + tout * pout) / 1e6


def _rate(name, hdr, used):
    """(limit, zostava) poziadaviek: z hlaviciek x-ratelimit-*-requests, inak z <SLUZBA>_DAILY_LIMIT"""
    lim = rem = None
    for k, v in (hdr or {}).items():
        if "ratelimit" in k and "req" in k and not any(w in k for w in ("minute", "second", "-min", "-sec")):
            try:
                x = int(float(v))
            except ValueError:
                continue
            if "remaining" in k:
                rem = x
            elif "limit" in k:
                lim = x
    env = os.environ.get(name.upper() + "_DAILY_LIMIT", "")
    if lim is None and env.isdigit():
        lim = int(env)
    if rem is None and lim is not None:
        rem = max(0, lim - used)
    return lim, rem


def _reason(msg):
    low = msg.lower()
    if "HTTP 401" in msg or "HTTP 403" in msg or ("api key" in low and ("not valid" in low or "invalid" in low)):
        return "key"
    if "HTTP 429" in msg or "quota" in low or "resource_exhausted" in low or "per day" in low:
        return "quota"
    return "error"


def load_keys():
    """nacita kluce zo suboru ~/.chappie_keys (riadky KLUC=hodnota); premenne prostredia maju prednost"""
    try:
        with open(KEYS_FILE) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.replace("export ", "", 1).split("=", 1)
                k, v = k.strip(), v.strip().strip("'\"")
                if k and v and not os.environ.get(k):
                    os.environ[k] = v
    except OSError:
        pass


def _shrink(data, mime, maxw=640):
    """zmensi snimok na max 640 px a ulozi ako JPEG (ak je Pillow), inak vrati povodny"""
    try:
        import io
        from PIL import Image
        im = Image.open(io.BytesIO(data)).convert("RGB")
        if im.width > maxw:
            im = im.resize((maxw, max(1, int(im.height * maxw / im.width))))
        b = io.BytesIO()
        im.save(b, "JPEG", quality=80)
        return b.getvalue(), "image/jpeg"
    except Exception:
        return data, mime


class OpenAICompatLLM:
    """Mistral / Groq / ina sluzba s rozhranim OpenAI (/chat/completions)"""

    def __init__(self, name, url, api_key, models, max_images, img_object, timeout=60, temperature=0.2, maxw=640):
        self.name, self.url, self.api_key = name, url, api_key
        self.models, self.max_images, self.img_object = list(models), max_images, img_object
        self.model = self.models[0] if self.models else ""
        self.timeout, self.temperature, self.maxw = timeout, temperature, maxw

    def _body(self, model, system, text, imgs, json_mode):
        if imgs:
            content = [{"type": "text", "text": text}]
            for data, m, label in imgs:
                data, m = _shrink(data, m, self.maxw)
                if label:
                    content.append({"type": "text", "text": label})
                u = "data:%s;base64,%s" % (m, base64.b64encode(data).decode("ascii"))
                content.append({"type": "image_url", "image_url": {"url": u} if self.img_object else u})
        else:
            content = text
        body = {"model": model, "temperature": self.temperature, "max_tokens": 4096,
                "messages": [{"role": "system", "content": system}, {"role": "user", "content": content}]}
        if json_mode:
            body["response_format"] = {"type": "json_object"}
        return body

    @staticmethod
    def _text(out, name):
        try:
            c = out["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError):
            raise RuntimeError("Neocakavana odpoved %s: %s" % (name, json.dumps(out)[:300]))
        if isinstance(c, list):
            c = "".join(p.get("text", "") for p in c if isinstance(p, dict))
        c = re.sub(r"<think>.*?</think>", "", c or "", flags=re.S).strip()
        if not c:
            raise RuntimeError("%s vratila prazdnu odpoved" % name)
        return c

    def _call(self, model, system, text, use):
        headers = {"Content-Type": "application/json", "Authorization": "Bearer " + self.api_key}
        out = None
        for json_mode in (True, False):       # ak model JSON rezim nepozna / vygeneroval zly JSON, skusi sa bez neho
            t0 = time.time()
            try:
                out = _post(self.url, headers, self._body(model, system, text, use, json_mode), self.timeout, 3)
                break
            except RuntimeError as e:
                low = str(e).lower()
                if json_mode and "HTTP 400" in str(e) and ("response_format" in low or "json" in low):
                    continue
                raise
        self.model = model
        print("[%s] %s odpovedal za %.1f s (obrazky: %d)" % (self.name, model, time.time() - t0, len(use)), flush=True)
        return self._text(out, self.name)

    def ask(self, system, user_text, image=None, mime=None):
        imgs = _images(image, mime)[-self.max_images:] if self.max_images > 0 else []
        sizes = [len(imgs)] + [n for n in (1, 0) if n < len(imgs)]
        err = None
        for n in sizes:
            use = imgs[len(imgs) - n:] if n else []
            text = user_text if n == len(imgs) else \
                user_text + "\n\n(NOTE: some or all images were left out because of the size limit of this AI service.)"
            too_big = False
            for model in self.models:
                try:
                    return self._call(model, system, text, use)
                except RuntimeError as e:
                    err, msg = e, str(e)
                    low = msg.lower()
                    if "per day" in low or "tpd" in low or "rpd" in low:
                        raise                                   # denny limit
                    if "HTTP 413" in msg or "too large" in low or "tokens per minute" in low or \
                            ("HTTP 400" in msg and "image" in low):
                        too_big = True
                        break                                   # skusit s menej obrazkami
                    if "HTTP 404" in msg or ("HTTP 400" in msg and "model" in low):
                        continue                                # iny model
                    raise
            if not too_big:
                break
        raise err


class ChainLLM:
    """skusa sluzby po poradi; tu, co zlyhala, na chvilu preskoci (stav je zdielany medzi ulohami)"""

    def __init__(self, backends):
        self.backends = backends          # [(nazov, llm), ...]
        self.last_info = None             # info o poslednej poziadavke (cita ho agent a posle klientovi)

    @staticmethod
    def dead_for(msg):
        low = msg.lower()
        if "HTTP 401" in msg or "HTTP 403" in msg or ("api key" in low and ("not valid" in low or "invalid" in low)):
            return 6 * 3600.0             # zly kluc
        if "HTTP 429" in msg or "quota" in low or "resource_exhausted" in low or "per day" in low:
            w = _retry_seconds(msg)
            return min(max(w, 60.0), 3600.0) if w else 900.0
        return 120.0                       # vypadok, siet, ...

    def ask(self, system, user_text, image=None, mime=None):
        now = time.time()
        order = [b for b in self.backends if _DEAD.get(b[0], 0) <= now] or self.backends
        err = None
        failed = []
        for name, llm in order:
            _RESP[0] = _RESP[1] = None
            try:
                out = llm.ask(system, user_text, image, mime)
            except Exception as e:
                err = e
                failed.append([name, _reason(str(e))])
                self.last_info = {"service": None, "failed": failed}
                _DEAD[name] = time.time() + self.dead_for(str(e))
                print("[retaz] %s zlyhala (%s) -> skusam dalsiu sluzbu" % (name, str(e).replace("\n", " ")[:160]),
                      flush=True)
                continue
            if _LAST[0] != name:
                print("[retaz] odpoveda: %s" % name, flush=True)
                _LAST[0] = name
            tok = _tokens(_RESP[1] or {})
            tin, tout = _tokens_split(_RESP[1] or {})
            cost = _cost_log(name, str(getattr(llm, "model", "")), tin, tout)
            used, used_tok = _usage_update(name, tok)
            lim, rem = _rate(name, _RESP[0], used)
            self.last_info = {"service": name, "model": str(getattr(llm, "model", "")), "tokens": tok, "used": used,
                              "tokens_in": tin, "tokens_out": tout, "cost": cost,
                              "used_tokens": used_tok, "limit": lim, "remaining": rem,
                              "period": "month" if name == "mistral" else "day", "failed": failed}
            return out
        raise RuntimeError("Vsetky AI sluzby zlyhali. Posledna chyba: %s" % err)


def _mistral(model=""):
    key = os.environ.get("MISTRAL_API_KEY")
    if not key:
        return None
    models = [model or os.environ.get("MISTRAL_MODEL", "")] if (model or os.environ.get("MISTRAL_MODEL")) \
        else ["mistral-small-latest", "mistral-medium-latest"]
    return OpenAICompatLLM("mistral", os.environ.get("MISTRAL_URL", "https://api.mistral.ai/v1/chat/completions"),
                           key, models, int(os.environ.get("MISTRAL_MAX_IMAGES", "4")), False)


def _groq(model=""):
    key = os.environ.get("GROQ_API_KEY")
    if not key:
        return None
    m = model or os.environ.get("GROQ_MODEL") or "qwen/qwen3.8-27b"
    return OpenAICompatLLM("groq", os.environ.get("GROQ_URL", "https://api.groq.com/openai/v1/chat/completions"),
                           key, [m], min(3, int(os.environ.get("GROQ_MAX_IMAGES", "1"))), True)


def _gemini(model="", **kw):
    if not (os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")):
        return None
    return GeminiLLM(model or "gemini-3.6-flash", **kw)


def make_llm(provider, model="", **kw):
    load_keys()
    p = provider.lower()
    if p in ("claude", "anthropic"):
        return ChainLLM([("claude", ClaudeLLM(model or "claude-sonnet-5-5", **kw))])
    if p == "mistral":
        llm = _mistral(model)
        if llm is None:
            raise RuntimeError("Chyba MISTRAL_API_KEY (subor ~/.chappie_keys)")
        return ChainLLM([("mistral", llm)])
    if p == "groq":
        llm = _groq(model)
        if llm is None:
            raise RuntimeError("Chyba GROQ_API_KEY (subor ~/.chappie_keys)")
        return ChainLLM([("groq", llm)])
    if p in ("gemini", "google", "chain", "auto"):
        order = os.environ.get("CHAPPIE_CHAIN", "gemini,mistral,groq").lower().replace(" ", "")
        if order in ("off", "0", "no", "none"):
            order = "gemini"
        chain = []
        for name in order.split(","):
            if name in ("gemini", "google"):
                llm, nm = _gemini(model, **kw), "gemini"
            elif name == "mistral":
                llm, nm = _mistral(), "mistral"
            elif name == "groq":
                llm, nm = _groq(), "groq"
            else:
                continue
            if llm is not None and nm not in [c[0] for c in chain]:
                chain.append((nm, llm))
        if not chain:
            raise RuntimeError("Chyba GEMINI_API_KEY (export GEMINI_API_KEY=...)")
        return ChainLLM(chain)
    raise ValueError("Neznamy provider '%s' (gemini | mistral | groq | claude)" % provider)


# --- strucny vysledok ulohy + info o AI (pouziva task_client) ---

AI_NAMES = {"gemini": "Gemini", "mistral": "Mistral", "groq": "Groq", "claude": "Claude"}

AI_TXT = {
    "sk": {"ok": "Hotovo", "fail": "Zlyhalo", "cancel": "Zrušené", "in": "za", "s": "s",
           "step": ("krok", "kroky", "krokov"), "iter": ("iterácia", "iterácie", "iterácií"),
           "req": ("požiadavka", "požiadavky", "požiadaviek"), "tok": "tokenov", "day": "dnes",
           "month": "tento mesiac", "left": "zostáva ~%s z %s (≈ %d takýchto úloh)",
           "nolimit": "limit neznámy (nastav: export %s_DAILY_LIMIT=N)", "warn": "⚠ Nedostupné: %s",
           "why": {"quota": "limit", "key": "zlý kľúč", "error": "chyba"}},
    "en": {"ok": "Done", "fail": "Failed", "cancel": "Cancelled", "in": "in", "s": "s",
           "step": ("step", "steps", "steps"), "iter": ("iteration", "iterations", "iterations"),
           "req": ("request", "requests", "requests"), "tok": "tokens", "day": "today",
           "month": "this month", "left": "~%s of %s left (≈ %d tasks like this)",
           "nolimit": "limit unknown (set: export %s_DAILY_LIMIT=N)", "warn": "⚠ Unavailable: %s",
           "why": {"quota": "limit", "key": "bad key", "error": "error"}},
}


def _plural(n, forms, lang):
    n = int(n)
    if lang == "en":
        f = forms[0] if n == 1 else forms[1]
    else:
        f = forms[0] if n == 1 else forms[1] if 2 <= n <= 4 else forms[2]
    return "%d %s" % (n, f)


def _num(n):
    return "{:,}".format(int(n)).replace(",", " ")


def ai_lines(infos, lang="sk"):
    """-> zoznam riadkov o AI (co zlyhalo, kto odpovedal, spotreba, kolko zostava)"""
    T = AI_TXT.get(lang, AI_TXT["sk"])
    failed, per, out = {}, {}, []
    for i in infos:
        for name, why in i.get("failed") or []:
            failed.setdefault(name, why)
        s = i.get("service")
        if s:
            p = per.setdefault(s, [0, 0, None, 0, 0, 0.0])
            p[0] += 1
            p[1] += int(i.get("tokens") or 0)
            p[2] = i
            p[3] += int(i.get("tokens_in") or 0)
            p[4] += int(i.get("tokens_out") or 0)
            p[5] += float(i.get("cost") or 0.0)
    if failed:
        out.append(T["warn"] % ", ".join("%s (%s)" % (AI_NAMES.get(n, n), T["why"].get(w, w)) for n, w in failed.items()))
    for s, (cnt, tok, last, t_in, t_out, cost) in per.items():
        head = AI_NAMES.get(s, s) + ((" (%s)" % last["model"]) if last.get("model") else "")
        parts = [head, _plural(cnt, T["req"], lang)]
        if tok:
            parts.append("%s %s" % (_num(tok), T["tok"]) + (" (%s in / %s out)" % (_num(t_in), _num(t_out)) if t_in or t_out else ""))
        if cost > 0:
            parts.append("≈ $%.4f" % cost)
        out.append("AI: " + " · ".join(parts))
        used = last.get("used")
        if used is not None:
            st = ["%s: %s" % (T["month"] if last.get("period") == "month" else T["day"], _plural(used, T["req"], lang))]
            lim, rem = last.get("limit"), last.get("remaining")
            if rem is not None and lim:
                st.append(T["left"] % (_num(rem), _num(lim), rem // max(1, cnt)))
            else:
                st.append(T["nolimit"] % s.upper())
            out.append("    " + " · ".join(st))
    return out


def result_lines(r, secs, infos, lang="sk", cancelled=False):
    """-> [(farba, text)]  r: objekt s poliami success, status, summary, steps, iterations"""
    T = AI_TXT.get(lang, AI_TXT["sk"])
    if cancelled or r.status == "cancelled":
        mark, col, word = "✗", "33", T["cancel"]
    elif r.success:
        mark, col, word = "✓", "32", T["ok"]
    else:
        mark, col, word = "✗", "31", T["fail"] + (" (%s)" % r.status if r.status else "")
    head = "%s %s %s %.0f %s" % (mark, word, T["in"], secs, T["s"])
    if r.steps or r.iterations:
        head += "  (%s, %s)" % (_plural(r.steps, T["step"], lang), _plural(r.iterations, T["iter"], lang))
    out = [(col, head), (None, "  " + r.summary)]
    for l in ai_lines(infos, lang):
        out.append(("33" if l.startswith("⚠") else "90", "  " + l))
    return out


def _test_png():
    import struct
    import zlib

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    raw = b"".join(b"\x00" + b"\xff\x00\x00" * 64 for _ in range(64))       # 64x64 cerveny stvorec
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 64, 64, 8, 2, 0, 0, 0)) + \
        chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")


if __name__ == "__main__":
    load_keys()
    png = _test_png()
    for nm, build in (("gemini", _gemini), ("mistral", _mistral), ("groq", _groq)):
        llm = build()
        if llm is None:
            print("%-8s bez kluca - preskakujem" % nm)
            continue
        try:
            r = llm.ask('Reply ONLY with a JSON object {"colour": "<main colour of the image>"}.',
                        "What is the main colour of the image?", [(png, "image/png", "test image")])
            print("%-8s OK: %s" % (nm, r.strip()[:120]))
        except Exception as e:
            print("%-8s CHYBA: %s" % (nm, str(e)[:300]))
