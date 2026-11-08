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
    headers = dict(headers)
    headers.setdefault("User-Agent", "chappie-agent/1.0")      # UA_PATCH: Cloudflare blokuje predvoleny 'Python-urllib' (403, kod 1010)
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


def _tokens_cached(js):
    """kolko vstupnych tokenov poskytovatel vzal z cache (Gemini, OpenAI-kompatibilne sluzby, Claude)"""
    try:
        if "usageMetadata" in js:
            return int(js["usageMetadata"].get("cachedContentTokenCount", 0) or 0)
        u = js.get("usage", {})
        d = u.get("prompt_tokens_details") or {}
        return int(d.get("cached_tokens") or u.get("cache_read_input_tokens") or 0)
    except (AttributeError, TypeError, ValueError):
        return 0


def _price(name):
    """(cena vstup, cena vystup) v USD za 1 mil. tokenov z <SLUZBA>_PRICE_IN / _PRICE_OUT (predvolene 0 = free tier)"""
    try:
        return (float(os.environ.get(name.upper() + "_PRICE_IN", "0") or 0),
                float(os.environ.get(name.upper() + "_PRICE_OUT", "0") or 0))
    except ValueError:
        return 0.0, 0.0


def _cost_log(name, model, tin, tout, tcached=0):
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
    m["cached"] = int(m.get("cached", 0)) + int(tcached)
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
        body = {"model": model, "temperature": self.temperature, "max_tokens": getattr(self, "max_tokens", 4096),
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
                    if "HTTP 404" in msg or ("HTTP 400" in msg and "model" in low) or \
                            (self.name == "openrouter" and ("HTTP 400" in msg or "HTTP 402" in msg or "HTTP 403" in msg or "HTTP 429" in msg)):
                        continue                                # iny model (OpenRouter: model je obmedzeny alebo vytazeny)
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
                if "LOCAL_SKIP" in str(e):           # lokalna AI ulohu neprevzala -> bez varovania a bez blokovania, dalsia sluzba
                    print("[retaz] %s preskocena (%s) -> dalsia sluzba" % (name, str(e).split("LOCAL_SKIP:")[-1].strip()),
                          flush=True)
                    continue
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
            tcached = _tokens_cached(_RESP[1] or {})
            cost = _cost_log(name, str(getattr(llm, "model", "")), tin, tout, tcached)
            used, used_tok = _usage_update(name, tok)
            lim, rem = _rate(name, _RESP[0], used)
            self.last_info = {"service": name, "model": str(getattr(llm, "model", "")), "tokens": tok, "used": used,
                              "tokens_in": tin, "tokens_out": tout, "tokens_cached": tcached, "cost": cost,
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


def _openrouter_pick(data, n=8):
    """z odpovede /models vyberie BEZPLATNE chatovacie modely (vstup text+obrazok, vystup len text; najvacsi kontext prvy)"""
    out = []
    for m in data or []:
        try:
            p = m.get("pricing") or {}
            arch = m.get("architecture") or {}
            mods = arch.get("input_modalities") or []
            outs = arch.get("output_modalities") or ["text"]
            free = float(p.get("prompt", 1)) == 0 and float(p.get("completion", 1)) == 0
            if free and "image" in mods and "text" in mods and outs == ["text"]:
                out.append((int(m.get("context_length") or 0), m["id"]))
        except (AttributeError, TypeError, ValueError, KeyError):
            continue
    out.sort(reverse=True)
    return [i for _, i in out[:n]]


def _openrouter_models():
    try:
        req = urllib.request.Request("https://openrouter.ai/api/v1/models", headers={"User-Agent": "chappie-agent/1.0"})
        with urllib.request.urlopen(req, timeout=15) as r:
            return _openrouter_pick(json.loads(r.read().decode("utf-8")).get("data"))
    except Exception:
        return []


def _openrouter(model=""):
    key = os.environ.get("OPENROUTER_API_KEY")
    if not key:
        return None
    env = model or os.environ.get("OPENROUTER_MODEL", "")
    models = [m for m in env.replace(" ", "").split(",") if m] or _openrouter_models()
    if not models:
        return None
    return OpenAICompatLLM("openrouter", os.environ.get("OPENROUTER_URL", "https://openrouter.ai/api/v1/chat/completions"),
                           key, models, int(os.environ.get("OPENROUTER_MAX_IMAGES", "3")), True)


def _ollama(model=""):
    """lokalna AI cez Ollamu (OpenAI-kompatibilne API); zapina sa nastavenim OLLAMA_MODEL"""
    env = model or os.environ.get("OLLAMA_MODEL", "")
    models = [m for m in env.replace(" ", "").split(",") if m]
    if not models:
        return None
    llm = OpenAICompatLLM("ollama", os.environ.get("OLLAMA_URL", "http://localhost:11434/v1/chat/completions"),
                          "ollama", models, int(os.environ.get("OLLAMA_MAX_IMAGES", "2")), True,
                          timeout=int(os.environ.get("OLLAMA_TIMEOUT", "300")),
                          maxw=int(os.environ.get("OLLAMA_IMG_W", "512")))
    llm.max_tokens = int(os.environ.get("OLLAMA_MAX_TOKENS", "600"))      # lokalny model je pomaly: nepusti ho generovat donekonecna
    return llm


# ===== Hybrid: lokalna AI pre jednoduche ulohy (pridal add_hybrid.py) =====
LOCAL_OK = ("stop", "stand", "walk_forward", "walk_backward", "step_left", "step_right", "turn_left", "turn_right",
            "raise_right_arm", "raise_left_arm", "raise_both_arms", "arms_down", "wave", "wave_left", "wave_both",
            "point_forward", "thumbs_up", "cheer", "salute", "clap", "bow", "dance", "wait")
LOCAL_GLOSS = {"clap": "applause, clap hands", "cheer": "joy or victory, both arms up", "wave": "wave hello with right hand",
               "bow": "polite bow", "dance": "dance on the spot", "salute": "military salute", "stop": "stop moving",
               "point_forward": "point forward with the arm", "thumbs_up": "thumbs up"}
LOCAL_VISION = re.compile(
    r"(vid|farb|červen|cerven|modr|zelen|žlt|zlt|škatu|skatu|box|krabic|find|nájdi|najdi|hľad|hlad|search|look|pozri|pozer|"
    r"opíš|opis|describe|koľko|kolko|how many|count|dver|door|stien|wall|prekáž|prekaz|obstacle|k nej|k nemu|toward|"
    r"push|posu|tlač|tlac|miest|place|room|where|kde|sten|kym|kým|popri|okolo|vedla|until|along|around)", re.I)
# co zo zadania sa musi objavit v plane (inak plan nepokryva zadanie a ulohu dostane Gemini)
LOCAL_NEED = (
    (re.compile(r"oto[cč]|turn|rotate", re.I), ("turn_",)),
    (re.compile(r"prejd|cho[dď]|krok|walk|dopredu|dozadu|forward|backward|step", re.I), ("walk_", "step_")),
    (re.compile(r"zam[aá]v|wave", re.I), ("wave",)),
    (re.compile(r"tanc|dance", re.I), ("dance",)),
    (re.compile(r"tliesk|clap|potlesk", re.I), ("clap",)),
    (re.compile(r"poklon|bow", re.I), ("bow",)),
    (re.compile(r"zdvihni|raise|ruk[uy]", re.I), ("raise_", "arms_down")),
)


def _complex_task(task):
    """True, ak zadanie nevyzera ako jednoducha uloha (rovnaky zoznam slov ako kratky prompt, prompt_select.py); bez prompt_select False"""
    try:
        from . import prompt_select
    except ImportError:
        try:
            import prompt_select
        except ImportError:
            return False
    t = prompt_select.fold(task)
    return len(t) > 220 or bool(prompt_select.COMPLEX.search(t)) or not prompt_select.SIMPLE.search(t)


class LocalFirstLLM:
    """vola lokalnu AI s kratkym promptom; ak ulohu nema zvladnut, vyhodi LOCAL_SKIP a retaz pokracuje dalsou sluzbou"""

    def __init__(self, llm):
        self.llm = llm
        self.model = llm.model

    @staticmethod
    def _skip(why):
        raise RuntimeError("LOCAL_SKIP: " + why)

    @staticmethod
    def _lite_system(specs, names):
        sk_ = []
        for n in names:
            p = specs[n].get("parameter")
            t = "%s(%s %g-%g, default %g)" % (n, p["unit"], p["min"], p["max"], p["default"]) if p else n
            sk_.append(t + (" = " + LOCAL_GLOSS[n] if n in LOCAL_GLOSS else ""))
        return (
            "You control a humanoid robot with ready-made skills. Answer with ONE JSON object and nothing else:\n"
            '{"actions": [{"skill": "<name>", "value": <number>}], "done": true, "answer": "short message"}\n'
            "Rules: use ONLY the skills listed below; 'value' must be inside the given range, omit it for skills without a "
            "parameter. A turn of any angle is ONE turn_left/turn_right action. Walking farther than 1 m: use one walk_forward "
            "with that value. Use exactly the numbers given in the task and do only what the task asks. Plan the whole task at "
            "once. EVERY action named in the task must appear in the plan, in the order given; if a turn has no angle, use the "
            "default angle of the skill. If the task cannot be done with these skills, or you would need "
            'to SEE something to do it, return "actions": [] and explain in \'answer\'.\n'
            'Example: task "Turn left 30 degrees and wave for 3 seconds" -> {"actions": [{"skill": "turn_left", "value": 30}, '
            '{"skill": "wave", "value": 3}], "done": true, "answer": "Done."}\n'
            "Skills: " + "; ".join(sk_))

    def ask(self, system, user_text, image=None, mime=None):
        m = re.search(r"TASK: (.*?)\n\nPrevious actions and results:\n(.*?)\n\nIteration (\d+) of", user_text, re.S)
        if not m:
            self._skip("neznamy format zadania")
        task, hist, it = m.group(1).strip(), m.group(2).strip(), int(m.group(3))
        if it != 1 or hist != "(none yet)":
            self._skip("nie je prva iteracia")
        nm = re.search(r"\nNOW: (.*)$", user_text, re.S)
        now = nm.group(1).strip() if nm else ""
        if "FALLEN" in now.upper():
            self._skip("robot lezi")
        if LOCAL_VISION.search(task):
            self._skip("uloha vyzaduje obraz/okolie")
        if _complex_task(task):
            self._skip("zlozitejsie zadanie")
        i = system.find("Skill catalog (JSON):")
        try:
            specs = {s["name"]: s for s in json.loads(system[i + len("Skill catalog (JSON):"):].strip())}
        except (ValueError, KeyError, TypeError):
            self._skip("nepodarilo sa precitat katalog")
        names = [n for n in LOCAL_OK if n in specs]
        if not names:
            self._skip("katalog nema zakladne skills")
        user = "TASK: " + task + (("\nSensor readings: " + now) if now else "")
        raw = self.llm.ask(self._lite_system(specs, names), user, None)
        self.model = self.llm.model
        a, b = raw.find("{"), raw.rfind("}")
        try:
            plan = json.loads(raw[a:b + 1])
        except ValueError:
            self._skip("neplatny JSON")
        acts = plan.get("actions")
        if not isinstance(acts, list) or not acts or len(acts) > 20:
            self._skip("prazdny alebo prilis dlhy plan")
        if plan.get("done") is not True:
            self._skip("model nepovedal done=true")
        clean = []
        for x in acts:
            sp = specs.get(x.get("skill")) if isinstance(x, dict) else None
            if sp is None or x["skill"] not in names:
                self._skip("skill mimo zakladnej sady: %s" % (x.get("skill") if isinstance(x, dict) else x))
            if "parameter" in sp:
                try:
                    v = float(x.get("value", sp["parameter"]["default"]))
                except (TypeError, ValueError):
                    self._skip("hodnota nie je cislo")
                if not (sp["parameter"]["min"] <= v <= sp["parameter"]["max"]):
                    self._skip("hodnota mimo limitov: %s %g" % (x["skill"], v))
                clean.append({"skill": x["skill"], "value": v})
            else:
                clean.append({"skill": x["skill"]})
        for rx, pfx in LOCAL_NEED:
            has = any(a["skill"].startswith(pfx) for a in clean)
            if rx.search(task) and not has:
                self._skip("plan nepokryva zadanie (chyba %s)" % "/".join(pfx))
            if has and not rx.search(task):
                self._skip("plan robi viac nez zadanie (%s)" % "/".join(pfx))
        lang = "en" if re.search(r"\b(turn|walk|wave|raise|please|forward|backward|degrees?|seconds?|then|the)\b", task, re.I)             else "sk"
        return json.dumps({"observation": "", "thought": "", "actions": clean, "done": True,
                           "answer": "Done." if lang == "en" else "Hotovo.", "language": lang},       # vlastna odpoved, model pisal aj po cesky
                          ensure_ascii=False)


def _ollama_lite():
    llm = _ollama(os.environ.get("OLLAMA_LITE_MODEL", ""))
    if llm is None:
        return None
    llm.temperature = 0.0
    llm.max_tokens = int(os.environ.get("OLLAMA_LITE_MAX_TOKENS", "300"))
    return LocalFirstLLM(llm)


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
    if p == "openrouter":
        llm = _openrouter(model)
        if llm is None:
            raise RuntimeError("Chyba OPENROUTER_API_KEY (subor ~/.chappie_keys) alebo sa nenasiel bezplatny model s obrazkami")
        return ChainLLM([("openrouter", llm)])
    if p == "ollama":
        llm = _ollama(model)
        if llm is None:
            raise RuntimeError("Chyba OLLAMA_MODEL (napr. export OLLAMA_MODEL=qwen2.5vl:3b alebo riadok v ~/.chappie_keys)")
        return ChainLLM([("ollama", llm)])
    if p in ("gemini", "google", "chain", "auto"):
        order = os.environ.get("CHAPPIE_CHAIN", "gemini,groq,openrouter,mistral,ollama").lower().replace(" ", "")
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
            elif name == "openrouter":
                llm, nm = _openrouter(), "openrouter"
            elif name == "ollama":
                llm, nm = _ollama(), "ollama"
            elif name == "ollama_lite":
                llm, nm = _ollama_lite(), "ollama_lite"
            else:
                continue
            if llm is not None and nm not in [c[0] for c in chain]:
                chain.append((nm, llm))
        if not chain:
            raise RuntimeError("Chyba GEMINI_API_KEY (export GEMINI_API_KEY=...)")
        return ChainLLM(chain)
    raise ValueError("Neznamy provider '%s' (gemini | mistral | groq | openrouter | ollama | claude)" % provider)


# --- strucny vysledok ulohy + info o AI (pouziva task_client) ---

AI_NAMES = {"gemini": "Gemini", "mistral": "Mistral", "groq": "Groq", "openrouter": "OpenRouter", "ollama": "Ollama", "ollama_lite": "Ollama (lite)", "claude": "Claude"}

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
            p = per.setdefault(s, [0, 0, None, 0, 0, 0.0, 0])
            p[0] += 1
            p[1] += int(i.get("tokens") or 0)
            p[2] = i
            p[3] += int(i.get("tokens_in") or 0)
            p[4] += int(i.get("tokens_out") or 0)
            p[5] += float(i.get("cost") or 0.0)
            p[6] += int(i.get("tokens_cached") or 0)
    if failed:
        out.append(T["warn"] % ", ".join("%s (%s)" % (AI_NAMES.get(n, n), T["why"].get(w, w)) for n, w in failed.items()))
    for s, (cnt, tok, last, t_in, t_out, cost, t_cached) in per.items():
        head = AI_NAMES.get(s, s) + ((" (%s)" % last["model"]) if last.get("model") else "")
        parts = [head, _plural(cnt, T["req"], lang)]
        if tok:
            parts.append("%s %s" % (_num(tok), T["tok"]) + (" (%s in / %s out%s)" % (_num(t_in), _num(t_out), (", %s cache" % _num(t_cached)) if t_cached else "") if t_in or t_out else ""))
        if cost > 0:
            parts.append("≈ $%.4f" % cost)
        out.append("AI: " + " · ".join(parts))
        used = last.get("used")
        if used is not None:
            st = ["%s: %s" % (T["month"] if last.get("period") == "month" else T["day"], _plural(used, T["req"], lang))]
            lim, rem = last.get("limit"), last.get("remaining")
            if rem is not None and lim:
                st.append(T["left"] % (_num(rem), _num(lim), rem // max(1, cnt)))
            elif s not in ("ollama", "ollama_lite"):                 # lokalna AI nema limity
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
    for nm, build in (("gemini", _gemini), ("mistral", _mistral), ("groq", _groq), ("openrouter", _openrouter), ("ollama", _ollama)):
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
