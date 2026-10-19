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
            return json.loads(r.read().decode("utf-8"))
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
    m = re.search(r"retry in (?:(\d+)h)?(?:(\d+)m)?(?:([0-9.]+)s)?", msg)
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
                if w is not None and w > 20:
                    raise                      # denny limit: cakat nema zmysel
                if w is not None:
                    delay = max(delay, w + 1)
        except (urllib.error.URLError, TimeoutError, ConnectionError):
            if k == tries - 1:
                raise RuntimeError("Siet: nedostupna sluzba AI")
        print("[gemini] docasna chyba, dalsi pokus %d/%d o %.0f s" % (k + 2, tries, delay), flush=True)
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


def make_llm(provider, model="", **kw):
    p = provider.lower()
    if p in ("gemini", "google"):
        return GeminiLLM(model or "gemini-3.6-flash", **kw)
    if p in ("claude", "anthropic"):
        return ClaudeLLM(model or "claude-sonnet-5-5", **kw)
    raise ValueError("Neznamy provider '%s' (gemini | claude)" % provider)
