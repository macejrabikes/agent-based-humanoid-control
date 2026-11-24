"""perception: jednoduchy detektor predmetov podla FARBY z obrazu kamery (bez AI, bez LLM; iba PIL a stdlib).

Z obrazu vyberie pixely danej farby (odtien, sytost a jas v HSV), najde najvacsi suvisly usek stlpcov a vrati jeho stred
v percentach sirky obrazu (rovnaka stupnica ako pravitko 0-100, teda priamo pouzitelne pre turn_towards / walk_to_object).
Siva (steny, podlaha) a biela sa nikdy nepovazuju za farbu. Pravitko dole a zltu znacku stredu hore obraz oreze.

  python3 perception.py obrazok.png blue          # vypise, kde je modry predmet
  python3 perception.py obrazok.png all           # vsetky farby
"""
import io
import sys

# odtien v stupnoch (0-360); "red" a "orange" sa prekryvaju: oranzovo-cerveny predmet je v oboch
HUES = {
    "red": [(335, 360), (0, 22)],
    "orange": [(12, 42)],
    "yellow": [(42, 72)],
    "green": [(72, 165)],
    "cyan": [(165, 200)],
    "blue": [(200, 262)],
    "purple": [(262, 345)],
    "pink": [(300, 345)],
}
ALIASES = {
    "red": ("red", "cerven", "rud"), "orange": ("orange", "oranz"), "yellow": ("yellow", "zlt", "zlta"),
    "green": ("green", "zelen"), "cyan": ("cyan", "tyrkys", "azur", "svetlomodr"), "blue": ("blue", "modr"),
    "purple": ("purple", "violet", "fialov", "magenta"), "pink": ("pink", "ruzov"),
}
MIN_SAT = 100          # 0-255: nizsia sytost = siva/biela
MIN_VAL = 55           # 0-255: tmavsie pixely su tien/ciern
MIN_FRAC = 0.0012      # najmensi usek: podiel plochy obrazu
MIN_COLS = 3           # a aspon tolko stlpcov (pri sirke 320 px)


def normalize_color(text):
    """'Modrá' / 'blue' / 'modry box' -> 'blue'; neznama farba -> None"""
    import unicodedata
    t = "".join(c for c in unicodedata.normalize("NFD", str(text).lower()) if unicodedata.category(c) != "Mn")
    for key, words in ALIASES.items():
        if any(w in t for w in words):
            return key
    return None


def _mask_for(hsv, color):
    h, s, v = hsv
    ranges = HUES[color]
    sat_ok = (s >= MIN_SAT) & (v >= MIN_VAL)
    hm = None
    for lo, hi in ranges:
        a, b = lo * 255.0 / 360.0, hi * 255.0 / 360.0
        m = (h >= a) & (h <= b)
        hm = m if hm is None else (hm | m)
    return sat_ok & hm


def detect(data, color, width=320):
    """data = bajty PNG/JPEG; vrati dict(found, percent, width_pct, frac, bottom, n) alebo {'found': False}"""
    from PIL import Image
    try:
        import numpy as np
    except ImportError:
        np = None
    color = normalize_color(color) or color
    if color not in HUES:
        return {"found": False, "error": "unknown colour '%s'" % color}
    im = Image.open(io.BytesIO(data)).convert("RGB")
    w0, h0 = im.size
    band = max(18, w0 // 30)                       # pravitko dole (rovnaky vzorec ako draw_ruler)
    im = im.crop((0, 12, w0, max(13, h0 - band - 4)))
    h1 = im.size[1]
    im = im.resize((width, max(8, int(round(h1 * width / float(w0))))))
    hsv_im = im.convert("HSV")
    w, h = im.size
    if np is not None:
        arr = np.asarray(hsv_im, dtype=np.int32)
        hsv = (arr[:, :, 0], arr[:, :, 1], arr[:, :, 2])
        mask = _mask_for(hsv, color)
        col = mask.sum(axis=0).astype(int).tolist()

        def y_max(a, b):
            ys = np.nonzero(mask[:, a:b + 1].any(axis=1))[0]
            return (int(ys.max()) + 1) if len(ys) else 0
    else:                                          # bez numpy (pomalsie)
        px = list(hsv_im.getdata())
        rng = [(lo * 255.0 / 360.0, hi * 255.0 / 360.0) for lo, hi in HUES[color]]
        col, pts = [0] * w, []
        for i, (hh, ss, vv) in enumerate(px):
            if ss >= MIN_SAT and vv >= MIN_VAL and any(a <= hh <= b for a, b in rng):
                x, y = i % w, i // w
                col[x] += 1
                pts.append((x, y))

        def y_max(a, b):
            return max([y + 1 for x, y in pts if a <= x <= b] or [0])
    # suvisle useky stlpcov (medzera do 2 stlpcov sa preskoci)
    runs, start, gap = [], None, 0
    for x in range(w):
        if col[x] > 0:
            if start is None:
                start = x
            gap = 0
        elif start is not None:
            gap += 1
            if gap > 2:
                runs.append((start, x - gap))
                start = None
    if start is not None:
        runs.append((start, w - 1 - gap if gap else w - 1))
    best = None
    for a, b in runs:
        area = sum(col[a:b + 1])
        if (b - a + 1) >= MIN_COLS * width / 320.0 and area / float(w * h) >= MIN_FRAC:
            if best is None or area > best[2]:
                best = (a, b, area)
    n = sum(1 for a, b in runs if (b - a + 1) >= MIN_COLS * width / 320.0 and sum(col[a:b + 1]) / float(w * h) >= MIN_FRAC)
    if best is None:
        return {"found": False}
    a, b, area = best
    bottom = y_max(a, b) / float(h)
    return {"found": True, "color": color, "percent": 100.0 * ((a + b + 1) / 2.0) / w, "width_pct": 100.0 * (b - a + 1) / w,
            "frac": area / float(w * h), "bottom": bottom, "n": n}


def annotate(data, det, out_path):
    """pre ladenie: ulozi obraz s ciarou na zistenej polohe"""
    from PIL import Image, ImageDraw
    im = Image.open(io.BytesIO(data)).convert("RGB")
    if det.get("found"):
        w, h = im.size
        x = int(det["percent"] / 100.0 * w)
        d = ImageDraw.Draw(im)
        d.line([x, 0, x, h], fill=(255, 0, 255), width=3)
        d.text((x + 4, 14), "%s %.0f%%" % (det["color"], det["percent"]), fill=(255, 0, 255))
    im.save(out_path)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    img = open(sys.argv[1], "rb").read()
    for c in (list(HUES) if sys.argv[2] == "all" else [sys.argv[2]]):
        r = detect(img, c)
        print(c, ("%.0f %% sirka %.0f %% plocha %.1f %% (%d usekov)" % (r["percent"], r["width_pct"], 100 * r["frac"], r["n"])) if r.get("found") else "-")
