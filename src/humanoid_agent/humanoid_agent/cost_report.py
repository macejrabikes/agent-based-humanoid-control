#!/usr/bin/env python3
"""chappie_cost: prehlad tokenov a nakladov AI z ~/.chappie_cost.json (iba stdlib).

  chappie_cost             dnes + poslednych 7 dni + spolu
  chappie_cost 30          poslednych 30 dni
  chappie_cost --reset     zmaze statistiku (po potvrdeni)
Ceny (USD za 1 mil. tokenov) sa citaju z <SLUZBA>_PRICE_IN / _PRICE_OUT (env alebo ~/.chappie_keys); bez nich 0 (free tier).
"""
import json
import os
import sys
import time

COST_FILE = os.path.expanduser("~/.chappie_cost.json")
KEYS_FILE = os.path.expanduser("~/.chappie_keys")


def load_prices():
    try:
        with open(KEYS_FILE) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.replace("export ", "", 1).split("=", 1)
                    k, v = k.strip(), v.strip().strip("'\"")
                    if "_PRICE_" in k and k not in os.environ:      # kluce API sa tu zamerne necitaju
                        os.environ[k] = v
    except OSError:
        pass


def price(name):
    try:
        return (float(os.environ.get(name.upper() + "_PRICE_IN", "0") or 0),
                float(os.environ.get(name.upper() + "_PRICE_OUT", "0") or 0))
    except ValueError:
        return 0.0, 0.0


def num(n):
    return "{:,}".format(int(n)).replace(",", " ")


def summarize(days, keys):
    """-> {sluzba: [poziadavky, vstup, vystup, cena]} za vybrane dni"""
    out = {}
    for k in keys:
        for svc, models in (days.get(k) or {}).items():
            row = out.setdefault(svc, [0, 0, 0, 0.0])
            pin, pout = price(svc)
            for m in models.values():
                row[0] += m.get("req", 0)
                row[1] += m.get("in", 0)
                row[2] += m.get("out", 0)
                row[3] += (m.get("in", 0) * pin + m.get("out", 0) * pout) / 1e6
    return out


def show(title, rows):
    print(title)
    if not rows:
        print("  (ziadne poziadavky)")
        return
    print("  %-10s %9s %12s %12s %10s" % ("sluzba", "poziad.", "vstup tok.", "vystup tok.", "cena USD"))
    tot = [0, 0, 0, 0.0]
    for svc, r in sorted(rows.items()):
        print("  %-10s %9s %12s %12s %10.4f" % (svc, num(r[0]), num(r[1]), num(r[2]), r[3]))
        tot = [a + b for a, b in zip(tot, r)]
    if len(rows) > 1:
        print("  %-10s %9s %12s %12s %10.4f" % ("spolu", num(tot[0]), num(tot[1]), num(tot[2]), tot[3]))
    print("  priemer na poziadavku: %s tokenov" % num((tot[1] + tot[2]) / max(1, tot[0])))


def main():
    load_prices()
    args = sys.argv[1:]
    if "--reset" in args:
        if input("Naozaj zmazat %s? [a/N] " % COST_FILE).strip().lower() in ("a", "y", "ano", "yes"):
            try:
                os.remove(COST_FILE)
                print("Zmazane.")
            except OSError:
                print("Subor neexistuje.")
        return
    try:
        with open(COST_FILE) as f:
            days = json.load(f).get("days", {})
    except (OSError, ValueError):
        days = {}
    if not days:
        print("Zatial ziadne data (%s). Zapisu sa po prvej ulohe pre agenta." % COST_FILE)
        return
    n = int(args[0]) if args and args[0].isdigit() else 7
    today = time.strftime("%Y-%m-%d")
    recent = sorted(days)[-n:]
    show("Dnes (%s)" % today, summarize(days, [today]))
    show("\nPoslednych %d dni s aktivitou" % len(recent), summarize(days, recent))
    show("\nSpolu od zaciatku merania (%s)" % sorted(days)[0], summarize(days, sorted(days)))
    if not any(os.environ.get(k) for k in os.environ if "_PRICE_" in k):
        print("\nCeny nie su nastavene (free tier = 0 USD). Nastav napr. GEMINI_PRICE_IN=0.30 a GEMINI_PRICE_OUT=2.50 v ~/.chappie_keys.")


if __name__ == "__main__":
    main()
