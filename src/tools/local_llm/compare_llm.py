#!/usr/bin/env python3
"""Porovnanie AI sluzieb (Gemini vs. lokalna Ollama ...) na jednoduchych ulohach agenta Chappie.

Pouziva SKUTOCNY systemovy prompt z agent_core.py a skutocny klient z llm_backends.py (po add_ollama.py),
posle 11 jednoduchych uloh (kazda = 1 prompt so snimkou) a pre kazdu sluzbu vypise:
  platny JSON | plan prejde validaciou | plan robi to, co sa chcelo | priemerny cas | tokeny
Nehybe robotom ani Gazebom. Pouzitie (na VM, kde je repozitar):
    python3 compare_llm.py gemini ollama --pause 5       # kazda sluzba samostatne (bez preskakovania v retazi)
    python3 compare_llm.py ollama --lite --n 1            # kratky prompt bez obrazka (male textove modely)
    python3 compare_llm.py ollama --n 1                   # rychly test, len 1 opakovanie
    python3 compare_llm.py ollama --catalog katalog.json  # presny katalog z robota (ros2 topic echo /skills/catalog --once --field data > katalog.json)
    python3 compare_llm.py ollama --image snimka.png      # vlastna snimka (inak 64x64 testovacia)
Bez --catalog sa pouzije ZJEDNODUSENY katalog (skrateny zoznam skills), takze skutocny prompt bude dlhsi a lokalny model pomalsi.
Nastavenia: CHAPPIE_WS (predvolene ~/agent-based-humanoid-control); kluce v ~/.chappie_keys; OLLAMA_MODEL=... pre Ollamu.
Vysledky sa ulozia do compare_vysledky.json.
"""
import argparse
import json
import os
import sys
import time

WS = os.path.expanduser(os.environ.get("CHAPPIE_WS", "~/agent-based-humanoid-control"))
sys.path.insert(0, os.path.join(WS, "src/humanoid_agent"))
from humanoid_agent import agent_core, llm_backends  # noqa: E402


def sk(name, desc, unit=None, lo=0, hi=0, default=0):
    d = {"name": name, "description": desc}
    if unit:
        d["parameter"] = {"name": "value", "unit": unit, "min": lo, "max": hi, "default": default}
    return d


CATALOG = [
    sk("stop", "Zastavi pohyb a robot ostane stat."),
    sk("walk_forward", "Prejde dopredu o danu vzdialenost.", "m", 0.1, 5.0, 1.0),
    sk("walk_to_obstacle", "Walks forward until a wall or obstacle is about 0.7 m in front of the robot (LiDAR) and stops there. "
       "'value' is the MAXIMUM distance it may walk.", "m", 0.5, 8.0, 5.0),
    sk("walk_backward", "Prejde dozadu o danu vzdialenost.", "m", 0.1, 2.0, 0.5),
    sk("step_left", "Posunie sa bokom dolava.", "m", 0.1, 2.0, 0.5),
    sk("step_right", "Posunie sa bokom doprava.", "m", 0.1, 2.0, 0.5),
    sk("turn_left", "Otoci sa dolava o dany uhol.", "deg", 5, 180, 90),
    sk("turn_right", "Otoci sa doprava o dany uhol.", "deg", 5, 180, 90),
    sk("turn_towards", "Turns on the spot so that something you SEE in the picture ends up straight ahead. 'value' = horizontal "
       "position of the object in PERCENT read on the ruler (1 = left edge, 50 = centre, 100 = right edge).", "%", 1, 100, 50),
    sk("raise_right_arm", "Zdvihne pravu ruku."), sk("raise_left_arm", "Zdvihne lavu ruku."),
    sk("raise_both_arms", "Zdvihne obe ruky."), sk("arms_down", "Spusti ruky dole."),
    sk("get_up", "Use when the robot has FALLEN: it stands up by itself (~10 s)."),
    sk("wave", "Waves hello with the RIGHT hand, 'value' = seconds of waving (default 4).", "s", 2, 30, 4),
    sk("cheer", "Cheering: raises BOTH arms above the head. 'value' = seconds (default 4).", "s", 2, 30, 4),
    sk("clap", "Claps hands in front of the chest. 'value' = seconds of clapping (default 4).", "s", 2, 30, 4),
    sk("bow", "Bows: leans the upper body forward, holds it for 'value' seconds (default 2) and comes back up.", "s", 1, 10, 2),
    sk("dance", "Dances on the spot. 'value' = seconds of dancing (default 8).", "s", 3, 60, 8),
    sk("push_forward", "Pushes an object in front of the robot with both arms for the given number of metres.", "m", 0.2, 3.0, 1.0),
]


def total(acts, skill):
    return sum(a["value"] for a in acts if a["skill"] == skill)


def only(acts, skill, lo, hi):
    ks = [a for a in acts if a["skill"] != "look"]
    return bool(ks) and all(a["skill"] == skill for a in ks) and lo <= total(ks, skill) <= hi


def seq(acts, spec):
    ks = [a for a in acts if a["skill"] != "look"]
    if [a["skill"] for a in ks[:len(spec)]] != [s[0] for s in spec]:
        return False
    for a, (_, lo, hi) in zip(ks, spec):
        if not (lo <= a["value"] <= hi):
            return False
    return True


def names(acts, want):
    return [a["skill"] for a in acts if a["skill"] != "look"] == want


TASKS = [
    ("Otoč sa doľava o 90 stupňov.", lambda a: only(a, "turn_left", 85, 95)),
    ("Turn right 45 degrees.", lambda a: only(a, "turn_right", 40, 50)),
    ("Choď 2 metre dopredu.", lambda a: only(a, "walk_forward", 1.8, 2.2)),
    ("Zamávaj pravou rukou 5 sekúnd.", lambda a: only(a, "wave", 4, 6)),
    ("Urob poklonu.", lambda a: names(a, ["bow"])),
    ("Choď dozadu pol metra.", lambda a: only(a, "walk_backward", 0.4, 0.6)),
    ("Otoč sa doprava o 180 stupňov a potom choď 1 meter dopredu.",
     lambda a: seq(a, [("turn_right", 170, 180), ("walk_forward", 0.9, 1.1)])),
    ("Zdvihni obe ruky.", lambda a: names(a, ["raise_both_arms"])),
    ("Zatancuj 10 sekúnd.", lambda a: only(a, "dance", 8, 12)),
    ("Zatlieskaj.", lambda a: names(a, ["clap"])),
    ("Skoč do výšky.", lambda a: all(x["skill"] not in ("walk_forward", "walk_backward") for x in a)),   # nepodporovane: nesmie vymyslat pohyb
]


def user_text(task):
    return ("TASK: %s\n\nPrevious actions and results:\n(none yet)\n\nIteration 1 of 10. You receive 1 image(s). "
            "Look at them and decide.\n\nNOW: heading 0 deg, x 0.0 m, y 0.0 m; LIDAR: front >10 m, left 3.2 m, right 2.8 m, back 1.5 m" % task)


def run(provider, tasks, n, system, image, catalog, pause=0.0, lite=False):
    os.environ["CHAPPIE_CHAIN"] = provider           # iba tato sluzba, bez preskakovania na inu
    llm = llm_backends.make_llm("chain")
    llm_backends._DEAD.clear()
    rows, t_all, tok_in, tok_out = [], [], 0, 0
    ok_json = ok_valid = ok_task = 0
    for task, check in tasks:
        for _ in range(n):
            time.sleep(pause)                        # kvoli limitu pozadaviek za minutu (Gemini free ~15/min)
            t0 = time.time()
            try:
                reply = llm.ask(system, ("TASK: " + task) if lite else user_text(task), [] if lite else [(image, "image/png", "CURRENT VIEW")])
            except Exception as e:
                rows.append({"task": task, "error": str(e)[:200]})
                print("  ✗ %-55s CHYBA: %s" % (task[:55], str(e)[:90]), flush=True)
                continue
            dt = time.time() - t0
            t_all.append(dt)
            info = llm.last_info or {}
            tok_in += int(info.get("tokens_in") or 0)
            tok_out += int(info.get("tokens_out") or 0)
            j = v = c = False
            acts = []
            try:
                plan = agent_core.parse_json(reply)
                j = True
                acts, errs = agent_core.validate_actions(plan.get("actions", []), catalog, 20, "sk")
                v = not errs
                c = v and bool(check(acts))
            except Exception:
                pass
            ok_json += j
            ok_valid += v
            ok_task += c
            rows.append({"task": task, "json": j, "valid": v, "correct": c, "s": round(dt, 1), "actions": acts})
            print("  %s %-55s %5.1f s  %s" % ("✓" if c else "✗", task[:55], dt,
                                              " ".join("%s(%g)" % (a["skill"], a["value"]) for a in acts)[:60]), flush=True)
    cnt = len(tasks) * n
    return {"provider": provider, "model": str(getattr(llm.backends[0][1], "model", "")), "requests": cnt,
            "json": ok_json, "valid": ok_valid, "correct": ok_task,
            "avg_s": round(sum(t_all) / len(t_all), 1) if t_all else None,
            "median_s": round(sorted(t_all)[len(t_all) // 2], 1) if t_all else None,
            "tok_in": tok_in // max(1, len(t_all)), "tok_out": tok_out // max(1, len(t_all)), "rows": rows}


def lite_prompt(catalog):
    """kratky prompt (~600 tokenov) bez obrazka: pre male lokalne modely a jednoduche textove ulohy"""
    sk_ = []
    for s in list(catalog):
        p = s.get("parameter")
        sk_.append("%s(%s %g-%g, default %g)" % (s["name"], p["unit"], p["min"], p["max"], p["default"]) if p else s["name"])
    return (
        "You control a humanoid robot with ready-made skills. Answer with ONE JSON object and nothing else:\n"
        '{"actions": [{"skill": "<name>", "value": <number>}], "done": true, "answer": "short message"}\n'
        "Rules: use ONLY the skills listed below; 'value' must be inside the given range, omit it for skills without a parameter. "
        "A turn of any angle is ONE turn_left/turn_right action. Walking farther than 1 m: use one walk_forward with that value. "
        "Use exactly the numbers given in the task and do only what the task asks. Plan the whole task at once. If the task cannot be done with these skills, return \"actions\": [] and explain in 'answer'.\n"
        'Example: task "Turn left 30 degrees and wave for 3 seconds" -> {'
        '"actions": [{"skill": "turn_left", "value": 30}, {"skill": "wave", "value": 3}], "done": true, "answer": "Done."}\n'
        "Skills: " + "; ".join(sk_))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("providers", nargs="+", help="gemini | groq | mistral | openrouter | ollama")
    ap.add_argument("--n", type=int, default=2, help="opakovania kazdej ulohy (predvolene 2)")
    ap.add_argument("--pause", type=float, default=0.0,
                    help="pauza medzi poziadavkami v s (pre Gemini free daj 5, inak narazi na limit 15/min a priemer je skresleny)")
    ap.add_argument("--catalog", help="JSON katalog skills (z /skills/catalog)")
    ap.add_argument("--lite", action="store_true",
                    help="kratky prompt (~600 tokenov) a bez obrazka - test, ci by maly textovy model stacil na jednoduche ulohy")
    ap.add_argument("--image", help="PNG/JPEG snimka (predvolene testovacia)")
    a = ap.parse_args()
    catalog = json.load(open(a.catalog)) if a.catalog else CATALOG
    if isinstance(catalog, str):
        catalog = json.loads(catalog)
    system = lite_prompt(catalog) if a.lite else \
        agent_core.SYSTEM_PROMPT.format(max_actions=20, catalog=json.dumps(list(catalog) + [agent_core.LOOK], ensure_ascii=False))
    image = open(a.image, "rb").read() if a.image else llm_backends._test_png()
    llm_backends.load_keys()
    res = []
    for p in a.providers:
        print("== %s" % p, flush=True)
        try:
            res.append(run(p, TASKS, a.n, system, image, catalog, a.pause, a.lite))
        except Exception as e:
            print("  preskakujem %s: %s" % (p, e))
    print("\n%-12s %-22s %6s %6s %6s %8s %8s %12s" % ("sluzba", "model", "JSON", "valid", "spravne", "median", "priemer", "tokeny in/out"))
    for r in res:
        print("%-12s %-22s %3d/%-3d %3d/%-3d %3d/%-3d %6s s %6s s %6s/%s" % (
            r["provider"], r["model"][:22], r["json"], r["requests"], r["valid"], r["requests"], r["correct"], r["requests"],
            r["median_s"], r["avg_s"], r["tok_in"], r["tok_out"]))
    json.dump(res, open("compare_vysledky.json", "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
