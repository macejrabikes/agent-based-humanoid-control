#!/usr/bin/env python3
"""Meranie lokalneho modelu cez Ollama (iba stdlib). Pouzitie:
    ollama pull gemma3:4b ; ollama pull qwen2.5vl:7b
    python3 bench_ollama.py gemma3:4b qwen2.5vl:7b [--prompt-tokens 6000] [--out 200] [--n 3]
Pre kazdy model zmeria: prefill (tok/s), generovanie (tok/s), cas jednej poziadavky
(studeny prompt aj prompt s rovnakou predponou = KV cache) a odhad casu pre 500 poziadaviek.
Vystup sa ulozi aj do bench_vysledky.json."""
import argparse, json, time, urllib.request

import os
HOST = os.environ.get("OLLAMA_BASE", "http://localhost:11434")      # napr. OLLAMA_BASE=http://192.168.x.1:11434 (Ollama na hostiteli)

def call(model, prompt, num_predict, num_ctx):
    body = json.dumps({"model": model, "prompt": prompt, "stream": False,
                       "options": {"num_predict": num_predict, "num_ctx": num_ctx, "temperature": 0}}).encode()
    t = time.time()
    r = urllib.request.urlopen(urllib.request.Request(HOST + "/api/generate", body,
                               {"Content-Type": "application/json"}), timeout=1800)
    js = json.loads(r.read())
    js["wall_s"] = time.time() - t
    return js

def make_prompt(tokens, tail, nonce=""):
    base = "Katalog skillov: walk_forward(m), turn_left(deg), turn_right(deg), wait(s), push_forward(m). "
    # priblizne 1 token ~ 4 znaky
    s = (base * (tokens * 4 // len(base) + 1))[: tokens * 4]
    return nonce + s + "\nUloha: " + tail + "\nOdpovedz iba JSON planom {\"actions\":[...]}."

def stats(js):
    pe = js.get("prompt_eval_count", 0); pd = js.get("prompt_eval_duration", 1) / 1e9
    ge = js.get("eval_count", 0); gd = js.get("eval_duration", 1) / 1e9
    return {"prompt_tok": pe, "prefill_tps": pe / pd if pd else 0, "gen_tok": ge,
            "gen_tps": ge / gd if gd else 0, "wall_s": js["wall_s"]}

ap = argparse.ArgumentParser()
ap.add_argument("models", nargs="+")
ap.add_argument("--prompt-tokens", type=int, default=6000)
ap.add_argument("--out", type=int, default=200)
ap.add_argument("--n", type=int, default=3)
a = ap.parse_args()
res = {}
for m in a.models:
    print("== %s" % m)
    call(m, "ahoj", 5, 2048)  # zahrievacie nacitanie modelu
    cold, warm = [], []
    for i in range(a.n):
        cold.append(stats(call(m, make_prompt(a.prompt_tokens, "otoc sa o %d stupnov" % (10 + i), "%s-%d " % (time.time(), i)), a.out, a.prompt_tokens + 1024)))
    # rovnaka predpona (systemovy prompt), meni sa len koniec -> Ollama znovu pouzije KV cache
    for i in range(a.n):
        warm.append(stats(call(m, make_prompt(a.prompt_tokens, "prejdi %d metrov dopredu" % (i + 1)), a.out, a.prompt_tokens + 1024)))
    avg = lambda L, k: sum(x[k] for x in L) / len(L)
    r = {"cold_s": avg(cold, "wall_s"), "warm_s": avg(warm, "wall_s"),
         "prefill_tps": avg(cold, "prefill_tps"), "gen_tps": avg(cold, "gen_tps")}
    # realisticky cas: prefill z merania + presne `--out` vygenerovanych tokenov (skutocny plan ma ~200 tokenov)
    r["real_cold_s"] = avg(cold, "wall_s") - avg(cold, "gen_tok") / avg(cold, "gen_tps") + a.out / avg(cold, "gen_tps")
    r["real_warm_s"] = avg(warm, "wall_s") - avg(warm, "gen_tok") / avg(cold, "gen_tps") + a.out / avg(cold, "gen_tps")
    r["avg_gen_tok"] = avg(cold, "gen_tok")
    r["500_cold_min"] = 500 * r["cold_s"] / 60
    r["500_warm_min"] = 500 * r["warm_s"] / 60
    res[m] = r
    print("  prefill %.0f tok/s, generovanie %.1f tok/s" % (r["prefill_tps"], r["gen_tps"]))
    print("  1 poziadavka: %.1f s (studena), %.1f s (s KV cache)" % (r["cold_s"], r["warm_s"]))
    print("  model vygeneroval priemerne %.0f tokenov" % r["avg_gen_tok"])
    print("  500 poziadaviek: %.0f min (studena), %.0f min (cache)" % (r["500_cold_min"], r["500_warm_min"]))
    print("  s planom %d tokenov: %.0f s / %.0f s na poziadavku -> 500 = %.1f h / %.1f h"
          % (a.out, r["real_cold_s"], r["real_warm_s"], 500 * r["real_cold_s"] / 3600, 500 * r["real_warm_s"] / 3600))
json.dump(res, open("bench_vysledky.json", "w"), indent=1)
