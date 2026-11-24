#!/usr/bin/env python3
"""Hybrid: jednoduche ulohy riesi lokalna AI (Ollama), vsetko ostatne ide dalej v retazi (Gemini ...): llm_backends.py.

Nova sluzba v retazi 'ollama_lite' (spusti po add_ollama.py!). Zapnutie:
    export CHAPPIE_CHAIN=ollama_lite,gemini,groq,openrouter,mistral
    OLLAMA_MODEL=qwen2.5:7b      (v ~/.chappie_keys; volitelne OLLAMA_LITE_MODEL=... ak ma byt iny nez pre sluzbu 'ollama')

Lokalny model dostane KRATKY prompt (~450 tokenov: format, pravidla, zoznam skills, bez obrazka) a jeho plan sa prijme len ked:
  * je to PRVA iteracia ulohy (ziadna historia) a robot nelezi (LIDAR nehlasi FALLEN),
  * zadanie neobsahuje slova o videni / hladani / predmetoch / stenach / dverach (zoznam LOCAL_VISION),
  * odpoved je platny JSON, 'done' je true, plan ma 1-20 akcii, vsetky su z povolenej zakladnej sady
    (otocky, chodza, gesta, pozy rúk, wait, stop) a hodnoty su v limitoch katalogu.
Inak sa lokalna odpoved zahodi (alebo sa model vobec nevola) a ulohu odpovie dalsia sluzba v retazi (Gemini).
Skills zavisle od okolia (walk_to_obstacle, push_forward, turn_towards, go_through_opening ...) nikdy nejdu lokalne.
Vypis po ulohe ukaze "Ollama (lite)" ked odpovedala lokalna AI; dovod preskocenia sa vypise ako '[retaz] ollama_lite preskocena (...)'.
Idempotentny: zaloha *.before_hybrid, vzdy sa aplikuje od zalohy.
  python3 add_hybrid.py              # aplikuj
  python3 add_hybrid.py --restore    # vrat povodne
Potom: colcon build --symlink-install --packages-select humanoid_agent a reset agenta (chappie_sim).
Test bez robota:  python3 compare_llm.py "ollama_lite,gemini" --pause 5 --n 1
"""
import os
import shutil
import sys

WS = os.path.expanduser(os.environ.get("CHAPPIE_WS", "~/agent-based-humanoid-control"))
PATH = os.path.join(WS, "src/humanoid_agent/humanoid_agent/llm_backends.py")
SUF = ".before_hybrid"

CODE = '''# ===== Hybrid: lokalna AI pre jednoduche ulohy (pridal add_hybrid.py) =====
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
            "You control a humanoid robot with ready-made skills. Answer with ONE JSON object and nothing else:\\n"
            '{"actions": [{"skill": "<name>", "value": <number>}], "done": true, "answer": "short message"}\\n'
            "Rules: use ONLY the skills listed below; 'value' must be inside the given range, omit it for skills without a "
            "parameter. A turn of any angle is ONE turn_left/turn_right action. Walking farther than 1 m: use one walk_forward "
            "with that value. Use exactly the numbers given in the task and do only what the task asks. Plan the whole task at "
            "once. EVERY action named in the task must appear in the plan, in the order given; if a turn has no angle, use the "
            "default angle of the skill. If the task cannot be done with these skills, or you would need "
            'to SEE something to do it, return "actions": [] and explain in \\'answer\\'.\\n'
            'Example: task "Turn left 30 degrees and wave for 3 seconds" -> {"actions": [{"skill": "turn_left", "value": 30}, '
            '{"skill": "wave", "value": 3}], "done": true, "answer": "Done."}\\n'
            "Skills: " + "; ".join(sk_))

    def ask(self, system, user_text, image=None, mime=None):
        m = re.search(r"TASK: (.*?)\\n\\nPrevious actions and results:\\n(.*?)\\n\\nIteration (\\d+) of", user_text, re.S)
        if not m:
            self._skip("neznamy format zadania")
        task, hist, it = m.group(1).strip(), m.group(2).strip(), int(m.group(3))
        if it != 1 or hist != "(none yet)":
            self._skip("nie je prva iteracia")
        nm = re.search(r"\\nNOW: (.*)$", user_text, re.S)
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
        user = "TASK: " + task + (("\\nSensor readings: " + now) if now else "")
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
        lang = "en" if re.search(r"\\b(turn|walk|wave|raise|please|forward|backward|degrees?|seconds?|then|the)\\b", task, re.I) \
            else "sk"
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


def _gemini(model="", **kw):'''

EDITS = [
    ('def _gemini(model="", **kw):', CODE),
    ('''            elif name == "ollama":
                llm, nm = _ollama(), "ollama"
''',
     '''            elif name == "ollama":
                llm, nm = _ollama(), "ollama"
            elif name == "ollama_lite":
                llm, nm = _ollama_lite(), "ollama_lite"
'''),
    ('''            except Exception as e:
                err = e
                failed.append([name, _reason(str(e))])''',
     '''            except Exception as e:
                if "LOCAL_SKIP" in str(e):           # lokalna AI ulohu neprevzala -> bez varovania a bez blokovania, dalsia sluzba
                    print("[retaz] %s preskocena (%s) -> dalsia sluzba" % (name, str(e).split("LOCAL_SKIP:")[-1].strip()),
                          flush=True)
                    continue
                err = e
                failed.append([name, _reason(str(e))])'''),
    ('"ollama": "Ollama", "claude": "Claude"}', '"ollama": "Ollama", "ollama_lite": "Ollama (lite)", "claude": "Claude"}'),
    ('elif s != "ollama":', 'elif s not in ("ollama", "ollama_lite"):'),
]


def main():
    if "--restore" in sys.argv:
        if os.path.exists(PATH + SUF):
            shutil.copy2(PATH + SUF, PATH)
            print("obnovene:", PATH)
        return
    if not os.path.exists(PATH):
        sys.exit("chyba subor: %s (nastav CHAPPIE_WS)" % PATH)
    if not os.path.exists(PATH + SUF):
        shutil.copy2(PATH, PATH + SUF)
    text = open(PATH + SUF, encoding="utf-8").read()
    if "def _ollama(" not in text:
        sys.exit("najprv spusti add_ollama.py (chyba _ollama)")
    for old, new in EDITS:
        if text.count(old) != 1:
            sys.exit("nenasiel som presne 1x kotvu:\n%s" % old[:120])
        text = text.replace(old, new)
    open(PATH, "w", encoding="utf-8").write(text)
    print("upravene:", PATH)
    print("Zapni:  export CHAPPIE_CHAIN=ollama_lite,gemini,groq,openrouter,mistral   (a OLLAMA_MODEL=qwen2.5:7b v ~/.chappie_keys)")
    print("Potom: colcon build --symlink-install --packages-select humanoid_agent a reset chappie_sim")
    print('Test:  python3 compare_llm.py "ollama_lite,gemini" --pause 5 --n 1')


if __name__ == "__main__":
    main()
