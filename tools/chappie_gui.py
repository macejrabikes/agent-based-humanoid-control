#!/usr/bin/env python3
"""chappie_gui.py - ovladaci panel (okno) pre Chappieho. Iba stdlib (Python 3.8+), ziadna instalacia balikov.

Co to robi (vsetko pouziva existujuce nastroje projektu, nic nenahradza):
  - vlavo: vyber mapy, nahlad, Spustit/Zastavit simulaciu (gen_world.py + chappie_sim world:=...)
  - v strede: pole pre ulohu (`chappie_task`), plan AI po krokoch, odpoved agenta, Zrusit = Ctrl+C
  - vpravo: Primitiva (z8/primitives.py, `chappie_do`), Ulohy (z7/benchmark/tasks.json), Log

Spustenie (v terminali, kde funguje chappie_sim, t.j. po `source ~/.bashrc`):
    python3 chappie_gui.py                 # otvori okno (Chrome/Chromium v "app" rezime, inak prehliadac)
    python3 chappie_gui.py --demo          # bez ROS/Gazeba/AI: falosny robot na vyskusanie okna
    python3 chappie_gui.py --no-open       # len vypise adresu
GUI je lokalna webova stranka na 127.0.0.1 s nahodnym tokenom (nikto iny sa k nemu nepripoji).
"""
import argparse
import csv
import glob
import hashlib
import importlib.util
import json
import mimetypes
import os
import re
import secrets
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)                                  # priecinok s z5, z6, z7, z8 (alebo koren repozitara)
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]|\x1b\][^\x07]*\x07")
BASH_NOISE = re.compile(r"cannot set terminal process group|no job control in this shell|Inappropriate ioctl for device|^\[\d+\]\s")
IMG_EXT = ("png", "jpg", "jpeg", "gif", "webp")
SKILL_RE = re.compile(r"^[a-z][a-z0-9_]{0,40}$")
NUM_RE = re.compile(r"^-?\d{1,5}([.,]\d{1,4})?$")
WORD_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,60}$")
ERR_WORDS = re.compile(r"fail|error|chyba|zlyh|traceback|not found|nenajden|exception|rejected|odmiet", re.I)

# --------------------------------------------------------------------------- primitiva (zoznam berie z z8/primitives.py)
BUILTIN_PRIM = {
    "MOVES": ["walk_forward", "walk_backward", "step_left", "step_right", "turn_left", "turn_right"],
    "POSTURE": ["stand", "stop", "get_up", "wait"],
    "GESTURES": ["arms_down", "raise_right_arm", "raise_left_arm", "raise_both_arms", "wave", "wave_left", "wave_both", "clap",
                 "bow", "thumbs_up", "cheer", "salute", "point_forward", "dance"],
    "SENSES": ["look", "where_am_i", "check_door"],
}
BUILTIN_ARCHIVED = ["walk_to_obstacle", "turn_towards", "walk_to_object", "go_around_obstacle", "follow_wall", "go_through_opening",
                    "search_room", "look_around", "push_forward", "find_object", "save_place", "go_to_place"]
# parameter: (jednotka, predvolena hodnota); skills bez parametra tu nie su
PARAMS = {"walk_forward": ("m", "1"), "walk_backward": ("m", "0.5"), "step_left": ("m", "0.3"), "step_right": ("m", "0.3"),
          "turn_left": ("°", "90"), "turn_right": ("°", "90"), "wait": ("s", "2"),
          "walk_to_obstacle": ("m", "5"), "turn_towards": ("%", "50"), "walk_to_object": ("%", "50"), "save_place": ("slot", "1"),
          "go_to_place": ("slot", "0"), "push_forward": ("m", "0.5"), "go_through_opening": ("m", "8"), "search_room": ("m", "3")}
PRIM_OFF = {"get_up", "stop", "dance"}                       # predvolene nezaskrtnute v "spusti vybrane" (nerobia sa bez dovodu)
ARCH_NOTE = "AI ich nevidí (archív), ale `chappie_do` ich vie spustiť. Hodnotu nastav podľa skillu."
GROUPS = {"A": "A · Jednoduché úlohy", "B": "B · Vnímanie a navigácia", "C": "C · Dvere a izby", "D": "D · Odmietnutie a bezpečnosť"}
FNS = {"chappie_box": 2, "chappie_lidar": 0, "chappie_snap": 0, "chappie_skills": 0, "chappie_cost": 1}   # funkcia -> max. pocet cislelnych argumentov


def load_module(path, name):
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        return mod
    except Exception as e:                                    # noqa: BLE001 - modul je volitelny
        sys.modules.pop(name, None)
        print("poznamka: nepodarilo sa nacitat %s: %s" % (path, e), file=sys.stderr)
        return None


def first_existing(paths):
    for p in paths:
        if p and os.path.exists(p):
            return p
    return None


def load_json(path, default=None):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def clean_line(s):
    return ANSI.sub("", s.rstrip("\r\n")).replace("\r", "")


def extract_json(text):
    i, j = text.find("{"), text.rfind("}")
    if i >= 0 and j > i:
        try:
            v = json.loads(text[i:j + 1])
            return v if isinstance(v, dict) else None
        except ValueError:
            return None
    return None


# --------------------------------------------------------------------------- priecinok behu agenta (~/chappie_runs/<cas>/)
STEP_RE = re.compile(r"^step_(\d+)_(.+?)\.(\w+)$")


def parse_run(path):
    """-> {'steps': [...], 'result': {...}}; kazdy krok: n, observation, thought, actions, done, answer, images, extras, raw"""
    steps = {}
    try:
        names = sorted(os.listdir(path))
    except OSError:
        return {"steps": [], "result": {}}
    for f in names:
        m = STEP_RE.match(f)
        if not m:
            continue
        n, kind, ext = int(m.group(1)), m.group(2), m.group(3).lower()
        st = steps.setdefault(n, {"n": n, "images": [], "extras": [], "actions": []})
        if ext in IMG_EXT:
            st["images"].append(f)
            continue
        try:
            with open(os.path.join(path, f), encoding="utf-8", errors="replace") as fh:
                text = fh.read(40000)
        except OSError:
            continue
        plan = extract_json(text) if kind.startswith("reply") else None
        if plan is not None:
            for k in ("observation", "thought", "done", "answer", "language"):
                if k in plan:
                    st[k] = plan[k]
            acts = []
            for a in plan.get("actions") or []:
                if isinstance(a, dict) and (a.get("skill") or a.get("name")):
                    acts.append({"skill": str(a.get("skill") or a.get("name")), "value": a.get("value")})
            st["actions"] = acts
        elif kind.startswith("reply"):
            st["raw"] = text[:2500]
        else:
            st["extras"].append({"name": f, "text": text[:3000]})
    res = load_json(os.path.join(path, "result.json"), {}) or {}
    return {"steps": [steps[k] for k in sorted(steps)], "result": res if isinstance(res, dict) else {}}


def plans_from_log(lines):
    """zaloha, ked nie je priecinok behu: plany (JSON s 'actions') priamo z vypisu chappie_task"""
    out = []
    for ln in lines:
        i, j = ln.find("{"), ln.rfind("}")
        if i < 0 or j <= i:
            continue
        try:
            d = json.loads(ln[i:j + 1])
        except ValueError:
            continue
        if isinstance(d, dict) and "actions" in d:
            st = {"n": len(out) + 1, "images": [], "extras": [], "actions": []}
            for k in ("observation", "thought", "done", "answer", "language"):
                if k in d:
                    st[k] = d[k]
            st["actions"] = [{"skill": str(a.get("skill") or a.get("name")), "value": a.get("value")}
                             for a in d.get("actions") or [] if isinstance(a, dict) and (a.get("skill") or a.get("name"))]
            out.append(st)
    return out


# --------------------------------------------------------------------------- aplikacia
class Busy(Exception):
    pass


class App:
    def __init__(self, a):
        self.a = a
        self.demo = a.demo
        self.ws = os.path.expanduser(a.ws)
        self.runs_dir = os.path.expanduser(a.runs_dir)
        self.bench_out = os.path.expanduser(a.bench_out)
        self.cost_file = os.path.expanduser(a.cost_file)
        self.lock = threading.RLock()
        self.log_lines, self.log_base = [], 0
        self.conv, self.conv_id = [], 0
        self.sim = {"status": "stopped", "proc": None, "name": "", "map_text": "", "started": None, "label": ""}
        self.busy = None
        self.cancel = False
        self.cur_proc = None
        self.pose = None
        self.pose_busy = False
        self.fake_dir = os.path.join(a.fake_dir)
        if self.demo:
            os.makedirs(self.fake_dir, exist_ok=True)
            os.makedirs(self.runs_dir, exist_ok=True)
        # nastroje projektu (hladaju sa vedla GUI: z5, z7, z8, alebo v repozitari)
        self.gen_script = first_existing([os.path.join(self.ws, "tools/gen_world.py"), os.path.join(PROJ, "z5/gen_world.py"),
                                          os.path.join(HERE, "gen_world.py")])
        self.bench_script = first_existing([os.path.join(self.ws, "tools/benchmark/run_benchmark.py"),
                                            os.path.join(PROJ, "z7/benchmark/run_benchmark.py"), os.path.join(HERE, "run_benchmark.py")])
        self.tasks_file = first_existing([a.tasks_file, os.path.join(self.ws, "tools/benchmark/tasks.json"),
                                          os.path.join(PROJ, "z7/benchmark/tasks.json"), os.path.join(HERE, "tasks.json")])
        prim_path = first_existing([os.path.join(self.ws, "src/humanoid_agent/humanoid_agent/primitives.py"),
                                    os.path.join(PROJ, "z8/primitives.py"), os.path.join(HERE, "primitives.py")])
        self.prim = self._load_prims(prim_path)
        self.bench = load_module(self.bench_script, "chappie_run_benchmark") if self.bench_script else None
        self.fake = os.path.join(HERE, "tests", "fake_chappie.py")
        self.maps = self._discover_maps()
        self.tasks = self._load_tasks()
        self.env_note = "demo (falošný robot)" if self.demo else "ROS cez bash -i (~/.bashrc)"

    # ------------------------------------------------------------------ konfiguracia z projektu
    def _load_prims(self, path):
        d = {k: list(v) for k, v in BUILTIN_PRIM.items()}
        arch, src = list(BUILTIN_ARCHIVED), "vstavaný zoznam"
        mod = load_module(path, "chappie_primitives") if path else None
        if mod:
            try:
                for k in d:
                    d[k] = list(getattr(mod, k))
                arch = [s for s in getattr(mod, "ARCHIVED", arch)]
                src = os.path.relpath(path, PROJ) if path.startswith(PROJ) else path
            except Exception:                                  # noqa: BLE001
                pass
        groups = [("Pohyb", d["MOVES"]), ("Postoj", d["POSTURE"]), ("Ruky a gestá", d["GESTURES"]), ("Senzory", d["SENSES"])]
        mk = lambda names: [{"name": n, "unit": PARAMS.get(n, ("", ""))[0], "default": PARAMS.get(n, ("", ""))[1],
                             "on": n not in PRIM_OFF and n != "look"} for n in names]
        return {"groups": [{"title": t, "items": mk(n)} for t, n in groups], "archived": mk(arch), "source": src}

    def _discover_maps(self):
        dirs = [os.path.join(PROJ, "z5/maps"), os.path.join(PROJ, "z7/benchmark/maps"), os.path.join(HERE, "maps"),
                os.path.join(self.ws, "maps"), os.path.join(self.ws, "tools/benchmark/maps"), os.path.expanduser("~/chappie_maps")]
        dirs += [os.path.expanduser(d) for d in self.a.maps_dir]
        gen_dir = os.path.join(self.ws, "src/humanoid_gazebo/worlds")
        out, seen = [], set()
        for d in dirs + [gen_dir]:
            for f in sorted(glob.glob(os.path.join(d, "*.txt"))):
                name = os.path.splitext(os.path.basename(f))[0]
                if name in seen:
                    continue
                try:
                    text = open(f, encoding="utf-8").read()
                except OSError:
                    continue
                if text.count("S") != 1 or "#" not in text:
                    continue
                seen.add(name)
                src = "vygenerované svety" if d == gen_dir else (os.path.relpath(d, PROJ) if d.startswith(PROJ) else d.replace(os.path.expanduser("~"), "~"))
                out.append({"id": hashlib.md5(f.encode()).hexdigest()[:8], "name": name, "source": src, "path": f, "text": text})
        return out

    def _load_tasks(self):
        cfg = load_json(self.tasks_file, None) if self.tasks_file else None
        out = []
        for t in (cfg or {}).get("tasks", []):
            out.append({"id": t["id"], "group": t["group"], "text": t["text"], "spawn": bool(t.get("spawn")), "note": t.get("note", "")})
        return out

    def config(self):
        return {"maps": [{k: m[k] for k in ("id", "name", "source", "text")} for m in self.maps], "tasks": self.tasks, "groups": GROUPS,
                "prim": self.prim, "arch_note": ARCH_NOTE, "ws": self.ws, "demo": self.demo, "env": self.env_note,
                "has_gen": bool(self.gen_script), "has_bench": bool(self.bench_script), "runs_dir": self.runs_dir,
                "bench_out": self.bench_out, "tasks_file": self.tasks_file or "", "gen_script": self.gen_script or "",
                "bench_script": self.bench_script or "", "prim_source": self.prim["source"],
                "default_label": "gui-" + time.strftime("%Y%m%d-%H%M")}

    # ------------------------------------------------------------------ log
    def log(self, src, text):
        with self.lock:
            self.log_lines.append({"t": time.strftime("%H:%M:%S"), "src": src, "text": text})
            if len(self.log_lines) > 30000:
                del self.log_lines[:10000]
                self.log_base += 10000

    def log_since(self, after):
        with self.lock:
            i = max(0, after - self.log_base)
            return {"n": self.log_base + len(self.log_lines), "lines": self.log_lines[i:i + 3000], "base": self.log_base + i}

    # ------------------------------------------------------------------ prikazy
    def _bash(self, script, *args):
        return ["bash", "-ic", script, "_"] + [str(x) for x in args]

    def cmd_do(self, skill, value):
        args = [skill] + ([value] if value not in (None, "") else [])
        if self.demo:
            return [sys.executable, self.fake, "do"] + args
        return self._bash('if type chappie_do >/dev/null 2>&1; then chappie_do "$@"; else echo "CHYBA: chappie_do nie je dostupne '
                          '(skontroluj ~/.bashrc, source install/setup.bash)"; exit 127; fi', *args)

    def cmd_task(self, text):
        if self.demo:
            return [sys.executable, self.fake, "task", text]
        return self._bash('if type chappie_task >/dev/null 2>&1; then chappie_task "$1"; else ros2 run humanoid_agent task_client "$1"; fi', text)

    def cmd_fn(self, fn, args):
        if self.demo:
            return [sys.executable, self.fake, "do", "where_am_i"]
        return self._bash('if type %s >/dev/null 2>&1; then %s "$@"; else echo "CHYBA: %s nie je dostupne"; exit 127; fi' % (fn, fn, fn), *args)

    def env(self, extra=None):
        e = dict(os.environ)
        e.setdefault("TERM", "dumb")
        e["PYTHONUNBUFFERED"] = "1"
        if self.demo:
            e["CHAPPIE_FAKE_DIR"] = self.fake_dir
            e["CHAPPIE_RUNS_DIR"] = self.runs_dir
        e.update(extra or {})
        return e

    def spawn(self, cmd, env=None):
        return subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                errors="replace", env=self.env(env), start_new_session=True, cwd=os.path.expanduser("~"))

    @staticmethod
    def signal_group(proc, sig):
        try:
            os.killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError, OSError):
            pass

    def stop_proc(self, proc, grace=12.0):
        """SIGINT (ako Ctrl+C), po case SIGTERM a SIGKILL celej skupiny"""
        self.signal_group(proc, signal.SIGINT)
        t0 = time.time()
        while proc.poll() is None and time.time() - t0 < grace:
            time.sleep(0.2)
        if proc.poll() is None:
            self.signal_group(proc, signal.SIGTERM)
            time.sleep(2)
        if proc.poll() is None:
            self.signal_group(proc, signal.SIGKILL)

    # ------------------------------------------------------------------ simulacia
    def sim_start(self, o):
        with self.lock:
            if self.sim["status"] in ("starting", "running", "stopping"):
                raise Busy("Simulácia už beží alebo sa ukončuje.")
            self.sim.update(status="starting", name="", map_text="", started=time.time(), label="")
        env = {"CG_WS": self.ws, "CG_GEN": self.gen_script or "", "CG_ASCII": "", "CG_RANDOM": "", "CG_SEED": "", "CG_NAME": ""}
        sel, map_text, label = o.get("map", "arena"), "", "Základná aréna"
        if sel.startswith("rand:"):
            kind = sel[5:]
            if kind not in ("rooms", "maze", "warehouse"):
                self._sim_fail("Neznámy typ náhodného sveta.")
                raise ValueError("neznámy typ sveta")
            seed = str(o.get("seed") or "").strip()
            if not re.fullmatch(r"\d{1,8}", seed or "x"):
                seed = str(int(time.time()) % 100000)
            env.update(CG_RANDOM=kind, CG_SEED=seed, CG_NAME="%s_%s" % (kind, seed))
            label = "Náhodný svet %s (seed %s)" % (kind, seed)
        elif sel != "arena":
            m = next((m for m in self.maps if m["id"] == sel), None)
            if not m:
                self._sim_fail("Mapa sa nenašla.")
                raise ValueError("mapa sa nenašla")
            name = re.sub(r"[^A-Za-z0-9_]", "_", m["name"])
            env.update(CG_ASCII=m["path"], CG_NAME=name)
            map_text, label = m["text"], "Mapa " + m["name"]
        if env["CG_NAME"] and not env["CG_GEN"]:
            self._sim_fail("Nenašiel som gen_world.py (očakávam ho v %s/tools alebo z5/)." % self.ws)
            raise ValueError("chýba gen_world.py")
        extra = []
        for tok in shlex.split(str(o.get("extra") or "")):
            if not re.fullmatch(r"[\w:=./,@+\-]+", tok):
                self._sim_fail("Neplatný argument: " + tok)
                raise ValueError("neplatný argument")
            extra.append(tok)
        env["CHAPPIE_PRIMITIVES"] = "1" if o.get("prim", True) else "0"
        chain = str(o.get("chain") or "").strip()
        if chain:
            if not re.fullmatch(r"[a-z,]{1,60}", chain):
                self._sim_fail("Reťaz služieb: len malé písmená a čiarky (napr. gemini,mistral,groq).")
                raise ValueError("neplatná reťaz")
            env["CHAPPIE_CHAIN"] = chain
        if self.demo:
            cmd = [sys.executable, self.fake, "gen", env["CG_NAME"] or "arena"]
            if env["CG_ASCII"]:
                cmd += ["--ascii", env["CG_ASCII"]]
            script = None
        else:
            script = ('if [ -n "$CG_ASCII" ]; then python3 "$CG_GEN" --ascii "$CG_ASCII" --name "$CG_NAME" --ws "$CG_WS" || exit 11;\n'
                      'elif [ -n "$CG_RANDOM" ]; then python3 "$CG_GEN" --random "$CG_RANDOM" --seed "$CG_SEED" --name "$CG_NAME" --ws "$CG_WS" || exit 11; fi\n'
                      'W=""; [ -n "$CG_NAME" ] && W="world:=$CG_NAME.sdf"\n'
                      'if type chappie_sim >/dev/null 2>&1; then chappie_sim $W "$@"; else ros2 launch humanoid_gazebo walk_demo.launch.py $W "$@"; fi')
            cmd = self._bash(script, *extra)
        self.log("sim", "=== štart simulácie: %s ===" % label)
        threading.Thread(target=self._sim_thread, args=(cmd, env, extra, label, map_text), daemon=True).start()

    def _sim_fail(self, msg):
        with self.lock:
            self.sim.update(status="stopped")
        self.log("sim", "CHYBA: " + msg)

    def _sim_thread(self, cmd, env, extra, label, map_text):
        try:
            if self.demo:                                      # najprv "generator", potom "simulacia"
                p = self.spawn(cmd, env)
                for ln in p.stdout:
                    self.log("sim", clean_line(ln))
                p.wait()
                cmd = [sys.executable, self.fake, "sim", env["CG_NAME"] or "arena"]
            proc = self.spawn(cmd, env)
        except OSError as e:
            self._sim_fail("nepodarilo sa spustiť: %s" % e)
            return
        name = env["CG_NAME"]
        if not map_text and name:
            for d in (os.path.join(self.ws, "src/humanoid_gazebo/worlds"), os.path.join(self.fake_dir, "worlds")):
                p = os.path.join(d, name + ".txt")
                if os.path.exists(p):
                    map_text = open(p, encoding="utf-8").read()
                    break
        with self.lock:
            self.sim.update(status="running", proc=proc, name=name, label=label, map_text=map_text, started=time.time())
            self.pose = None
        for ln in proc.stdout:
            ln = clean_line(ln)
            if ln and not BASH_NOISE.search(ln):
                self.log("sim", ln)
                if not self.sim["map_text"] and name:
                    pass
        rc = proc.wait()
        self.log("sim", "=== simulácia skončila (kód %s) ===" % rc)
        with self.lock:
            self.sim.update(status="stopped", proc=None, started=None)
            self.pose = None

    def sim_stop(self):
        with self.lock:
            proc, st = self.sim["proc"], self.sim["status"]
            if st != "running" or not proc:
                raise Busy("Simulácia nebeží.")
            self.sim["status"] = "stopping"
        self.log("sim", "=== zastavujem simuláciu (Ctrl+C) ===")
        threading.Thread(target=lambda: self.stop_proc(proc, grace=25.0), daemon=True).start()

    def kill_gz(self):
        """zvysky Gazeba po padnutom/zastavenom launch (gz sim, ruby ...)"""
        if self.demo:
            self.log("gui", "demo: nič na zabitie")
            return
        for pat in ("gz sim", "ruby.*gz", "parameter_bridge", "robot_state_publisher"):
            subprocess.run(["pkill", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.log("gui", "poslané pkill pre zvyšky Gazeba (gz sim, parameter_bridge, ...)")

    # ------------------------------------------------------------------ akcie (jedna naraz)
    def submit(self, items, title, finish_hook=None):
        with self.lock:
            if self.busy:
                raise Busy("Robot už pracuje: %s. Počkaj alebo úlohu zruš." % self.busy["title"])
            if not items:
                raise ValueError("nič na spustenie")
            self.busy = {"title": title, "started": time.time(), "total": len(items), "done": 0}
            self.cancel = False
        threading.Thread(target=self._worker, args=(items,), daemon=True).start()

    def new_entry(self, kind, title, **kw):
        with self.lock:
            self.conv_id += 1
            e = {"id": self.conv_id, "kind": kind, "title": title, "status": "running", "started": time.time(), "ended": None,
                 "out": [], "steps": [], "result": {}, "answer": "", "usage": None, "rc": None, "run": ""}
            e.update(kw)
            self.conv.append(e)
            if len(self.conv) > 120:
                del self.conv[:20]
            return e

    def _worker(self, items):
        try:
            for i, it in enumerate(items):
                if self.cancel:
                    break
                with self.lock:
                    self.busy["done"] = i
                    self.busy["current"] = it.get("label", "")
                self._run_item(it)
                if it.get("pause") and not self.cancel:
                    time.sleep(it["pause"])
        finally:
            with self.lock:
                self.busy = None
                self.cur_proc = None
            if self.sim["status"] == "running":
                self.refresh_pose()

    def _item_cmd(self, it):
        t = it["type"]
        if t == "task":
            return self.cmd_task(it["text"]), it["text"], 3600
        if t == "do":
            return self.cmd_do(it["skill"], it.get("value")), "%s %s" % (it["skill"], it.get("value") or ""), 900
        if t == "fn":
            return self.cmd_fn(it["fn"], it.get("args", [])), "%s %s" % (it["fn"], " ".join(map(str, it.get("args", [])))), 40
        if t == "bench":
            return it["cmd"], "benchmark " + " ".join(it["cmd"][4:] if not self.demo else it["cmd"][1:])[:200], 86400
        raise ValueError(t)

    def _run_item(self, it):
        cmd, title, timeout = self._item_cmd(it)
        is_task = it["type"] == "task"
        e = self.new_entry(it["type"], title.strip(), label=it.get("label", ""))
        self.log("gui", "▶ %s: %s" % (it["type"], title.strip()))
        runs_before = set(os.listdir(self.runs_dir)) if is_task and os.path.isdir(self.runs_dir) else set()
        cost0 = None
        if is_task and self.bench:
            try:
                cost0 = self.bench.cost_totals(self.cost_file)
            except Exception:                                  # noqa: BLE001
                cost0 = None
        t0 = time.time()
        try:
            proc = self.spawn(cmd)
        except OSError as ex:
            e.update(status="error", ended=time.time(), rc=-1)
            e["out"].append("nepodarilo sa spustiť: %s" % ex)
            return
        self.cur_proc = proc
        src = {"task": "task", "do": "do", "fn": "do", "bench": "bench"}[it["type"]]

        def reader():
            for ln in proc.stdout:
                ln = clean_line(ln)
                if not ln or BASH_NOISE.search(ln):
                    continue
                with self.lock:
                    e["out"].append(ln)
                    if len(e["out"]) > 600:
                        del e["out"][:200]
                self.log(src, ln)
        rt = threading.Thread(target=reader, daemon=True)
        rt.start()
        sig_t = None
        while proc.poll() is None:
            if is_task:
                self._update_task(e, runs_before)
            if self.cancel and sig_t is None:
                sig_t = time.time()
                self.signal_group(proc, signal.SIGINT)
            elif sig_t and time.time() - sig_t > 15:
                self.signal_group(proc, signal.SIGKILL)
            elif time.time() - t0 > timeout and sig_t is None:
                sig_t = time.time()
                e["out"].append("(časový limit %d s, ruším)" % timeout)
                self.signal_group(proc, signal.SIGINT)
            time.sleep(0.5)
        rt.join(5)
        rc = proc.returncode
        if is_task:
            time.sleep(0.4)
            self._update_task(e, runs_before, final=True)
            if cost0 is not None:
                try:
                    use = self.bench.cost_diff(cost0, self.bench.cost_totals(self.cost_file))
                    if use:
                        svc = max(use, key=lambda s: use[s].get("req", 0))
                        e["usage"] = {"service": svc, "req": sum(u.get("req", 0) for u in use.values()),
                                      "tin": sum(u.get("in", 0) for u in use.values()), "tout": sum(u.get("out", 0) for u in use.values())}
                except Exception:                              # noqa: BLE001
                    pass
        text = "\n".join(e["out"])
        if self.cancel:
            status = "cancelled"
        elif rc != 0:
            status = "error"
        elif it["type"] in ("do", "fn") and ERR_WORDS.search(text):
            status = "warn"
        else:
            status = "ok"
        with self.lock:
            e.update(status=status, ended=time.time(), rc=rc)
        self.log("gui", "■ %s -> %s (kód %s, %.1f s)" % (title.strip()[:60], status, rc, time.time() - t0))

    def _update_task(self, e, runs_before, final=False):
        run = None
        try:
            new = [d for d in os.listdir(self.runs_dir) if d not in runs_before and os.path.isdir(os.path.join(self.runs_dir, d))]
            new.sort(key=lambda d: os.path.getmtime(os.path.join(self.runs_dir, d)))
            if new:
                run = new[-1]
        except OSError:
            pass
        steps, res = [], {}
        if run:
            p = parse_run(os.path.join(self.runs_dir, run))
            steps, res = p["steps"], p["result"]
        if not steps:
            steps = plans_from_log(e["out"])
        answer = ""
        for st in steps:
            if st.get("answer") and (st.get("done") or not answer):
                answer = st["answer"]
        if not answer and res.get("summary"):
            answer = str(res["summary"])
        with self.lock:
            e.update(steps=steps, result={k: res[k] for k in ("status", "success", "iterations", "prompt_mode", "summary") if k in res},
                     answer=answer, run=run or "")

    def cancel_all(self):
        with self.lock:
            if not self.busy:
                raise Busy("Nič nebeží.")
            self.cancel = True
            proc = self.cur_proc
        self.log("gui", "⏹ rušim (Ctrl+C)")
        if proc and proc.poll() is None:
            self.signal_group(proc, signal.SIGINT)

    # ------------------------------------------------------------------ poloha robota (odometria na mape)
    def refresh_pose(self):
        with self.lock:
            if self.pose_busy or self.sim["status"] != "running":
                return
            self.pose_busy = True
        threading.Thread(target=self._pose_thread, daemon=True).start()

    def _pose_thread(self):
        try:
            p = None
            if self.demo:
                fp = os.path.join(self.fake_dir, "pose.json")
                d = load_json(fp, None)
                if d:
                    p = (d["x"], d["y"], 0.0, d["yaw"])
            elif self.bench:
                rc, out = self.bench.run(["bash", "-ic", "ros2 topic echo %s --once --field pose.pose" % shlex.quote(self.a.odom_topic)], timeout=20)
                p = self.bench.parse_odom(out) if rc == 0 else None
            with self.lock:
                self.pose = {"x": p[0], "y": p[1], "yaw": p[3], "t": time.time()} if p else None
        except Exception:                                      # noqa: BLE001
            with self.lock:
                self.pose = None
        finally:
            self.pose_busy = False

    # ------------------------------------------------------------------ stav pre GUI
    def state(self):
        now = time.time()
        with self.lock:
            sim = {"status": self.sim["status"], "name": self.sim["name"], "label": self.sim["label"], "map_text": self.sim["map_text"],
                   "up": int(now - self.sim["started"]) if self.sim["started"] and self.sim["status"] == "running" else 0}
            conv = []
            for e in self.conv[-60:]:
                c = dict(e)
                c["out"] = e["out"][-80:]
                c["elapsed"] = int((e["ended"] or now) - e["started"])
                c.pop("started"), c.pop("ended")
                conv.append(c)
            busy = dict(self.busy, elapsed=int(now - self.busy["started"])) if self.busy else None
            if busy:
                busy.pop("started")
            return {"sim": sim, "busy": busy, "conv": conv, "pose": self.pose, "log_n": self.log_base + len(self.log_lines)}

    def clear_conv(self):
        with self.lock:
            self.conv = [e for e in self.conv if e["status"] == "running"]

    # ------------------------------------------------------------------ benchmark
    def bench_items(self, o):
        if not self.bench_script:
            raise ValueError("Nenašiel som run_benchmark.py (z7/benchmark alebo tools/benchmark v repozitári).")
        label = str(o.get("label") or "").strip()
        tasks = str(o.get("tasks") or "").strip()
        world = str(o.get("world") or "arena").strip()
        mode = str(o.get("mode") or "run")
        reset = str(o.get("reset") or "teleport")
        reps = str(o.get("reps") or "1")
        if not WORD_RE.match(label or "x") or not WORD_RE.match(world) or not re.fullmatch(r"[A-Za-z0-9,]{0,200}", tasks) \
                or not re.fullmatch(r"\d{1,2}", reps) or reset not in ("teleport", "go_home", "restart", "none") \
                or mode not in ("run", "check", "selftest", "dry"):
            raise ValueError("Neplatná hodnota v nastaveniach benchmarku.")
        args = []
        if mode == "check":
            args = ["--check"]
        elif mode == "selftest":
            args = ["--selftest", "--reset", reset, "--world", world]
        else:
            if not label:
                raise ValueError("Zadaj štítok behu.")
            args = ["--label", label, "--reps", reps, "--reset", reset, "--world", world, "--runs-dir", self.runs_dir, "--out", self.bench_out]
            if tasks:
                args += ["--tasks", tasks]
            if mode == "dry" or self.demo:
                args.append("--dry-run")
        if self.demo:
            cmd = [sys.executable, self.bench_script] + args
        else:
            cmd = self._bash('python3 "$@"', self.bench_script, *args)
        return [{"type": "bench", "cmd": cmd, "label": "benchmark"}]

    def bench_labels(self):
        try:
            return sorted([d for d in os.listdir(self.bench_out) if os.path.exists(os.path.join(self.bench_out, d, "summary.csv"))], reverse=True)
        except OSError:
            return []

    def bench_results(self, label):
        if not WORD_RE.match(label or ""):
            return {"rows": [], "error": "neplatný štítok"}
        p = os.path.join(self.bench_out, label, "summary.csv")
        try:
            with open(p, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
        except OSError:
            return {"rows": [], "error": "summary.csv sa nenašlo"}
        return {"rows": rows}

    def image_path(self, run, fn):
        run, fn = os.path.basename(run), os.path.basename(fn)
        p = os.path.realpath(os.path.join(self.runs_dir, run, fn))
        if not p.startswith(os.path.realpath(self.runs_dir) + os.sep) or not os.path.isfile(p) or p.rsplit(".", 1)[-1].lower() not in IMG_EXT:
            return None
        return p


# --------------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    app = None
    token = ""
    port = 0
    server_version = "ChappieGUI"

    def log_message(self, *a):
        pass

    def _ok_host(self):
        h = (self.headers.get("Host") or "").lower()
        return h in ("127.0.0.1:%d" % self.port, "localhost:%d" % self.port)

    def _send(self, code, body, ctype="application/json; charset=utf-8", extra=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False)
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _auth(self, q):
        if not self._ok_host():
            self._send(403, {"error": "zlý Host"})
            return False
        if (q.get("t") or [""])[0] != self.token:
            self._send(403, {"error": "zlý token"})
            return False
        return True

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if not self._auth(q):
            return
        app = self.app
        if u.path == "/":
            self._send(200, PAGE.replace("__TOKEN__", self.token), "text/html; charset=utf-8")
        elif u.path == "/api/config":
            self._send(200, app.config())
        elif u.path == "/api/state":
            self._send(200, app.state())
        elif u.path == "/api/log":
            self._send(200, app.log_since(int((q.get("after") or ["0"])[0] or 0)))
        elif u.path == "/api/bench/labels":
            self._send(200, {"labels": app.bench_labels()})
        elif u.path == "/api/bench/results":
            self._send(200, app.bench_results((q.get("label") or [""])[0]))
        elif u.path == "/img":
            p = app.image_path((q.get("r") or [""])[0], (q.get("f") or [""])[0])
            if not p:
                self._send(404, {"error": "obrázok sa nenašiel"})
            else:
                self._send(200, open(p, "rb").read(), mimetypes.guess_type(p)[0] or "application/octet-stream")
        else:
            self._send(404, {"error": "neexistuje"})

    def do_POST(self):
        u = urlparse(self.path)
        if not self._auth(parse_qs(u.query)):
            return
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(min(n, 65536)) or b"{}")
            if not isinstance(body, dict):
                raise ValueError("telo musí byť objekt")
        except ValueError as e:
            self._send(400, {"error": "zlé telo požiadavky: %s" % e})
            return
        try:
            self._send(200, self.route(u.path, body))
        except Busy as e:
            self._send(409, {"error": str(e)})
        except ValueError as e:
            self._send(400, {"error": str(e)})
        except Exception as e:                                 # noqa: BLE001
            self._send(500, {"error": "%s: %s" % (type(e).__name__, e)})

    def route(self, path, b):
        app = self.app
        if path == "/api/sim/start":
            app.sim_start(b)
        elif path == "/api/sim/stop":
            app.sim_stop()
        elif path == "/api/sim/kill_gz":
            app.kill_gz()
        elif path == "/api/cancel":
            app.cancel_all()
        elif path == "/api/task":
            text = str(b.get("text") or "").strip()
            if not text or len(text) > 2000:
                raise ValueError("Zadaj text úlohy (max. 2000 znakov).")
            app.submit([{"type": "task", "text": text}], text[:60])
        elif path == "/api/do":
            app.submit([self._do_item(b)], "skill")
        elif path == "/api/fn":
            fn, args = str(b.get("fn") or ""), [str(a) for a in b.get("args") or []]
            if fn not in FNS or len(args) > FNS[fn] or not all(NUM_RE.match(a) for a in args):
                raise ValueError("Nepovolený príkaz alebo argument.")
            app.submit([{"type": "fn", "fn": fn, "args": [a.replace(",", ".") for a in args]}], fn)
        elif path == "/api/queue":
            items = []
            pause = max(0.0, min(30.0, float(b.get("pause") or 0)))
            for x in b.get("items") or []:
                if x.get("type") == "task":
                    text = str(x.get("text") or "").strip()
                    if not text or len(text) > 2000:
                        raise ValueError("Prázdna úloha v zozname.")
                    items.append({"type": "task", "text": text, "label": x.get("id", ""), "pause": pause})
                else:
                    it = self._do_item(x)
                    it["pause"] = pause
                    items.append(it)
                if b.get("home_after") and x.get("type") == "task":
                    items.append({"type": "do", "skill": "go_to_place", "value": "0", "label": "späť na štart", "pause": pause})
            app.submit(items, "zoznam (%d)" % len(items))
        elif path == "/api/pose":
            app.refresh_pose()
        elif path == "/api/clear":
            app.clear_conv()
        elif path == "/api/bench/start":
            app.submit(app.bench_items(b), "benchmark")
        else:
            return {"error": "neexistuje"}
        return {"ok": True}

    @staticmethod
    def _do_item(x):
        skill, value = str(x.get("skill") or ""), x.get("value")
        value = "" if value is None else str(value).strip()
        if not SKILL_RE.match(skill):
            raise ValueError("Neplatný názov skillu.")
        if value and not NUM_RE.match(value):
            raise ValueError("Hodnota musí byť číslo.")
        return {"type": "do", "skill": skill, "value": value.replace(",", "."), "label": x.get("label", "")}


def open_window(url, browser):
    cands = [browser] if browser else ["google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "brave-browser", "microsoft-edge"]
    for c in cands:
        exe = shutil.which(c) if c else None
        if exe:
            try:
                subprocess.Popen([exe, "--app=" + url, "--window-size=1500,950"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                 start_new_session=True)
                return "okno (%s)" % os.path.basename(exe)
            except OSError:
                continue
    webbrowser.open(url)
    return "prehliadač (predvolený)"


def make_server(a, token=None):
    app = App(a)
    Handler.app, Handler.token = app, token or secrets.token_urlsafe(16)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    Handler.port = srv.server_address[1]
    srv.daemon_threads = True
    return srv, app


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ws", default=os.environ.get("CHAPPIE_WS", "~/agent-based-humanoid-control"), help="workspace s repozitárom")
    ap.add_argument("--port", type=int, default=0, help="port (predvolene náhodný voľný)")
    ap.add_argument("--demo", action="store_true", help="falošný robot a agent (bez ROS, Gazeba a AI)")
    ap.add_argument("--no-open", action="store_true", help="neotváraj okno, len vypíš adresu")
    ap.add_argument("--browser", help="príkaz prehliadača pre okno (predvolene Chrome/Chromium v app režime)")
    ap.add_argument("--runs-dir", default="~/chappie_runs")
    ap.add_argument("--bench-out", default="~/chappie_bench")
    ap.add_argument("--cost-file", default="~/.chappie_cost.json")
    ap.add_argument("--tasks-file", default=None)
    ap.add_argument("--maps-dir", action="append", default=[], help="ďalší priečinok s ASCII mapami (viackrát)")
    ap.add_argument("--odom-topic", default="/chappie/odom")
    ap.add_argument("--fake-dir", default="/tmp/chappie_fake")
    a = ap.parse_args()
    if a.demo and a.runs_dir == "~/chappie_runs":
        a.runs_dir = "/tmp/chappie_fake/runs"
        a.bench_out = "/tmp/chappie_fake/bench"
    if a.demo and not os.path.exists(os.path.join(HERE, "tests", "fake_chappie.py")):
        sys.exit("--demo potrebuje tests/fake_chappie.py vedľa tohto súboru")
    srv, app = make_server(a)
    url = "http://127.0.0.1:%d/?t=%s" % (srv.server_address[1], Handler.token)
    print("Chappie GUI beží: %s%s" % (url, "   [DEMO]" if a.demo else ""))
    print("Mapy: %d, úlohy: %d, primitíva z: %s, gen_world: %s, benchmark: %s" % (
        len(app.maps), len(app.tasks), app.prim["source"], app.gen_script or "NENÁJDENÝ", app.bench_script or "NENÁJDENÝ"))
    if not a.no_open:
        print("Otvorené ako:", open_window(url, a.browser))
    print("Ukončenie: Ctrl+C v tomto termináli (simulácia spustená z okna sa zastaví tiež).")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        proc = app.sim.get("proc")
        if proc and proc.poll() is None:
            app.stop_proc(proc, grace=20)
        srv.server_close()


PAGE = r'''<!DOCTYPE html>
<html lang="sk"><head><meta charset="utf-8"><title>Chappie</title>
<style>
*{box-sizing:border-box}body{margin:0;font:14px/1.4 system-ui,sans-serif;color:#222;background:#fff;height:100vh;display:grid;grid-template-columns:300px 1fr 330px}
section{padding:12px;overflow:auto;border-right:1px solid #ddd;display:flex;flex-direction:column;gap:8px}section:last-child{border:0}
h2{font-size:13px;margin:6px 0 0;color:#555}
select,input,textarea,button{font:inherit}select,input[type=text],textarea{width:100%;padding:5px;border:1px solid #bbb;border-radius:3px}
button{padding:4px 10px;border:1px solid #aaa;border-radius:3px;background:#f4f4f4;cursor:pointer}button:hover:not(:disabled){background:#e8e8e8}button:disabled{opacity:.45}
.row{display:flex;gap:6px;align-items:center}.row>.f{flex:1}.dim{color:#777;font-size:12px}
canvas{max-width:100%;border:1px solid #ddd}
#conv{flex:1;overflow:auto;display:flex;flex-direction:column;gap:8px}
.me{background:#e8f0fe;padding:6px 10px;border-radius:4px;white-space:pre-wrap}
.step{border-left:3px solid #ccc;padding:2px 10px}.step div{margin:1px 0}
.ans{background:#eaf6ea;border:1px solid #b9dfb9;padding:6px 10px;border-radius:4px}
.err{color:#b00}pre{margin:2px 0;padding:6px;background:#f6f6f6;border:1px solid #e2e2e2;font-size:12px;white-space:pre-wrap;max-height:200px;overflow:auto}
.tabs button{border-radius:3px 3px 0 0}.tabs button.on{background:#fff;border-bottom-color:#fff;font-weight:600}
section:last-child{overflow:hidden}.pane{display:none}.pane.on{display:block;flex:1;min-height:0;overflow:auto}.it{display:flex;gap:6px;align-items:center;margin:3px 0}.it button{text-align:left;flex:1}.it input[type=text]{width:52px}
.tk{margin:4px 0;display:flex;gap:6px}.tk span{flex:1;cursor:pointer}
#logbox{font:12px/1.35 ui-monospace,monospace;white-space:pre-wrap;word-break:break-word}
</style></head><body>
<section>
 <h2>Mapa</h2>
 <select id="selMap"></select>
 <canvas id="map" width="280" height="100"></canvas>
 <div class="row"><button id="bStart" class="f">Spustiť simuláciu</button><button id="bStop">Zastaviť</button></div>
 <div class="dim" id="simInfo">simulácia nebeží</div>
</section>
<section>
 <div id="conv"></div>
 <textarea id="taskText" rows="2" placeholder="Úloha pre robota (Enter = odoslať)"></textarea>
 <div class="row"><button id="bSend">Odoslať</button><button id="bCancel" disabled>Zrušiť</button><span class="f dim" id="busy"></span><button id="bClear">Vyčistiť</button></div>
</section>
<section>
 <div class="tabs"><button class="on" data-p="pPrim">Primitíva</button><button data-p="pTasks">Úlohy</button><button data-p="pLog">Log</button></div>
 <div class="pane on" id="pPrim"></div><div class="pane" id="pTasks"></div><div class="pane" id="pLog"><div id="logbox"></div></div>
</section>
<script>
const T="__TOKEN__",$=s=>document.querySelector(s),$$=s=>[...document.querySelectorAll(s)];
const esc=s=>String(s==null?"":s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
let CFG,ST,lastConv="";
async function api(p,b){const r=await fetch(p+(p.includes("?")?"&":"?")+"t="+T,b===undefined?{}:{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(b)});let j={};try{j=await r.json()}catch(e){}if(!r.ok)throw new Error(j.error||r.status);return j}
async function act(p,b){try{await api(p,b);poll()}catch(e){alert(e.message)}}
const COL={"#":"#888","=":"#bb9","B":"#e80","R":"#d22","C":"#3a4","P":"#c4a","T":"#779","p":"#38d","D":"#2c5","S":"#fc0"};
function drawMap(text,pose){const cv=$("#map"),c=cv.getContext("2d");
 if(!text){cv.width=280;cv.height=40;c.fillStyle="#999";c.fillText("bez náhľadu",8,24);return}
 const rows=text.replace(/\s+$/,"").split("\n"),w=Math.max(...rows.map(r=>r.length)),k=Math.max(3,Math.min(14,Math.floor(280/w)));
 cv.width=w*k;cv.height=rows.length*k;let sr=0,sc=0;
 rows.forEach((r,y)=>[...r].forEach((ch,x)=>{if(ch==="S"){sr=y;sc=x}if(COL[ch]){c.fillStyle=COL[ch];c.fillRect(x*k,y*k,k,k)}}));
 if(pose){c.fillStyle="#000";c.beginPath();c.arc((sc+.5+pose.x/.5)*k,(sr+.5-pose.y/.5)*k,k*.6,0,7);c.fill()}}
function onMap(){const m=CFG.maps.find(m=>m.id===$("#selMap").value);if(ST&&ST.sim.status==="running"&&ST.sim.map_text)return;drawMap(m?m.text:"")}
function stepHtml(s){const a=(s.actions||[]).map(a=>esc(a.skill)+(a.value!=null&&a.value!==""?" "+esc(a.value):"")).join(", ");
 return`<div class="step"><div class="dim">krok ${s.n}</div>${s.observation?`<div>Vidí: ${esc(s.observation)}</div>`:""}${s.thought?`<div>Myslí: ${esc(s.thought)}</div>`:""}<div><b>${a||"–"}</b></div>${s.raw?`<pre>${esc(s.raw)}</pre>`:""}</div>`}
const STX={running:"beží…",ok:"hotovo",warn:"skontroluj výpis",error:"chyba",cancelled:"zrušené"};
function entry(e){const st=`<span class="dim ${e.status==="error"?"err":""}">${STX[e.status]||e.status} · ${e.elapsed} s</span>`;
 if(e.kind==="task"){const u=e.usage;
  return`<div class="me">${esc(e.title)}</div>${st}${u?` <span class="dim">${esc(u.service)}, ${u.tin}+${u.tout} tokenov</span>`:""}${(e.steps||[]).map(stepHtml).join("")}${e.answer?`<div class="ans"><b>Odpoveď:</b> ${esc(e.answer)}</div>`:""}`}
 return`<div><b>${esc(e.title)}</b> ${st}${e.out.length?`<pre>${esc(e.out.slice(-10).join("\n"))}</pre>`:""}</div>`}
function render(s){ST=s;const sim=s.sim,b=s.busy,run=sim.status==="running";
 $("#simInfo").textContent=run?`beží: ${sim.label}`:({stopped:"simulácia nebeží",starting:"štartuje…",stopping:"zastavuje sa…"}[sim.status]);
 $("#bStart").disabled=sim.status!=="stopped";$("#bStop").disabled=!run;$("#bSend").disabled=!!b;$("#bCancel").disabled=!b;$$("[data-do],[data-run]").forEach(x=>x.disabled=!!b);
 $("#busy").textContent=b?"pracuje: "+b.title:"";
 if(run&&sim.map_text)drawMap(sim.map_text,s.pose);
 const h=s.conv.map(entry).join("");if(h!==lastConv){lastConv=h;const c=$("#conv"),n=c.scrollHeight-c.scrollTop-c.clientHeight<80;c.innerHTML=h;if(n)c.scrollTop=c.scrollHeight}}
async function poll(){try{render(await api("/api/state"))}catch(e){}}
let logN=0,logAll=[];
async function pollLog(){try{const j=await api("/api/log?after="+logN);if(j.lines.length){logAll.push(...j.lines);logAll=logAll.slice(-3000);logN=j.base+j.lines.length;if($("#pLog").classList.contains("on"))showLog()}}catch(e){}}
function showLog(){const p=$("#pLog"),n=p.scrollHeight-p.scrollTop-p.clientHeight<60;$("#logbox").innerHTML=logAll.slice(-1000).map(l=>`<div>${l.t} [${l.src}] ${esc(l.text)}</div>`).join("");if(n)p.scrollTop=p.scrollHeight}
function val(n){const i=$(`[data-v="${n}"]`);return i?i.value.replace(",","."):""}
function build(){
 let h="";for(const g of CFG.prim.groups){h+=`<h2>${g.title}</h2>`+g.items.map(i=>`<div class="it"><button data-do="${i.name}">${i.name}</button>${i.unit?`<input type="text" data-v="${i.name}" value="${i.default}"><span class="dim">${i.unit}</span>`:""}</div>`).join("")}
 $("#pPrim").innerHTML=h;
 $("#pTasks").innerHTML=CFG.tasks.length?CFG.tasks.map(t=>`<div class="tk"><button data-run="${t.id}">▶</button><span data-fill="${t.id}"><b>${t.id}</b> ${esc(t.text)}</span></div>`).join(""):"<div class='dim'>tasks.json sa nenašiel</div>"}
document.addEventListener("click",e=>{const t=e.target.closest("button,[data-fill]");if(!t)return;const d=t.dataset;
 if(d.p){$$(".tabs button").forEach(x=>x.classList.toggle("on",x===t));$$(".pane").forEach(x=>x.classList.toggle("on",x.id===d.p));if(d.p==="pLog")showLog();return}
 if(d.do)return act("/api/do",{skill:d.do,value:val(d.do)});
 if(d.fill){$("#taskText").value=CFG.tasks.find(x=>x.id===d.fill).text;return}
 if(d.run)return act("/api/task",{text:CFG.tasks.find(x=>x.id===d.run).text});
 ({bStart:()=>act("/api/sim/start",{map:$("#selMap").value}),bStop:()=>act("/api/sim/stop",{}),bCancel:()=>act("/api/cancel",{}),bClear:()=>act("/api/clear",{}),
  bSend:async()=>{const v=$("#taskText").value.trim();if(!v)return;await act("/api/task",{text:v});$("#taskText").value=""}}[t.id]||(()=>{}))()});
$("#selMap").onchange=onMap;
$("#taskText").addEventListener("keydown",e=>{if(e.key==="Enter"&&!e.shiftKey){e.preventDefault();$("#bSend").click()}});
(async()=>{CFG=await api("/api/config");
 const gr={};CFG.maps.forEach(m=>(gr[m.source]=gr[m.source]||[]).push(m));
 let o=`<option value="arena">Základná aréna</option><option value="rand:rooms">náhodné izby</option><option value="rand:maze">náhodné bludisko</option><option value="rand:warehouse">náhodný sklad</option>`;
 for(const s in gr)o+=`<optgroup label="${esc(s)}">${gr[s].map(m=>`<option value="${m.id}">${esc(m.name)}</option>`).join("")}</optgroup>`;
 $("#selMap").innerHTML=o;const f=CFG.maps.find(m=>m.name==="benchmark")||CFG.maps[0];if(f)$("#selMap").value=f.id;
 build();onMap();poll();setInterval(poll,700);setInterval(pollLog,1000);
 setInterval(()=>{if(ST&&ST.sim.status==="running"&&!ST.busy)api("/api/pose",{}).catch(()=>{})},4000)})();
</script></body></html>
'''

if __name__ == "__main__":
    main()

