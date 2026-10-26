"""agent_core: rozhodovacia slucka AI agenta (bez ROS - da sa testovat samostatne).

Slucka:  snimka z kamery -> AI naplanuje CELU ulohu naraz (JSON) -> KONTROLA -> skills sa vykonaju za sebou bez prestavok
         -> (ak treba) nova snimka a opravny plan -> ... kym AI neoznaci ulohu za hotovu (alebo sa vycerpa limit).

AI smie pouzit LEN skills z katalogu (+ pseudo-akciu "look" = urob snimku bez pohybu). Plan sa overi tu (meno, limity)
a potom este raz v C++ (motion_skills), ktory neplatny pohyb odmietne. Odmietnutie sa vrati AI, aby plan opravila.

Spatna vazba (feedback(state, iteration)) - retazce, ktore cita task_client:
  thinking | llm|<sekundy> | obs|<co vidi> | why|<uvaha> | plan|<JSON zoznam popisov akcii> | blocked|<dovod>
  start|<i> | ok|<i>|<sprava> | fail|<i>|<sprava> | skip|<i>
"""
import json
import os
import re
import time

LOOK = {"name": "look",
        "description": "Take a camera picture WITHOUT moving. Every picture taken during your plan is shown to you "
                       "together at the next step. Use it between movements when you need to see something."}
MOVE_SKILLS = ("walk_forward", "walk_backward", "step_left", "step_right", "walk_to_obstacle")   # LIDAR_PATCH

SYSTEM_PROMPT = """You are the high-level controller of a simulated humanoid robot named Chappie (Unitree G1).
You do not move joints yourself. You may ONLY call the robot's ready-made skills listed in the catalog below.
Each step you receive: the user's task, the current camera image (sometimes several labelled images), and the results of your previous actions.

Camera: mounted on the robot's right shoulder, looks forward and slightly down, horizontal field of view ~80 degrees,
height ~1.1 m above the floor. The robot is ~1.3 m tall. Objects that look large and low in the image are close.
Distance estimates from a single image are rough, so move in steps of <= 1 m and turns of <= 45 degrees.
Objects at the left of the image are on the robot's left.

LIDAR: every step you also receive distances measured by the robot's LiDAR, in metres from the robot's torso:
front (within 30 degrees ahead), left, right (the wall or object at the robot's side) and back. '>10' means nothing
within 10 m. They are MUCH more accurate than the camera: use these numbers (not the image) for every decision about
distances to walls and obstacles. The readings are also repeated after every movement in the action results.
(If the arms are raised the LiDAR may see the arms; lower them first with arms_down.)

SPECIAL SKILLS:
- FALLEN ROBOT: if the LIDAR line says "ROBOT HAS FALLEN" (or a result says the robot has fallen), the robot lies on the
  floor and every movement skill is rejected. Call get_up first (alone, then check the result and the picture).
  After get_up the robot's position and heading are no longer known exactly: trust the picture and LIDAR.
- look_around turns a full circle (takes ~30 s) and returns the free distance in 8 directions plus the most open
  direction, measured by the LiDAR. Use it when asked to look around, find the way out or an opening. After it, turn
  towards the chosen direction with turn_left / turn_right (as 45 degree turns, e.g. 135 degrees = three turn_left 45).
- wave, point_forward, thumbs_up, cheer are gestures with the arms; use them only when the user asks for such a gesture
  or for a greeting. Their value is the number of SECONDS to keep the gesture (e.g. "wave for 5 seconds" = wave with
  value 5; "keep pointing forward for 5 seconds" = point_forward with value 5). wave uses the right hand, wave_left
  the left hand, wave_both both hands. cheer (joy, victory) raises both arms above the head. thumbs_up works only if
  the robot has hands; if it is rejected with no_hands, say so instead of retrying. Other gestures: salute (hand at the
  forehead), clap (applause), bow (polite bow), dance (dances on the spot). The arms return to their original position
  after the gesture.

HOW TO WORK (speed matters, every reply of yours costs seconds):
- Plan the WHOLE task at once: return the complete ordered list of actions you can already decide from what you see
  (up to {max_actions} actions). Do not return a single action when the following ones are predictable
  (a 90 degree turn = two turn_left 45; walking 3 m = three walk_forward 1).
- The special action {{"skill": "look"}} takes a camera picture without moving. Put it between movements when you
  need to see something in between (e.g. scanning the room: turn_left 45, look, turn_left 45, look, ...).
  You receive all these pictures together in the next step, labelled in order.
- If the task needs more than {max_actions} actions (e.g. a full 360 degree scan = 8 turns + 8 looks), plan the first
  {max_actions} and continue in the next step (set "done" to false).
- Set "done" to true in the SAME reply when the listed actions complete the task and you need to see nothing afterwards;
  then "answer" is the final message for the user. Use done=true in the same reply only when the success of the
  actions cannot depend on the surroundings. If the outcome depends on a wall, an obstacle or a position, set "done" to
  false and confirm the outcome from the next picture and LIDAR before you give the answer.
  If you must see the outcome (searching, approaching or describing something), set "done" to false:
  you get the pictures and give the final answer in the next step.
- If an action fails or is rejected, the rest of your plan is skipped and you replan from the new image.

Rules:
- Use only skills from the catalog (and "look"), with values inside their min/max limits.
  Omit "value" for skills without a parameter.
- Every picture is labelled with the robot's estimated heading and position relative to the START of the task
  (heading > 0 = turned left, < 0 = turned right; x = metres forward, y = metres left). Views of neighbouring headings
  OVERLAP (the camera sees ~80 degrees, turns are 45), so the same object appears in several pictures.
  When counting or listing objects, match them across pictures by colour, shape and bearing, and count every
  physical object only ONCE.
- Never claim success unless the images or the results confirm it.
- To go up to a wall or obstacle ("walk until you reach the wall") use the skill walk_to_obstacle. Never guess that
  distance with walk_forward. A walk that ends with status blocked, timeout or no_obstacle did NOT do what you wanted:
  look at the LIDAR and the picture and replan, do not repeat the same walk.
  Exception: when walk_to_obstacle (or a walk towards a wall) ends with blocked, the robot is already standing at the
  wall - that is the goal reached, continue with the next part of the task.
- The robot walks at about 0.5 m/s: "walk forward for N seconds" means about 0.5*N metres (5 seconds = 2.5 m),
  done as walk_forward steps of <= 1 m.
- Following a wall: the wall must stay at about the same distance at the robot's side (read it from the LIDAR). Walk in
  steps of <= 1 m and plan at most 3 steps at a time, then check the LIDAR in the results. The wall has ended when the
  side distance suddenly jumps by more than 1 m (or becomes '>10'); then stop there. Turn by <= 45 degrees to correct drift.
- Remember what is already done. The results list every executed action; each part of a multi-part task is done ONCE.
  Never repeat a finished part (for example a turn that was already made) unless its result says it failed.
- If the task needs something that is not in the catalog (e.g. jumping), do the possible part, say clearly what you
  cannot do in "answer", and set "done" to true.
- Language: write "observation", "thought" and "answer" in the same language as the user's task
  (an English task gets English replies, a Slovak task Slovak replies). Set "language" to "en" for English
  and to "sk" for Slovak or Czech (this switches the labels of the console output).
- Keep "observation" and "thought" to one or two short sentences. Put ALL findings the user asked for into "answer"
  (complete and self-contained, e.g. every unique object once with colour, shape and bearing), and only when
  "done" is true - do not describe the results of the task in earlier steps.

Answer with ONE JSON object and nothing else:
{{"observation": "what you see, short",
  "thought": "your reasoning/plan, short",
  "actions": [{{"skill": "<name>", "value": <number or omitted>}}],
  "done": false,
  "answer": "",
  "language": "sk or en"}}

Skill catalog (JSON):
{catalog}
"""


def parse_json(text):
    """Vytiahne JSON objekt z odpovede AI (aj ked je obaleny v ```)."""
    if text is None:
        raise ValueError("prazdna odpoved")
    t = text.strip()
    m = re.search(r"```(?:json)?\s*(.*?)```", t, re.S)
    if m:
        t = m.group(1).strip()
    a, b = t.find("{"), t.rfind("}")
    if a < 0 or b <= a:
        raise ValueError("v odpovedi nie je JSON objekt")
    return json.loads(t[a:b + 1])


MSG = {
    "sk": {"list": "'actions' musi byt zoznam",
           "many": "prilis vela akcii (%d), max %d - ostatne ignorujem",
           "noskill": "akcia %d nema pole 'skill'",
           "unknown": "akcia %d: neznamy skill '%s'",
           "nonum": "akcia %d: hodnota '%s' nie je cislo",
           "limits": "akcia %d: %s %g %s je mimo limitov <%s, %s>",
           "json": "odpoved AI nebola platny JSON (%s)"},
    "en": {"list": "'actions' must be a list",
           "many": "too many actions (%d), max %d - the rest is ignored",
           "noskill": "action %d has no 'skill' field",
           "unknown": "action %d: unknown skill '%s'",
           "nonum": "action %d: value '%s' is not a number",
           "limits": "action %d: %s %g %s is outside the limits <%s, %s>",
           "json": "AI reply was not valid JSON (%s)"},
}


def validate_actions(actions, catalog, max_actions, lang="sk"):
    """Vrati (ciste_akcie, chyby). Kontrola mena skillu a limitov hodnoty ('look' je povoleny bez parametra)."""
    m = MSG.get(lang, MSG["sk"])
    specs = {s["name"]: s for s in catalog}
    specs["look"] = LOOK
    ok, errors = [], []
    if not isinstance(actions, list):
        return [], [m["list"]]
    if len(actions) > max_actions:
        errors.append(m["many"] % (len(actions), max_actions))
        actions = actions[:max_actions]
    for i, a in enumerate(actions):
        if not isinstance(a, dict) or "skill" not in a:
            errors.append(m["noskill"] % (i + 1))
            continue
        name = a["skill"]
        spec = specs.get(name)
        if spec is None:
            errors.append(m["unknown"] % (i + 1, name))
            continue
        value = 0.0
        if "parameter" in spec:
            p = spec["parameter"]
            raw = a.get("value", p.get("default", 0))
            try:
                value = float(raw)
            except (TypeError, ValueError):
                errors.append(m["nonum"] % (i + 1, raw))
                continue
            if not (p["min"] <= value <= p["max"]):
                errors.append(m["limits"] % (i + 1, name, value, p["unit"], p["min"], p["max"]))
                continue
        ok.append({"skill": name, "value": value})
    return ok, errors


class Agent:
    """robot: .execute(skill, value) -> dict(success, status, message, value_done)
       camera: .snapshot() -> bytes (PNG/JPEG), mime v .mime
       llm: .ask(system, user_text, images, mime=None) -> str     (images: zoznam (bytes, mime, popis))
       catalog: zoznam skills (JSON z /skills/catalog)"""

    MAX_IMAGES = 10

    def __init__(self, robot, camera, llm, catalog, max_iterations=10, max_actions=20,
                 max_total_distance=15.0, max_failures=3, run_dir=None, log=print, should_cancel=None, lidar=None):
        self.robot, self.camera, self.llm, self.catalog = robot, camera, llm, catalog
        self.lidar = lidar          # objekt s .read() -> {"front":..,"left":..,"right":..,"back":..,"arms_up":..} alebo None
        self.max_iterations, self.max_actions = max_iterations, max_actions
        self.max_total_distance, self.max_failures = max_total_distance, max_failures
        self.run_dir, self.log = run_dir, log
        self.should_cancel = should_cancel or (lambda: False)
        self.units = {s["name"]: s["parameter"].get("unit", "") for s in catalog if "parameter" in s}
        full_catalog = list(catalog) + [LOOK]
        self.system = SYSTEM_PROMPT.format(max_actions=max_actions, catalog=json.dumps(full_catalog, ensure_ascii=False))

    @staticmethod
    def _pose_text(p):
        return "heading %+.0f deg, position x=%.1f m forward, y=%.1f m left" % (p["h"], p["x"], p["y"])

    @staticmethod
    def _update_pose(p, skill, requested, done, success=True):
        import math
        d = float(requested) if success else float(done or 0.0)      # pri neuspechu len to, co sa naozaj spravilo
        if skill == "turn_left":
            p["h"] += d
        elif skill == "turn_right":
            p["h"] -= d
        elif skill in MOVE_SKILLS:
            d = float(done) if (done or skill == "walk_to_obstacle" or not success) else d
            ang = math.radians(p["h"] + {"walk_forward": 0, "walk_to_obstacle": 0, "walk_backward": 180,
                                          "step_left": 90, "step_right": -90}[skill])
            p["x"] += d * math.cos(ang)
            p["y"] += d * math.sin(ang)

    def _lidar_text(self):
        """vzdialenosti z lidaru ako text pre AI ('' ak lidar nie je pripojeny)"""
        if self.lidar is None:
            return ""
        try:
            d = self.lidar.read()
        except Exception:
            d = None
        if not d:
            return "LIDAR: no data"

        def f(k):
            v = d.get(k)
            return "?" if v is None else (">10" if float(v) >= 9.99 else "%.1f" % float(v))
        t = "LIDAR (m): front %s, left %s, right %s, back %s" % (f("front"), f("left"), f("right"), f("back"))
        if d.get("arms_up"):
            t += " (arms are raised - readings may show the arms)"
        if d.get("upright") is False:                                  # SKILLS2_PATCH
            t += " | ROBOT HAS FALLEN (it lies on the floor) - call get_up"
        return t

    def _lidar_suffix(self, a):
        """lidar po pohybe - pripoji sa k vysledku akcie v historii"""
        if a["skill"] in MOVE_SKILLS or a["skill"].startswith("turn_"):
            t = self._lidar_text()
            return (" | " + t.replace("LIDAR (m): ", "lidar after this: ")) if t else ""
        return ""

    def _where(self, pose):
        t = self._lidar_text()
        return self._pose_text(pose) + (("; " + t) if t else "")

    def label(self, a):
        if a["skill"] in self.units:
            return ("%s %g %s" % (a["skill"], a["value"], self.units[a["skill"]])).strip()
        return a["skill"]

    def run(self, task, feedback=None):
        fb = feedback or (lambda state, it: None)
        history = []          # riadky textu: co sa vykonalo a aky bol vysledok
        total_dist = 0.0
        failures = 0
        steps_done = 0
        pose = {"h": 0.0, "x": 0.0, "y": 0.0}      # odhad polohy (dead reckoning) voci zaciatku ulohy
        lang = "sk"           # jazyk hlaseni (AI ho potvrdi v poli "language")
        pics = []             # snimky z 'look' z predchadzajuceho planu: (bytes, mime, popis)
        last_look = False
        if self.run_dir:
            os.makedirs(self.run_dir, exist_ok=True)
        for it in range(1, self.max_iterations + 1):
            if self.should_cancel():
                return self._result(False, "cancelled", "Uloha bola zrusena.", steps_done, it - 1)
            fb("thinking", it)
            images = list(pics)
            if not (pics and last_look):
                cur = self.camera.snapshot()
                mime = getattr(self.camera, "mime", "image/png")
                self._save("step_%02d_image" % it, cur, mime)
                images.append((cur, mime, "CURRENT VIEW (taken now, after all your previous actions; %s)" % self._where(pose)))
            images = images[-self.MAX_IMAGES:]
            user = "TASK: %s\n\nPrevious actions and results:\n%s\n\nIteration %d of %d. You receive %d image(s). " \
                   "Look at them and decide." % (task, "\n".join(history[-40:]) or "(none yet)", it,
                                                  self.max_iterations, len(images))
            lt = self._lidar_text()
            if lt:
                user += "\n\nNOW: " + self._pose_text(pose) + "; " + lt
            t0 = time.time()
            try:
                reply = self.llm.ask(self.system, user, images)
            except Exception as e:     # siet, kluc, limit...
                self.log("LLM chyba: %s" % e)
                return self._result(False, "llm_error", "Chyba AI sluzby: %s" % e, steps_done, it)
            dt = time.time() - t0
            fb("llm|%.1f" % dt, it)
            self._save("step_%02d_reply" % it, reply.encode("utf-8"), "text/plain")
            try:
                plan = parse_json(reply)
            except Exception as e:
                failures += 1
                history.append("[system] Your last reply was not valid JSON (%s). Reply with ONE JSON object." % e)
                fb("blocked|" + MSG[lang]["json"] % e, it)
                if failures >= self.max_failures:
                    return self._result(False, "bad_plan", "AI opakovane nevratila platny JSON.", steps_done, it)
                continue
            pics, last_look = [], False
            pl = str(plan.get("language", "")).lower()[:2]
            if pl in MSG and pl != lang:
                lang = pl
                fb("lang|" + lang, it)
            obs, why = str(plan.get("observation", "")).strip(), str(plan.get("thought", "")).strip()
            self.log("[%d] (%.1f s) %s | %s" % (it, dt, obs, why))
            will_finish = plan.get("done") is True
            if obs and will_finish:          # opis ("co vidim") sa ukaze az v zavere, nie uz na zaciatku
                fb("obs|" + obs, it)
            if why:
                fb("why|" + why, it)
            actions, errors = validate_actions(plan.get("actions", []), self.catalog, self.max_actions, lang)
            done = plan.get("done") is True
            if actions:
                fb("plan|" + json.dumps([self.label(a) for a in actions], ensure_ascii=False), it)
            for e in errors:
                fb("blocked|" + e, it)
            if done and not actions and not errors:
                return self._result(True, "ok", str(plan.get("answer", "")), steps_done, it)
            if errors:
                failures += 1
                history.append("[system] Rejected by validation: " + "; ".join(errors))
                self.log("Plan zamietnuty: %s" % errors)
                if failures >= self.max_failures:
                    return self._result(False, "bad_plan", "AI opakovane navrhla neplatne akcie: " + "; ".join(errors),
                                        steps_done, it)
                if not actions:
                    continue
            if not actions and not errors:
                failures += 1
                history.append("[system] Your plan had no actions and was not marked done.")
                if failures >= self.max_failures:
                    return self._result(False, "bad_plan", "AI opakovane nenavrhla ziadne akcie.", steps_done, it)
                continue
            plan_ok = True
            prev = "start"
            for idx, a in enumerate(actions):
                if self.should_cancel():
                    return self._result(False, "cancelled", "Uloha bola zrusena.", steps_done, it)
                fb("start|%d" % idx, it)
                if a["skill"] == "look":
                    img = self.camera.snapshot()
                    mime = getattr(self.camera, "mime", "image/png")
                    pics.append((img, mime, "PICTURE from your 'look' action %d (%s)" % (idx + 1, self._where(pose))))
                    self._save("step_%02d_look%02d" % (it, idx + 1), img, mime)
                    history.append("look -> picture %d taken" % len(pics))
                    fb("ok|%d|snimka" % idx, it)
                    last_look = True
                    continue
                last_look = False
                res = self.robot.execute(a["skill"], a["value"])
                steps_done += 1
                msg = str(res.get("message", ""))
                if a["skill"] in MOVE_SKILLS or a["skill"].startswith("turn_"):
                    self._update_pose(pose, a["skill"], a["value"], res.get("value_done", 0.0), bool(res.get("success")))
                history.append("[%d] %s(%g) -> %s: %s%s" % (it, a["skill"], a["value"], res.get("status"), msg,
                                                           self._lidar_suffix(a)))
                if not res.get("success"):
                    history.append("(pose estimate now: %s)" % self._pose_text(pose))
                self.log("   %s -> %s: %s" % (self.label(a), res.get("status"), msg))
                prev = self.label(a)
                if a["skill"] in MOVE_SKILLS:
                    total_dist += float(res.get("value_done", 0.0))
                if res.get("success"):
                    failures = 0
                    fb("ok|%d|%s" % (idx, msg), it)
                else:
                    failures += 1
                    plan_ok = False
                    fb("fail|%d|%s" % (idx, msg or str(res.get("status"))), it)
                    for j in range(idx + 1, len(actions)):
                        fb("skip|%d" % j, it)
                    break          # po chybe sa AI znova pozrie a rozhodne
                if total_dist > self.max_total_distance:
                    return self._result(False, "distance_limit",
                                        "Prekroceny limit celkovej vzdialenosti (%.0f m)." % self.max_total_distance,
                                        steps_done, it)
            if failures >= self.max_failures:
                return self._result(False, "too_many_failures", "Prilis vela neuspesnych krokov po sebe.", steps_done, it)
            if done and plan_ok and not errors:
                return self._result(True, "ok", str(plan.get("answer", "")), steps_done, it)
        return self._result(False, "max_iterations", "Vycerpany pocet iteracii bez dokoncenia ulohy.",
                            steps_done, self.max_iterations)

    def _save(self, name, data, mime):
        if not self.run_dir:
            return
        ext = {"image/png": "png", "image/jpeg": "jpg", "text/plain": "json"}.get(mime, "bin")
        with open(os.path.join(self.run_dir, "%s.%s" % (name, ext)), "wb") as f:
            f.write(data)

    def _result(self, success, status, summary, steps, iterations):
        r = {"success": success, "status": status, "summary": summary, "steps": steps, "iterations": iterations}
        if self.run_dir:
            with open(os.path.join(self.run_dir, "result.json"), "w") as f:
                json.dump(r, f, ensure_ascii=False, indent=2)
        return r

# STUCK_PATCH
