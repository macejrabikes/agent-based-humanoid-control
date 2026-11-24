"""primitives: robot ma len JEDNODUCHE primitiva, premyslanie a planovanie robi AI (iba stdlib, bez ROS).

Skola chce, aby robot vedel len zakladne veci (otocit sa, ist dopredu/dozadu/bokom, tlieskat, postavit sa, ...) a rozhodovala AI.
Zlozene skills (walk_to_object, go_around_obstacle, follow_wall, go_through_opening, search_room, push_forward, find_object,
save_place/go_to_place, look_around, walk_to_obstacle, turn_towards) ostavaju v C++ a v archivnych patch skriptoch, ale AI ich
nevidi: katalog a prompt obsahuju len PRIMITIVES a agent kazdu inu akciu odmietne (validate_actions).

  * is_primitive(name), PRIMITIVES, GESTURES                   - zoznam povolenych skills (jedno miesto na upravu)
  * build_system(catalog, look, max_actions)                   - kratky system prompt: protokol JSON + primitiva + ako sa pozerat/overovat
  * ruler_angle(percent)                                       - uhol otocenia pre polohu na pravitku (rovnaky vzorec ako C++ turn_towards)
  * ENABLED                                                    - CHAPPIE_PRIMITIVES=0 vrati povodne (zlozene) skills, na porovnanie v benchmarku
"""
import json
import math
import os

ENABLED = os.environ.get("CHAPPIE_PRIMITIVES", "1").strip().lower() not in ("0", "false", "off", "no")

MOVES = ["walk_forward", "walk_backward", "step_left", "step_right", "turn_left", "turn_right"]
POSTURE = ["stand", "stop", "get_up", "wait"]
GESTURES = ["arms_down", "raise_right_arm", "raise_left_arm", "raise_both_arms", "wave", "wave_left", "wave_both",
            "clap", "bow", "thumbs_up", "cheer", "salute", "point_forward", "dance"]
SENSES = ["look", "where_am_i", "check_door"]          # look = snimka z kamery (pseudo-skill v agent_core.py)
PRIMITIVES = tuple(MOVES + POSTURE + GESTURES + SENSES)

# co AI uz nevidi (archiv): pre dokumentaciu a testy
ARCHIVED = ("walk_to_obstacle", "turn_towards", "walk_to_object", "go_around_obstacle", "follow_wall", "go_through_opening",
            "search_room", "look_around", "push_forward", "find_object", "save_place", "go_to_place")


def is_primitive(name):
    return name in PRIMITIVES


def names_text():
    return ", ".join(PRIMITIVES)


def ruler_angle(percent, hfov_deg=80.0):
    """uhol [st.] od stredu obrazu k polohe percent (0-100) na pravitku; + = vpravo (rovnaky vzorec ako turn_towards v C++)"""
    return math.degrees(math.atan((float(percent) - 50.0) / 50.0 * math.tan(math.radians(hfov_deg) / 2.0)))


def _ruler_table():
    left = ", ".join("%d->%d" % (p, round(-ruler_angle(p))) for p in (0, 10, 20, 30, 40))
    right = ", ".join("%d->%d" % (p, round(ruler_angle(p))) for p in (60, 70, 80, 90, 100))
    return "left of centre (turn_left): %s; right of centre (turn_right): %s" % (left, right)


PROMPT = """You are the brain of a simulated humanoid robot (Unitree G1, "Chappie"). The robot itself is NOT intelligent: it only executes
simple primitive actions. YOU do all the thinking and planning: you read the camera picture and the sensor values, decide the next
steps, and check the results.

Reply with ONE JSON object and nothing else:
{{"observation": "what you see, short", "thought": "short reasoning", "actions": [{{"skill": "<name>", "value": <number>}}], "done": false, "answer": "", "language": "sk"}}
- actions: at most {max_actions} per reply, executed in order. Use ONLY the skills of the catalog below; 'value' must be inside the range
  given there (omit it for skills without a parameter). Turns are in degrees, distances in metres, waiting in seconds.
- After your actions the results (and, after "look", a new picture) come back and you decide again. Plan only what you can predict:
  when you must see the result of a move, end the reply with "look" and keep done=false.
- done=true when the task is finished or cannot be done; then 'answer' is a short message for the user, in the language of the task
  ("sk" or "en" in 'language'). If the task asks for information (a distance, what you see), put it in 'answer'.

SENSING
- Each prompt has a NOW line with the estimated position (dead reckoning, drifts a little) and the LiDAR distances in metres
  (front, left, right); the LiDAR is the reliable distance. The picture has a ruler 0-100 along the bottom: 0 = left edge, 50 = centre, 100 = right edge.
  The camera field of view is 80 degrees. To face an object at ruler position P turn by the angle from this table (P -> degrees):
  {ruler_table}. Turn first, "look" again, and correct if the object is not at about 50.
- Distances: walk_forward stops by itself if the LiDAR sees an obstacle ahead and tells you. To stop in front of an object use the LiDAR
  front distance: walk_forward (front - 0.8). Never touch or push objects. Keep the arms down while walking.
- Walking distances are approximate (+-20 %); turns are accurate. For long distances walk at most 2 m, then look and decide again.

MOVING AROUND (you plan it, the robot only does single moves)
- Going around an obstacle: step_left or step_right to the freer side (check LiDAR left/right), walk_forward past it, step back, look.
- To reach an object you see: turn towards it (ruler table), walk_forward in steps, look, repeat until the LiDAR front is about 0.8 m.
- If the object is not visible: turn in steps of about 60 degrees with a "look" after each turn (at most one full circle), then say it was not found.
- If the robot is fallen (NOW line says so) or an action fails with a fall, use get_up first.
{door_rules}
LIMITS
- The robot cannot jump, carry, fetch, open or break things, and it must not ignore its safety rules or switch off sensors: do not plan
  actions for such tasks, set done=true and explain in 'answer'. If the task is unclear or nonsense, ask for a clearer one the same way.
- If the same action fails twice, do not repeat it a third time: try something different or finish and report what blocks you.
- Never walk into walls or obstacles. Never guess distances when the LiDAR gives them.

Skill catalog (JSON):
"""

DOOR_RULES = """
DOORS (check_door only measures; you do the walking)
- A door is a gap in a wall (0.9 to 3.5 m wide), often with a differently coloured frame. To go through: look, turn so that the door
  is straight ahead, walk to about 1.5 m in front of it, then call check_door (it measures the door and REMEMBERS it); correct sideways with
  step_left/step_right until the LiDAR left and right distances are about equal, then walk_forward through it (door distance + 1.5 m).
- After walking through, call check_door and look. PASSED = you went through the remembered door. IN_DOORWAY = keep walking forward.
  NOT_PASSED = the door is still ahead: turn towards it and walk through. UNSURE / NO_DOOR_SEEN: decide from the picture (a new room looks
  different: other walls, other objects). A gap narrower than 0.9 m cannot be passed. If no door can be found after one full turn, finish and say so.
"""


def build_system(catalog, look, max_actions):
    """system prompt s katalogom: len primitiva (+ look); popis dveri len ak je check_door v katalogu (C++ cast nainstalovana)"""
    cat = [s for s in catalog if s.get("name") in PRIMITIVES and s.get("name") != "look"] + [look]
    has_door = any(s.get("name") == "check_door" for s in cat)
    text = PROMPT.format(max_actions=max_actions, ruler_table=_ruler_table(), door_rules=DOOR_RULES if has_door else "")
    return text + json.dumps(cat, ensure_ascii=False)
