"""task_client: odosle ulohu AI agentovi (/agent/task) a prehladne vypisuje priebeh.

Pouzitie:  chappie_task "otoc sa dolava o 90 stupnov a povedz co vidis"
           (= ros2 run humanoid_agent task_client "uloha"; Ctrl+C ulohu zrusi)

Vypis:  Rozmýšľam... -> Vidím / Úvaha -> Plánované akcie ([ ] čaká, [>] prebieha, [✓] hotovo, [X] zlyhalo, [-] preskočené)
        -> Zamietnuté akcie (kontrola výstupu AI) -> na konci výsledok.
"""
import json
import shutil
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from humanoid_interfaces.action import Task

T = {
    "sk": {"wait": "Čakám na agenta...", "task": "Úloha", "think": "Rozmýšľam...", "see": "Vidím", "why": "Úvaha",
           "plan": "Plánované akcie", "blocked": "Zamietnuté akcie", "nosrv": "Agent nebeží (/agent/task). Spustil si chappie_sim?",
           "usage": 'Použitie:  chappie_task "úloha"   napr.  chappie_task "otoč sa doľava o 90 stupňov"',
           "rej": "Úloha nebola prijatá (agent je možno obsadený inou úlohou).", "cancel": "Rušim úlohu...",
           "intr": "Prerušené."},
    "en": {"wait": "Waiting for the agent...", "task": "Task", "think": "Thinking...", "see": "I see", "why": "Reasoning",
           "plan": "Planned actions", "blocked": "Rejected actions", "nosrv": "The agent is not running (/agent/task). Did you start chappie_sim?",
           "usage": 'Usage:  chappie_task "task"   e.g.  chappie_task "turn left 90 degrees"',
           "rej": "The task was not accepted (the agent may be busy with another task).", "cancel": "Cancelling the task...",
           "intr": "Interrupted."},
}
EN_WORDS = set("the a an to and then look around tell me what you see go walk turn left right forward back backward "
               "meters metres meter step raise hand arm your find move stop how many colors colour color shapes of in is "
               "are please take pick up down say describe".split())
SK_WORDS = set("otoc otoč sa doprava dolava prejdi chod choď povedz kolko koľko ake aké akej farby tvarov vidis vidíš "
               "metre metrov cuvni cúvni pozri ruku zdvihni dopredu dozadu poobzeraj poobzeraaj najdi nájdi potom dole "
               "hore skoc skoč stoj krok k do na ze že".split())


def guess_lang(task):
    """odhad jazyka ulohy (AI ho neskor potvrdi); predvolene slovencina"""
    if any(ch in task.lower() for ch in "áäčďéíĺľňóôŕšťúýž"):
        return "sk"
    words = __import__("re").findall(r"[a-zá-ž']+", task.lower())
    en = sum(w in EN_WORDS for w in words)
    sk = sum(w in SK_WORDS for w in words)
    return "en" if en > sk else "sk"


MARK = {"wait": "[ ]", "run": "[>]", "ok": "[✓]", "fail": "[X]", "skip": "[-]"}


class View:
    """vykresluje priebeh; na terminali prekresluje blok planu na mieste, inak vypisuje po riadkoch"""

    def __init__(self, out=None):
        self.out = out or sys.stdout
        self.tty = self.out.isatty()
        self.items = []          # [stav, popis, sprava]
        self.tail = []           # zamietnute akcie (zobrazene pod planom)
        self.drawn = 0           # kolko riadkov bloku je uz vykreslenych
        self.active = False
        self.llm_wait = False
        self.blocked_hdr = False
        self.t0 = time.time()
        self.col = self.tty
        self.lang = "sk"

    def c(self, code, s):
        return "\033[%sm%s\033[0m" % (code, s) if self.col else s

    def w(self, s=""):
        self.out.write(s + "\n")
        self.out.flush()

    def width(self):
        return max(40, shutil.get_terminal_size((100, 20)).columns - 1)

    def lines(self):
        out = []
        for i, (st, label, msg) in enumerate(self.items):
            mark = MARK[st]
            colour = {"ok": "32", "fail": "31", "run": "33", "skip": "90"}.get(st)
            txt = "  %s %d. %s" % (mark, i + 1, label)
            if msg and st in ("fail",):
                txt += " - " + msg
            txt = txt[:self.width()]
            out.append(self.c(colour, txt) if colour else txt)
        if self.tail:
            out.append("    %s:" % T[self.lang]["blocked"])
            for t in self.tail:
                out.append(self.c("31", ("  [X] " + t)[:self.width()]))
        return out

    def draw(self, first=False):
        ls = self.lines()
        if self.tty and not first and self.drawn:
            self.out.write("\033[%dA" % self.drawn)
        for l in ls:
            self.out.write(("\033[2K" if self.tty else "") + l + "\n")
        self.out.flush()
        self.drawn = len(ls)

    def close_block(self):
        self.blocked_hdr = False
        self.active = False
        self.items, self.tail, self.drawn = [], [], 0

    def event(self, state, it):
        kind, _, rest = state.partition("|")
        if kind == "thinking":
            self.close_block()
            self.out.write(self.c("36", "[%d] %s" % (it, T[self.lang]["think"])))
            self.out.flush()
            self.llm_wait = True
        elif kind == "lang":
            if rest in T:
                self.lang = rest
        elif kind == "llm":
            self.out.write(self.c("90", " (%s s)" % rest) + "\n")
            self.llm_wait = False
        elif kind == "obs":
            self.w("    %s: %s" % (T[self.lang]["see"], rest))
        elif kind == "why":
            self.w("    %s: %s" % (T[self.lang]["why"], rest))
        elif kind == "plan":
            try:
                labels = json.loads(rest)
            except ValueError:
                labels = [rest]
            self.items = [["wait", l, ""] for l in labels]
            self.tail, self.drawn, self.active = [], 0, True
            self.w("    %s:" % T[self.lang]["plan"])
            self.draw(first=True)
        elif kind == "blocked":
            if self.active and self.tty:
                self.tail.append(rest)
                self.draw()
            else:
                if not self.blocked_hdr:
                    self.w("    %s:" % T[self.lang]["blocked"])
                    self.blocked_hdr = True
                self.w(self.c("31", "  [X] " + rest))
        elif kind in ("start", "ok", "fail", "skip"):
            try:
                idx = int(rest.split("|")[0])
                msg = rest.partition("|")[2]
            except ValueError:
                return
            if 0 <= idx < len(self.items):
                self.items[idx][0] = {"start": "run"}.get(kind, kind)
                self.items[idx][2] = msg
                if self.tty:
                    self.draw()
                elif kind != "start":
                    self.w("    %s %d. %s%s" % (MARK[self.items[idx][0]], idx + 1, self.items[idx][1],
                                                 (" - " + msg) if kind == "fail" and msg else ""))


_LANG = ["sk"]


def main():
    try:
        _main()
    except KeyboardInterrupt:
        print("\n" + T[_LANG[0]]["intr"])
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(130)


def _main():
    task = " ".join(sys.argv[1:]).strip()
    lang = guess_lang(task)
    _LANG[0] = lang
    if not task:
        print(T["sk"]["usage"])
        sys.exit(2)
    rclpy.init()
    node = Node("chappie_task_client")
    client = ActionClient(node, Task, "/agent/task")
    print(T[lang]["wait"])
    if not client.wait_for_server(timeout_sec=15.0):
        print(T[lang]["nosrv"])
        rclpy.shutdown()
        sys.exit(1)
    view = View()
    view.lang = lang
    goal = Task.Goal()
    goal.task = task
    print("%s: %s\n" % (T[lang]["task"], task))
    t0 = time.time()
    send = client.send_goal_async(goal, feedback_callback=lambda m: view.event(m.feedback.state, m.feedback.iteration))
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        print(T[lang]["rej"])
        rclpy.shutdown()
        sys.exit(1)
    res = gh.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, res)
    except KeyboardInterrupt:
        print("\n" + T[view.lang]["cancel"])
        gh.cancel_goal_async()
        rclpy.spin_until_future_complete(node, res, timeout_sec=15.0)
    wrapped = res.result()
    r = wrapped.result
    names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
             GoalStatus.STATUS_CANCELED: "CANCELED"}
    print("\nResult:")
    print("    success: %s" % str(r.success).lower())
    print("status: %s" % r.status)
    print("summary: %s" % r.summary)
    print("steps: %d" % r.steps)
    print("iterations: %d" % r.iterations)
    print("time: %.0f s" % (time.time() - t0))
    print("\nGoal finished with status: %s" % names.get(wrapped.status, str(wrapped.status)))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if r.success else 1)


if __name__ == "__main__":
    main()
