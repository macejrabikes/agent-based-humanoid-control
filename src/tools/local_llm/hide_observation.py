#!/usr/bin/env python3
"""task_client.py: skryje riadok 'Vidím: ...' (observation od AI). Vsetko podstatne je aj tak v zaverecnej sprave.

Riadok sa da vratit bez patchu:  export CHAPPIE_SHOW_OBS=1   (nastav v terminali, kde spustas chappie_task).
Idempotentny: zaloha *.before_obs, vzdy sa aplikuje od zalohy.
  python3 hide_observation.py              # aplikuj
  python3 hide_observation.py --restore    # vrat povodne
Potom: colcon build --symlink-install --packages-select humanoid_agent (chappie_task staci spustit znova).
"""
import os
import shutil
import sys

WS = os.path.expanduser(os.environ.get("CHAPPIE_WS", "~/agent-based-humanoid-control"))
PATH = os.path.join(WS, "src/humanoid_agent/humanoid_agent/task_client.py")
SUF = ".before_obs"
EDITS = [
    ('import json\nimport shutil\n', 'import json\nimport os\nimport shutil\n'),
    ('''        elif kind == "obs":
            self.w("    %s: %s" % (T[self.lang]["see"], rest))''',
     '''        elif kind == "obs":
            if os.environ.get("CHAPPIE_SHOW_OBS") == "1":       # OBS_PATCH: predvolene sa 'Vidim' neukazuje
                self.w("    %s: %s" % (T[self.lang]["see"], rest))'''),
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
    for old, new in EDITS:
        if text.count(old) != 1:
            sys.exit("nenasiel som presne 1x kotvu:\n%s" % old[:120])
        text = text.replace(old, new)
    open(PATH, "w", encoding="utf-8").write(text)
    print("upravene:", PATH)
    print("Potom: colcon build --symlink-install --packages-select humanoid_agent")


if __name__ == "__main__":
    main()
