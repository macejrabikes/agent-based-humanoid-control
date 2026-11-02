# Agent-Based Humanoid Control

Bakalársky projekt: **Riadenie humanoidného robota v simulačnom prostredí pomocou agentových systémov**

- **Univerzita:** Technická univerzita v Košiciach (TUKE), Fakulta elektrotechniky a informatiky
- **Študijný program:** Inteligentné systémy
- **Pracovisko:** Ústav umelej inteligencie (ÚUI)
- **Vedúci práce:** doc. Ing. Peter Papcun, PhD.
- **Autor:** Matej Rabik

## Abstrakt

Projekt prepája jazykový model (LLM) s humanoidným robotom v simulácii. Upravený model Unitree G1 s názvom **Chappie** beží v Gazebo (ROS 2 Jazzy, Gazebo Harmonic). Používateľ zadá úlohu v prirodzenom jazyku, napríklad `chappie_task "otoč sa k červenej škatuli, choď k nej a posuň ju o meter"`. AI agent dostane snímku z kamery, textový popis okolia z LiDARu, históriu a katalóg hotových funkcií (skills). Vráti plán ako zoznam akcií, ktorý sa skontroluje a potom vykoná. Neplatné akcie robot odmietne.

Prínos projektu:

- **Bezpečné rozhranie medzi AI a robotom:** LLM nevydáva priame povely motorom, vyberá len z overených skills (chôdza, otáčanie, gestá, vstávanie, navigácia, tlačenie). Výstup AI sa validuje.
- **Navigácia a vnímanie:** LiDAR (zastavenie pred prekážkou), pamäť miest (`save_place`, `go_to_place`), natočenie k predmetu podľa pravítka na obraze z kamery (`turn_towards`), tlačenie predmetov (`push_forward`), prechod dverami (`go_through_opening`: otvor v stene nájde LiDAR), prehľadávanie izby (`search_room`).
- **Odolnosť voči výpadkom AI služieb:** reťaz Gemini → Groq → OpenRouter → Mistral, vyčerpanú službu preskočí, vedie si počítadlá limitov a meria spotrebu tokenov (`chappie_cost`, `tools/probe_limits.py`).

## Štruktúra repozitára

| Cesta | Obsah |
|---|---|
| `README.md` | tento súbor |
| `requirements.txt` | Python závislosti a poznámka k systémovým závislostiam |
| `src/` | ROS 2 balíky (zdrojový kód, okomentovaný) |
| `src/humanoid_description` | URDF/Xacro model robota Chappie, senzory (kamera, LiDAR Mid-360), meshe |
| `src/humanoid_gazebo` | spúšťacie súbory a svety (`arena.sdf`) pre Gazebo |
| `src/humanoid_walk` | C++ node s natrénovanou politikou chôdze a vstávania |
| `src/humanoid_skills` | C++ knižnica skills (`skill_core.hpp`), node `motion_skills`, pózy rúk |
| `src/humanoid_agent` | Python AI agent: `agent_core.py` (plán), `llm_backends.py` (Gemini/Groq/OpenRouter/Mistral), `task_client.py` (príkaz `chappie_task`) |
| `src/humanoid_interfaces` | ROS 2 akcie `Skill` a `Task` |
| `src/humanoid_teleop` | ovládanie z klávesnice (WASD) |
| `src/humanoid_control` | konfigurácia kontrolérov |
| `tools/` | pomocné skripty (generovanie modelu a kolízií, export politiky), generátor svetov `gen_world.py`, meranie limitov AI `probe_limits.py` a 2D test hľadania dverí `test_door_sim.cpp` |
| `maps/` | ASCII mapy pre generátor svetov |
| `mujoco_demo/` | záložné demo chôdze v MuJoCo |
| `doc/` | technická dokumentácia: návod na spustenie (`doc/NAVOD.md`), prehľad príkazov `chappie_*` (`doc/PRIKAZY.md`), prehľad skillov (`doc/SKILLY.md`), literatúra (`doc/LITERATURA.md`) |

## Architektúra

```
chappie_task "úloha"
   └─> /agent/task ──> agent_node (Agent.run: kamera + LiDAR + história + katalóg skills ─> LLM ─> JSON plán ─> validácia)
                                 └─> /skills/execute ──> motion_skills (C++, SkillExecutor)
                                          └─> /cmd_vel a trajektórie rúk ──> walk_policy ──> Gazebo
```

## Spustenie

Podrobný návod je v [`doc/NAVOD.md`](doc/NAVOD.md). Skrátene:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch humanoid_gazebo walk_demo.launch.py        # simulácia + agent
ros2 run humanoid_agent task_client "otoč sa doľava o 90 stupňov"
```

### Svety a testy

```bash
python3 tools/gen_world.py --random rooms --seed 3 --name rooms_3     # náhodné izby s dverami (ďalej maze, warehouse alebo --ascii maps/izba.txt)
ros2 launch humanoid_gazebo walk_demo.launch.py world:=rooms_3.sdf    # simulácia s týmto svetom
# test hľadania dverí a prehľadávania izby bez ROS (2D simulátor LiDARu):
g++ -std=c++17 -O1 -I src/humanoid_skills/include tools/test_door_sim.cpp -o /tmp/test_door && /tmp/test_door
python3 tools/probe_limits.py --service gemini     # zmeria limity AI služby
python3 src/humanoid_agent/humanoid_agent/cost_report.py            # spotreba tokenov a nákladov AI (v návode ako príkaz chappie_cost)
```

Prehľad skillov je v [`doc/SKILLY.md`](doc/SKILLY.md).

API kľúče sa nikdy neukladajú do repozitára. Čítajú sa z premenných prostredia alebo zo súboru `~/.chappie_keys`.

## Použitie umelej inteligencie

V súlade s pokynmi ÚUI uvádzam, kde a ako bola AI použitá.

1. **AI ako súčasť systému (runtime):** modely Gemini, Groq, OpenRouter a Mistral rozhodujú, ktoré skills sa použijú. Ide o samotný predmet práce.
2. **AI ako pomocník pri vývoji:** asistent Claude (Anthropic) pomáhal pri návrhu a písaní častí kódu (napr. LiDAR filter, navigačné skills, reťaz LLM služieb) a pri formulácii dokumentácie. Autor kód a texty skontroloval, otestoval v simulácii a zodpovedá za ne.
3. **Konzultácia so školiteľom:** *doplniť dátum a súhlas školiteľa s použitím AI.*

Použité cudzie zdroje: model Unitree G1 a jeho licencia (`doc/UNITREE_LICENSE`), politika chôdze `walk_mjlab` a ďalšie zdroje uvedené nižšie.

## Referencie

- Unitree G1 (URDF/MJCF): <https://github.com/unitreerobotics/unitree_ros>, <https://github.com/unitreerobotics/unitree_mujoco>
- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
- Gazebo Harmonic: <https://gazebosim.org/docs/harmonic/>
- MuJoCo: <https://mujoco.readthedocs.io/>
- mjlab (tréning politiky chôdze): <https://github.com/mujocolab/mjlab>
- Google Gemini API: <https://ai.google.dev/gemini-api/docs>
- Mistral API: <https://docs.mistral.ai/>
- Groq API: <https://console.groq.com/docs>
- OpenRouter API: <https://openrouter.ai/docs>
- Ahn, M. et al.: *Do As I Can, Not As I Say: Grounding Language in Robotic Affordances* (SayCan), CoRL 2022. <https://arxiv.org/abs/2204.01691>
- Yao, S. et al.: *ReAct: Synergizing Reasoning and Acting in Language Models*, ICLR 2023. <https://arxiv.org/abs/2210.03629>
- Huang, W. et al.: *Inner Monologue: Embodied Reasoning through Planning with Language Models*, CoRL 2022. <https://arxiv.org/abs/2207.05608>
- Liang, J. et al.: *Code as Policies: Language Model Programs for Embodied Control*, ICRA 2023. <https://arxiv.org/abs/2209.07753>
- Macenski, S. et al.: *Robot Operating System 2: Design, architecture, and uses in the wild*, Science Robotics 7(66), 2022. <https://arxiv.org/abs/2211.07752>
- Koenig, N., Howard, A.: *Design and use paradigms for Gazebo, an open-source multi-robot simulator*, IROS 2004. <https://ieeexplore.ieee.org/document/1389727>
- Anguelov, D. et al.: *Detecting and Modeling Doors with Mobile Robots*, ICRA 2004. <https://dblp.org/rec/conf/icra/AnguelovKPT04.html>
- Yamauchi, B.: *A frontier-based approach for autonomous exploration*, CIRA 1997. <https://ieeexplore.ieee.org/document/613851>
- Úplný zoznam s poznámkami: [`doc/LITERATURA.md`](doc/LITERATURA.md)

## Licencia

Pozri `LICENSE` a `doc/UNITREE_LICENSE`.
