# Príkazy `chappie_*`

Textové skratky sú aliasy a funkcie v `~/.bashrc` autora (nie sú v repozitári). Po zmene daj `source ~/.bashrc`.
Skutočnú prácu robia ROS 2 uzly v `src/`, skratky ich len spúšťajú. Kód funkcie vypíše `type chappie_snap` (alebo `grep -n chappie_ ~/.bashrc`).

## Build a spustenie

| Príkaz | Na čo slúži |
|---|---|
| `chappie_build` | `colcon build --symlink-install` a `source install/setup.bash` v `~/agent-based-humanoid-control` |
| `chappie_sim` | Spustí Gazebo so simuláciou, robotom, skills a agentom (`ros2 launch humanoid_gazebo walk_demo.launch.py`). Argumenty sa pripoja, napr. `chappie_sim world:=rooms_3.sdf follow:=false` |
| `chappie_world TYP [seed]` | Vygeneruje svet a spustí v ňom simuláciu. Typy: `rooms`, `maze`, `warehouse` alebo súbor ASCII mapy. Príklad: `chappie_world maze 5` |
| `chappie_teleop` | Ovládanie robota z klávesnice (WASD), balík `humanoid_teleop` |
| `chappie_mujoco` | Záložné demo chôdze v MuJoCo (`mujoco_demo/mujoco_wasd_demo.py`) |

## Ovládanie robota

| Príkaz | Na čo slúži |
|---|---|
| `chappie_task "text"` | Pošle úlohu AI agentovi: plán, priebeh, výsledok, použitá AI a spotreba. `Ctrl+C` úlohu zruší. Klient je `humanoid_agent/task_client.py`, plán robí `agent_core.py` |
| `chappie_task_raw "text"` | Pošle úlohu priamo cez `ros2 action send_goal /agent/task ... --feedback`, so surovým výpisom namiesto formátovaného (`task_client.py`) |
| `chappie_do SKILL [hodnota]` | Spustí jeden skill priamo bez AI, napr. `chappie_do walk_forward 1` (vykoná C++ uzol `motion_skills`) |
| `chappie_skills` | Vypíše zoznam skills z `/skills/catalog` |
| `chappie_task_all` | Vyskúša po jednom všetky skills (test) |

## Simulácia a diagnostika

| Príkaz | Na čo slúži |
|---|---|
| `chappie_box [x y]` | Vytvorí tlačiteľnú modrú škatuľu (4 kg, nízke trenie), napr. `chappie_box 2 0` |
| `chappie_cam POHĽAD` | Pohľad kamery v okne Gazebo (nie kamera robota): `back`, `left`, `right`, `front`, `front_right`, `front_left`. Treba bežiacu simuláciu |
| `chappie_lidar` | Vypíše vzdialenosti z LiDARu (`front`, `back`, `left`, `right`) z topicu `/lidar/distances`. Počíta ich `motion_skills` |
| `chappie_snap` | Zavolá službu `/camera/snapshot` (`std_srvs/Trigger`): uloží snímku z kamery robota |
| `chappie_cost [N]` | Prehľad tokenov a odhad nákladov AI (dnes, posledných N dní, spolu). `--reset` zmaže štatistiku |

## Súbory

- `~/.chappie_keys`: API kľúče (Gemini, Mistral, Groq) a voliteľné ceny `<SLUŽBA>_PRICE_IN/OUT`. Nikdy do gitu.
- `~/.chappie_usage.json`: počítadlá požiadaviek a tokenov pre limity služieb.
- `~/.chappie_cost.json`: história tokenov po dňoch, službách a modeloch (číta ju `chappie_cost`).

## Bez skratiek

```bash
ros2 launch humanoid_gazebo walk_demo.launch.py           # = chappie_sim
ros2 run humanoid_agent task_client "úloha"               # = chappie_task
ros2 run humanoid_teleop wasd_teleop                      # = chappie_teleop
python3 tools/gen_world.py --random rooms --seed 3 --name rooms_3
```
