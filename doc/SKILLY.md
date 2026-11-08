# Skilly robota Chappie

Skill je jednoduchá, overená akcia, ktorú robot vie vykonať. AI agent skladá úlohu z týchto skillov (v `skill_core.hpp`, uzol `motion_skills`). Mimo skillov agent používa `look` (nasníma kameru).

## Pohyb

| Skill | Parameter | Čo robí |
|---|---|---|
| `stop`, `stand` | - | zastaví sa a stojí |
| `walk_forward` | m | ide dopredu o danú vzdialenosť, zastaví sa pred prekážkou (LiDAR) |
| `walk_backward` | m | ide dozadu |
| `walk_to_obstacle` | max. m | ide dopredu, kým nie je prekážka asi 0,7 m pred ním (podľa LiDARu) |
| `step_left`, `step_right` | m | bočný krok |
| `turn_left`, `turn_right` | stupne | otočenie na mieste (podľa IMU) |
| `turn_towards` | % | otočí sa k miestu v obraze kamery (0 = vľavo, 100 = vpravo) |

## Ruky a pózy

`raise_right_arm`, `raise_left_arm`, `raise_both_arms`, `arms_down`, `get_up` (postaví sa po páde).

## Gestá

`wave`, `wave_left`, `wave_both`, `point_forward` (ukáže dopredu), `thumbs_up`, `cheer`, `salute`, `clap`, `bow`, `dance`.

## Orientácia a miesta

| Skill | Parameter | Čo robí |
|---|---|---|
| `where_am_i` | - | povie polohu z odometrie |
| `save_place` | slot 1 až 9 | uloží aktuálne miesto |
| `go_to_place` | slot 0 až 9 | vráti sa na uložené miesto (0 = štart) |
| `look_around` | - | otočí sa dookola a zmeria voľnú vzdialenosť v smeroch |

## Dvere a izby

| Skill | Parameter | Čo robí |
|---|---|---|
| `go_through_opening` | max. m (predvolene 8) | nájde dvere (medzeru v stene 0,9 až 3,5 m) z LiDARu, sám ich hľadá (otočí sa dookola a presúva sa po izbe), vycentruje sa a prejde nimi. Po prechode overí LiDARom, že okolo je stena. Výsledky: `ok`, `no_opening`, `not_through`, `blocked` |
| `search_room` | max. m (predvolene 3) | presunie sa na ďalšie neprehľadané miesto v izbe (k najvolnejšiemu priestoru). Pamätá si navštívené miesta. Keď je izba prehľadaná, skončí `explored`. Na hľadanie predmetu sa kombinuje s `look_around` a `look` |

## Iné

`push_forward` (tlačenie ľahkej škatule, experimentálne), `wait` (počká danú dobu).

## Bezpečnosť

- Robot sa pred prekážkou zastaví (LiDAR).
- Pri skilloch s odometriou (`go_to_place`, `go_through_opening`, `search_room`) musí mať robot ruky dole.
- Agent má limity: počet krokov, celková prejdená vzdialenosť, počet zlyhaní po sebe.

### Zlozene skills (SKILLS3_PATCH)

| skill | value | co robi |
|---|---|---|
| `walk_to_object` | % na pravitku obrazka (1-100) | otoci sa na predmet z obrazu a dojde k nemu podla LiDARu (zastavi 0,7 m pred nim); nahradza `turn_towards` + `walk_to_obstacle` |
| `go_around_obstacle` | min. dopredna vzdialenost v m (0,5-5) | uhne na volnejsiu stranu, prejde popri prekazke a vrati sa na povodnu priamku; chyby `no_obstacle`, `no_room`, `blocked` |
| `follow_wall` | max. dlzka v m (0,5-15) | ide popri stene po boku robota (odstup a rovnobeznost z LiDARu), kym stena neskonci, nepride roh/prekazka alebo neprejde value; chyby `no_wall`, `blocked` |

Test bez ROS a Gazeba: `g++ -std=c++17 -O1 -I src/humanoid_skills/include tools/test_skills3_sim.cpp -o /tmp/test_s3 && /tmp/test_s3`.
