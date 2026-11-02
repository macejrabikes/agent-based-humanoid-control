// skill_core.hpp - logika pohybovych skills BEZ ROS (da sa testovat samostatne).
//
// Skill = pomenovana schopnost robota s limitmi (napr. walk_forward 0.1 - 5 m).
// Kazdy skill sa skonci vysledkom {success, status, message, value_done}.
// Neplatny prikaz (zle meno, hodnota mimo limitov, robot lezi, ...) sa odmietne PRED pohybom.
//
// Chodza je v C++ uzle walk_policy_node; tento kod len posiela rychlosti (vx, vy, wz)
// a sleduje postup podla IMU (otocenie) a odometrie (vzdialenost).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace skills
{

enum class Kind {Stop, Walk, Turn, Arms, Wait, Gesture, Look, Getup, GoTo, Place, Where, Push, Door, Room};   // SKILLS2_PATCH, NAV_PATCH, OPEN_PATCH

struct SkillSpec
{
  std::string name;
  std::string description;
  Kind kind;
  double dir_x = 0, dir_y = 0;   // smer chodze v suradniciach robota (Walk)
  double sign = 0;               // smer otacania: +1 dolava, -1 doprava (Turn)
  std::string pose;              // nazov pozy (Arms)
  bool has_value = false;
  std::string unit;
  double vmin = 0, vmax = 0, vdef = 0;
  bool to_obstacle = false;      // Walk: chod, kym nie je pred robotom prekazka (lidar)
  std::string gesture;           // Gesture: nazov gesta
  double gesture_s = 0;          // (nepouzite)
  double gesture_ramp = 1.4;     // Gesture: nabeh a navrat ruky [s] (celkova dlzka = hodnota + 2 * nabeh)
  int arm = 1;                   // Gesture: ktora ruka: 1 = prava, 2 = lava, 3 = obe
  bool needs_hands = false;      // Gesture: funguje len ak ma robot ruky s prstami (with_hands:=true)
};

struct Params
{
  double walk_speed = 0.5, back_speed = 0.3, side_speed = 0.3, turn_rate = 0.6;   // [m/s], [rad/s]
  double accel = 0.8, decel = 0.8;           // [m/s^2] (pre otacanie rad/s^2)
  double lag = 0.3;                          // o kolko skor davam stop [s] (dobeh politiky)
  double heading_kp = 1.5, heading_max = 0.4;
  double upright_gz = -0.8;                  // na start musi byt gz < tohto (sklon < ~37 st.)
  double fall_gz = -0.6;                     // pocas pohybu gz > tohto = robot spadol
  double settle_s = 0.6, arm_s = 3.0;
  double stop_dist = 0.70;     // walk_to_obstacle: zastav, ked je prekazka vpredu blizsie ako toto [m] (od lidaru)
  double block_dist = 0.55;    // walk_*: prekazka v smere pohybu blizsie ako toto -> skill konci 'blocked' [m]
  bool lidar_guard = true;     // zastavit chodzu pred prekazkou (LIDAR_PATCH)
  double camera_hfov_deg = 80.0;   // TT_PATCH: vodorovny pohlad kamery [st.] (turn_towards)
  double push_speed = 0.30;    // NAV_PATCH: push_forward - rychlost chodze pri tlaceni [m/s]
  double dance_step = 0.10;    // GEST_PATCH: dance - amplituda bocnych krokov [m/s] (0 = nohy stoja)
  double goto_tol = 0.15;      // NAV_PATCH: go_to_place - cielom je kruh s tymto polomerom [m]
  double push_gap = 0.42;      // PUSH_PATCH: push_forward - vzdialenost predmetu od lidaru, kde sa natiahnu ruky [m] (ruky dosiahnu ~0.34 m)
  double door_min_w = 0.9, door_max_w = 3.5;   // OPEN_PATCH: go_through_opening - sirka priechodneho otvoru [m] (robot je ~0.5 m siroky)
  double door_pre = 1.4, door_exit = 1.2;       // OPEN_PATCH: bod pred dverami a vzdialenost za rovinou steny, kde skill konci [m]
  double door_speed = 0.35;                     // OPEN_PATCH: rychlost pri prechode [m/s]
  double stuck_s = 4.0, stuck_min = 0.15;   // STUCK_PATCH: ak za stuck_s prejde < stuck_min [m], robot stoji o prekazku
};

// OPEN_PATCH: najblizsia prekazka v kazdom 2-stupnovom vysecku okolo robota (index 0 = -180 st. = za robotom,
// index 90 = priamo dopredu, uhol rastie dolava). Z toho sa hladaju otvory v stene (find_openings).
constexpr int kBins = 180;
constexpr double kBinDeg = 2.0;
struct LidarBins
{
  std::array<double, kBins> r, x, y;     // vodorovna vzdialenost [m] (99 = nic) a poloha najblizsieho bodu v suradniciach robota
  LidarBins() {r.fill(99.0); x.fill(0.0); y.fill(0.0);}
};

struct Sensors
{
  bool have_imu = false;
  bool have_hands = false;   // robot ma prsty (joint_states obsahuje right_hand_thumb_0_joint)
  double sim_t = -1.0;       // cas simulacie [s] z /joint_states (zaporne = nie je k dispozicii)
  double yaw = 0.0;       // [rad] z IMU
  double gz = -1.0;       // z-zlozka gravitacie v suradniciach panvy (stoji: -1)
  bool have_odom = false;
  double ox = 0.0, oy = 0.0;
  double oyaw = 0.0;         // NAV_PATCH: natocenie robota v suradniciach odometrie [rad]
  bool have_lidar = false;   // cerstve data z lidaru
  double lid_front = 99, lid_back = 99, lid_left = 99, lid_right = 99;   // najblizsia prekazka v sektore [m], 99 = nic
  LidarBins bins;            // OPEN_PATCH: vzdialenosti po 2 stupnoch (pre hladanie otvorov)
};

// NAV_PATCH: zapamatane miesto (suradnice odometrie)
struct Place {double x = 0, y = 0, yaw = 0;};

struct Result
{
  bool success = false;
  std::string status;      // ok | invalid_skill | invalid_argument | out_of_range | not_upright | busy |
                           // no_imu | timeout | robot_fell | cancelled
  std::string message;
  double value_done = 0.0;
};

struct Output
{
  double vx = 0, vy = 0, wz = 0;
  bool publish_cmd = false;     // posielat rychlosti na /cmd_vel
  bool send_pose = false;       // jednorazovo poslat pozu ruk
  std::string pose;
  std::string gesture;          // jednorazovo spustit gesto (meno) - prazdne = nic
  int gesture_arm = 1;          // ktora ruka: 1 = prava, 2 = lava, 3 = obe
  double gesture_total = 0;     // celkova dlzka gesta [s]
  bool send_getup = false;      // jednorazovo spustit vstavanie zo zeme
  bool finished = false;
  Result result;
  double progress = 0.0;
  double value_done = 0.0;
  std::string state;
};

inline double wrap_pi(double a)
{
  while (a > M_PI) {a -= 2 * M_PI;}
  while (a < -M_PI) {a += 2 * M_PI;}
  return a;
}

inline double clampd(double v, double lo, double hi) {return std::max(lo, std::min(hi, v));}

// ---- LiDAR: najblizsie prekazky v sektoroch okolo robota -------------------------------------------
// Body su v suradniciach senzora: x dopredu, y dolava, z hore. Pocita sa VODOROVNA vzdialenost.
struct LidarParams
{
  double floor_z = -0.95;               // body nizsie ako toto (vodorovna vyska od senzora) = podlaha
  double self_x = 0.45, self_y = 0.35;  // obdlznik okolo senzora = telo a ruky robota, ignoruje sa (SYM_PATCH: 0.22 -> 0.45, ramena boli vo vzdialenosti 0.35)
  double front_deg = 30, side_lo = 60, side_hi = 120, back_deg = 150;
  double max_range = 10.0;
  double mount_pitch = 0.04;            // sklon senzora voci panve (rpy v URDF) [rad]
  bool flip = false;                    // SYM_PATCH: senzor je otoceny (roll 180 st.): y a z sa pred vypoctom obratia
  double up_x = 0, up_y = 0, up_z = 1;  // "hore" v suradniciach panvy (z IMU); vyrovna naklon robota pri chodzi
};

struct LidarSectors
{
  double front = 99, back = 99, left = 99, right = 99;
  int seen = 0;       // kolko platnych bodov prislo (kontrola, ze lidar naozaj meria)
  LidarBins bins;     // OPEN_PATCH
  void add(double x, double y, double z, const LidarParams & p)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {return;}
    if (p.flip) {y = -y; z = -z;}                  // SYM_PATCH
    ++seen;
    // senzor -> suradnice panvy (sklon pripevnenia) a vodorovna vyska podla IMU (naklon robota)
    const double cp = std::cos(p.mount_pitch), sp = std::sin(p.mount_pitch);
    const double px = x * cp + z * sp, pz = -x * sp + z * cp;
    const double level_z = p.up_x * px + p.up_y * y + p.up_z * pz;
    if (level_z < p.floor_z) {return;}
    if (std::fabs(px) < p.self_x && std::fabs(y) < p.self_y) {return;}
    const double h = std::hypot(px, y);
    if (h > p.max_range) {return;}
    const double deg = std::atan2(y, px) * 180.0 / M_PI;
    const double a = std::fabs(deg);
    int k = static_cast<int>(std::floor((deg + 180.0) / kBinDeg));      // OPEN_PATCH
    k = std::max(0, std::min(kBins - 1, k));
    if (h < bins.r[k]) {bins.r[k] = h; bins.x[k] = px; bins.y[k] = y;}
    if (a <= p.front_deg) {front = std::min(front, h);}
    if (a >= p.back_deg) {back = std::min(back, h);}
    if (deg >= p.side_lo && deg <= p.side_hi) {left = std::min(left, h);}
    if (deg <= -p.side_lo && deg >= -p.side_hi) {right = std::min(right, h);}
  }
};

// PointCloud2 (little endian, FLOAT32 x/y/z na offsetoch ox/oy/oz) -> sektory
inline LidarSectors lidar_from_cloud(const unsigned char * data, size_t n, size_t point_step,
  size_t ox, size_t oy, size_t oz, const LidarParams & p)
{
  LidarSectors r;
  for (size_t i = 0; i < n; ++i) {
    const unsigned char * q = data + i * point_step;
    float x, y, z;
    std::memcpy(&x, q + ox, 4);
    std::memcpy(&y, q + oy, 4);
    std::memcpy(&z, q + oz, 4);
    r.add(x, y, z, p);
  }
  return r;
}

// OPEN_PATCH: otvor v stene (dvere, prechod) najdeny v LiDAR sken-e. Suradnice v systeme robota (x dopredu, y dolava).
struct Opening
{
  double cx = 0, cy = 0;        // stred otvoru [m]
  double nx = 1, ny = 0;        // jednotkova normala steny, smeruje cez otvor (od robota prec)
  double width = 0;             // sirka medzi hranami otvoru [m]
  double dist = 0;              // vzdialenost stredu od robota [m]
  double bearing = 0;           // smer na stred [st.], + = vlavo
};

// Otvor = usek lucov, ktorych vzdialenost je o dost vacsia nez vzdialenost oboch okrajov (blizke hrany steny).
// Okraje sa hladaju ako skok vzdialenosti medzi susednymi 2-stupnovymi vysecmi: skok hore zdola doprava = pravy okraj,
// skok dole = lavy okraj. Dvojica okrajov je otvor, ak je medzi nimi (aspon 3 vysece) skoro vsetko "daleko"; ak je viac moznosti,
// vyhra uzsi otvor (kazdy okraj patri najviac jednemu otvoru).
// Hlada sa v rozsahu +-fov_deg od smeru dopredu. Vrati VSETKY otvory s width >= min_w (volajuci si vyberie podla sirky a smeru).
inline std::vector<Opening> find_openings(const LidarBins & b, double min_w, double max_w, double max_dist, double fov_deg = 110.0)
{
  std::vector<Opening> out;
  const int k0 = std::max(0, static_cast<int>(std::floor((180.0 - fov_deg) / kBinDeg)));
  const int k1 = std::min(kBins - 1, static_cast<int>(std::floor((180.0 + fov_deg) / kBinDeg)));
  const double jump = 0.6;       // [m] skok vzdialenosti = okraj
  std::array<double, kBins> r = b.r;
  for (auto & v : r) {if (v > 8.0) {v = 99.0;}}              // za 8 m su merania riedke a sumive -> berie sa ako "nic"
  std::vector<int> rise, fall;
  for (int i = k0; i < k1; ++i) {
    if (r[i] < 90.0 && r[i + 1] - r[i] > jump) {rise.push_back(i);}      // blizka stena vpravo, dalej sa to otvara
    if (r[i + 1] < 90.0 && r[i] - r[i + 1] > jump) {fall.push_back(i);}  // za otvorom zase blizka stena vlavo (blizka je vysec i + 1)
  }
  struct Cand {double width; int s, j; Opening op;};
  std::vector<Cand> cands;
  for (int j : fall) {
    for (int s : rise) {
      if (s > j - 3) {continue;}
      const double ra = r[s], rc = r[j + 1];
      if (std::fabs(rc - ra) > max_w) {continue;}
      const double x1 = b.x[s], y1 = b.y[s], x2 = b.x[j + 1], y2 = b.y[j + 1];
      const double raw_w = std::hypot(x2 - x1, y2 - y1);
      // hrany su krajne body steny v susednych vysecoch: skutocna hrana je niekde medzi nimi, takze raw_w je o ~1 vysec (kBinDeg) vacsie
      const double width = raw_w - kBinDeg * M_PI / 180.0 * 0.5 * (ra + rc);
      if (raw_w < 1e-3 || width < min_w || width > max_w) {continue;}
      int far_n = 0;
      for (int k = s + 1; k <= j; ++k) {if (r[k] >= std::max(ra, rc) + 1.0) {++far_n;}}
      if (far_n < 0.8 * (j - s)) {continue;}
      Opening op;
      op.cx = 0.5 * (x1 + x2);
      op.cy = 0.5 * (y1 + y2);
      const double tx = (x2 - x1) / raw_w, ty = (y2 - y1) / raw_w;
      op.nx = ty;
      op.ny = -tx;                                                // normala; ma smerovat od robota k otvoru
      op.dist = std::hypot(op.cx, op.cy);
      if (op.nx * op.cx + op.ny * op.cy < 0.0) {op.nx = -op.nx; op.ny = -op.ny;}
      if (op.nx * op.cx + op.ny * op.cy < 0.35 * op.dist) {continue;}      // pozeram sa na stenu zo skraja (pozdlz steny), nie je to otvor
      op.width = width;
      op.bearing = std::atan2(op.cy, op.cx) * 180.0 / M_PI;
      if (op.dist > max_dist) {continue;}
      // OPEN_PATCH: po oboch stranach musi byt dlha rovna stena/zarubna v jednej rovine (nie skrinka, valec alebo stlp stojaci v izbe)
      {
        double fnx = op.nx, fny = op.ny;
        {   // smer steny z bodov okolo otvoru (nie z okrajov, tie byvaju na bocnej ploche steny)
          double m[2][2] = {{0, 0}, {0, 0}};
          int cnt[2] = {0, 0};
          for (int side = 0; side < 2; ++side) {
            std::vector<int> ks;
            for (int dk = 3; dk <= 9; ++dk) {
              const int k = side == 0 ? j + 1 + dk : s - dk;
              if (k >= 0 && k < kBins && r[k] < 90.0) {ks.push_back(k);}
            }
            if (ks.size() < 3) {continue;}
            std::vector<double> d;
            for (int k : ks) {d.push_back(fnx * b.x[k] + fny * b.y[k]);}
            std::sort(d.begin(), d.end());
            const double med = d[static_cast<size_t>(0.7 * (d.size() - 1))];      // predmety pred stenou su blizsie, stena je v horne casti
            for (int k : ks) {
              if (std::fabs(fnx * b.x[k] + fny * b.y[k] - med) < 0.3) {m[side][0] += b.x[k]; m[side][1] += b.y[k]; ++cnt[side];}
            }
          }
          if (cnt[0] >= 3 && cnt[1] >= 3) {
            double ux = m[0][0] / cnt[0] - m[1][0] / cnt[1], uy = m[0][1] / cnt[0] - m[1][1] / cnt[1];
            const double un = std::hypot(ux, uy);
            if (un > 0.3) {
              ux /= un; uy /= un;
              fnx = uy; fny = -ux;
              if (fnx * op.nx + fny * op.ny < 0.0) {fnx = -fnx; fny = -fny;}
            }
          }
        }
        auto depth = [&](int k) {return fnx * b.x[k] + fny * b.y[k];};
        // rovina steny na jednej strane = median hlbky bodov o niekolko vysecov dalej od okraja (hrana otvoru byva na bocnej ploche steny)
        auto plane = [&](int from, int step, bool & ok) {
          std::vector<double> d;
          for (int dk = 3; dk <= 9; ++dk) {
            const int k = from + step * dk;
            if (k >= 0 && k < kBins && r[k] < 90.0) {d.push_back(depth(k));}
          }
          ok = d.size() >= 3;
          if (!ok) {return 0.0;}
          std::sort(d.begin(), d.end());
          return d[static_cast<size_t>(0.7 * (d.size() - 1))];
        };
        auto flank = [&](int from, int step, double d0, double px, double py, double dx, double dy) {
          double ext = 0.0;
          int bad = 0, hidden = 0;
          bool started = false;
          for (int k = from; k >= 0 && k < kBins; k += step) {
            if (r[k] >= 90.0) {if (++bad > (started ? 2 : 14)) {break;} continue;}
            const double dd = depth(k) - d0;
            if (std::fabs(dd) < 0.25) {
              started = true; bad = 0;
              ext = std::max(ext, (b.x[k] - px) * dx + (b.y[k] - py) * dy);
            } else if (dd < 0.0 && ++hidden < 40) {      // predmet v izbe pred stenou zakryva vyhlad - preskoc
              continue;
            } else if (++bad > (started ? 2 : 14)) {break;}
          }
          return ext + 0.5 * kBinDeg * M_PI / 180.0 * 0.5 * (ra + rc);
        };
        bool okl = false, okr = false;
        const double dl = plane(j + 1, 1, okl), dr = plane(s, -1, okr);
        if (!okl && !okr) {continue;}
        if (okl && okr && std::fabs(dl - dr) > 0.3) {continue;}
        const double d0 = (okl && okr) ? 0.5 * (dl + dr) : (okl ? dl : dr);
        const double e1 = depth(s), e2 = depth(j + 1);          // okraje otvoru lezia v rovine steny (najviac o hrubku steny hlbsie)
        if (std::min(e1, e2) < d0 - 0.15 || std::max(e1, e2) > d0 + 0.7) {continue;}
        const double el = okl ? flank(j + 1, 1, d0, x2, y2, tx, ty) : -1.0;
        const double er = okr ? flank(s, -1, d0, x1, y1, -tx, -ty) : -1.0;
        if (std::max(el, er) < 1.0 || (okl && okr && std::min(el, er) < 0.3)) {continue;}
      }
      cands.push_back({width, s, j, op});
    }
  }
  std::sort(cands.begin(), cands.end(), [](const Cand & p, const Cand & q) {return p.width < q.width;});
  std::vector<bool> used_s(kBins, false), used_j(kBins, false);
  for (const auto & c : cands) {                        // najuzsie otvory maju prednost; okraj patri najviac jednemu otvoru
    if (used_s[c.s] || used_j[c.j]) {continue;}
    used_s[c.s] = used_j[c.j] = true;
    out.push_back(c.op);
  }
  return out;
}

inline std::string fmt_dist(double d)
{
  std::ostringstream o;
  if (d >= 9.99) {o << ">10";} else {o.precision(2); o << std::fixed << d;}
  return o.str();
}

class SkillExecutor
{
public:
  explicit SkillExecutor(const Params & p = Params())
  : p_(p)
  {
    using K = Kind;
    auto add = [this](SkillSpec s) {specs_.push_back(s);};
    SkillSpec s;
    s = {}; s.name = "stop"; s.kind = K::Stop; s.description = "Zastavi pohyb a robot ostane stat."; add(s);
    s = {}; s.name = "stand"; s.kind = K::Stop; s.description = "Robot stoji na mieste (to iste ako stop)."; add(s);
    s = {}; s.name = "walk_forward"; s.kind = K::Walk; s.dir_x = 1; s.has_value = true; s.unit = "m";
    s.vmin = 0.1; s.vmax = 5.0; s.vdef = 1.0; s.description = "Prejde dopredu o danu vzdialenost."; add(s);
    s = {}; s.name = "walk_to_obstacle"; s.kind = K::Walk; s.dir_x = 1; s.to_obstacle = true; s.has_value = true;
    s.unit = "m"; s.vmin = 0.5; s.vmax = 8.0; s.vdef = 5.0;
    s.description = "Walks forward until a wall or obstacle is about 0.7 m in front of the robot (measured by LiDAR) and stops there. "
      "'value' is the MAXIMUM distance it may walk; if no obstacle is found within it, the skill fails with status no_obstacle.";
    add(s);
    s = {}; s.name = "walk_backward"; s.kind = K::Walk; s.dir_x = -1; s.has_value = true; s.unit = "m";
    s.vmin = 0.1; s.vmax = 2.0; s.vdef = 0.5; s.description = "Prejde dozadu o danu vzdialenost."; add(s);
    s = {}; s.name = "step_left"; s.kind = K::Walk; s.dir_y = 1; s.has_value = true; s.unit = "m";
    s.vmin = 0.1; s.vmax = 2.0; s.vdef = 0.5; s.description = "Posunie sa bokom dolava."; add(s);
    s = {}; s.name = "step_right"; s.kind = K::Walk; s.dir_y = -1; s.has_value = true; s.unit = "m";
    s.vmin = 0.1; s.vmax = 2.0; s.vdef = 0.5; s.description = "Posunie sa bokom doprava."; add(s);
    s = {}; s.name = "turn_left"; s.kind = K::Turn; s.sign = 1; s.has_value = true; s.unit = "deg";
    s.vmin = 5; s.vmax = 180; s.vdef = 90; s.description = "Otoci sa dolava o dany uhol."; add(s);
    s = {}; s.name = "turn_right"; s.kind = K::Turn; s.sign = -1; s.has_value = true; s.unit = "deg";
    s.vmin = 5; s.vmax = 180; s.vdef = 90; s.description = "Otoci sa doprava o dany uhol."; add(s);
    s = {}; s.name = "turn_towards"; s.kind = K::Turn; s.has_value = true; s.unit = "%"; s.vmin = 1; s.vmax = 100; s.vdef = 50;
    s.description = "Turns on the spot so that something you SEE in the camera picture ends up straight ahead. 'value' = the horizontal "
      "position of the object in the picture in PERCENT, read on the ruler at the bottom of the picture (1 = left edge, 50 = centre, 100 = right edge). "
      "The camera view is 80 degrees wide, so 90 turns about 34 degrees right, 75 about 23 degrees right and 10 about 34 degrees left. "
      "Use it before walking to an object you see instead of guessing turn angles; afterwards check the next picture.";
    add(s);
    s = {}; s.name = "raise_right_arm"; s.kind = K::Arms; s.pose = "raise_right_arm";
    s.description = "Zdvihne pravu ruku."; add(s);
    s = {}; s.name = "raise_left_arm"; s.kind = K::Arms; s.pose = "raise_left_arm";
    s.description = "Zdvihne lavu ruku."; add(s);
    s = {}; s.name = "raise_both_arms"; s.kind = K::Arms; s.pose = "raise_both_arms";
    s.description = "Zdvihne obe ruky."; add(s);
    s = {}; s.name = "arms_down"; s.kind = K::Arms; s.pose = "arms_down";
    s.description = "Spusti ruky dole."; add(s);
    s = {}; s.name = "get_up"; s.kind = K::Getup;
    s.description = "Use when the robot has FALLEN (lies on the floor): it stands up by itself (takes ~10 s). "
      "All other movement skills are rejected while the robot lies. If it already stands, nothing happens.";
    add(s);
    s = {}; s.name = "wave"; s.kind = K::Gesture; s.gesture = "wave"; s.arm = 1; s.gesture_ramp = 1.4;
    s.has_value = true; s.unit = "s"; s.vmin = 2; s.vmax = 30; s.vdef = 4;
    s.description = "Waves hello with the RIGHT hand, 'value' = how many seconds to keep waving (default 4, about one wave per second), "
      "then the arm returns to its original position. Robot must stand with arms down."; add(s);
    s = {}; s.name = "wave_left"; s.kind = K::Gesture; s.gesture = "wave"; s.arm = 2; s.gesture_ramp = 1.4;
    s.has_value = true; s.unit = "s"; s.vmin = 2; s.vmax = 30; s.vdef = 4;
    s.description = "Same as wave but with the LEFT hand ('value' = seconds of waving, default 4)."; add(s);
    s = {}; s.name = "wave_both"; s.kind = K::Gesture; s.gesture = "wave"; s.arm = 3; s.gesture_ramp = 1.4;
    s.has_value = true; s.unit = "s"; s.vmin = 2; s.vmax = 30; s.vdef = 4;
    s.description = "Same as wave but with BOTH hands ('value' = seconds of waving, default 4)."; add(s);
    s = {}; s.name = "point_forward"; s.kind = K::Gesture; s.gesture = "point_forward"; s.gesture_ramp = 2.0;
    s.has_value = true; s.unit = "s"; s.vmin = 1; s.vmax = 30; s.vdef = 3;
    s.description = "Points straight forward with the right arm (index finger out if the robot has hands). 'value' = how many seconds to "
      "keep pointing (default 3), then the arm returns to its original position. Robot must stand with arms down."; add(s);
    s = {}; s.name = "thumbs_up"; s.kind = K::Gesture; s.gesture = "thumbs_up"; s.gesture_ramp = 2.0; s.needs_hands = true;
    s.has_value = true; s.unit = "s"; s.vmin = 1; s.vmax = 30; s.vdef = 3;
    s.description = "Shows a thumbs up (fist, thumb up) with the right hand in front of the body (approval, 'OK'). 'value' = seconds to "
      "hold it (default 3). Works ONLY if the robot has hands with fingers, otherwise it is rejected (no_hands). "
      "Robot must stand with arms down."; add(s);
    s = {}; s.name = "cheer"; s.kind = K::Gesture; s.gesture = "cheer"; s.gesture_ramp = 1.4;
    s.has_value = true; s.unit = "s"; s.vmin = 2; s.vmax = 30; s.vdef = 4;
    s.description = "Cheering (joy, victory): raises BOTH arms high above the head and pumps them up and down. 'value' = seconds of "
      "cheering (default 4), then the arms return to their original position. Not a greeting - to greet someone use wave."; add(s);
    s = {}; s.name = "salute"; s.kind = K::Gesture; s.gesture = "salute"; s.gesture_ramp = 1.8;
    s.has_value = true; s.unit = "s"; s.vmin = 1; s.vmax = 30; s.vdef = 3;
    s.description = "Military salute with the right hand at the forehead. 'value' = seconds to hold the salute (default 3), "
      "then the arm returns to its original position. Robot must stand with arms down."; add(s);
    s = {}; s.name = "clap"; s.kind = K::Gesture; s.gesture = "clap"; s.gesture_ramp = 1.6;
    s.has_value = true; s.unit = "s"; s.vmin = 2; s.vmax = 30; s.vdef = 4;
    s.description = "Claps hands in front of the chest (applause, ~1.5 claps per second). 'value' = seconds of clapping (default 4), "
      "then the arms return to their original position. Robot must stand with arms down."; add(s);
    s = {}; s.name = "bow"; s.kind = K::Gesture; s.gesture = "bow"; s.gesture_ramp = 2.5;
    s.has_value = true; s.unit = "s"; s.vmin = 1; s.vmax = 10; s.vdef = 2;
    s.description = "Bows: leans the upper body forward slowly (about 20 degrees), holds it for 'value' seconds (default 2) and comes back up. "
      "Use for a polite greeting or thanks. Robot must stand still with arms down."; add(s);
    s = {}; s.name = "dance"; s.kind = K::Gesture; s.gesture = "dance"; s.gesture_ramp = 1.6;
    s.has_value = true; s.unit = "s"; s.vmin = 3; s.vmax = 60; s.vdef = 8;
    s.description = "Dances on the spot: arms pump alternately while the upper body sways left and right. 'value' = seconds of dancing "
      "(default 8). The feet do not move. Robot must stand with arms down."; add(s);
    s = {}; s.name = "push_forward"; s.kind = K::Push; s.has_value = true; s.unit = "m"; s.vmin = 0.2; s.vmax = 3.0; s.vdef = 1.0;
    s.description = "Pushes an object in front of the robot: both arms are stretched forward at chest height and the robot walks slowly "
      "forward by 'value' metres (default 1), pushing a light box or similar object ahead of it. First walk_to_obstacle (it stops 0.7 m "
      "before the object), then push_forward: it first steps closer to the object (about 0.4 m) with the arms down, then stretches the arms. The arms return to their original position afterwards. "
      "If the robot cannot move (object too heavy or blocked) it stops with status 'blocked'. Takes 10-20 s. Robot must stand with arms down.";
    add(s);
    s = {}; s.name = "save_place"; s.kind = K::Place; s.has_value = true; s.unit = "slot"; s.vmin = 1; s.vmax = 9; s.vdef = 1;
    s.description = "Remembers the robot's CURRENT position and heading in memory slot 'value' (1-9; slot 0 is always the START position). "
      "Use it to mark a place (e.g. next to an object) so that go_to_place can come back later. You must remember yourself which slot means what.";
    add(s);
    s = {}; s.name = "go_to_place"; s.kind = K::GoTo; s.has_value = true; s.unit = "slot"; s.vmin = 0; s.vmax = 9; s.vdef = 0;
    s.description = "Walks in a straight line to a remembered place: slot 0 = the START position (value 0 = go back to the start), slots 1-9 = places "
      "stored with save_place. It turns towards the place, walks there and finally turns to the heading the robot had when the place was saved. "
      "Stops with status 'blocked' if an obstacle is in the way (LiDAR): then go around it with turns and walk_forward and call it again. "
      "Fails with unknown_place if the slot is empty.";
    add(s);
    s = {}; s.name = "where_am_i"; s.kind = K::Where;
    s.description = "Reports the robot's exact position (x forward, y left, in metres, relative to the START position and start heading) and its heading, "
      "plus distance and direction to every saved place. Use it to find out where you are or how far a saved place is. It does not move the robot.";
    add(s);
    s = {}; s.name = "look_around"; s.kind = K::Look;
    s.description = "Turns a full circle in 45 degree steps (takes ~30 s) and measures the free distance ahead in 8 directions "
      "with the LiDAR. The result lists the direction (0 = start direction, + left, - right) and distance of each, and the most open "
      "direction. The robot ends facing the start direction. Use it to find the way out or to look for walls and openings.";
    add(s);
    s = {}; s.name = "go_through_opening"; s.kind = K::Door; s.has_value = true; s.unit = "m"; s.vmin = 1.0; s.vmax = 12.0; s.vdef = 8.0;
    s.description = "Finds an OPENING in a wall (a doorway or any gap 0.9 to 3.5 m wide) with the LiDAR and walks through it to about 1.2 m behind the wall. "
      "It SEARCHES BY ITSELF: it looks 110 degrees to each side, turns on the spot up to 360 degrees and, if there is no door in sight, walks to another "
      "part of the room (towards the most open space, up to 4 times) and looks again, watching for a door all the time. 'value' = the maximum distance to "
      "the opening in metres from where it looks (default 8). If several doors are visible it takes the one closest to the direction the robot faces and "
      "not the one it has just come through, so turn_towards a specific door first if the task names one. It turns to the door, steps in front of it, "
      "aligns with the door frame, centres itself and walks straight through. Fails with status no_opening if no passable gap is found anywhere "
      "(the whole search is done, do not repeat it at once) and with status blocked if the robot gets stuck. Takes 15-90 s. Afterwards you are in the next room: "
      "look at the picture. Robot must stand with arms down.";
    add(s);
    s = {}; s.name = "search_room"; s.kind = K::Room; s.has_value = true; s.unit = "m"; s.vmin = 1.0; s.vmax = 5.0; s.vdef = 3.0;
    s.description = "Moves the robot to the NEXT UNEXPLORED SPOT of the room (towards the most open space, i.e. roughly through the middle, never to a spot it "
      "has already visited) so the camera sees a new part of the room. 'value' = the longest hop in metres (default 3). Use it to look for something in a room "
      "('find the red box'): look_around, and if the object is not in the pictures call search_room and look_around again, until the object is found or "
      "search_room fails with status explored (the whole room has been visited). Stops in front of obstacles. Takes 5-20 s. Robot must stand with arms down.";
    add(s);
    s = {}; s.name = "wait"; s.kind = K::Wait; s.has_value = true; s.unit = "s";
    s.vmin = 0.1; s.vmax = 30; s.vdef = 1.0; s.description = "Pocka dany pocet sekund."; add(s);
    for (auto & k : specs_) {
      if (k.kind == K::Walk && !k.to_obstacle) {
        k.description += " Stops early with status 'blocked' if an obstacle is closer than 0.55 m in the walking direction (LiDAR).";
      }
    }
  }

  const std::vector<SkillSpec> & specs() const {return specs_;}
  bool active() const {return active_;}
  bool arms_up() const {return arms_up_;}

  // NAV_PATCH: vychodzia poloha (slot 0) sa ulozi pri prvej odometrii
  void observe(const Sensors & s)
  {
    if (!start_saved_ && s.have_odom) {
      places_[0] = Place{s.ox, s.oy, s.oyaw};
      start_saved_ = true;
    }
  }

  // zoznam skills ako JSON (pre AI agenta)
  std::string catalog_json() const
  {
    std::ostringstream o;
    o << "[";
    for (size_t i = 0; i < specs_.size(); ++i) {
      const auto & s = specs_[i];
      o << (i ? "," : "") << "{\"name\":\"" << s.name << "\",\"description\":\"" << s.description << "\"";
      if (s.has_value) {
        o << ",\"parameter\":{\"name\":\"value\",\"unit\":\"" << s.unit << "\",\"min\":" << s.vmin
          << ",\"max\":" << s.vmax << ",\"default\":" << s.vdef << "}";
      }
      o << "}";
    }
    o << "]";
    return o.str();
  }

  // Overi prikaz a spusti ho. Pri odmietnuti vrati false a vyplni res.
  bool start(const std::string & name, double value, const Sensors & s, Result & res)
  {
    res = Result();
    if (active_) {
      return fail(res, "busy", "Iny skill este bezi (" + cur_.name + "). Pockaj alebo ho zrus.");
    }
    const SkillSpec * spec = nullptr;
    for (const auto & k : specs_) {if (k.name == name) {spec = &k;}}
    if (!spec) {
      std::string all;
      for (const auto & k : specs_) {all += (all.empty() ? "" : ", ") + k.name;}
      return fail(res, "invalid_skill", "Neznamy skill '" + name + "'. Dostupne: " + all);
    }
    double v = 0.0;
    if (spec->has_value) {
      if (!std::isfinite(value)) {return fail(res, "invalid_argument", "Hodnota nie je cislo.");}
      v = (value == 0.0) ? spec->vdef : value;
      if (v < spec->vmin || v > spec->vmax) {
        std::ostringstream o;
        o << name << ": hodnota " << v << " " << spec->unit << " je mimo limitov <" << spec->vmin
          << ", " << spec->vmax << "> " << spec->unit;
        return fail(res, "out_of_range", o.str());
      }
    }
    if (spec->kind != Kind::Stop) {
      if (!s.have_imu) {return fail(res, "no_imu", "Nemam data z IMU, neviem posudit stav robota.");}
      if (s.gz > p_.upright_gz && spec->kind != Kind::Getup) {
        return fail(res, "not_upright",
          "Robot nestoji rovno (lezi alebo je velmi naklonený), pohyb odmietnuty. Robot has fallen: use the skill get_up first.");
      }
    }
    if (spec->kind == Kind::Gesture && arms_up_) {
      return fail(res, "arms_up", "Ruky su hore - najprv arms_down, potom gesto.");
    }
    if (spec->needs_hands && !s.have_hands) {
      return fail(res, "no_hands", "Gesto '" + name + "' potrebuje robota s prstami (simulacia musi bezat s with_hands:=true). "
        "The robot has no hands with fingers: this gesture is not possible, use another skill (e.g. wave).");
    }
    if (spec->kind == Kind::Place || spec->kind == Kind::Where || spec->kind == Kind::GoTo || spec->kind == Kind::Push ||
      spec->kind == Kind::Door || spec->kind == Kind::Room) {
      if (!s.have_odom) {
        return fail(res, "no_odom", "Nemam data z odometrie (/chappie/odom), neviem urcit polohu.");
      }
    }
    if ((spec->kind == Kind::GoTo || spec->kind == Kind::Push || spec->kind == Kind::Door || spec->kind == Kind::Room) && arms_up_) {
      return fail(res, "arms_up", "Ruky su hore - najprv arms_down.");
    }
    if (spec->kind == Kind::Place || spec->kind == Kind::GoTo) {
      if (std::fabs(v - std::round(v)) > 1e-6) {
        return fail(res, "invalid_argument", name + ": slot musi byt cele cislo.");
      }
      if (spec->kind == Kind::GoTo && places_.find(static_cast<int>(std::lround(v))) == places_.end()) {
        std::ostringstream m;
        m << "go_to_place: slot " << std::lround(v) << " is empty. Saved slots:";
        for (const auto & kv : places_) {m << " " << kv.first;}
        m << ". Use save_place first (slot 0 = start).";
        return fail(res, "unknown_place", m.str());
      }
    }
    if (spec->to_obstacle || spec->kind == Kind::Door || spec->kind == Kind::Room) {
      if (!s.have_lidar) {
        return fail(res, "no_lidar", "Nemam cerstve data z lidaru (/lidar/points), neviem hladat prekazku.");
      }
      if (arms_up_) {
        return fail(res, "arms_up", "Ruky su hore a lidar by videl ich - najprv arms_down.");
      }
    }
    cur_ = *spec;
    value_ = v;
    if (spec->name == "turn_towards") {                 // TT_PATCH: percento obrazka -> uhol (model dierkovej kamery)
      const double hf = p_.camera_hfov_deg * M_PI / 180.0;
      const double deg = std::atan((v - 50.0) / 50.0 * std::tan(hf / 2.0)) * 180.0 / M_PI;     // + = predmet je vpravo
      cur_.sign = (deg > 0) ? -1.0 : 1.0;               // vpravo = zaporne otacanie
      value_ = std::max(2.0, std::fabs(deg));
    }
    active_ = true;
    t_ = 0.0;
    started_ = false;
    phase_ = Phase::Move;
    v_cur_ = 0.0;
    settle_t_ = 0.0;
    d_est_ = 0.0;
    stuck_t_ = 0.0;
    stuck_d0_ = 0.0;
    turned_ = 0.0;
    corrections_ = 0;
    yaw0_ = yaw_prev_ = s.yaw;
    x0_ = s.ox;
    y0_ = s.oy;
    odom_used_ = s.have_odom;
    pose_sent_ = false;
    getup_sent_ = false;
    gest_total_ = (spec->kind == Kind::Gesture) ? v + 2.0 * spec->gesture_ramp : 0.0;
    upright_t_ = 0.0;
    look_k_ = 0;
    look_target_ = 0.0;
    look_front_.clear();
    goto_ph_ = 0;                       // NAV_PATCH
    w_cur_ = 0.0;
    goto_prev_d_ = 0.0;
    push_stage_ = 0;
    push_t_ = 0.0;
    push_ok_ = true;
    push_status_ = "ok";
    push_msg_.clear();
    push_moved_ = 0.0;
    push_x0_ = push_y0_ = 0.0;
    door_ph_ = 0; door_t_ = 0.0; door_settle_ = 0.0; door_turning_ = false; vy_cur_ = 0.0; w_cur_ = 0.0;     // OPEN_PATCH
    door_prev_ = 0.0; door_face_t_ = 0.0;
    door_rot_n_ = 0; door_rotating_ = false; door_rot_t_ = 0.0;
    door_relocating_ = false; door_reloc_n_ = 0; door_move_t_ = door_look_t_ = 0.0;
    door_vis_.clear(); door_vis_.push_back({s.ox, s.oy});
    room_t_ = 0.0; room_moving_ = false; room_move_t_ = 0.0; lost_t_ = 0.0;
    if (spec->kind == Kind::GoTo) {
      const Place & pl = places_[static_cast<int>(std::lround(v))];
      tx_ = pl.x; ty_ = pl.y; tyaw_ = pl.yaw;
      goto_d0_ = std::hypot(tx_ - s.ox, ty_ - s.oy);
      goto_timeout_ = 2.0 * goto_d0_ / p_.walk_speed + 30.0;
      goto_ph_ = (goto_d0_ < p_.goto_tol) ? 2 : 0;
      goto_prev_d_ = goto_d0_;
    }
    if (spec->kind == Kind::Push) {
      gest_total_ = (0.5 + v / p_.push_speed * 1.25 + 2.0) + 2.0 * kPushRamp;
      // PUSH_PATCH: walk_to_obstacle zastavi na stop_dist (0.7 m), ruky ale dosiahnu len ~0.34 m -> najprv dojdi bez rúk bližšie.
      // Lidar pod ~0.45 m uz nevidi (filter tela), preto sa vzdialenost zmeria raz na zaciatku a dojde sa podla odometrie.
      push_appr_ = 0.0;
      if (s.have_odom && s.have_lidar && s.lid_front < p_.stop_dist + 0.25) {
        push_appr_ = clampd(s.lid_front - p_.push_gap, 0.0, 0.5);
      }
      if (push_appr_ > 0.03) {push_stage_ = -1; push_ax0_ = s.ox; push_ay0_ = s.oy;}
    }
    phase_ = (spec->kind == Kind::Look) ? Phase::Settle : Phase::Move;     // look_around: najprv zmeraj smer 0
    return true;
  }

  Output cancel()
  {
    Output o;
    if (!active_) {return o;}
    active_ = false;
    o.publish_cmd = true;
    o.finished = true;
    o.result.success = false;
    o.result.status = "cancelled";
    o.result.message = "Skill " + cur_.name + " bol zruseny.";
    o.result.value_done = measured_(Sensors());
    return o;
  }

  Output update(const Sensors & s, double dt)
  {
    Output o;
    if (!active_) {return o;}
    t_ += dt;
    const bool dance_steps = cur_.kind == Kind::Gesture && cur_.gesture == "dance" && p_.dance_step > 0.0;   // GEST_PATCH
    o.publish_cmd = cur_.kind != Kind::Arms && cur_.kind != Kind::Wait && (cur_.kind != Kind::Gesture || dance_steps) &&
      cur_.kind != Kind::Getup;

    // pad robota
    if (cur_.kind != Kind::Stop && cur_.kind != Kind::Getup && s.have_imu && s.gz > p_.fall_gz) {
      arms_up_ = false;
      return finish_(o, false, "robot_fell",
        "Robot sa pocas pohybu prevratil, skill prerušený. The robot has fallen: use the skill get_up.", s);
    }

    switch (cur_.kind) {
      case Kind::Stop:
        o.state = "stopping";
        if (t_ >= 0.3) {return finish_(o, true, "ok", "Robot stoji.", s);}
        break;
      case Kind::Wait:
        o.state = "waiting";
        o.progress = clampd(t_ / value_, 0, 1);
        if (t_ >= value_) {return finish_(o, true, "ok", "Cakanie hotove.", s);}
        break;
      case Kind::Arms:
        o.state = "arms";
        if (!pose_sent_) {
          o.send_pose = true; o.pose = cur_.pose; pose_sent_ = true;
          arms_up_ = (cur_.pose != "arms_down");
        }
        o.progress = clampd(t_ / p_.arm_s, 0, 1);
        if (t_ >= p_.arm_s) {return finish_(o, true, "ok", "Poza ruk '" + cur_.pose + "' hotova.", s);}
        break;
      case Kind::Walk:
        walk_(o, s, dt);
        break;
      case Kind::Turn:
        turn_(o, s, dt);
        break;
      case Kind::Gesture:
        o.state = "gesture";
        if (!pose_sent_) {
          o.gesture = cur_.gesture;
          o.gesture_arm = cur_.arm;
          o.gesture_total = gest_total_;
          pose_sent_ = true;
          arms_up_ = true;
          gest_tw_ = t_;
          gest_t0_ = s.sim_t;
        }
        {
          // cas gesta sa meria v CASE SIMULACIE (rovnako ako ho meria regulator trajektorie), nie v case steny:
          // ak Gazebo bezi pomalsie ako realny cas, gesto by sa inak skoncilo skor, nez ho ruka dokonci
          const double wall = t_ - gest_tw_;
          const double el = (s.sim_t >= 0.0 && gest_t0_ >= 0.0) ? (s.sim_t - gest_t0_) : wall;
          o.progress = clampd(el / (gest_total_ + 0.8), 0, 1);
          if (dance_steps) {      // GEST_PATCH: nohy sa pri tanci striedaju (kroky do strany spolu s pohupovanim trupu)
            const double ramp = cur_.gesture_ramp, ta = el - ramp;
            const double env = clampd(ta / 1.0, 0.0, 1.0) * clampd((gest_total_ - ramp - el) / 1.0, 0.0, 1.0);
            o.vy = -env * p_.dance_step * std::sin(2.0 * M_PI * 0.75 * ta);      // rytmus ako ruky
            if (s.have_odom) {        // drz poziciu: dopredny drift z krokov sa vracia spat
              const double ex = std::cos(s.oyaw) * (s.ox - x0_) + std::sin(s.oyaw) * (s.oy - y0_);
              o.vx = clampd(-1.5 * ex, -0.12, 0.12);
            }
            o.wz = clampd(p_.heading_kp * wrap_pi(yaw0_ - s.yaw), -p_.heading_max, p_.heading_max);
          }
          // trajektoria sa sama vrati do zakladnej polohy (nic dalsie sa neposiela)
          if (el >= gest_total_ + 0.8 || wall > 4.0 * (gest_total_ + 0.8) + 5.0) {
            arms_up_ = false;
            return finish_(o, true, "ok", "Gesto '" + cur_.name + "' hotove, ruky su v povodnej polohe.", s);
          }
        }
        break;
      case Kind::Look:
        look_(o, s, dt);
        break;
      case Kind::Getup:
        getup_(o, s, dt);
        break;
      case Kind::GoTo:
        goto_(o, s, dt);
        break;
      case Kind::Place:
        place_(o, s);
        break;
      case Kind::Where:
        where_(o, s);
        break;
      case Kind::Push:
        push_(o, s, dt);
        break;
      case Kind::Door:
        door_(o, s, dt);
        break;
      case Kind::Room:
        room_(o, s, dt);
        break;
    }
    if (!o.finished && cur_.kind != Kind::Stop && t_ > timeout_()) {
      std::string tmsg = "Skill " + cur_.name + " trval prilis dlho.";
      if (cur_.kind == Kind::Turn) {       // SAFE_PATCH: napoveda pre AI, co robit po zaseknuti
        tmsg += " The robot probably cannot turn because an arm or the body touches an obstacle (LIDAR: front " +
          fmt_dist(s.lid_front) + ", left " + fmt_dist(s.lid_left) + ", right " + fmt_dist(s.lid_right) + ", back " +
          fmt_dist(s.lid_back) + " m). First walk_backward 0.5 m (away from the nearest obstacle), then turn again.";
      }
      return finish_(o, false, "timeout", tmsg, s);
    }
    o.value_done = measured_(s);
    return o;
  }

private:
  enum class Phase {Move, Settle};

  bool fail(Result & res, const char * status, const std::string & msg)
  {
    res.success = false;
    res.status = status;
    res.message = msg;
    res.value_done = 0.0;
    return false;
  }

  double timeout_() const
  {
    switch (cur_.kind) {
      case Kind::Walk: {
        const double v = (cur_.dir_x > 0) ? p_.walk_speed : (cur_.dir_x < 0 ? p_.back_speed : p_.side_speed);
        return 2.0 * value_ / v + 8.0;
      }
      case Kind::Turn: return 2.0 * (value_ * M_PI / 180.0) / p_.turn_rate + 8.0;
      case Kind::Look: return 90.0;
      case Kind::Getup: return 35.0;
      case Kind::GoTo: return goto_timeout_;
      case Kind::Push: return gest_total_ * 3.0 + 15.0;
      case Kind::Door: return 150.0;
      case Kind::Room: return 60.0;
      default: return 1e9;
    }
  }

  double measured_(const Sensors & s) const
  {
    if (!active_) {return 0.0;}
    if (cur_.kind == Kind::Walk) {
      return odom_used_ && s.have_odom ? std::hypot(s.ox - x0_, s.oy - y0_) : d_est_;
    }
    if (cur_.kind == Kind::Turn) {return std::fabs(turned_) * 180.0 / M_PI;}
    if (cur_.kind == Kind::GoTo || cur_.kind == Kind::Push || cur_.kind == Kind::Door || cur_.kind == Kind::Room) {
      return s.have_odom ? std::hypot(s.ox - x0_, s.oy - y0_) : 0.0;
    }
    return 0.0;
  }

  Output & finish_(Output & o, bool ok, const char * status, const std::string & msg, const Sensors & s)
  {
    o.vx = o.vy = o.wz = 0.0;
    o.publish_cmd = true;
    o.finished = true;
    o.result.success = ok;
    o.result.status = status;
    o.result.message = msg;
    o.result.value_done = measured_(s);
    o.value_done = o.result.value_done;
    o.progress = ok ? 1.0 : o.progress;
    active_ = false;
    return o;
  }

  // vzdialenost prekazky v smere pohybu
  double dir_dist_(const Sensors & s) const
  {
    if (cur_.dir_x > 0) {return s.lid_front;}
    if (cur_.dir_x < 0) {return s.lid_back;}
    return cur_.dir_y > 0 ? s.lid_left : s.lid_right;
  }

  void walk_(Output & o, const Sensors & s, double dt)
  {
    const double vmax = (cur_.dir_x > 0) ? p_.walk_speed : (cur_.dir_x < 0 ? p_.back_speed : p_.side_speed);
    const double d = measured_(s);
    double remaining = value_ - d;
    if (cur_.to_obstacle) {
      if (!s.have_lidar || arms_up_) {
        finish_(o, false, "no_lidar", "Stratil som data z lidaru, zastavujem.", s);
        return;
      }
      remaining = std::min(remaining, s.lid_front - p_.stop_dist);
    } else if (p_.lidar_guard && s.have_lidar && !arms_up_ && phase_ == Phase::Move &&
      dir_dist_(s) < p_.block_dist)
    {
      std::ostringstream m;
      m << cur_.name << ": prekazka " << fmt_dist(dir_dist_(s)) << " m v smere pohybu, zastavujem (prejdene "
        << measured_(s) << " m z " << value_ << " m)";
      finish_(o, false, "blocked", m.str(), s);
      return;
    }
    o.progress = clampd(d / value_, 0, 1);
    if (phase_ == Phase::Move && odom_used_ && v_cur_ > 0.2) {
      stuck_t_ += dt;
      if (stuck_t_ >= p_.stuck_s) {
        if (d - stuck_d0_ < p_.stuck_min) {
          std::ostringstream m;
          m << cur_.name << ": robot sa nehybe (za " << p_.stuck_s << " s prejdene len " << (d - stuck_d0_)
            << " m), pred nim je prekazka (stena/predmet); prejdene spolu " << d << " m";
          if (s.have_lidar) {m << ", lidar v smere pohybu " << fmt_dist(dir_dist_(s)) << " m";}
          finish_(o, false, "blocked", m.str(), s);
          return;
        }
        stuck_t_ = 0.0;
        stuck_d0_ = d;
      }
    } else {
      stuck_t_ = 0.0;
      stuck_d0_ = d;
    }
    if (phase_ == Phase::Move) {
      o.state = "walking";
      const bool close = remaining <= std::max(0.02, v_cur_ * p_.lag);
      if (close) {phase_ = Phase::Settle; settle_t_ = 0.0;}
      const double vt = std::min(vmax, std::sqrt(2.0 * p_.decel * std::max(remaining, 0.0)) + 0.1);
      const double dv = clampd(vt - v_cur_, -p_.decel * dt, p_.accel * dt);
      v_cur_ = std::max(0.0, v_cur_ + dv);
    }
    if (phase_ == Phase::Settle) {
      o.state = "settling";
      v_cur_ = 0.0;
      settle_t_ += dt;
    }
    o.vx = cur_.dir_x * v_cur_;
    o.vy = cur_.dir_y * v_cur_;
    const double err = wrap_pi(yaw0_ - s.yaw);
    o.wz = clampd(p_.heading_kp * err, -p_.heading_max, p_.heading_max);
    d_est_ += v_cur_ * dt;     // odhad bez odometrie
    const double left_to_go = cur_.to_obstacle ?
      std::min(value_ - measured_(s), s.lid_front - p_.stop_dist) : value_ - measured_(s);
    if (phase_ == Phase::Settle && settle_t_ >= p_.settle_s && (odom_used_ || cur_.to_obstacle) &&
      corrections_ < 2 && left_to_go > 0.12)
    {
      ++corrections_;      // dobeh bol kratky -> dorovnaj
      phase_ = Phase::Move;
      v_cur_ = 0.0;
    }
    if (phase_ == Phase::Settle && settle_t_ >= p_.settle_s && cur_.to_obstacle) {
      std::ostringstream m;
      if (s.lid_front <= p_.stop_dist + 0.15) {
        m << cur_.name << ": zastavil som pri prekazke, vpredu je " << fmt_dist(s.lid_front)
          << " m (prejdene " << measured_(s) << " m)";
        finish_(o, true, "ok", m.str(), s);
      } else {
        m << cur_.name << ": po " << measured_(s) << " m (limit " << value_ << " m) som prekazku nenasiel, "
          << "vpredu je " << fmt_dist(s.lid_front) << " m";
        finish_(o, false, "no_obstacle", m.str(), s);
      }
      return;
    }
    if (phase_ == Phase::Settle && settle_t_ >= p_.settle_s) {
      std::ostringstream m;
      m << cur_.name << ": prejdene " << measured_(s) << " m z " << value_ << " m";
      if (!odom_used_) {m << " (odhad podla rychlosti, odometria nie je dostupna)";}
      finish_(o, true, "ok", m.str(), s);
    }
  }

  void turn_(Output & o, const Sensors & s, double dt)
  {
    turned_ += wrap_pi(s.yaw - yaw_prev_);
    yaw_prev_ = s.yaw;
    const double target = cur_.sign * value_ * M_PI / 180.0;
    const double rem = target - turned_;
    o.progress = clampd(std::fabs(turned_) / std::fabs(target), 0, 1);
    double w = 0.0;
    if (phase_ == Phase::Move) {
      o.state = "turning";
      const bool wrong_way = rem * cur_.sign < 0;     // pretocil som sa
      const bool close = std::fabs(rem) <= std::max(0.05, std::fabs(v_cur_) * p_.lag) || wrong_way;
      if (close) {phase_ = Phase::Settle; settle_t_ = 0.0;}
      const double wt = std::min(p_.turn_rate, std::sqrt(2.0 * p_.decel * std::max(std::fabs(rem), 0.0)) + 0.08);
      const double dv = clampd(wt - v_cur_, -p_.decel * dt, p_.accel * dt);
      v_cur_ = std::max(0.0, v_cur_ + dv);
    }
    if (phase_ == Phase::Settle) {
      o.state = "settling";
      v_cur_ = 0.0;
      settle_t_ += dt;
    }
    w = cur_.sign * v_cur_;
    o.wz = w;
    if (phase_ == Phase::Settle && settle_t_ >= p_.settle_s && corrections_ < 2 &&
      rem * cur_.sign > 0.07)
    {
      ++corrections_;      // otocenie bolo kratke -> dorovnaj
      phase_ = Phase::Move;
      v_cur_ = 0.0;
    }
    if (phase_ == Phase::Settle && settle_t_ >= p_.settle_s) {
      std::ostringstream m;
      m << cur_.name << ": otocene o " << std::fabs(turned_) * 180.0 / M_PI << " deg z " << value_ << " deg";
      finish_(o, true, "ok", m.str(), s);
    }
  }

  // ---- SKILLS2_PATCH: vstavanie zo zeme
  void getup_(Output & o, const Sensors & s, double dt)
  {
    o.state = "getting_up";
    if (!getup_sent_) {
      if (s.gz < -0.9) {
        finish_(o, true, "ok", "Robot uz stoji, vstavanie nebolo treba.", s);
        return;
      }
      o.send_getup = true;
      getup_sent_ = true;
      upright_t_ = 0.0;
      return;
    }
    upright_t_ = (t_ > 3.0 && s.have_imu && s.gz < -0.9) ? upright_t_ + dt : 0.0;
    o.progress = clampd(t_ / 15.0, 0, 0.95);
    if (upright_t_ >= 1.5) {
      finish_(o, true, "ok", "Robot vstal a stoji. Its position and heading have changed: re-check the camera and LiDAR.", s);
    } else if (t_ > 30.0) {
      finish_(o, false, "getup_failed", "Robot sa nepostavil do 30 s (vstavanie zlyhalo).", s);
    }
  }

  // ---- SKILLS2_PATCH: rozhliadnutie sa (8 x 45 stupnov, v kazdom smere zmeria vzdialenost pred robotom)
  void look_(Output & o, const Sensors & s, double dt)
  {
    turned_ += wrap_pi(s.yaw - yaw_prev_);
    yaw_prev_ = s.yaw;
    const double step = 45.0 * M_PI / 180.0;
    const double rem = look_target_ - turned_;
    o.progress = clampd((look_k_ + 0.5) / 9.0, 0, 1);
    if (phase_ == Phase::Move) {
      o.state = "looking_around";
      const bool close = rem <= std::max(0.05, v_cur_ * p_.lag) || rem < 0;
      if (close) {phase_ = Phase::Settle; settle_t_ = 0.0;}
      const double wt = std::min(p_.turn_rate, std::sqrt(2.0 * p_.decel * std::max(rem, 0.0)) + 0.08);
      const double dv = clampd(wt - v_cur_, -p_.decel * dt, p_.accel * dt);
      v_cur_ = std::max(0.0, v_cur_ + dv);
    }
    if (phase_ == Phase::Settle) {
      o.state = "measuring";
      v_cur_ = 0.0;
      settle_t_ += dt;
    }
    o.wz = v_cur_;
    if (phase_ != Phase::Settle || settle_t_ < p_.settle_s + 0.3) {return;}
    if (rem > 0.07 && corrections_ < 2) {       // otocenie bolo kratke -> dorovnaj
      ++corrections_;
      phase_ = Phase::Move;
      v_cur_ = 0.0;
      return;
    }
    look_front_.push_back(s.have_lidar ? s.lid_front : -1.0);        // vzdialenost pred robotom v tomto smere
    corrections_ = 0;
    if (look_k_ >= 8) {
      look_finish_(o, s);
      return;
    }
    ++look_k_;
    look_target_ = look_k_ * step;
    phase_ = Phase::Move;
    v_cur_ = 0.0;
  }

  void look_finish_(Output & o, const Sensors & s)
  {
    std::ostringstream m;
    m << "look_around: free distance ahead per direction (0 = start direction, + left, - right): ";
    double bd = -1.0;
    for (int k = 0; k < 8; ++k) {
      int deg = k * 45;
      if (deg > 180) {deg -= 360;}
      const double d = look_front_[k];
      if (d < 0) {m << (k ? ", " : "") << deg << ": no data"; continue;}
      m << (k ? ", " : "") << deg << ": " << fmt_dist(std::min(d, 10.0)) << " m";
      bd = std::max(bd, std::min(d, 10.0));
    }
    int best = 1000;       // pri remize (rozdiel < 0.3 m) vyhra smer blizsie k pociatocnemu
    for (int k = 0; k < 8; ++k) {
      int deg = k * 45;
      if (deg > 180) {deg -= 360;}
      if (look_front_[k] >= 0 && std::min(look_front_[k], 10.0) >= bd - 0.3 && std::abs(deg) < std::abs(best)) {best = deg;}
    }
    if (bd >= 0 && best != 1000) {
      m << ". Most open direction: " << best << " deg (" << fmt_dist(bd) << " m)";
    }
    m << ". The robot is again facing its start direction.";
    finish_(o, true, "ok", m.str(), s);
  }

  // ---- NAV_PATCH: pamat miest, cesta na miesto, tlacenie
  static constexpr double kPushRamp = 2.0;        // nabeh / navrat rúk pri push_forward [s] (rovnaky ako v motion_skills.cpp)

  static std::string fx(double v, int prec)
  {
    std::ostringstream o;
    o.precision(prec);
    o << std::fixed << v;
    return o.str();
  }

  // poloha v systeme STARTU: x dopredu od vychodzieho smeru, y dolava
  void rel_(double x, double y, double & rx, double & ry) const
  {
    Place st;
    const auto it = places_.find(0);
    if (it != places_.end()) {st = it->second;}
    const double dx = x - st.x, dy = y - st.y, c = std::cos(st.yaw), sn = std::sin(st.yaw);
    rx = c * dx + sn * dy;
    ry = -sn * dx + c * dy;
  }

  double start_yaw_() const
  {
    const auto it = places_.find(0);
    return it != places_.end() ? it->second.yaw : 0.0;
  }

  std::string pos_text_(double x, double y, double yaw) const
  {
    double rx, ry;
    rel_(x, y, rx, ry);
    return "x=" + fx(rx, 2) + " m, y=" + fx(ry, 2) + " m, heading " + fx(wrap_pi(yaw - start_yaw_()) * 180.0 / M_PI, 0) + " deg";
  }

  void place_(Output & o, const Sensors & s)
  {
    const int slot = static_cast<int>(std::lround(value_));
    const bool had = places_.count(slot) > 0;
    places_[slot] = Place{s.ox, s.oy, s.oyaw};
    finish_(o, true, "ok", "save_place: place " + std::to_string(slot) + (had ? " overwritten" : " saved") + " at " +
      pos_text_(s.ox, s.oy, s.oyaw) + " (start frame: x forward, y left; heading + = left). go_to_place " +
      std::to_string(slot) + " brings the robot back here.", s);
  }

  void where_(Output & o, const Sensors & s)
  {
    std::ostringstream m;
    m << "where_am_i: robot at " << pos_text_(s.ox, s.oy, s.oyaw) << " (start frame: x forward from the start, y left; "
      << "heading 0 = start heading, + left).";
    if (places_.size() > 1 || (places_.size() == 1 && places_.count(0) == 0)) {
      m << " Saved places (distance, direction relative to the robot's heading: + left, - right):";
      for (const auto & kv : places_) {
        const double dx = kv.second.x - s.ox, dy = kv.second.y - s.oy;
        const double dir = wrap_pi(std::atan2(dy, dx) - s.oyaw) * 180.0 / M_PI;
        double rx, ry;
        rel_(kv.second.x, kv.second.y, rx, ry);
        m << " [" << kv.first << (kv.first == 0 ? " = start" : "") << ": at x=" << fx(rx, 1) << " y=" << fx(ry, 1) << ", ";
        if (std::hypot(dx, dy) < 0.15) {
          m << "the robot is here]";
        } else {
          m << fx(std::hypot(dx, dy), 1) << " m away, direction " << (dir >= 0 ? "+" : "") << fx(dir, 0) << " deg]";
        }
      }
    }
    finish_(o, true, "ok", m.str(), s);
  }

  void goto_(Output & o, const Sensors & s, double dt)
  {
    if (!s.have_odom) {
      finish_(o, false, "no_odom", "Stratil som odometriu, zastavujem.", s);
      return;
    }
    const int slot = static_cast<int>(std::lround(value_));
    const double dx = tx_ - s.ox, dy = ty_ - s.oy;
    const double dist = std::hypot(dx, dy);
    const double herr = wrap_pi(std::atan2(dy, dx) - s.oyaw);        // + = cil je vlavo
    double vt = 0.0, wt = 0.0;
    o.progress = clampd(1.0 - dist / std::max(goto_d0_, 0.1), 0.0, 1.0);
    if (goto_ph_ == 0) {                           // 1) otoc sa smerom k cielu
      o.state = "turning_to_target";
      if (dist < p_.goto_tol) {
        goto_ph_ = 2; settle_t_ = 0.0;
      } else if (std::fabs(herr) < 0.10) {
        settle_t_ += dt;
        if (settle_t_ >= p_.settle_s) {goto_ph_ = 1; stuck_t_ = 0.0; goto_prev_d_ = dist;}
      } else {
        settle_t_ = 0.0;
        wt = clampd(1.2 * herr, -p_.turn_rate, p_.turn_rate);
      }
    } else if (goto_ph_ == 1) {                    // 2) chod k cielu a drz smer
      o.state = "walking_to_target";
      if (dist < p_.goto_tol) {
        goto_ph_ = 2; settle_t_ = 0.0;
      } else {
        if (p_.lidar_guard && s.have_lidar && !arms_up_ && s.lid_front < p_.block_dist) {
          finish_(o, false, "blocked", "go_to_place " + std::to_string(slot) + ": obstacle " + fmt_dist(s.lid_front) +
            " m in front, " + fx(dist, 2) + " m from the place. Go around it (turn, walk_forward) and call go_to_place again.", s);
          return;
        }
        vt = std::min(p_.walk_speed, std::sqrt(2.0 * p_.decel * std::max(dist - 0.5 * p_.goto_tol, 0.0)) + 0.1);
        wt = clampd(1.5 * herr, -p_.heading_max, p_.heading_max);
        if (std::fabs(herr) > 0.7 && dist > 0.5) {goto_ph_ = 0; settle_t_ = 0.0;}    // zlý smer -> znova sa otoc
        if (v_cur_ > 0.2) {
          stuck_t_ += dt;
          if (stuck_t_ >= p_.stuck_s) {
            if (goto_prev_d_ - dist < p_.stuck_min) {
              finish_(o, false, "blocked", "go_to_place " + std::to_string(slot) + ": the robot does not move (something blocks the way), " +
                fx(dist, 2) + " m from the place. Go around the obstacle and call go_to_place again.", s);
              return;
            }
            stuck_t_ = 0.0; goto_prev_d_ = dist;
          }
        } else {
          stuck_t_ = 0.0; goto_prev_d_ = dist;
        }
      }
    }
    if (goto_ph_ == 2) {                           // 3) natoc sa tak, ako stal pri ulozeni miesta
      o.state = "turning_to_final_heading";
      const double ferr = wrap_pi(tyaw_ - s.oyaw);
      if (std::fabs(ferr) < 0.10) {
        settle_t_ += dt;
        if (settle_t_ >= p_.settle_s) {
          finish_(o, true, "ok", "go_to_place " + std::to_string(slot) + ": arrived (" + fx(dist, 2) + " m from the place, heading error " +
            fx(ferr * 180.0 / M_PI, 0) + " deg). Now at " + pos_text_(s.ox, s.oy, s.oyaw) + ".", s);
          return;
        }
      } else {
        settle_t_ = 0.0;
        wt = clampd(1.2 * ferr, -p_.turn_rate, p_.turn_rate);
      }
    }
    v_cur_ += clampd(vt - v_cur_, -p_.decel * dt, p_.accel * dt);
    w_cur_ += clampd(wt - w_cur_, -p_.accel * dt, p_.accel * dt);
    o.vx = v_cur_;
    o.wz = w_cur_;
  }

  void push_(Output & o, const Sensors & s, double dt)
  {
    if (push_stage_ == -1) {         // PUSH_PATCH: pristup k predmetu s rukami dole
      o.state = "approaching_object";
      const double done = std::hypot(s.ox - push_ax0_, s.oy - push_ay0_);
      const double left = push_appr_ - done;
      push_t_ += dt;
      if (left <= 0.02 || push_t_ > 2.0 * push_appr_ / p_.push_speed + 6.0) {
        v_cur_ = 0.0;
        push_stage_ = 0;
        push_t_ = 0.0;
      } else {
        const double vt = std::min(p_.push_speed, std::sqrt(2.0 * p_.decel * std::max(left, 0.0)) + 0.05);
        v_cur_ += clampd(vt - v_cur_, -p_.decel * dt, p_.accel * dt);
      }
      o.vx = v_cur_;
      o.wz = clampd(p_.heading_kp * wrap_pi(yaw0_ - s.yaw), -p_.heading_max, p_.heading_max);
      o.progress = 0.0;
      return;
    }
    if (!pose_sent_) {
      o.gesture = "push";
      o.gesture_arm = 3;
      o.gesture_total = gest_total_;
      pose_sent_ = true;
      arms_up_ = true;
      gest_tw_ = t_;
      gest_t0_ = s.sim_t;
    }
    // cas sa meria v case simulacie (rovnako ako trajektoria rúk)
    const double wall = t_ - gest_tw_;
    const double el = (s.sim_t >= 0.0 && gest_t0_ >= 0.0) ? (s.sim_t - gest_t0_) : wall;
    const double t_start = kPushRamp + 0.6, t_end = gest_total_ - kPushRamp - 0.4;
    double vt = 0.0;
    if (push_stage_ == 0 && el >= t_start) {
      push_stage_ = 1; push_x0_ = s.ox; push_y0_ = s.oy; stuck_t_ = 0.0; stuck_d0_ = 0.0;
    }
    if (push_stage_ == 1) {
      o.state = "pushing";
      push_moved_ = std::hypot(s.ox - push_x0_, s.oy - push_y0_);
      if (push_moved_ >= value_ - 0.03) {
        push_stage_ = 2;
        push_msg_ = "push_forward: pushed " + fx(push_moved_, 2) + " m forward (asked " + fx(value_, 2) +
          " m). The arms are back in their original position.";
      } else if (el >= t_end) {
        push_stage_ = 2; push_ok_ = false; push_status_ = "timeout";
        push_msg_ = "push_forward: time ran out after " + fx(push_moved_, 2) + " m of " + fx(value_, 2) + " m (simulation is slow?).";
      } else {
        vt = std::min(p_.push_speed, std::sqrt(2.0 * p_.decel * std::max(value_ - push_moved_, 0.0)) + 0.05);
        stuck_t_ += dt;
        if (stuck_t_ >= 3.0) {
          if (push_moved_ - stuck_d0_ < 0.2) {
            push_stage_ = 2; push_ok_ = false; push_status_ = "blocked";
            push_msg_ = "push_forward: the robot cannot move forward (the object is too heavy or blocked, e.g. against a wall); "
              "pushed only " + fx(push_moved_, 2) + " m of " + fx(value_, 2) + " m. The arms are back in their original position.";
          }
          stuck_t_ = 0.0; stuck_d0_ = push_moved_;
        }
      }
    }
    if (push_stage_ == 2) {o.state = "arms_returning";}
    v_cur_ += clampd(vt - v_cur_, -p_.decel * dt, p_.accel * dt);
    o.vx = v_cur_;
    o.wz = clampd(p_.heading_kp * wrap_pi(yaw0_ - s.yaw), -p_.heading_max, p_.heading_max);
    o.progress = clampd(el / (gest_total_ + 0.8), 0, 1);
    if (el >= gest_total_ + 0.8 || wall > 4.0 * (gest_total_ + 0.8) + 5.0) {
      arms_up_ = false;
      if (push_stage_ < 2) {
        push_ok_ = false; push_status_ = "timeout";
        push_msg_ = "push_forward: did not finish in time (simulation is slow?).";
      }
      finish_(o, push_ok_, push_status_.c_str(), push_msg_, s);
    }
  }

  // ---- OPEN_PATCH: go_through_opening - najde otvor v stene, nastavi sa pred nim, vycentruje sa a prejde nim
  void door_world_(const Sensors & s, double x, double y, double & wx, double & wy) const
  {
    const double c = std::cos(s.oyaw), sn = std::sin(s.oyaw);
    wx = s.ox + c * x - sn * y;
    wy = s.oy + sn * x + c * y;
  }

  // ulozi otvor (v suradniciach odometrie); alpha < 1 = zjemnenie predosleho odhadu novym meranim
  void door_set_(const Sensors & s, const Opening & op, double alpha)
  {
    double wx, wy;
    door_world_(s, op.cx, op.cy, wx, wy);
    const double c = std::cos(s.oyaw), sn = std::sin(s.oyaw);
    double nx = c * op.nx - sn * op.ny, ny = sn * op.nx + c * op.ny;
    if (alpha < 1.0) {
      wx = door_cx_ + alpha * (wx - door_cx_);
      wy = door_cy_ + alpha * (wy - door_cy_);
      nx = (1.0 - alpha) * door_nx_ + alpha * nx;
      ny = (1.0 - alpha) * door_ny_ + alpha * ny;
      door_w_ += alpha * (op.width - door_w_);
    } else {
      door_w_ = op.width;
    }
    const double n = std::hypot(nx, ny);
    door_cx_ = wx; door_cy_ = wy;
    door_nx_ = nx / n; door_ny_ = ny / n;
  }

  // zjemni polohu otvoru novym meraniam (berie len otvor blizko povodneho)
  void door_refine_(const Sensors & s)
  {
    const auto ops = find_openings(s.bins, 0.3, 6.0, 12.0, 120.0);
    const Opening * best = nullptr;
    double best_d = 0.8;
    for (const auto & op : ops) {
      double wx, wy;
      door_world_(s, op.cx, op.cy, wx, wy);
      const double c = std::cos(s.oyaw), sn = std::sin(s.oyaw);
      const double nx = c * op.nx - sn * op.ny, ny = sn * op.nx + c * op.ny;
      const double d = std::hypot(wx - door_cx_, wy - door_cy_);
      if (d < best_d && nx * door_nx_ + ny * door_ny_ > 0.8 && std::fabs(op.width - door_w_) < 0.5) {best = &op; best_d = d;}
    }
    if (best) {door_set_(s, *best, 0.4);}
  }

  // po prejdeni: okolo otvoru musi byt vidiet stenu (nie volny priestor). Vrati pocet bodov steny vlavo a vpravo od otvoru.
  void door_verify_(const Sensors & s, int & nl, int & nr) const
  {
    nl = nr = 0;
    const double tx = -door_ny_, ty = door_nx_;
    for (int k = 0; k < kBins; ++k) {
      if (s.bins.r[k] > 8.0) {continue;}
      double wx, wy;
      door_world_(s, s.bins.x[k], s.bins.y[k], wx, wy);
      const double rx = wx - door_cx_, ry = wy - door_cy_;
      const double depth = rx * door_nx_ + ry * door_ny_, lat = rx * tx + ry * ty;
      if (depth < -0.4 || depth > 0.9) {continue;}                     // v rovine steny (aj s jej hrubkou)
      if (lat > door_w_ / 2 + 0.1 && lat < door_w_ / 2 + 1.5) {++nl;}
      if (lat < -door_w_ / 2 - 0.1 && lat > -door_w_ / 2 - 1.5) {++nr;}
    }
  }

  // vyber otvoru zo zoznamu: najblizsi smerom, ktory robot pozera; dvere, cez ktore prave presiel, az ako poslednu moznost (fb)
  const Opening * door_pick_(const Sensors & s, const std::vector<Opening> & all, int & narrow, double & narrow_w, int & wide,
    const Opening *& fb) const
  {
    const Opening * best = nullptr;
    fb = nullptr;
    double best_cost = 0.0, fb_cost = 0.0;
    for (const auto & op : all) {
      if (op.width < p_.door_min_w) {++narrow; narrow_w = std::max(narrow_w, op.width); continue;}
      if (op.width > p_.door_max_w) {++wide; continue;}
      const double cost = std::fabs(op.bearing) + 3.0 * op.dist;
      double wx, wy;
      door_world_(s, op.cx, op.cy, wx, wy);
      bool bad = false;
      for (const auto & v : door_bad_) {if (std::hypot(wx - v.first, wy - v.second) < 1.2) {bad = true;}}
      if (bad) {continue;}
      if (door_last_ok_ && std::hypot(wx - door_last_x_, wy - door_last_y_) < 1.2) {
        if (!fb || cost < fb_cost) {fb = &op; fb_cost = cost;}
        continue;
      }
      if (!best || cost < best_cost) {best = &op; best_cost = cost;}
    }
    return best;
  }

  // dalsie miesto na prehladavanie izby: smerom k najvolnejsiemu priestoru (stred izby), nie tam, kde uz bol
  bool room_plan_(const Sensors & s, const std::vector<std::pair<double, double>> & vis, double maxhop, double minsep, double & rx, double & ry) const
  {
    double best_c = 0.0, best_a = 0.0, best_l = 0.0;
    for (int a = -180; a < 180; a += 6) {
      double c = 8.0;
      for (int d = -18; d <= 18; d += 2) {
        const int k = ((static_cast<int>(std::floor((a + d + 180) / kBinDeg)) % kBins) + kBins) % kBins;
        c = std::min(c, std::min(s.bins.r[k], 8.0));
      }
      if (c < 2.2) {continue;}
      const double l = std::min(maxhop, c - 1.0);
      const double tx = s.ox + l * std::cos(s.oyaw + a * M_PI / 180.0), ty = s.oy + l * std::sin(s.oyaw + a * M_PI / 180.0);
      double dv = 4.0;
      for (const auto & v : vis) {dv = std::min(dv, std::hypot(tx - v.first, ty - v.second));}
      if (dv < minsep) {continue;}                        // tam uz bol
      const double score = c + 0.8 * std::min(dv, 4.0);                  // volny priestor a zaroven kus od miest, kde uz bol
      if (score > best_c) {best_c = score; best_a = a; best_l = l;}
    }
    if (best_c <= 0.0) {return false;}
    rx = s.ox + best_l * std::cos(s.oyaw + best_a * M_PI / 180.0);
    ry = s.oy + best_l * std::sin(s.oyaw + best_a * M_PI / 180.0);
    return true;
  }

  bool door_plan_reloc_(const Sensors & s)
  {
    if (!room_plan_(s, door_vis_, 3.0, 1.4, door_rx_, door_ry_)) {return false;}
    door_vis_.push_back({door_rx_, door_ry_});
    door_relocating_ = true; door_move_t_ = 0.0; door_look_t_ = 0.0; door_turning_ = true;
    ++door_reloc_n_;
    return true;
  }

  // senzory musia byt k dispozicii; kratky vypadok (do 3 s) sa pretrva v pokoji, az dlhsi skill ukonci
  bool door_sensors_ok_(Output & o, const Sensors & s, double dt, const std::string & who)
  {
    if (s.have_odom && s.have_lidar && !arms_up_) {lost_t_ = 0.0; return true;}
    lost_t_ += dt;
    if (arms_up_ || lost_t_ > 3.0) {
      finish_(o, false, "no_lidar", who + ": lost the LiDAR or odometry data for more than 3 s, stopping.", s);
      return false;
    }
    v_cur_ = 0.0; w_cur_ = 0.0; vy_cur_ = 0.0;
    o.vx = o.vy = o.wz = 0.0;
    return false;
  }

  // jazda k bodu (pri hladani dveri)
  void door_walk_to_(Output & o, const Sensors & s, double dt, double tx, double ty, bool & arrived)
  {
    const double dx = tx - s.ox, dy = ty - s.oy, dist = std::hypot(dx, dy);
    const double herr = wrap_pi(std::atan2(dy, dx) - s.oyaw);
    double vt = 0.0, wt = 0.0;
    arrived = dist < 0.25 || (p_.lidar_guard && s.lid_front < 0.8 && v_cur_ > 0.05 && dist > 0.5);
    if (!arrived) {
      if (std::fabs(herr) > 0.6) {door_turning_ = true;} else if (std::fabs(herr) < 0.2) {door_turning_ = false;}
      if (door_turning_) {
        wt = clampd(1.2 * herr, -p_.turn_rate, p_.turn_rate);
      } else {
        vt = std::min(p_.walk_speed, std::sqrt(2.0 * p_.decel * std::max(dist - 0.1, 0.0)) + 0.1);
        wt = clampd(1.5 * herr, -p_.heading_max, p_.heading_max);
      }
    }
    v_cur_ += clampd(vt - v_cur_, -p_.decel * dt, p_.accel * dt);
    w_cur_ += clampd(wt - w_cur_, -p_.accel * dt, p_.accel * dt);
    vy_cur_ += clampd(0.0 - vy_cur_, -p_.accel * dt, p_.accel * dt);
    o.vx = v_cur_; o.vy = vy_cur_; o.wz = w_cur_;
  }

  // search_room: presun na dalsie neprehladane miesto v izbe
  void room_(Output & o, const Sensors & s, double dt)
  {
    if (!door_sensors_ok_(o, s, dt, "search_room")) {return;}
    o.state = "searching_room";
    room_t_ += dt;
    if (!room_moving_) {
      if (room_t_ < 0.6) {return;}
      bool near_known = false;
      for (const auto & v : room_vis_) {if (std::hypot(s.ox - v.first, s.oy - v.second) < 5.0) {near_known = true;}}
      if (!near_known) {room_vis_.clear();}                // iny priestor (napr. po prechode dverami) - zacni odznova
      bool at_known = false;
      for (const auto & v : room_vis_) {if (std::hypot(s.ox - v.first, s.oy - v.second) < 0.8) {at_known = true;}}
      if (!at_known) {room_vis_.push_back({s.ox, s.oy});}
      if (!room_plan_(s, room_vis_, value_, 2.5, room_rx_, room_ry_)) {
        std::ostringstream m;
        m << "search_room: nothing new to explore here - " << room_vis_.size() << " spot(s) of this room have been visited and the free space around is "
          << "used up (LIDAR: front " << fmt_dist(s.lid_front) << ", left " << fmt_dist(s.lid_left) << ", right " << fmt_dist(s.lid_right)
          << ", back " << fmt_dist(s.lid_back) << " m). The whole room has been searched: finish and tell the user what you found (or did not find).";
        finish_(o, false, "explored", m.str(), s);
        return;
      }
      room_vis_.push_back({room_rx_, room_ry_});
      room_moving_ = true; room_move_t_ = 0.0; door_turning_ = true;
    }
    room_move_t_ += dt;
    bool arrived = false;
    door_walk_to_(o, s, dt, room_rx_, room_ry_, arrived);
    o.progress = clampd(room_move_t_ / 12.0, 0.0, 0.9);
    if (arrived || room_move_t_ > 25.0) {
      v_cur_ = 0.0; w_cur_ = 0.0;
      std::ostringstream m;
      m << "search_room: moved to a new spot of the room, now at " << pos_text_(s.ox, s.oy, s.oyaw) << " (" << room_vis_.size()
        << " spot(s) visited so far). Now look_around or look to see this part of the room.";
      finish_(o, true, "ok", m.str(), s);
    }
  }

  void door_(Output & o, const Sensors & s, double dt)
  {
    if (!door_sensors_ok_(o, s, dt, "go_through_opening")) {return;}
    door_t_ += dt;
    double vt = 0.0, wt = 0.0, vyt = 0.0;
    if (door_ph_ == 0) {                                   // 0) pockaj na ustaleny sken a vyber otvor
      o.state = "searching_opening";
      if (door_rotating_) {                                // otacanie na mieste, aby sa presiel cely okolit (po 120 st.)
        door_rot_t_ += dt;
        const double e = wrap_pi(door_rot_tgt_ - s.oyaw);
        if (std::fabs(e) < 0.12 || door_rot_t_ > 10.0) {
          door_rotating_ = false; door_t_ = 0.0;
        } else {
          wt = clampd(1.5 * e, -p_.turn_rate, p_.turn_rate);
          if (std::fabs(wt) < 0.25) {wt = (e > 0 ? 0.25 : -0.25);}
        }
        v_cur_ += clampd(0.0 - v_cur_, -p_.decel * dt, p_.accel * dt);
        w_cur_ += clampd(wt - w_cur_, -p_.accel * dt, p_.accel * dt);
        o.vx = v_cur_; o.vy = 0.0; o.wz = w_cur_;
        return;
      }
      std::vector<Opening> all;
      const Opening * best = nullptr, * fb = nullptr;
      double narrow_w = 0.0;
      int narrow = 0, wide = 0;
      if (door_relocating_) {                              // presun na dalsie miesto; pocas jazdy sa stale pozera po dverach
        door_move_t_ += dt;
        door_look_t_ += dt;
        if (door_look_t_ >= 0.25) {
          door_look_t_ = 0.0;
          all = find_openings(s.bins, 0.3, 8.0, value_, 110.0);
          best = door_pick_(s, all, narrow, narrow_w, wide, fb);
        }
        if (!best) {
          bool arrived = false;
          door_walk_to_(o, s, dt, door_rx_, door_ry_, arrived);
          if (arrived || door_move_t_ > 16.0) {door_relocating_ = false; door_t_ = 0.0; door_rot_n_ = 0;}
          return;
        }
        door_relocating_ = false;
      } else {
        if (door_t_ < 0.6) {return;}
        all = find_openings(s.bins, 0.3, 8.0, value_, 110.0);
        best = door_pick_(s, all, narrow, narrow_w, wide, fb);
        if (!best && door_rot_n_ < 2) {                    // v tomto smere nic - otoc sa o 120 st. a hladaj dalej
          ++door_rot_n_;
          door_rot_tgt_ = wrap_pi(s.oyaw + 2.0944);
          door_rotating_ = true; door_rot_t_ = 0.0;
          return;
        }
        if (!best && door_reloc_n_ < 4 && door_plan_reloc_(s)) {return;}      // nic nevidno ani dookola - presun sa inam v izbe
        if (!best && fb) {best = fb;}                      // jedine dvere su tie, cez ktore prisiel
      }
      if (!best) {
        std::ostringstream m;
        m << "go_through_opening: no passable opening found within " << fx(value_, 1) << " m. I looked all the way round (360 degrees) at "
          << (door_reloc_n_ + 1) << " place(s) in this room and found no door (a gap in a wall " << fx(p_.door_min_w, 2) << "-" << fx(p_.door_max_w, 1) << " m wide).";
        if (narrow > 0) {
          m << " I saw " << narrow << " gap(s) that are too narrow for the robot (the widest " << fx(narrow_w, 2) << " m).";
        }
        if (wide > 0) {m << " There is also a very wide open area (no door frame).";}
        m << " LIDAR now: front " << fmt_dist(s.lid_front) << ", left " << fmt_dist(s.lid_left) << ", right " << fmt_dist(s.lid_right)
          << ", back " << fmt_dist(s.lid_back) << " m. The whole search is done: do NOT call it again right away. Finish and tell the user the door was not found, "
          << "or try once more only after something has changed (e.g. a different part of the world).";
        finish_(o, false, "no_opening", m.str(), s);
        return;
      }
      door_set_(s, *best, 1.0);
      door_w0_ = best->width; door_d0_ = best->dist; door_b0_ = best->bearing;
      const double sa0 = (s.ox - door_cx_) * door_nx_ + (s.oy - door_cy_) * door_ny_;
      door_ph_ = (sa0 > -0.6 * p_.door_pre) ? 2 : 1;       // uz je pri dverach -> rovno sa narovnaj
      door_settle_ = 0.0; stuck_t_ = 0.0; door_prev_ = 1e9;
    }
    const double sa = (s.ox - door_cx_) * door_nx_ + (s.oy - door_cy_) * door_ny_;      // vzdialenost od roviny dveri (< 0 = pred nimi)
    const double dcenter = std::hypot(door_cx_ - s.ox, door_cy_ - s.oy);
    if (door_ph_ == 1) {                                   // 1) chod k bodu pred dverami
      o.state = "approaching_opening";
      if (dcenter > 0.9) {door_refine_(s);}
      const double px = door_cx_ - door_nx_ * p_.door_pre, py = door_cy_ - door_ny_ * p_.door_pre;
      const double dx = px - s.ox, dy = py - s.oy, dist = std::hypot(dx, dy);
      const double herr = wrap_pi(std::atan2(dy, dx) - s.oyaw);
      o.progress = clampd(0.5 * (1.0 - dcenter / std::max(door_d0_, 0.5)), 0.0, 0.5);
      if (dist < 0.2 || sa > -0.6 * p_.door_pre) {
        door_ph_ = 2; door_settle_ = 0.0;
      } else {
        if (std::fabs(herr) > 0.6) {door_turning_ = true;} else if (std::fabs(herr) < 0.2) {door_turning_ = false;}
        if (door_turning_) {
          wt = clampd(1.2 * herr, -p_.turn_rate, p_.turn_rate);
        } else {
          if (p_.lidar_guard && s.lid_front < p_.block_dist && dcenter > 1.3 && v_cur_ > 0.05) {
            finish_(o, false, "blocked", "go_through_opening: an obstacle " + fmt_dist(s.lid_front) + " m ahead is in the way to the opening (" +
              fx(dcenter, 1) + " m away). Walk around it (turn, walk_forward) and call go_through_opening again.", s);
            return;
          }
          vt = std::min(p_.walk_speed, std::sqrt(2.0 * p_.decel * std::max(dist - 0.1, 0.0)) + 0.1);
          wt = clampd(1.5 * herr, -p_.heading_max, p_.heading_max);
          if (v_cur_ > 0.2) {
            stuck_t_ += dt;
            if (stuck_t_ >= p_.stuck_s) {
              if (door_prev_ - dist < p_.stuck_min) {
                finish_(o, false, "blocked", "go_through_opening: the robot does not move (something blocks the way), the opening is " +
                  fx(dcenter, 1) + " m away. Go around the obstacle and call it again.", s);
                return;
              }
              stuck_t_ = 0.0; door_prev_ = dist;
            }
          } else {
            stuck_t_ = 0.0; door_prev_ = dist;
          }
        }
      }
    }
    if (door_ph_ == 2) {                                   // 2) narovnaj sa kolmo na stenu
      o.state = "facing_opening";
      door_face_t_ += dt;
      const double herr = wrap_pi(std::atan2(door_ny_, door_nx_) - s.oyaw);
      o.progress = 0.5;
      if (std::fabs(herr) < 0.1 || door_face_t_ > 12.0) {          // po 12 s sa ide aj s mensou odchylkou (aby sa nezasekol)
        door_settle_ += dt;
        if (door_settle_ >= p_.settle_s) {door_ph_ = 3; stuck_t_ = 0.0; door_prev_ = sa; door_settle_ = 0.0;}
      } else {
        door_settle_ = 0.0;
        wt = clampd(1.2 * herr, -p_.turn_rate, p_.turn_rate);
      }
    }
    if (door_ph_ == 3) {                                   // 3) prejdi rovno, drz stred medzi stlpikmi
      o.state = "passing_opening";
      const double e = (s.ox - door_cx_) * (-door_ny_) + (s.oy - door_cy_) * door_nx_;     // + = robot je vlavo od stredu
      const double herr = wrap_pi(std::atan2(door_ny_, door_nx_) - s.oyaw);
      const double rem = p_.door_exit - sa;
      wt = clampd(1.5 * herr, -0.3, 0.3);
      vyt = clampd(-1.2 * e, -0.2, 0.2);
      if (std::fabs(sa) < 0.45 && s.lid_left < 1.4 && s.lid_right < 1.4) {          // v otvore: vystred podla lidaru
        vyt = clampd(0.5 * (s.lid_left - s.lid_right), -0.15, 0.15);
      }
      o.progress = clampd(0.5 + 0.5 * (sa + p_.door_pre) / (p_.door_exit + p_.door_pre), 0.5, 1.0);
      if (sa < -0.4 && std::fabs(e) > 0.12) {
        vt = 0.0;                                         // najprv sa posun bokom do stredu
      } else if (rem <= std::max(0.03, v_cur_ * p_.lag)) {
        vt = 0.0; vyt = 0.0; wt = 0.0;
        door_settle_ += dt;
        if (door_settle_ >= p_.settle_s && v_cur_ < 0.05) {
          std::ostringstream m;
          m << "go_through_opening: passed through the opening (" << fx(door_w0_, 2) << " m wide, it was " << fx(door_d0_, 1) << " m away, "
            << fx(std::fabs(door_b0_), 0) << " deg to the " << (door_b0_ >= 0 ? "left" : "right") << "). The robot now stands "
            << fx(std::max(sa, 0.0), 1) << " m behind the wall, facing away from the doorway. Now at " << pos_text_(s.ox, s.oy, s.oyaw)
            << ". Check the picture and the LIDAR: you are in the next room.";
          int nl = 0, nr = 0;
          door_verify_(s, nl, nr);
          if (nl + nr < 6 || std::max(nl, nr) < 4) {          // okolo "dveri" nie je stena -> nebola to medzera v stene
            door_bad_.push_back({door_cx_, door_cy_});
            std::ostringstream f;
            f << "go_through_opening: FAILED - I drove through a gap (" << fx(door_w0_, 2) << " m wide), but the LiDAR now sees no wall on either side of it "
              << "(wall points left " << nl << ", right " << nr << "), so it was NOT a doorway (probably a gap between objects). I did not change rooms. "
              << "Now at " << pos_text_(s.ox, s.oy, s.oyaw) << ". Look at the picture for a real door with a frame and call go_through_opening again "
              << "(this gap will be ignored).";
            finish_(o, false, "not_through", f.str(), s);
            return;
          }
          door_last_ok_ = true; door_last_x_ = door_cx_; door_last_y_ = door_cy_; room_vis_.clear();
          m << " Wall check: " << nl << "+" << nr << " wall points around the doorway behind me.";
          finish_(o, true, "ok", m.str(), s);
          return;
        }
      } else {
        vt = std::min(p_.door_speed, std::sqrt(2.0 * p_.decel * std::max(rem, 0.0)) + 0.08);
        if (v_cur_ > 0.2) {
          stuck_t_ += dt;
          if (stuck_t_ >= p_.stuck_s) {
            if (sa - door_prev_ < p_.stuck_min) {
              finish_(o, false, "blocked", "go_through_opening: the robot is stuck in the opening (the doorway is probably too narrow or something "
                "blocks it); it is " + fx(std::max(sa, 0.0), 2) + " m behind the wall plane. Use walk_backward and try again or look at the picture.", s);
              return;
            }
            stuck_t_ = 0.0; door_prev_ = sa;
          }
        } else {
          stuck_t_ = 0.0; door_prev_ = sa;
        }
      }
    }
    v_cur_ += clampd(vt - v_cur_, -p_.decel * dt, p_.accel * dt);
    w_cur_ += clampd(wt - w_cur_, -p_.accel * dt, p_.accel * dt);
    vy_cur_ += clampd(vyt - vy_cur_, -p_.accel * dt, p_.accel * dt);
    o.vx = v_cur_;
    o.vy = vy_cur_;
    o.wz = w_cur_;
  }

  Params p_;
  std::vector<SkillSpec> specs_;
  SkillSpec cur_;
  bool active_ = false, started_ = false, pose_sent_ = false, odom_used_ = false, arms_up_ = false;
  Phase phase_ = Phase::Move;
  int corrections_ = 0;
  double stuck_t_ = 0, stuck_d0_ = 0;
  double value_ = 0, t_ = 0, v_cur_ = 0, settle_t_ = 0, d_est_ = 0, turned_ = 0;
  double yaw0_ = 0, yaw_prev_ = 0, x0_ = 0, y0_ = 0;
  bool getup_sent_ = false;
  double gest_total_ = 0.0, gest_tw_ = 0.0, gest_t0_ = -1.0;    // SKILLS2_PATCH
  double upright_t_ = 0, look_target_ = 0;
  int look_k_ = 0;
  std::vector<double> look_front_;
  // NAV_PATCH
  std::map<int, Place> places_;
  bool start_saved_ = false;
  int goto_ph_ = 0, push_stage_ = 0;
  double tx_ = 0, ty_ = 0, tyaw_ = 0, goto_d0_ = 0, goto_timeout_ = 60, goto_prev_d_ = 0, w_cur_ = 0;
  bool push_ok_ = true;
  std::string push_status_ = "ok", push_msg_;
  double push_moved_ = 0, push_x0_ = 0, push_y0_ = 0;
  double push_appr_ = 0, push_ax0_ = 0, push_ay0_ = 0, push_t_ = 0;   // PUSH_PATCH
  // OPEN_PATCH: go_through_opening
  int door_ph_ = 0;
  bool door_turning_ = false, door_rotating_ = false;
  int door_rot_n_ = 0;
  double door_rot_tgt_ = 0, door_rot_t_ = 0;
  bool door_relocating_ = false, door_last_ok_ = false;
  int door_reloc_n_ = 0;
  double door_rx_ = 0, door_ry_ = 0, door_move_t_ = 0, door_look_t_ = 0, door_last_x_ = 0, door_last_y_ = 0;
  std::vector<std::pair<double, double>> door_vis_;      // kde uz robot hladal dvere
  std::vector<std::pair<double, double>> door_bad_;      // medzery, o ktorych sa ukazalo, ze nie su dvere (persistuje)
  std::vector<std::pair<double, double>> room_vis_;      // kde uz robot prehladaval izbu (search_room), pamata si aj medzi volaniami
  bool room_moving_ = false;
  double room_t_ = 0, room_move_t_ = 0, room_rx_ = 0, room_ry_ = 0, lost_t_ = 0;
  double door_t_ = 0, door_settle_ = 0, door_prev_ = 0, vy_cur_ = 0, door_face_t_ = 0;
  double door_cx_ = 0, door_cy_ = 0, door_nx_ = 1, door_ny_ = 0, door_w_ = 0, door_w0_ = 0, door_d0_ = 0, door_b0_ = 0;
};

}  // namespace skills

// STUCK_PATCH
