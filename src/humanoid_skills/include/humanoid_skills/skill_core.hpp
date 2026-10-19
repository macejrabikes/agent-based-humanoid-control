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
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace skills
{

enum class Kind {Stop, Walk, Turn, Arms, Wait};

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
  double stuck_s = 4.0, stuck_min = 0.15;   // STUCK_PATCH: ak za stuck_s prejde < stuck_min [m], robot stoji o prekazku
};

struct Sensors
{
  bool have_imu = false;
  double yaw = 0.0;       // [rad] z IMU
  double gz = -1.0;       // z-zlozka gravitacie v suradniciach panvy (stoji: -1)
  bool have_odom = false;
  double ox = 0.0, oy = 0.0;
  bool have_lidar = false;   // cerstve data z lidaru
  double lid_front = 99, lid_back = 99, lid_left = 99, lid_right = 99;   // najblizsia prekazka v sektore [m], 99 = nic
};

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
  double self_x = 0.22, self_y = 0.35;  // obdlznik okolo senzora = telo a ruky robota, ignoruje sa
  double front_deg = 30, side_lo = 60, side_hi = 120, back_deg = 150;
  double max_range = 10.0;
  double mount_pitch = 0.04;            // sklon senzora voci panve (rpy v URDF) [rad]
  double up_x = 0, up_y = 0, up_z = 1;  // "hore" v suradniciach panvy (z IMU); vyrovna naklon robota pri chodzi
};

struct LidarSectors
{
  double front = 99, back = 99, left = 99, right = 99;
  int seen = 0;       // kolko platnych bodov prislo (kontrola, ze lidar naozaj meria)
  void add(double x, double y, double z, const LidarParams & p)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {return;}
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
    s = {}; s.name = "raise_right_arm"; s.kind = K::Arms; s.pose = "raise_right_arm";
    s.description = "Zdvihne pravu ruku."; add(s);
    s = {}; s.name = "raise_left_arm"; s.kind = K::Arms; s.pose = "raise_left_arm";
    s.description = "Zdvihne lavu ruku."; add(s);
    s = {}; s.name = "raise_both_arms"; s.kind = K::Arms; s.pose = "raise_both_arms";
    s.description = "Zdvihne obe ruky."; add(s);
    s = {}; s.name = "arms_down"; s.kind = K::Arms; s.pose = "arms_down";
    s.description = "Spusti ruky dole."; add(s);
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
      if (s.gz > p_.upright_gz) {
        return fail(res, "not_upright", "Robot nestoji rovno (lezi alebo je velmi naklonený), pohyb odmietnuty.");
      }
    }
    if (spec->to_obstacle) {
      if (!s.have_lidar) {
        return fail(res, "no_lidar", "Nemam cerstve data z lidaru (/lidar/points), neviem hladat prekazku.");
      }
      if (arms_up_) {
        return fail(res, "arms_up", "Ruky su hore a lidar by videl ich - najprv arms_down.");
      }
    }
    cur_ = *spec;
    value_ = v;
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
    o.publish_cmd = cur_.kind != Kind::Arms && cur_.kind != Kind::Wait;

    // pad robota
    if (cur_.kind != Kind::Stop && s.have_imu && s.gz > p_.fall_gz) {
      return finish_(o, false, "robot_fell", "Robot sa pocas pohybu prevratil, skill prerušený.", s);
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
    }
    if (!o.finished && cur_.kind != Kind::Stop && t_ > timeout_()) {
      return finish_(o, false, "timeout", "Skill " + cur_.name + " trval prilis dlho.", s);
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

  Params p_;
  std::vector<SkillSpec> specs_;
  SkillSpec cur_;
  bool active_ = false, started_ = false, pose_sent_ = false, odom_used_ = false, arms_up_ = false;
  Phase phase_ = Phase::Move;
  int corrections_ = 0;
  double stuck_t_ = 0, stuck_d0_ = 0;
  double value_ = 0, t_ = 0, v_cur_ = 0, settle_t_ = 0, d_est_ = 0, turned_ = 0;
  double yaw0_ = 0, yaw_prev_ = 0, x0_ = 0, y0_ = 0;
};

}  // namespace skills

// STUCK_PATCH
