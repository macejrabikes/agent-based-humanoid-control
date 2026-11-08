// test_skills3_sim.cpp - 2D simulator (bez ROS a Gazeba) na testy skillov walk_to_object, go_around_obstacle a follow_wall.
//
// Svet = obdlzniky, robot = kruh s polomerom 0.3 m, LiDAR = ray-casting po 1 stupni (so suradnicami ako v skutocnom LidarSectors::add,
// teda aj s "slepou zonou" okolo tela), odometria je v inom suradnicovom systeme nez svet a ma 2 % chybu mierky. Pouziva SKUTOCNY
// SkillExecutor zo skill_core.hpp.
//
// Spustenie (z korena repozitara):
//   g++ -std=c++17 -O1 -Wall -Wextra -I src/humanoid_skills/include tools/test_skills3_sim.cpp -o /tmp/test_s3 && /tmp/test_s3
//   /tmp/test_s3 --fuzz 200         (nahodne scenare; FUZZ_SEED=n zmeni nahodu)
//   /tmp/test_s3 -v                 (vypise cestu robota a spravu skillu)
// Navratovy kod 0 = vsetky scenare dopadli podla ocakavania.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "humanoid_skills/skill_core.hpp"

using namespace skills;

struct Box {double x0, y0, x1, y1;};
struct World
{
  std::vector<Box> b;
  void add(double x0, double y0, double x1, double y1) {b.push_back({x0, y0, x1, y1});}
};

static double ray(const World & w, double ox, double oy, double ang, double maxr)
{
  const double dx = std::cos(ang), dy = std::sin(ang);
  double best = maxr;
  for (const auto & q : w.b) {
    double t0 = 0.0, t1 = best;
    bool hit = true;
    const double o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {q.x0, q.y0}, hi[2] = {q.x1, q.y1};
    for (int k = 0; k < 2 && hit; ++k) {
      if (std::fabs(d[k]) < 1e-12) {
        if (o[k] < lo[k] || o[k] > hi[k]) {hit = false;}
      } else {
        double a = (lo[k] - o[k]) / d[k], c = (hi[k] - o[k]) / d[k];
        if (a > c) {std::swap(a, c);}
        t0 = std::max(t0, a);
        t1 = std::min(t1, c);
        if (t0 > t1) {hit = false;}
      }
    }
    if (hit && t0 < best) {best = t0;}
  }
  return best;
}

static bool collides(const World & w, double x, double y, double r)
{
  for (const auto & q : w.b) {
    const double cx = std::max(q.x0, std::min(x, q.x1)), cy = std::max(q.y0, std::min(y, q.y1));
    if (std::hypot(x - cx, y - cy) < r) {return true;}
  }
  return false;
}

static bool g_verbose = false;

struct Outcome {bool done = false; bool ok = false; std::string status, msg; double x = 0, y = 0, yaw = 0, t = 0; int collisions = 0;};

// jeden beh skillu v svete w; robot zacina v (x0, y0, yaw0)
static Outcome run(const World & w, double x0, double y0, double yaw0, const std::string & skill, double value, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.01);
  Params p;
  SkillExecutor ex(p);
  LidarParams lp;
  lp.flip = false; lp.mount_pitch = 0.0; lp.up_x = 0; lp.up_y = 0; lp.up_z = 1;
  const double R = 0.30;
  double x = x0, y = y0, yaw = yaw0, vx = 0, vy = 0, wz = 0, t = 0;
  const double dt = 0.02;
  Outcome out;
  auto sense = [&]() {
    Sensors s;
    s.have_imu = true; s.gz = -1.0; s.yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    s.have_odom = true;
    const double c = std::cos(yaw0), sn = std::sin(yaw0);
    s.ox = 1.02 * (c * (x - x0) + sn * (y - y0));
    s.oy = 1.02 * (-sn * (x - x0) + c * (y - y0));
    s.oyaw = wrap_pi(yaw - yaw0);
    s.sim_t = t;
    LidarSectors ls;
    for (int a = 0; a < 360; ++a) {
      const double rel = (a - 180) * M_PI / 180.0;
      const double r = ray(w, x, y, yaw + rel, 10.0) + noise(rng);
      if (r > 0.05 && r < 10.0) {ls.add(r * std::cos(rel), r * std::sin(rel), 0.0, lp);}
    }
    s.have_lidar = ls.seen >= 50;
    s.lid_front = ls.front; s.lid_back = ls.back; s.lid_left = ls.left; s.lid_right = ls.right;
    s.bins = ls.bins;
    return s;
  };
  Sensors s = sense();
  ex.observe(s);
  Result res;
  if (!ex.start(skill, value, s, res)) {
    out.status = res.status; out.msg = res.message; out.x = x; out.y = y; out.yaw = yaw; out.done = true;
    return out;
  }
  for (int i = 0; i < 12000; ++i) {                       // 240 s
    s = sense();
    ex.observe(s);
    const Output o = ex.update(s, dt);
    const double cvx = o.publish_cmd ? o.vx : 0.0, cvy = o.publish_cmd ? o.vy : 0.0, cwz = o.publish_cmd ? o.wz : 0.0;
    const double a = dt / 0.25;                             // dobeh politiky chodze
    vx += (cvx - vx) * a; vy += (cvy - vy) * a; wz += (cwz - wz) * a;
    yaw += wz * dt;
    const double nx = x + (std::cos(yaw) * vx - std::sin(yaw) * vy) * dt, ny = y + (std::sin(yaw) * vx + std::cos(yaw) * vy) * dt;
    if (collides(w, nx, ny, R)) {
      if (!collides(w, x, y, R)) {++out.collisions;}
      vx = vy = 0;
    } else {
      x = nx; y = ny;
    }
    t += dt;
    if (g_verbose && i % 50 == 0) {std::printf("    t=%5.1f  x=%5.2f y=%5.2f yaw=%6.1f  %s\n", t, x, y, yaw * 180 / M_PI, o.state.c_str());}
    if (o.finished) {
      out.done = true; out.ok = o.result.success; out.status = o.result.status; out.msg = o.result.message;
      out.x = x; out.y = y; out.yaw = yaw; out.t = t;
      return out;
    }
  }
  out.status = "sim_timeout"; out.x = x; out.y = y; out.yaw = yaw; out.t = t; out.done = false;
  return out;
}

// obvodove steny: miestnost x -2..12, y -5..5
static World room()
{
  World w;
  w.add(-2.5, -5.5, 12.5, -5.0); w.add(-2.5, 5.0, 12.5, 5.5); w.add(-2.5, -5.5, -2.0, 5.5); w.add(12.0, -5.5, 12.5, 5.5);
  return w;
}

// percento na pravitku obrazka pre predmet v smere bearing [rad] (+ = vlavo); inverzia vzorca z skill_core.hpp
static double percent_for(double bearing)
{
  const double hf = 80.0 * M_PI / 180.0;
  return 50.0 - 50.0 * std::tan(bearing) / std::tan(hf / 2.0);
}

struct Test {std::string name; bool pass; std::string detail;};
static std::vector<Test> g_tests;

static void report(const std::string & name, bool pass, const std::string & detail)
{
  g_tests.push_back({name, pass, detail});
  std::printf("%s  %-52s %s\n", pass ? "OK    " : "CHYBA ", name.c_str(), detail.c_str());
}

static std::string fmt(const char * f, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0)
{
  char buf[200];
  std::snprintf(buf, sizeof(buf), f, a, b, c, d, e);
  return buf;
}

// ---- walk_to_object
static void t_toobj(const std::string & name, double ox, double oy, double sx, double sy, double syaw, double size, unsigned seed, bool expect_ok = true)
{
  World w = room();
  w.add(ox - size / 2, oy - size / 2, ox + size / 2, oy + size / 2);
  const double bearing = wrap_pi(std::atan2(oy - sy, ox - sx) - syaw);
  const Outcome r = run(w, sx, sy, syaw, "walk_to_object", percent_for(bearing), seed);
  // vzdialenost od predmetu v smere pohladu robota
  const double d = ray(w, r.x, r.y, r.yaw, 10.0);
  const double herr = std::fabs(wrap_pi(std::atan2(oy - r.y, ox - r.x) - r.yaw)) * 180 / M_PI;
  bool pass;
  if (expect_ok) {pass = r.ok && r.collisions == 0 && d > 0.45 && d < 1.0 && herr < 12.0;}
  else {pass = !r.ok && r.collisions == 0;}
  report(name, pass, expect_ok ? fmt("odstup %.2f m, smer %.0f st., %.0f s", d, herr, r.t) + " [" + r.status + "]" : "[" + r.status + "]");
  if (!pass && g_verbose) {std::printf("      %s\n", r.msg.c_str());}
}

// ---- go_around_obstacle
static void t_around(const std::string & name, World w, double sx, double sy, double syaw, double value, double back_x, bool expect_ok, const std::string & status, unsigned seed)
{
  const Outcome r = run(w, sx, sy, syaw, "go_around_obstacle", value, seed);
  bool pass;
  if (expect_ok) {const double along = std::cos(syaw) * (r.x - sx) + std::sin(syaw) * (r.y - sy), off = -std::sin(syaw) * (r.x - sx) + std::cos(syaw) * (r.y - sy);
    pass = r.ok && r.collisions == 0 && along > back_x && std::fabs(off) < 0.3 && std::fabs(wrap_pi(r.yaw - syaw)) < 0.25;}
  else {pass = !r.ok && r.collisions == 0 && (status.empty() || r.status == status);}
  report(name, pass, fmt("x=%.2f y=%.2f kurz %.0f st., %.0f s, kolizie %.0f", r.x, r.y, (r.yaw - syaw) * 180 / M_PI, r.t, r.collisions) + " [" + r.status + "]");
  if (!pass && g_verbose) {std::printf("      %s\n", r.msg.c_str());}
}

// ---- follow_wall
static void t_follow(const std::string & name, World w, double sx, double sy, double syaw, double value, double end_x_lo, double end_x_hi,
  bool expect_ok, const std::string & status, unsigned seed)
{
  const Outcome r = run(w, sx, sy, syaw, "follow_wall", value, seed);
  bool pass;
  if (expect_ok) {pass = r.ok && r.collisions == 0 && r.x >= end_x_lo && r.x <= end_x_hi;}
  else {pass = !r.ok && r.collisions == 0 && (status.empty() || r.status == status);}
  report(name, pass, fmt("x=%.2f y=%.2f kurz %.0f st., %.0f s, kolizie %.0f", r.x, r.y, (r.yaw - syaw) * 180 / M_PI, r.t, r.collisions) + " [" + r.status + "]");
  if ((!pass || g_verbose) && !r.msg.empty()) {std::printf("      %s\n", r.msg.c_str());}
}

static int fuzz(int n)
{
  std::mt19937 rng(std::getenv("FUZZ_SEED") ? std::atoi(std::getenv("FUZZ_SEED")) : 777);
  auto U = [&](double a, double b) {return std::uniform_real_distribution<double>(a, b)(rng);};
  int ok_obj = 0, bad_obj = 0, ok_ar = 0, bad_ar = 0, ok_fw = 0, bad_fw = 0, coll = 0;
  for (int i = 0; i < n; ++i) {
    {   // walk_to_object: predmet 0.3-0.7 m v dosahu 2-7 m a v zornom poli +-35 st.
      const double sx = U(-1, 2), sy = U(-3, 3), syaw = U(-0.5, 0.5);
      const double dist = U(2.0, 7.0), br = U(-35, 35) * M_PI / 180.0;
      const double ox = sx + dist * std::cos(syaw + br), oy = sy + dist * std::sin(syaw + br), sz = U(0.3, 0.7);
      if (std::fabs(oy) < 4.3 && ox < 11.0) {
        World w = room();
        w.add(ox - sz / 2, oy - sz / 2, ox + sz / 2, oy + sz / 2);
        const Outcome r = run(w, sx, sy, syaw, "walk_to_object", percent_for(br), 100 + i);
        const double d = ray(w, r.x, r.y, r.yaw, 10.0);
        if (r.collisions) {++coll;}
        if (r.ok && !r.collisions && d > 0.45 && d < 1.0) {++ok_obj;}
        else {++bad_obj; std::printf("  walk_to_object #%d: start (%.1f,%.1f,%.2f) objekt (%.1f,%.1f) %.1f -> %s d=%.2f\n", i, sx, sy, syaw, ox, oy, sz, r.status.c_str(), d);}
      }
    }
    {   // go_around_obstacle: debna 0.3-1.4 m siroka 0.7-2.5 m pred robotom
      const double w_ = U(0.3, 1.4), dep = U(0.3, 1.0), oy = U(-0.4, 0.4), dist = U(1.0, 2.2);
      World w = room();
      w.add(dist, oy - w_ / 2, dist + dep, oy + w_ / 2);
      const Outcome r = run(w, 0.0, 0.0, 0.0, "go_around_obstacle", std::max(2.0, dist + dep + 0.8), 300 + i);
      if (r.collisions) {++coll;}
      if (r.ok && !r.collisions && r.x > dist + dep && std::fabs(r.y) < 0.3) {++ok_ar;}
      else {++bad_ar; std::printf("  go_around #%d: debna x=%.1f w=%.1f d=%.1f y=%.1f -> %s end (%.2f,%.2f)\n", i, dist, w_, dep, oy, r.status.c_str(), r.x, r.y);}
    }
    {   // follow_wall: rovna stena 0.8-1.4 m od robota, mierne natocenie robota, stena konci
      const double wd = U(0.8, 1.4), len = U(3.0, 7.0), syaw = U(-0.25, 0.25);
      World w = room();
      w.add(-1.9, wd, len, wd + 0.2);                      // stena vlavo od robota, konci v x = len
      const Outcome r = run(w, 0.0, 0.0, syaw, "follow_wall", 12.0, 500 + i);
      if (r.collisions) {++coll;}
      if (r.ok && !r.collisions && r.x > len - 1.2 && r.x < len + 1.2) {++ok_fw;}
      else {++bad_fw; std::printf("  follow_wall #%d: d=%.1f dlzka=%.1f yaw=%.2f -> %s end (%.2f,%.2f)\n", i, wd, len, syaw, r.status.c_str(), r.x, r.y);}
    }
  }
  std::printf("fuzz %d: walk_to_object %d OK / %d zle, go_around %d OK / %d zle, follow_wall %d OK / %d zle, kolizie %d\n", n, ok_obj, bad_obj, ok_ar, bad_ar, ok_fw, bad_fw, coll);
  return (bad_obj + bad_ar + bad_fw + coll) == 0 ? 0 : 1;
}

int main(int argc, char ** argv)
{
  int nfuzz = 0;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "-v")) {g_verbose = true;}
    if (!std::strcmp(argv[i], "--fuzz") && i + 1 < argc) {nfuzz = std::atoi(argv[++i]);}
  }
  if (nfuzz > 0) {return fuzz(nfuzz);}

  std::printf("== walk_to_object ==\n");
  t_toobj("predmet rovno pred robotom, 4 m", 4.0, 0.0, 0, 0, 0, 0.5, 1);
  t_toobj("predmet 20 st. vlavo, 5 m", 5.0 * std::cos(0.35), 5.0 * std::sin(0.35), 0, 0, 0, 0.5, 2);
  t_toobj("predmet 30 st. vpravo, 3 m", 3.0 * std::cos(-0.52), 3.0 * std::sin(-0.52), 0, 0, 0, 0.4, 3);
  t_toobj("robot natoceny 0.4 rad, predmet vzadu vlavo", 3.0, 2.5, 0, 0, 0.4, 0.5, 4);
  t_toobj("uzky stlp 0.15 m, 4 m", 4.0, 0.3, 0, 0, 0, 0.15, 5);
  t_toobj("blizko (1.5 m)", 1.5, 0.0, 0, 0, 0, 0.5, 6);
  {
    World w = room();                                  // percento ukazuje na prazdno (nic na LiDARe)
    const Outcome r = run(w, 0, 0, 0, "walk_to_object", 50, 7);
    // 12 m dozadu je stena v 12 m -> mimo 8 m dosahu: no_object
    report("prazdny priestor -> no_object", !r.ok && r.status == "no_object" && r.collisions == 0, "[" + r.status + "]");
  }
  {
    World w = room();                                  // kriva hodnota mimo limitov
    const Outcome r = run(w, 0, 0, 0, "walk_to_object", 150, 8);
    report("hodnota 150 % je odmietnuta", !r.ok && r.status == "out_of_range", "[" + r.status + "]");
  }

  std::printf("== go_around_obstacle ==\n");
  {
    World w = room(); w.add(2.5, -0.3, 3.0, 0.3);
    t_around("debna 0.5x0.6 rovno pred robotom (0.7 m)", w, 1.8, 0, 0, 2.0, 1.6, true, "", 11);
  }
  {
    World w = room(); w.add(2.5, -0.3, 3.0, 0.3);
    t_around("ta ista debna zo 2.5 m", w, 0.0, 0, 0, 3.0, 3.6, true, "", 12);
  }
  {
    World w = room(); w.add(2.5, -0.1, 3.5, 1.4);    // viac vlavo: obchadzat vpravo
    t_around("debna posunuta vlavo -> ide vpravo", w, 1.8, 0, 0, 2.0, 1.9, true, "", 13);
  }
  {
    World w = room(); w.add(2.5, -1.4, 3.5, 0.1);    // viac vpravo: obchadzat vlavo
    t_around("debna posunuta vpravo -> ide vlavo", w, 1.8, 0, 0, 2.0, 1.9, true, "", 14);
  }
  {
    World w = room(); w.add(2.5, -0.15, 2.8, 0.15);
    t_around("tenky stlp 0.3 m", w, 1.8, 0, 0, 2.0, 1.6, true, "", 15);
  }
  {
    World w = room(); w.add(2.5, -0.5, 4.5, 0.5);
    t_around("hlboka prekazka 2 m", w, 1.8, 0, 0, 2.0, 3.0, true, "", 16);
  }
  {
    World w = room(); w.add(2.5, -4.9, 2.8, 4.9);    // dlha stena
    t_around("dlha stena -> no_room", w, 1.8, 0, 0, 2.0, 0, false, "no_room", 17);
  }
  {
    World w = room();
    t_around("nic pred robotom -> no_obstacle", w, 0.0, 0, 0, 2.0, 0, false, "no_obstacle", 18);
  }
  {
    World w = room(); w.add(2.5, -0.3, 3.0, 0.3); w.add(1.0, 0.9, 4.5, 1.1);   // stena vlavo vedla cesty
    t_around("stena vlavo -> obchadza vpravo", w, 1.8, 0, 0, 2.0, 1.6, true, "", 19);
  }
  {
    World w = room(); w.add(2.45, 0.06, 2.95, 0.56);          // lezi na ciare z (1.6, 0) pod uhlom 0.3 rad
    t_around("robot natoceny 0.3 rad", w, 1.6, 0, 0.3, 2.0, 1.9, true, "", 20);
  }

  std::printf("== follow_wall ==\n");
  {
    World w = room(); w.add(-1.9, 1.0, 6.0, 1.2);
    t_follow("stena vlavo 1.0 m, konci v x=6", w, 0, 0, 0, 12.0, 5.0, 7.0, true, "", 21);
  }
  {
    World w = room(); w.add(-1.9, -1.2, 6.0, -1.0);
    t_follow("stena vpravo 1.0 m, konci v x=6", w, 0, 0, 0, 12.0, 5.0, 7.0, true, "", 22);
  }
  {
    World w = room(); w.add(-1.9, 1.0, 6.0, 1.2);
    t_follow("robot natoceny 0.2 rad k stene", w, 0, 0, 0.2, 12.0, 5.0, 7.0, true, "", 23);
  }
  {
    World w = room(); w.add(-1.9, 1.0, 6.0, 1.2);
    t_follow("robot natoceny -0.2 rad od steny", w, 0, 0, -0.2, 12.0, 5.0, 7.0, true, "", 24);
  }
  {
    World w = room(); w.add(-1.9, 0.6, 6.0, 0.8);
    t_follow("stena blizko (0.6 m)", w, 0, 0, 0, 12.0, 5.0, 7.0, true, "", 25);
  }
  {
    World w = room(); w.add(-1.9, 1.0, 12.0, 1.2); w.add(6.0, -5.0, 6.2, 1.2);   // roh: kolma stena v x=6
    t_follow("stena konci rohom (kolma stena v x=6)", w, 0, 0, 0, 12.0, 4.5, 6.0, true, "", 26);
  }
  {
    World w = room(); w.add(-1.9, 1.0, 12.0, 1.2);
    t_follow("limit dlzky 3 m", w, 0, 0, 0, 3.0, 2.5, 3.8, true, "", 27);
  }
  {
    World w = room();                                   // zadna stena v dosahu 1.8 m (obvodove steny su 5 m)
    t_follow("ziadna stena na boku -> no_wall", w, 0, 0, 0, 5.0, 0, 0, false, "no_wall", 28);
  }
  {
    World w = room(); w.add(4.0, -1.5, 4.2, 1.5);       // stena rovno pred robotom, nie na boku
    t_follow("stena len pred robotom -> no_wall", w, 0, 0, 0, 5.0, 0, 0, false, "no_wall", 29);
  }

  int bad = 0;
  for (const auto & t : g_tests) {bad += t.pass ? 0 : 1;}
  std::printf("\n%d z %zu testov OK\n", (int)g_tests.size() - bad, g_tests.size());
  return bad == 0 ? 0 : 1;
}
