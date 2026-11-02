// test_door_sim.cpp - 2D simulator (bez ROS a Gazeba) na test skillu go_through_opening.
//
// Svet = obdlzniky (steny), robot = kruh s polomerom 0.3 m, LiDAR = ray-casting po 1 stupni, odometria v inom
// suradnicovom systeme ako svet (pre test prepoctov). Pouziva SKUTOCNY SkillExecutor zo skill_core.hpp.
//
// Spustenie (z korena repozitara):
//   g++ -std=c++17 -O1 -Wall -Wextra -I src/humanoid_skills/include tools/test_door_sim.cpp -o /tmp/test_door && /tmp/test_door
// Dalsie rezimy:  --fuzz N  (N nahodnych scenarov; FUZZ_SEED=n zmeni nahodu, FUZZ_DUMP=i vypise sken i-teho scenara)   --one x y yaw door_y sirka hrubka [value]  (jeden scenar s vypisom LiDAR skenu)
// Vystup: tabulka scenarov (OK/CHYBA), navratovy kod 0 = vsetky scenare dopadli podla ocakavania.
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// vzdialenost k prvej stene v smere (ang) z bodu (ox, oy); slab method
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

struct Scenario
{
  std::string name;
  double x, y, yaw;              // start robota vo svete
  double door_y, door_w, thick;  // dvere v stene x = 4
  bool has_door;
  bool two_doors;                // druhe dvere na y = -door_y (rovnaka sirka)
  bool crate;                    // debna vedla cesty k dveram (neblokuje ju)
  bool expect_ok;                // ocakavany vysledok skillu
  std::string expect_status;
  std::vector<Box> extra;        // dalsie prekazky (stlpy, nabytok)
  double value = 6.0;            // parameter skillu: najvacsia vzdialenost otvoru [m]
  std::string skill = "go_through_opening";   // "search_room" = skill sa vola opakovane, kym neskonci stavom explored
};

static bool g_dump = false;

struct Outcome {bool ok; std::string status, msg; double x, y, t; int collisions;};

static Outcome run(const Scenario & sc, unsigned seed)
{
  World w;
  const double xw = 4.0;
  w.add(-6, -8.5, 14, -8);  w.add(-6, 8, 14, 8.5);  w.add(-6.5, -8.5, -6, 8.5);  w.add(14, -8.5, 14.5, 8.5);     // obvodove steny
  auto wall = [&](double yd, double width) {
    w.add(xw, -8, xw + sc.thick, yd - width / 2);
    w.add(xw, yd + width / 2, xw + sc.thick, 8);
  };
  if (!sc.has_door) {
    w.add(xw, -8, xw + sc.thick, 8);
  } else if (sc.two_doors) {
    w.add(xw, -8, xw + sc.thick, -sc.door_y - sc.door_w / 2);
    w.add(xw, -sc.door_y + sc.door_w / 2, xw + sc.thick, sc.door_y - sc.door_w / 2);
    w.add(xw, sc.door_y + sc.door_w / 2, xw + sc.thick, 8);
  } else {
    wall(sc.door_y, sc.door_w);
  }
  w.add(9, -3, 9.5, 3);          // nabytok v druhej izbe
  if (sc.crate) {w.add(2.0, 1.25, 2.6, 1.85);}
  for (const auto & q : sc.extra) {w.b.push_back(q);}

  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.01);
  Params p;
  SkillExecutor ex(p);
  LidarParams lp;
  lp.flip = false; lp.mount_pitch = 0.0; lp.up_x = 0; lp.up_y = 0; lp.up_z = 1;
  const double R = 0.30;
  double x = sc.x, y = sc.y, yaw = sc.yaw, vx = 0, vy = 0, wz = 0, t = 0;
  const double x0 = sc.x, y0 = sc.y, yaw0 = sc.yaw;        // odometria zacina v 0 a je otocena voci svetu
  const double dt = 0.02;
  int coll = 0;
  auto sense = [&]() {
    Sensors s;
    s.have_imu = true; s.gz = -1.0; s.yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    s.have_odom = true;
    const double c = std::cos(yaw0), sn = std::sin(yaw0);
    s.ox = 1.02 * (c * (x - x0) + sn * (y - y0));          // 2 % chyba mierky odometrie
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
  if (g_dump) {                                              // --one: vypis sken a najdene otvory (na ladenie)
    std::printf("sken (index vysecu: vzdialenost [m]; 90 = priamo dopredu, vyssie = vlavo):\n");
    for (int k = 30; k < 150; ++k) {std::printf("%d:%.1f ", k, s.bins.r[k]);}
    std::printf("\notvory:\n");
    for (const auto & op : find_openings(s.bins, 0.3, 8.0, 6.0, 110.0)) {
      std::printf("  sirka %.2f m, stred (%.2f, %.2f), %.1f m daleko, smer %.0f st.\n", op.width, op.cx, op.cy, op.dist, op.bearing);
    }
  }
  Outcome out{false, "", "", 0, 0, 0, 0};
  const bool room = sc.skill == "search_room";
  int hops = 0;
  bool last_explored = false;
  double min_hop = 99.0;
  for (int call = 0; call < (room ? 14 : 1); ++call) {
    Result res;
    if (!ex.start(sc.skill, sc.value, s, res)) {
      out.status = res.status; out.msg = res.message; out.x = x; out.y = y;
      return out;
    }
    const double hx = x, hy = y;
    bool fin = false;
    Output o;
    for (int i = 0; i < 12000 && !fin; ++i) {                          // 240 s
      s = sense();
      ex.observe(s);
      o = ex.update(s, dt);
      const double cvx = o.publish_cmd ? o.vx : 0.0, cvy = o.publish_cmd ? o.vy : 0.0, cwz = o.publish_cmd ? o.wz : 0.0;
      const double a = dt / 0.25;                              // dobeh politiky chodze
      vx += (cvx - vx) * a; vy += (cvy - vy) * a; wz += (cwz - wz) * a;
      yaw += wz * dt;
      const double nx = x + (std::cos(yaw) * vx - std::sin(yaw) * vy) * dt, ny = y + (std::sin(yaw) * vx + std::cos(yaw) * vy) * dt;
      if (collides(w, nx, ny, R)) {
        if (!collides(w, x, y, R)) {++coll;}                   // dotkol sa steny
        vx = vy = 0;
      } else {
        x = nx; y = ny;
      }
      t += dt;
      if (g_dump && i % 50 == 0) {std::printf("  t=%5.1f  x=%5.2f y=%5.2f yaw=%5.2f  %s\n", t, x, y, yaw, o.state.c_str());}
      if (o.finished) {fin = true;}
    }
    if (!fin) {
      out = {false, "sim_timeout", "skill sa neskoncil", x, y, t, coll};
      return out;
    }
    if (!room) {
      out = {o.result.success, o.result.status, o.result.message, x, y, t, coll};
      return out;
    }
    if (o.result.status == "explored") {last_explored = true; break;}
    if (!o.result.success) {out = {false, o.result.status, o.result.message, x, y, t, coll}; return out;}
    ++hops;
    min_hop = std::min(min_hop, std::hypot(x - hx, y - hy));
    if (g_dump) {std::printf("  hop %d -> (%.2f, %.2f)\n", hops, x, y);}
  }
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%d presunov, najkratsi %.1f m", hops, min_hop);
  out = {hops >= 3 && min_hop > 0.9 && last_explored, last_explored ? (hops >= 3 ? "ok" : "malo_presunov") : "neskoncilo", buf, x, y, t, coll};
  return out;
}

// nahodne scenare: nahodna poloha a smer robota, sirka a poloha dveri, hrubka steny, stlpy v izbe
static int fuzz(int n)
{
  std::mt19937 rng(std::getenv("FUZZ_SEED") ? std::atoi(std::getenv("FUZZ_SEED")) : 12345);
  auto U = [&](double a, double b) {return std::uniform_real_distribution<double>(a, b)(rng);};
  int ok = 0, fail_nc = 0, collided = 0, noop = 0, blocked = 0, other = 0;
  for (int i = 0; i < n; ++i) {
    Scenario q{"fuzz", U(-1.5, 3.0), U(-4.5, 4.5), 0, U(-3, 3), U(0.9, 2.5), (rng() % 2) ? 0.5 : 0.1, true, false, false, true, "ok", {}};
    if (std::hypot(4.0 - q.x, q.door_y - q.y) > 5.5) {--i; continue;}                 // dalej ako 5.5 m skill (value 6) nehlada
    if (std::fabs(q.door_y - q.y) > 1.2 * (4.0 - q.x)) {--i; continue;}               // dvere pod velmi ostrym uhlom (> ~50 st. od normaly steny) nie je vidiet
    const double bearing = U(-70, 70) * M_PI / 180.0;                        // kde je stred dveri voci smeru robota
    q.yaw = std::atan2(q.door_y - q.y, 4.0 - q.x) - bearing;
    for (int k = 0; k < 3; ++k) {                                              // 3 stlpy 0.4 m v izbe pred stenou
      const double px = U(-4, 3.2), py = U(-6, 6);
      if (std::hypot(px - q.x, py - q.y) < 1.2) {continue;}
      bool occl = false;                                                      // stlp nesmie zakryvat vyhlad na dvere
      for (double e = -0.5; e <= 0.5; e += 0.5) {
        const double tx = 4.0 - q.x, ty = q.door_y + e * q.door_w - q.y, L2 = tx * tx + ty * ty;
        const double u = std::max(0.0, std::min(1.0, ((px - q.x) * tx + (py - q.y) * ty) / L2));
        if (std::hypot(q.x + u * tx - px, q.y + u * ty - py) < 0.9) {occl = true;}
      }
      if (occl) {continue;}
      q.extra.push_back({px - 0.2, py - 0.2, px + 0.2, py + 0.2});
    }
    g_dump = std::getenv("FUZZ_DUMP") && std::atoi(std::getenv("FUZZ_DUMP")) == i;
    const Outcome r = run(q, 100 + i);
    if (r.collisions > 0) {++collided; std::printf("  kolizia #%d: start (%.1f, %.1f, %.2f) door y=%.1f w=%.1f thick=%.1f -> %s\n", i, q.x, q.y, q.yaw, q.door_y, q.door_w, q.thick, r.status.c_str());}
    if (r.status == "timeout" || r.status == "sim_timeout") {std::printf("  timeout #%d: start (%.1f, %.1f, %.2f) door y=%.1f w=%.1f thick=%.1f\n", i, q.x, q.y, q.yaw, q.door_y, q.door_w, q.thick);}
    if (r.ok && r.x > 4.0 + q.thick + 0.5) {++ok;}
    else if (r.status == "no_opening") {++noop;}
    else if (r.status == "blocked") {++blocked;}
    else {++other; std::printf("  ? %s: start (%.1f, %.1f, %.2f) door y=%.1f w=%.1f -> %s\n    %s\n", r.status.c_str(), q.x, q.y, q.yaw, q.door_y, q.door_w, r.status.c_str(), r.msg.c_str());}
    if (!r.ok && r.status == "blocked") {std::printf("  blocked: start (%.1f, %.1f, %.2f) door y=%.1f w=%.1f thick=%.1f\n    %s\n", q.x, q.y, q.yaw, q.door_y, q.door_w, q.thick, r.msg.c_str());}
    if (!r.ok && r.status == "no_opening") {std::printf("  no_opening #%d: start (%.1f, %.1f, %.2f) door y=%.1f w=%.1f pillars %zu\n", i, q.x, q.y, q.yaw, q.door_y, q.door_w, q.extra.size());}
  }
  (void)fail_nc;
  std::printf("fuzz %d: prejdene %d, no_opening %d, blocked %d, ine %d; kolizie so stenou: %d\n", n, ok, noop, blocked, other, collided);
  return collided == 0 && other == 0 ? 0 : 1;
}

int main(int argc, char ** argv)
{
  if (argc > 2 && std::string(argv[1]) == "--fuzz") {return fuzz(std::atoi(argv[2]));}
  if (argc > 7 && std::string(argv[1]) == "--one") {         // --one x y yaw door_y sirka hrubka: jeden scenar s vypisom skenu
    g_dump = true;
    const Scenario q{"jeden", std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6]),
      std::atof(argv[7]), true, false, false, true, "ok", {}, argc > 8 ? std::atof(argv[8]) : 6.0};
    const Outcome r = run(q, 1);
    std::printf("%s %s: x=%.2f y=%.2f cas %.1f s, kolizie %d\n  %s\n", r.ok ? "OK" : "CHYBA", r.status.c_str(), r.x, r.y, r.t, r.collisions, r.msg.c_str());
    return r.ok ? 0 : 1;
  }
  const bool verbose = argc > 1;
  std::vector<Scenario> sc = {
    //  nazov                               x     y    yaw    door_y w     thick door  two    crate  ok     status
    {"dvere rovno pred robotom",           0.0,  0.0,  0.0,   0.0, 1.5, 0.5, true, false, false, true,  "ok", {}},
    {"dvere 1.0 m (uzke)",                 0.0,  0.0,  0.0,   0.0, 1.0, 0.5, true, false, false, true,  "ok", {}},
    {"dvere 1.0 m, tenka stena",           0.0,  1.5, -0.3,   0.0, 1.0, 0.1, true, false, false, true,  "ok", {}},
    {"robot bokom a natoceny inak",        0.0,  2.5,  0.4,   0.0, 1.5, 0.5, true, false, false, true,  "ok", {}},
    {"sikmo zdola, natoceny na sever",     1.0, -3.0,  1.2,   0.0, 1.5, 0.5, true, false, false, true,  "ok", {}},
    {"dvere posunute (y = 1)",             0.0,  0.0,  0.0,   1.0, 1.1, 0.5, true, false, false, true,  "ok", {}},
    {"blizko steny (0.7 m)",               3.3,  0.0,  0.0,   0.0, 1.5, 0.5, true, false, false, true,  "ok", {}},
    {"dvoje dvere, vyberie jedny",         0.0,  0.5,  0.0,   2.5, 1.2, 0.5, true, true,  false, true,  "ok", {}},
    {"dvere 0.8 m (neprejdem)",            0.0,  0.0,  0.0,   0.0, 0.8, 0.5, true, false, false, false, "no_opening", {}},
    {"dvere 0.6 m (neprejdem)",            0.0,  0.0,  0.0,   0.0, 0.6, 0.5, true, false, false, false, "no_opening", {}},
    {"plna stena bez dveri",               0.0,  0.0,  0.0,   0.0, 1.5, 0.5, false, false, false, false, "no_opening", {}},
    {"dvere za chrbtom",                   0.0,  0.0,  3.14159, 0.0, 1.5, 0.5, true, false, false, true, "ok", {}},
    {"dve skrinky 0.9 m s medzerou 1.5 m (nie dvere)", -2.0, 0.0, 0.0, 0.0, 1.5, 0.5, false, false, false, false, "no_opening", {{0.5, -1.65, 1.4, -0.75}, {0.5, 0.75, 1.4, 1.65}}},
    {"dva tenke stlpy s medzerou (nie dvere)", -2.0, 0.0, 0.0, 0.0, 1.5, 0.5, false, false, false, false, "no_opening", {{0.5, -1.0, 0.9, -0.75}, {0.5, 0.75, 0.9, 1.0}}},
    {"dvere daleko (15 m), treba prejst izbou", -5.0, -6.0, 1.57, 6.0, 1.5, 0.5, true, false, false, true, "ok", {}, 8.0},
    {"dvere daleko, robot otoceny od nich",     -5.0,  3.0, 3.14159, -4.0, 1.5, 0.5, true, false, false, true, "ok", {}, 8.0},
    {"search_room: prehlada izbu (skrina uprostred)", -5.0, -6.0, 0.0, 6.0, 1.5, 0.5, true, false, false, true, "ok", {{-2.0, -1.0, -0.8, 1.0}}, 3.0, "search_room"},
    {"debna vedla cesty (neblokuje)",      0.0,  0.0,  0.0,   0.0, 1.5, 0.5, true, false, true,  true,  "ok", {}},
  };
  int bad = 0;
  std::printf("%-36s %-9s %-11s %6s %6s %6s %s\n", "scenar", "vysledok", "status", "x", "y", "cas", "kolizie");
  for (const auto & q : sc) {
    for (unsigned seed = 1; seed <= 3; ++seed) {
      const Outcome r = run(q, seed);
      bool good = (r.ok == q.expect_ok) && r.status == q.expect_status;
      if (q.expect_ok) {
        const double dy = q.two_doors ? std::min(std::fabs(r.y - q.door_y), std::fabs(r.y + q.door_y)) : std::fabs(r.y - q.door_y);
        good = good && r.collisions == 0 && (q.skill == "search_room" || (r.x > 4.0 + q.thick + 0.5 && dy < 0.3));
      }
      if (!good) {++bad;}
      std::printf("%-36s %-9s %-11s %6.2f %6.2f %5.1fs %d%s\n", (q.name + (seed == 1 ? "" : " #" + std::to_string(seed))).c_str(),
        good ? "OK" : "CHYBA", r.status.c_str(), r.x, r.y, r.t, r.collisions, (verbose || !good) ? ("\n    " + r.msg).c_str() : "");
    }
  }
  std::printf("\n%s (%d chyb)\n", bad == 0 ? "VSETKO V PORIADKU" : "SU CHYBY", bad);
  return bad == 0 ? 0 : 1;
}
