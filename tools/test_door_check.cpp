// Test door_check.hpp na simulovanom LiDARe (ray casting v 2D svete, bez ROS a Gazeba).
//   mini (bez workspace):  python3 ../make_mini_core.py && g++ -std=c++17 -O1 -Wall -Wextra -DMINI_CORE -I . -I .. test_door_check.cpp -o /tmp/test_dc && /tmp/test_dc
//   v workspace (po add_primitives.py): g++ -std=c++17 -O1 -Wall -Wextra -I src/humanoid_skills/include tools/test_door_check.cpp -o /tmp/test_dc && /tmp/test_dc
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#ifdef MINI_CORE
#include "mini_core.hpp"
namespace skills
{
#include "door_check.hpp"
}
#else
#include "humanoid_skills/skill_core.hpp"
#endif

using namespace skills;

struct Seg {double x1, y1, x2, y2;};
static std::vector<Seg> world;

static void add_rect(double x0, double x1, double y0, double y1)
{
  world.push_back({x0, y0, x1, y0}); world.push_back({x1, y0, x1, y1});
  world.push_back({x1, y1, x0, y1}); world.push_back({x0, y1, x0, y0});
}

// hala x -2..3, izba x 3.15..8, delicia stena hrubka 0.15 s dverami y -w/2..w/2 (stred dveri v y = door_y)
static void build(double door_w, double door_y)
{
  world.clear();
  add_rect(-2, 8, -3, 3);                                     // obvodove steny
  const double lo = door_y - door_w / 2, hi = door_y + door_w / 2;
  world.push_back({3.0, -3, 3.0, lo}); world.push_back({3.15, -3, 3.15, lo}); world.push_back({3.0, lo, 3.15, lo});
  world.push_back({3.0, hi, 3.0, 3}); world.push_back({3.15, hi, 3.15, 3}); world.push_back({3.0, hi, 3.15, hi});
}

static double cast(double ox, double oy, double dx, double dy)
{
  double best = 99.0;
  for (const auto & s : world) {
    const double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
    const double den = dx * ey - dy * ex;
    if (std::fabs(den) < 1e-9) {continue;}
    const double t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / den;
    const double u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
    if (t > 0.05 && u >= 0.0 && u <= 1.0 && t < best) {best = t;}
  }
  return best;
}

static LidarBins scan(double x, double y, double yaw)
{
  LidarBins b;
  for (int i = 0; i < kBins; ++i) {
    const double a = (-180.0 + i * kBinDeg) * M_PI / 180.0;
    const double d = cast(x, y, std::cos(yaw + a), std::sin(yaw + a));
    b.r[i] = d;
    if (d < 90.0) {b.x[i] = d * std::cos(a); b.y[i] = d * std::sin(a);}
  }
  return b;
}

static int bad = 0;
static void check(const char * name, bool ok, const std::string & extra = "")
{
  if (!ok) {++bad;}
  std::printf("%s  %s %s\n", ok ? "OK    " : "CHYBA ", name, extra.c_str());
}

static door_check::Report run(double x, double y, double yawdeg, door_check::Memory & mem)
{
  const double yaw = yawdeg * M_PI / 180.0;
  const LidarBins b = scan(x, y, yaw);
  return door_check::analyse(b, [](const LidarBins & bb, double fov) {return find_openings(bb, 0.9, 3.5, 6.0, fov);}, x, y, yaw, mem);
}

int main()
{
  build(1.5, 0.0);
  {
    door_check::Memory m;
    auto r = run(0.0, 0.0, 0.0, m);
    check("pred dverami: NOT_PASSED", r.verdict == "NOT_PASSED", door_check::text(r));
    check("  sirka ~1.5 m, vzdialenost ~3 m", r.ahead.found && std::fabs(r.ahead.width - 1.5) < 0.3 && std::fabs(r.ahead.dist - 3.1) < 0.5);
    check("  dvere sa zapamatali", m.valid && std::fabs(m.cx - 3.07) < 0.3 && std::fabs(m.cy) < 0.2 && m.nx > 0.9);
    r = run(1.8, 0.1, 5.0, m);
    check("blizsie, mierne bokom: stale NOT_PASSED", r.verdict == "NOT_PASSED", door_check::text(r));
    r = run(3.07, 0.0, 0.0, m);
    check("v dverach: IN_DOORWAY", r.verdict == "IN_DOORWAY", door_check::text(r));
    r = run(4.5, 0.1, 0.0, m);
    check("v izbe za dverami: PASSED", r.verdict == "PASSED", door_check::text(r));
    r = run(5.5, -0.3, -20.0, m);
    check("v izbe, natoceny inak: PASSED", r.verdict == "PASSED", door_check::text(r));
    r = run(4.5, 2.4, 0.0, m);
    check("za rovinou, ale ďaleko od otvoru: nie PASSED", r.verdict != "PASSED", door_check::text(r));
    r = run(0.0, 0.0, 180.0, m);
    check("v hale, otoceny chrbtom k dveram (pamat): NOT_PASSED", r.verdict == "NOT_PASSED", door_check::text(r));
  }
  {
    door_check::Memory m;
    auto r = run(0.0, 0.0, 180.0, m);
    check("bez pamate, dvere za chrbtom: UNSURE (nevie povedat)", r.verdict == "UNSURE" && r.behind.found, door_check::text(r));
    r = run(-1.2, 2.0, 90.0, m);
    check("pri stene bez dveri: NO_DOOR_SEEN alebo UNSURE, nie PASSED", r.verdict != "PASSED", door_check::text(r));
  }
  {
    door_check::Memory m;
    auto r = run(-1.0, -1.5, 45.0, m);
    check("dvere vpravo/vlavo vpredu (45 st.): najdene", r.ahead.found && r.verdict == "NOT_PASSED", door_check::text(r));
    r = run(4.2, 0.0, 0.0, m);
    check("  potom prechod: PASSED", r.verdict == "PASSED", door_check::text(r));
  }
  build(0.6, 0.0);
  {
    door_check::Memory m;
    auto r = run(0.0, 0.0, 0.0, m);
    check("uzka medzera 0.6 m (nie je prechodna): dvere sa nehlasia", !r.ahead.found && r.verdict != "PASSED", door_check::text(r));
  }
  build(1.5, 1.2);
  {
    door_check::Memory m;
    auto r = run(0.0, 0.0, 0.0, m);
    check("dvere posunute vlavo (y=1.2): najdene, bearing > 0", r.ahead.found && r.ahead.bearing > 10.0, door_check::text(r));
    r = run(4.5, 1.2, 0.0, m);
    check("  prechod cez ne: PASSED", r.verdict == "PASSED", door_check::text(r));
    r = run(4.5, -1.5, 0.0, m);
    check("  za stenou, ale bokom od dveri: nie PASSED", r.verdict != "PASSED", door_check::text(r));
  }
  std::printf("\n%s\n", bad == 0 ? "VYSLEDOK: vsetko OK" : "VYSLEDOK: SU CHYBY");
  return bad == 0 ? 0 : 1;
}
