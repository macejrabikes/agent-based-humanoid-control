// Overenie: C++ politika vs. referencne vystupy z PyTorchu.
//   ros2 run humanoid_walk g1_policy_check <g1_policy.bin> <g1_policy_ref.txt>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "g1_policy.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "pouzitie: g1_policy_check <g1_policy.bin> <g1_policy_ref.txt>\n");
    return 2;
  }
  humanoid_walk::G1Policy p;
  p.load(argv[1]);
  std::ifstream f(argv[2]);
  std::string line;
  int n = 0;
  double worst = 0.0;
  while (std::getline(f, line)) {
    std::istringstream in(line);
    float x[47], y[12], out[12];
    for (float & v : x) {in >> v;}
    for (float & v : y) {in >> v;}
    p.forward(x, out);
    for (int k = 0; k < 12; ++k) {
      worst = std::fmax(worst, std::fabs(static_cast<double>(out[k]) - y[k]));
    }
    ++n;
  }
  std::printf("krokov: %d, najvacsi rozdiel proti PyTorchu: %.3g -> %s\n", n, worst,
    (n > 0 && worst < 1e-4) ? "OK" : "CHYBA");
  return (n > 0 && worst < 1e-4) ? 0 : 1;
}
