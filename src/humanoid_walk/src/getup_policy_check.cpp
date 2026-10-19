// Overenie: C++ politika vstavania vs. referencne vystupy z PyTorchu.
//   ros2 run humanoid_walk getup_policy_check <getup_policy.bin> <getup_policy_ref.txt>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "getup_policy.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "pouzitie: getup_policy_check <getup_policy.bin> <getup_policy_ref.txt>\n");
    return 2;
  }
  humanoid_walk::GetupPolicy p;
  p.load(argv[1]);
  std::ifstream f(argv[2]);
  std::string line;
  int n = 0;
  double worst = 0.0;
  while (std::getline(f, line)) {
    std::istringstream in(line);
    float x[93], y[29], out[29];
    for (float & v : x) {in >> v;}
    for (float & v : y) {in >> v;}
    p.forward(x, out);
    for (int k = 0; k < 29; ++k) {
      worst = std::fmax(worst, std::fabs(static_cast<double>(out[k]) - y[k]));
    }
    ++n;
  }
  std::printf("vektorov: %d, najvacsi rozdiel proti PyTorchu: %.3g -> %s\n", n, worst,
    (n > 0 && worst < 1e-3) ? "OK" : "CHYBA");
  return (n > 0 && worst < 1e-3) ? 0 : 1;
}
