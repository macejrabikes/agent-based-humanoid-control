// WalkMjlab: politika "stoj a chod" natrenovana v mjlab (12 klbov nohy), bez LibTorch.
// Siet: (obs - mean) / (std + 0.01) -> Linear(45,512) ELU -> Linear(512,256) ELU -> Linear(256,128) ELU -> Linear(128,12).
// Vahy: binarny subor walk_mjlab.bin (magic "GUPL", dims, mean, std, W0 b0 ... W3 b3).
//
// Pozorovanie (45): gyro v suradniciach panvy [rad/s] (3), gravitacia - jednotkovy vektor (3, vzpriamene 0,0,-1),
//   prikaz [vx, vy, wz] (3), q - default (12), dq [rad/s] (12), posledna surova akcia (12). Nic sa nenasobi.
// Ciel kĺbu: q_target = default + action_scale * action  (potom PD v regulatore, kp/kd v controllers.yaml).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace humanoid_walk
{
namespace walk_mjlab
{
constexpr int kObs = 45;
constexpr int kAct = 12;
constexpr float kDefault[kAct] = {
  -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f, -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f};
constexpr float kScale[kAct] = {
  0.54755f, 0.35066f, 0.54755f, 0.35066f, 0.43858f, 0.43858f,
  0.54755f, 0.35066f, 0.54755f, 0.35066f, 0.43858f, 0.43858f};
// rozsah prikazov, na ktory bola politika natrenovana
constexpr double kCmdMin[3] = {-0.4, -0.4, -0.8};
constexpr double kCmdMax[3] = {0.8, 0.4, 0.8};
}  // namespace walk_mjlab

class WalkMjlab
{
public:
  static constexpr int kObs = walk_mjlab::kObs;
  static constexpr int kH1 = 512;
  static constexpr int kH2 = 256;
  static constexpr int kH3 = 128;
  static constexpr int kAct = walk_mjlab::kAct;

  void load(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("Nemozem otvorit subor s vahami chodze (mjlab): " + path);
    }
    char magic[4];
    int32_t dims[5];
    f.read(magic, 4);
    f.read(reinterpret_cast<char *>(dims), sizeof(dims));
    if (!f || std::memcmp(magic, "GUPL", 4) != 0 || dims[0] != kObs || dims[1] != kH1 ||
      dims[2] != kH2 || dims[3] != kH3 || dims[4] != kAct)
    {
      throw std::runtime_error("Subor s vahami chodze (mjlab) ma zly format: " + path);
    }
    read(f, mean_, kObs);
    read(f, std_, kObs);
    read(f, w0_, kH1 * kObs); read(f, b0_, kH1);
    read(f, w1_, kH2 * kH1); read(f, b1_, kH2);
    read(f, w2_, kH3 * kH2); read(f, b2_, kH3);
    read(f, w3_, kAct * kH3); read(f, b3_, kAct);
    inv_std_.resize(kObs);
    for (int i = 0; i < kObs; ++i) {inv_std_[i] = 1.0f / (std_[i] + 0.01f);}
    loaded_ = true;
  }

  bool loaded() const {return loaded_;}

  // obs: 45 surovych hodnot (normalizaciu robi politika), action: 12 hodnot (surovy vystup)
  void forward(const float * obs, float * action) const
  {
    float x[kObs], a[kH1], b[kH2], c[kH3];
    for (int i = 0; i < kObs; ++i) {x[i] = (obs[i] - mean_[i]) * inv_std_[i];}
    layer(w0_, b0_, x, kObs, a, kH1, true);
    layer(w1_, b1_, a, kH1, b, kH2, true);
    layer(w2_, b2_, b, kH2, c, kH3, true);
    layer(w3_, b3_, c, kH3, action, kAct, false);
  }

private:
  static void layer(
    const std::vector<float> & w, const std::vector<float> & b, const float * in, int n_in,
    float * out, int n_out, bool elu)
  {
    for (int r = 0; r < n_out; ++r) {
      const float * wr = &w[static_cast<size_t>(r) * n_in];
      float s = b[r];
      for (int k = 0; k < n_in; ++k) {s += wr[k] * in[k];}
      out[r] = (elu && s < 0.0f) ? std::expm1(s) : s;
    }
  }

  static void read(std::ifstream & f, std::vector<float> & v, size_t n)
  {
    v.resize(n);
    f.read(reinterpret_cast<char *>(v.data()), static_cast<std::streamsize>(n * sizeof(float)));
    if (!f) {
      throw std::runtime_error("Subor s vahami chodze (mjlab) je kratky alebo poskodeny");
    }
  }

  std::vector<float> mean_, std_, inv_std_, w0_, b0_, w1_, b1_, w2_, b2_, w3_, b3_;
  bool loaded_{false};
};

}  // namespace humanoid_walk
