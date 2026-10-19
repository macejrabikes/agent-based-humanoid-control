// GetupPolicy: politika "postav sa zo zeme" bez LibTorch.
// Siet (mjlab): (obs - mean) / (std + 1e-8) -> Linear(93,512) ELU -> Linear(512,256) ELU
//               -> Linear(256,128) ELU -> Linear(128,29). Bez pamate (kazdy krok je nezavisly).
// Vahy citam z binarneho suboru getup_policy.bin (magic "GUPL", dims, mean, std, W0 b0 ... W3 b3).
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

class GetupPolicy
{
public:
  static constexpr int kObs = 93;
  static constexpr int kH1 = 512;
  static constexpr int kH2 = 256;
  static constexpr int kH3 = 128;
  static constexpr int kAct = 29;

  void load(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("Nemozem otvorit subor s vahami vstavania: " + path);
    }
    char magic[4];
    int32_t dims[5];
    f.read(magic, 4);
    f.read(reinterpret_cast<char *>(dims), sizeof(dims));
    if (!f || std::memcmp(magic, "GUPL", 4) != 0 || dims[0] != kObs || dims[1] != kH1 ||
      dims[2] != kH2 || dims[3] != kH3 || dims[4] != kAct)
    {
      throw std::runtime_error("Subor s vahami vstavania ma zly format: " + path);
    }
    read(f, mean_, kObs);
    read(f, std_, kObs);
    read(f, w0_, kH1 * kObs); read(f, b0_, kH1);
    read(f, w1_, kH2 * kH1); read(f, b1_, kH2);
    read(f, w2_, kH3 * kH2); read(f, b2_, kH3);
    read(f, w3_, kAct * kH3); read(f, b3_, kAct);
    inv_std_.resize(kObs);
    for (int i = 0; i < kObs; ++i) {inv_std_[i] = 1.0f / (std_[i] + 1e-8f);}
    loaded_ = true;
  }

  bool loaded() const {return loaded_;}

  // obs: 93 hodnot (surove, normalizaciu robi politika), action: 29 hodnot (surovy vystup)
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
      throw std::runtime_error("Subor s vahami vstavania je kratky alebo poskodeny");
    }
  }

  std::vector<float> mean_, std_, inv_std_, w0_, b0_, w1_, b1_, w2_, b2_, w3_, b3_;
  bool loaded_{false};
};

}  // namespace humanoid_walk
