// G1Policy: predtrenovana politika chodze Unitree G1 bez LibTorch.
// Siet: LSTM(47 -> 64), Linear(64 -> 32), ELU, Linear(32 -> 12); LSTM ma pamat (h, c).
// Vahy citam z binarneho suboru, ktory vytvara tools/export_g1_policy.py.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace humanoid_walk
{

class G1Policy
{
public:
  static constexpr int kObs = 47;
  static constexpr int kHidden = 64;
  static constexpr int kMid = 32;
  static constexpr int kAct = 12;

  void load(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("Nemozem otvorit subor s vahami: " + path);
    }
    char magic[4];
    int32_t dims[4];
    f.read(magic, 4);
    f.read(reinterpret_cast<char *>(dims), sizeof(dims));
    if (!f || std::memcmp(magic, "G1PL", 4) != 0 || dims[0] != kObs || dims[1] != kHidden ||
      dims[2] != kMid || dims[3] != kAct)
    {
      throw std::runtime_error("Subor s vahami ma zly format: " + path);
    }
    read(f, w_ih_, 4 * kHidden * kObs);
    read(f, w_hh_, 4 * kHidden * kHidden);
    read(f, b_ih_, 4 * kHidden);
    read(f, b_hh_, 4 * kHidden);
    read(f, w1_, kMid * kHidden);
    read(f, b1_, kMid);
    read(f, w2_, kAct * kMid);
    read(f, b2_, kAct);
    reset();
  }

  // vynuluje pamat LSTM (volat pri starte chodze a po padu)
  void reset()
  {
    h_.fill(0.0f);
    c_.fill(0.0f);
  }

  // obs: 47 hodnot, action: 12 hodnot
  void forward(const float * obs, float * action)
  {
    float gates[4 * kHidden];
    for (int r = 0; r < 4 * kHidden; ++r) {
      float s = b_ih_[r] + b_hh_[r];
      const float * wi = &w_ih_[r * kObs];
      for (int k = 0; k < kObs; ++k) {
        s += wi[k] * obs[k];
      }
      const float * wh = &w_hh_[r * kHidden];
      for (int k = 0; k < kHidden; ++k) {
        s += wh[k] * h_[k];
      }
      gates[r] = s;
    }
    // poradie brany v PyTorchu: i, f, g, o
    for (int k = 0; k < kHidden; ++k) {
      const float i = sigmoid(gates[k]);
      const float f = sigmoid(gates[kHidden + k]);
      const float g = std::tanh(gates[2 * kHidden + k]);
      const float o = sigmoid(gates[3 * kHidden + k]);
      c_[k] = f * c_[k] + i * g;
      h_[k] = o * std::tanh(c_[k]);
    }
    float mid[kMid];
    for (int r = 0; r < kMid; ++r) {
      float s = b1_[r];
      const float * w = &w1_[r * kHidden];
      for (int k = 0; k < kHidden; ++k) {
        s += w[k] * h_[k];
      }
      mid[r] = s > 0.0f ? s : std::expm1(s);   // ELU, alpha = 1
    }
    for (int r = 0; r < kAct; ++r) {
      float s = b2_[r];
      const float * w = &w2_[r * kMid];
      for (int k = 0; k < kMid; ++k) {
        s += w[k] * mid[k];
      }
      action[r] = s;
    }
  }

private:
  static float sigmoid(float x) {return 1.0f / (1.0f + std::exp(-x));}

  static void read(std::ifstream & f, std::vector<float> & v, size_t n)
  {
    v.resize(n);
    f.read(reinterpret_cast<char *>(v.data()), static_cast<std::streamsize>(n * sizeof(float)));
    if (!f) {
      throw std::runtime_error("Subor s vahami je kratky alebo poskodeny");
    }
  }

  std::vector<float> w_ih_, w_hh_, b_ih_, b_hh_, w1_, b1_, w2_, b2_;
  std::array<float, kHidden> h_{}, c_{};
};

}  // namespace humanoid_walk
