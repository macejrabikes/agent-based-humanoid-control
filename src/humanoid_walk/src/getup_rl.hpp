// GetupRl: vstavanie zo zeme politikou natrenovanou v mjlab (Unitree G1, 29 klbov).
// Cista logika bez ROS: dostane gravitaciu a uhlovu rychlost v suradniciach panvy, polohy a rychlosti
// 29 klbov (v poradi politiky) a vrati cielove polohy klbov. Rozhoduje aj o tom, ci robot uz stoji.
//
// Pozorovanie (93): base_ang_vel(3), projected_gravity(3), q - default (29), dq (29), last_action (29).
// Cielova poloha: q_target = default + scale * action (PD s kp, kd z getup_rl_params.hpp riesi regulator).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "getup_policy.hpp"
#include "getup_rl_params.hpp"

namespace humanoid_walk
{

class GetupRl
{
public:
  static constexpr int kN = getup_params::kJoints;
  enum class Status {kRunning, kStanding, kFailed};

  struct Config
  {
    double dt{0.02};            // perioda politiky [s]
    double min_time{1.0};       // najskor po tejto dobe sa moze vyhlasit "stoji" [s]
    double stable_time{0.4};    // tak dlho musi stat vzpriamene [s]
    double max_time{6.0};       // po tejto dobe sa vstavanie vzdava [s]
    double up_min{0.95};        // minimalna vzpriamenost trupu (-gz)
    double knee_max{1.3};       // oba kolena menej ohnute nez toto [rad] (nie je to klaci/drep)
  };

  void load(const std::string & weights) {policy_.load(weights);}
  bool loaded() const {return policy_.loaded();}
  Config & config() {return cfg_;}

  void start()
  {
    last_action_.fill(0.0f);
    targets_ = getup_params::kDefault;
    ticks_ = 0;
    stable_ticks_ = 0;
    bad_ticks_ = 0;
    status_ = Status::kRunning;
  }

  // gravity, omega: v suradniciach panvy; q, dq: v poradi politiky
  Status step(
    const std::array<double, 3> & gravity, const std::array<double, 3> & omega,
    const std::array<float, kN> & q, const std::array<float, kN> & dq)
  {
    float obs[getup_params::kObsDim];
    for (int i = 0; i < 3; ++i) {
      obs[i] = static_cast<float>(omega[i]);
      obs[3 + i] = static_cast<float>(gravity[i]);
    }
    for (int i = 0; i < kN; ++i) {
      obs[6 + i] = q[i] - getup_params::kDefault[i];
      obs[6 + kN + i] = dq[i];
      obs[6 + 2 * kN + i] = last_action_[i];
    }
    float act[kN];
    policy_.forward(obs, act);
    bool finite = true;
    for (int i = 0; i < kN; ++i) {finite = finite && std::isfinite(act[i]);}
    if (!finite) {
      if (++bad_ticks_ > 5) {status_ = Status::kFailed;}
      return status_;                       // povodne ciele ostavaju
    }
    bad_ticks_ = 0;
    for (int i = 0; i < kN; ++i) {
      last_action_[i] = act[i];
      targets_[i] = getup_params::kDefault[i] + getup_params::kScale[i] * act[i];
    }
    ++ticks_;
    const double t = ticks_ * cfg_.dt;

    // stoji? vzpriameny trup a vystrete kolena (politika v stoji este trochu "tancuje", preto
    // nepozadujem pokoj; kolena indexy 3 a 9 = left_knee, right_knee)
    const bool upright = (-gravity[2] > cfg_.up_min) && q[3] < cfg_.knee_max && q[9] < cfg_.knee_max;
    stable_ticks_ = upright ? stable_ticks_ + 1 : 0;
    if (status_ == Status::kRunning) {
      if (t >= cfg_.min_time && stable_ticks_ * cfg_.dt >= cfg_.stable_time) {
        status_ = Status::kStanding;
      } else if (t >= cfg_.max_time) {
        status_ = Status::kFailed;
      }
    }
    return status_;
  }

  const std::array<float, kN> & targets() const {return targets_;}
  Status status() const {return status_;}
  double time() const {return ticks_ * cfg_.dt;}

private:
  GetupPolicy policy_;
  Config cfg_;
  std::array<float, kN> last_action_{}, targets_{};
  long ticks_{0};
  int stable_ticks_{0}, bad_ticks_{0};
  Status status_{Status::kFailed};
};

}  // namespace humanoid_walk
