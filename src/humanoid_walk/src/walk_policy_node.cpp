// walk_policy_node: predtrenovana chodza Unitree G1 (unitree_rl_gym) pre ROS2 + Gazebo, v C++.
//
// Vstupy:  /joint_states, /imu/pelvis, /cmd_vel, /walk_policy/enable (std_msgs/Bool)
// Vystup:  /leg_pid_controller/reference (ciele 12 klbov nohy, 50 Hz)
//          /chappie/detach (std_msgs/Empty) - uvolni robota zo zaveska
//
// Rezimy:
//   vypnuty (default)  drzi vychodiskovu pozu
//   zapnuty            spusta politiku; pri zapnuti sa robot uvolni zo zaveska
// Ak sa robot nakloni o viac ako ~65 stupnov, politika sa sama vypne.
//
// Funkcia je rovnaka ako tools/walk_policy.py (Python verzia), len bez PyTorchu.
#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <array>
#include <cstdio>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "control_msgs/msg/multi_dof_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "g1_policy.hpp"
#include "walk_mjlab.hpp"
#include "getup_rl.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

namespace
{
const std::vector<std::string> kLegOrder = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
constexpr int kN = 12;
constexpr int kObs = 47;
// obmedzenia prikazov [min, max] pre vx, vy, wz
constexpr double kCmdMin[3] = {-0.5, -0.5, -1.0};
constexpr double kCmdMax[3] = {1.0, 0.5, 1.0};

using Mat3 = std::array<std::array<double, 3>, 3>;

Mat3 rpy_to_mat(double r, double p, double y)
{
  const double cr = std::cos(r), sr = std::sin(r);
  const double cp = std::cos(p), sp = std::sin(p);
  const double cy = std::cos(y), sy = std::sin(y);
  // R = Rz(y) * Ry(p) * Rx(r)
  return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
    {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
    {-sp, cp * sr, cp * cr}}};
}

std::array<double, 3> mul(const Mat3 & m, const std::array<double, 3> & v)
{
  std::array<double, 3> o{};
  for (int i = 0; i < 3; ++i) {
    o[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
  }
  return o;
}
}  // namespace

class WalkPolicyNode : public rclcpp::Node
{
  enum class HybState {kStand, kRamp, kWalk};      // WALK_HYBRID

public:
  WalkPolicyNode()
  : Node("walk_policy",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}))
  {
    std::string weights = declare_parameter<std::string>("weights_file", "");
    if (weights.empty()) {
      weights = ament_index_cpp::get_package_share_directory("humanoid_walk") +
        "/config/g1_policy.bin";
    }
    policy_.load(weights);

    // WALK_HYBRID: walk_backend = hybrid (stanie v8 + stara chodza, predvolene) | classic (len stara chodza)
    use_hybrid_ = declare_parameter<std::string>("walk_backend", "hybrid") != "classic";
    if (use_hybrid_) {
      std::string sw = declare_parameter<std::string>("stand_weights_file", "");
      if (sw.empty()) {
        sw = ament_index_cpp::get_package_share_directory("humanoid_walk") + "/config/walk_mjlab.bin";
      }
      walk_mjlab_.load(sw);
    }
    stand_after_ = declare_parameter<double>("stand_after", 1.5);          // po kolkych s bez prikazu sa prepne na stanie
    ramp_s_ = declare_parameter<double>("stand_to_walk_s", 0.5);           // prechod stanie -> start chodze [s]
    stand_blend_s_ = declare_parameter<double>("walk_to_stand_s", 0.3);    // prechod chodza -> stanie [s]
    cmd_dead_ = declare_parameter<double>("cmd_deadband", 0.05);
    {
      // zisky regulatora nohy (musia sediet s controllers.yaml) a zisky politiky stania (mjlab)
      const auto okp = declare_parameter<std::vector<double>>("ctrl_kp", {100, 100, 100, 150, 40, 40});
      const auto okd = declare_parameter<std::vector<double>>("ctrl_kd", {2, 2, 2, 4, 2, 2});
      const double mkp[6] = {40.179, 99.098, 40.179, 99.098, 28.501, 28.501};
      const double mkd[6] = {2.558, 6.309, 2.558, 6.309, 1.814, 1.814};
      if (okp.size() != 6 || okd.size() != 6) {throw std::runtime_error("ctrl_kp / ctrl_kd: treba 6 hodnot");}
      for (int i = 0; i < kN; ++i) {
        const int j = i % 6;
        stand_kratio_[i] = static_cast<float>(mkp[j] / okp[j]);
        stand_dratio_[i] = static_cast<float>((mkd[j] - okd[j]) / okd[j]);
      }
    }

    const auto def = declare_parameter<std::vector<double>>(
      "default_angles", {-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0});
    const auto cs = declare_parameter<std::vector<double>>("cmd_scale", {2.0, 2.0, 0.25});
    const auto rpy = declare_parameter<std::vector<double>>("imu_rpy", {0.0, 0.0, 0.0});
    if (def.size() != kN || cs.size() != 3 || rpy.size() != 3) {
      throw std::runtime_error("Zle dlzky parametrov default_angles / cmd_scale / imu_rpy");
    }
    for (int i = 0; i < kN; ++i) {default_[i] = static_cast<float>(def[i]);}
    for (int i = 0; i < 3; ++i) {cmd_scale_[i] = cs[i];}
    R_p_i_ = rpy_to_mat(rpy[0], rpy[1], rpy[2]);
    hold_pose_ = default_;
    if (use_hybrid_) {for (int i = 0; i < kN; ++i) {hold_pose_[i] = humanoid_walk::walk_mjlab::kDefault[i];}}

    ang_vel_scale_ = declare_parameter<double>("ang_vel_scale", 0.25);
    dof_pos_scale_ = declare_parameter<double>("dof_pos_scale", 1.0);
    dof_vel_scale_ = declare_parameter<double>("dof_vel_scale", 0.05);
    action_scale_ = declare_parameter<double>("action_scale", 0.25);
    period_ = declare_parameter<double>("period", 0.8);
    hold_ticks_max_ = declare_parameter<int>("hold_ticks", 150);
    fall_gravity_z_ = declare_parameter<double>("fall_gravity_z", -0.4);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.0);
    const double dt = declare_parameter<double>("control_dt", 0.02);
    detach_topic_ = declare_parameter<std::string>("detach_topic", "/chappie/detach");
    const std::string ref_topic = declare_parameter<std::string>(
      "reference_topic", "/leg_pid_controller/reference");

    q_.fill(0.0f);
    dq_.fill(0.0f);
    action_.fill(0.0f);
    q_min_.fill(9.0f);
    q_max_.fill(-9.0f);
    t_min_.fill(9.0f);
    t_max_.fill(-9.0f);

    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {on_joints(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/pelvis", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {imu_ = *m; have_imu_ = true;});
    // --- drzanie miesta
    // v hybride robot stoji sam; korekcia polohy by ho nutila kracat
    hold_enabled_ = declare_parameter<bool>("hold_position", !use_hybrid_);
    hold_kp_ = declare_parameter<double>("hold_kp", 1.0);
    hold_kv_ = declare_parameter<double>("hold_kv", 1.0);
    hold_max_ = declare_parameter<double>("hold_max", 0.5);
    hold_yaw_kp_ = declare_parameter<double>("hold_yaw_kp", 1.0);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/chappie/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(*m);});
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_cmd(*m);});
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/walk_policy/enable", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {on_enable(m->data);});
    ref_pub_ = create_publisher<control_msgs::msg::MultiDOFCommand>(ref_topic, 10);
    detach_pub_ = create_publisher<std_msgs::msg::Empty>(detach_topic_, 10);

    // --- padanie a vstavanie
    getup_file_ = declare_parameter<std::string>("getup_file", "");
    fall_sub_ = create_subscription<std_msgs::msg::String>(
      "/chappie/fall", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {on_fall(m->data);});
    getup_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/chappie/getup", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) {on_getup();});
    posture_pub_ = create_publisher<std_msgs::msg::String>("/chappie/posture", 10);
    pose_pub_ = create_publisher<std_msgs::msg::String>("/skills/pose", 10);
    handoff_t0_ = declare_parameter<double>("getup_handoff_t0", 18.9);     // od kedy hladat okamih odovzdania [s]
    handoff_gx_ = declare_parameter<double>("getup_handoff_gx", 0.05);     // max. sklon trupu dopredu/dozadu
    handoff_blend_s_ = declare_parameter<double>("getup_handoff_blend", 1.0);
    upper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      declare_parameter<std::string>("upper_topic", "/upper_body_controller/joint_trajectory"), 10);

    // --- vstavanie politikou natrenovanou v mjlab (RL)
    rl_mode_ = declare_parameter<std::string>("getup_mode", "rl");          // rl | keyframe
    rl_weights_ = declare_parameter<std::string>("getup_rl_weights", "");
    rl_ctrl_ = declare_parameter<std::string>("getup_rl_controller", "getup_pid_controller");
    rl_legs_ctrl_ = declare_parameter<std::string>("getup_rl_legs_controller", "leg_pid_controller");
    rl_upper_ctrl_ = declare_parameter<std::string>("getup_rl_upper_controller", "upper_body_controller");
    rl_jsb_ = declare_parameter<std::string>("getup_rl_broadcaster", "joint_state_broadcaster");
    rl_arm_phase_ = declare_parameter<double>("getup_rl_arm_phase", 1.5);         // ruky dole po postaveni [s]
    rl_handoff_blend_ = declare_parameter<double>("getup_rl_handoff_blend", 0.4); // prelinanie na chodzu [s]
    rl_tilt_ = declare_parameter<double>("getup_rl_handoff_tilt", 0.10);   // max. |gx|,|gy| pri odovzdani
    rl_wait_ = declare_parameter<double>("getup_rl_wait", 4.0);            // ako dlho cakat na rovny trup [s]
    declare_parameter<bool>("getup_rl_auto_walk", true);                   // false = po vstani drzi RL, chodzu zapnes sam
    getup_rl_.config().max_time = declare_parameter<double>("getup_rl_max_time", 8.0);
    rl_ref_pub_ = create_publisher<control_msgs::msg::MultiDOFCommand>("/" + rl_ctrl_ + "/reference", 10);
    rl_switch_cli_ = create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    timer_ = create_timer(std::chrono::duration<double>(dt), [this]() {step();});
    RCLCPP_INFO(get_logger(),
      "walk_policy (C++, %s): perioda %.3f s, politika VYPNUTA (drzim vychodiskovu pozu)",
      use_hybrid_ ? "hybrid" : "classic", dt);
  }

private:
  // ------------------------------------------------------------ callbacky
  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    for (size_t k = 0; k < msg.name.size() && k < msg.position.size(); ++k) {
      all_pos_[msg.name[k]] = msg.position[k];
      if (k < msg.velocity.size()) {all_vel_[msg.name[k]] = msg.velocity[k];}
    }
    if (joint_index_.empty()) {
      for (const auto & name : kLegOrder) {
        const auto it = std::find(msg.name.begin(), msg.name.end(), name);
        if (it == msg.name.end()) {
          RCLCPP_ERROR(get_logger(), "V /joint_states chyba klb: %s", name.c_str());
          return;
        }
        joint_index_.push_back(static_cast<size_t>(it - msg.name.begin()));
      }
    }
    for (int i = 0; i < kN; ++i) {
      const size_t j = joint_index_[i];
      if (j < msg.position.size()) {q_[i] = static_cast<float>(msg.position[j]);}
      if (j < msg.velocity.size()) {dq_[i] = static_cast<float>(msg.velocity[j]);}
    }
    have_joints_ = true;
  }

  void on_cmd(const geometry_msgs::msg::Twist & m)
  {
    cmd_[0] = std::clamp(m.linear.x, kCmdMin[0], kCmdMax[0]);
    cmd_[1] = std::clamp(m.linear.y, kCmdMin[1], kCmdMax[1]);
    cmd_[2] = std::clamp(m.angular.z, kCmdMin[2], kCmdMax[2]);
    cmd_stamp_ = now_s();
    have_cmd_stamp_ = true;
  }

  // ------------------------------------------------------------ drzanie miesta
  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    const double t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
    const double x = m.pose.pose.position.x, y = m.pose.pose.position.y;
    if (have_odom_) {
      const double dt = t - odom_stamp_;
      if (dt > 0.004 && dt < 0.5) {
        const double a = 0.3;     // vyhladenie rychlosti
        odom_vx_ += a * ((x - odom_x_) / dt - odom_vx_);
        odom_vy_ += a * ((y - odom_y_) / dt - odom_vy_);
      }
    }
    const auto & q = m.pose.pose.orientation;
    odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_x_ = x;
    odom_y_ = y;
    odom_stamp_ = t;
    odom_wall_ = now_s();
    have_odom_ = true;
  }

  static double wrap_pi(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }

  // dopln prikaz c o korekciu, ktora drzi robota na mieste (ked nema ziadny prikaz)
  void hold_update(std::array<double, 3> & c)
  {
    const double t = now_s();
    const double dt = t - hold_last_t_;
    hold_last_t_ = t;
    if (dt > 0.1) {hold_idle_ = 0.0; hold_anchor_ = false;}     // politika bola vypnuta
    if (!hold_enabled_) {return;}
    const bool idle = std::fabs(cmd_[0]) < 0.03 && std::fabs(cmd_[1]) < 0.03 && std::fabs(cmd_[2]) < 0.1;
    if (!idle) {hold_idle_ = 0.0; hold_anchor_ = false; return;}
    hold_idle_ += std::min(dt, 0.1);
    if (hold_idle_ < 1.0) {return;}                                 // nech dobehne
    const double qx = imu_.orientation.x, qy = imu_.orientation.y;
    const double qz = imu_.orientation.z, qw = imu_.orientation.w;
    const double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    const bool odom_ok = have_odom_ && (t - odom_wall_) < 0.5;
    if (!hold_anchor_) {
      hold_anchor_ = true;
      hold_yaw0_ = yaw;
      hold_odom_anchor_ = odom_ok;
      hold_x0_ = odom_x_;
      hold_y0_ = odom_y_;
      return;
    }
    c[2] = std::clamp(hold_yaw_kp_ * wrap_pi(hold_yaw0_ - yaw), -0.3, 0.3);
    if (odom_ok && hold_odom_anchor_) {
      const double cy = std::cos(odom_yaw_), sy = std::sin(odom_yaw_);
      const double ex = hold_x0_ - odom_x_, ey = hold_y0_ - odom_y_;
      const double ebx = cy * ex + sy * ey, eby = -sy * ex + cy * ey;
      const double vbx = cy * odom_vx_ + sy * odom_vy_, vby = -sy * odom_vx_ + cy * odom_vy_;
      c[0] = std::clamp(hold_kp_ * ebx - hold_kv_ * vbx, -hold_max_, hold_max_);
      c[1] = std::clamp(hold_kp_ * eby - hold_kv_ * vby, -hold_max_, hold_max_);
    }
    c[0] = std::clamp(c[0], kCmdMin[0], kCmdMax[0]);
    c[1] = std::clamp(c[1], kCmdMin[1], kCmdMax[1]);
    c[2] = std::clamp(c[2], kCmdMin[2], kCmdMax[2]);
  }

  void on_enable(bool on)
  {
    if (on) {
      if (!enabled_ && !enable_requested_) {
        enable_requested_ = true;
        RCLCPP_INFO(get_logger(), "Zapnutie vyziadane, pockam kym je robot pripraveny a drzi pozu");
      }
    } else {
      enable_requested_ = false;
      if (enabled_) {
        enabled_ = false;
        hold_ticks_ = 0;
        RCLCPP_INFO(get_logger(), "Politika VYPNUTA, drzim vychodiskovu pozu");
      }
    }
  }

  // ------------------------------------------------------------ padanie a vstavanie
  // Poloha z projekcie gravitacie v suradniciach panvy (stoji: gz = -1,
  // na tvari: gx = +1, na chrbte: gx = -1).
  static const char * classify_posture(const std::array<double, 3> & g)
  {
    if (g[2] < -0.8) {return "upright";}
    if (g[0] > 0.7) {return "face_down";}
    if (g[0] < -0.7) {return "on_back";}
    if (std::fabs(g[1]) > 0.7) {return "on_side";}
    return "tilted";
  }

  void update_posture(const std::array<double, 3> & g)
  {
    const std::string c = classify_posture(g);
    if (c == candidate_posture_) {
      ++candidate_ticks_;
    } else {
      candidate_posture_ = c;
      candidate_ticks_ = 0;
    }
    if (candidate_ticks_ == 10 && c != posture_) {   // 0.2 s stabilne
      posture_ = c;
      std_msgs::msg::String msg;
      msg.data = posture_;
      posture_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "Poloha robota: %s", posture_.c_str());
    }
  }

  void on_fall(const std::string &)
  {
    // testovacie padnutie dopredu: politika sa vypne, clenky sa vychylia a robot spadne na tvar
    enabled_ = false;
    enable_requested_ = false;
    hold_ticks_ = 0;
    getup_active_ = false;
    handoff_left_ = 0;
    fall_ticks_ = 30;   // 0.6 s
    RCLCPP_WARN(get_logger(), "Testovacie padnutie dopredu");
  }

  void run_fall()
  {
    std::array<float, kN> t = default_;
    for (int s = 0; s < 2; ++s) {
      t[6 * s + 3] = 0.0f;     // kolena rovno
      t[6 * s + 4] = -0.8f;    // clenky -> robot sa prevrati na tvar
    }
    publish_targets(t);
    --fall_ticks_;
  }

  bool load_getup()
  {
    std::string path = getup_file_;
    if (path.empty()) {
      path = ament_index_cpp::get_package_share_directory("humanoid_walk") +
        "/config/getup_prone.txt";
    }
    std::ifstream f(path);
    if (!f) {
      RCLCPP_ERROR(get_logger(), "Nemozem otvorit trajektoriu vstavania: %s", path.c_str());
      return false;
    }
    getup_names_.clear();
    getup_q_.clear();
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      std::istringstream is(line);
      if (line.rfind("dt ", 0) == 0) {
        std::string k;
        is >> k >> getup_dt_;
      } else if (line.rfind("joints ", 0) == 0) {
        std::string k, n;
        is >> k;
        while (is >> n) {getup_names_.push_back(n);}
      } else {
        std::vector<float> row;
        float v;
        while (is >> v) {row.push_back(v);}
        if (row.size() == getup_names_.size()) {getup_q_.push_back(row);}
      }
    }
    if (getup_names_.empty() || getup_q_.size() < 10) {
      RCLCPP_ERROR(get_logger(), "Trajektoria vstavania je prazdna alebo poskodena: %s",
        path.c_str());
      return false;
    }
    getup_leg_idx_.assign(kN, -1);
    for (int i = 0; i < kN; ++i) {
      for (size_t j = 0; j < getup_names_.size(); ++j) {
        if (getup_names_[j] == kLegOrder[i]) {getup_leg_idx_[i] = static_cast<int>(j);}
      }
    }
    RCLCPP_INFO(get_logger(), "Trajektoria vstavania: %zu klbov, %zu krokov, %.1f s",
      getup_names_.size(), getup_q_.size(), getup_q_.size() * getup_dt_);
    return true;
  }

  // ------------------------------------------------------------ vstavanie politikou (RL)
  // Priebeh: (1) regulatory sa prepnu na momentove riadenie vsetkych 29 klbov (getup_pid_controller),
  // (2) politika (50 Hz) postavi robota, (3) ked stoji, ruky sa dostanu dole, (4) regulatory sa vratia
  // (nohy: leg_pid_controller, driek a ruky: upper_body_controller) a prevezme chodza.
  static constexpr int kAll = humanoid_walk::GetupRl::kN;

  bool rl_joints_known() const
  {
    for (const auto & n : humanoid_walk::getup_params::kNames) {
      if (all_pos_.find(n) == all_pos_.end()) {return false;}
    }
    return true;
  }

  void rl_request_switch(bool to_rl)
  {
    using Switch = controller_manager_msgs::srv::SwitchController;
    auto req = std::make_shared<Switch::Request>();
    if (to_rl) {
      req->activate_controllers = {rl_ctrl_};
      req->deactivate_controllers = {rl_legs_ctrl_, rl_upper_ctrl_};
    } else {
      req->activate_controllers = {rl_legs_ctrl_, rl_upper_ctrl_};
      req->deactivate_controllers = {rl_ctrl_};
    }
    // joint_state_broadcaster cita stavy z PID regulatorov, takze ho controller_manager nechce nechat
    // bezat pri ich vypnuti. Restart = je v zozname vypnutia aj zapnutia (trva to jeden cyklus).
    if (!rl_jsb_.empty()) {
      req->activate_controllers.push_back(rl_jsb_);
      req->deactivate_controllers.push_back(rl_jsb_);
    }
    req->strictness = Switch::Request::STRICT;
    req->activate_asap = true;
    req->timeout.sec = 3;
    req->timeout.nanosec = 0;
    rl_switch_state_ = 1;
    rl_switch_wait_ = 0;
    rl_switch_cli_->async_send_request(
      req, [this, to_rl](rclcpp::Client<Switch>::SharedFuture f) {
        const bool ok = f.get()->ok;
        if (ok && !to_rl) {publish_targets(rl_leg_hold_);}   // nohy dostanu ciel hned po prepnuti
        rl_switch_state_ = ok ? 2 : 3;
      });
  }

  // true = spracovane (bezi, alebo odmietnute); false = RL sa neda spustit (chyba nastavenia)
  bool on_getup_rl()
  {
    if (rl_state_ != RlState::kIdle) {return true;}
    if (posture_ == "upright") {
      RCLCPP_WARN(get_logger(), "Vstavanie: robot uz stoji");
      return true;
    }
    try {
      if (!getup_rl_.loaded()) {
        std::string path = rl_weights_;
        if (path.empty()) {
          path = ament_index_cpp::get_package_share_directory("humanoid_walk") +
            "/config/getup_policy.bin";
        }
        getup_rl_.load(path);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Vstavanie (RL): %s", e.what());
      return false;
    }
    if (!rl_joints_known()) {
      RCLCPP_ERROR(get_logger(),
        "Vstavanie (RL): v /joint_states nie je vsetkych 29 klbov (bezi robot s volnymi rukami, "
        "weld_upper:=false?)");
      return false;
    }
    if (!rl_switch_cli_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Vstavanie (RL): nie je dostupna sluzba /controller_manager/switch_controller");
      return false;
    }
    // vypni chodzu a stare vstavanie
    enabled_ = false;
    enable_requested_ = false;
    getup_active_ = false;
    fall_ticks_ = 0;
    handoff_left_ = 0;
    hold_ticks_ = 0;
    rl_ticks_ = 0;
    rl_success_ = false;
    rl_state_ = RlState::kSwitchIn;
    rl_request_switch(true);
    RCLCPP_INFO(get_logger(), "Vstavanie (RL): prepinam regulatory na momentove riadenie (poloha: %s)",
      posture_.c_str());
    return true;
  }

  void rl_diag(const char * tag, const std::array<double, 3> & g)
  {
    auto p = [this](const char * n) {
        const auto it = all_pos_.find(n);
        return it == all_pos_.end() ? 0.0 : it->second;
      };
    double arm = 0.0;
    for (int i = 15; i < kAll; ++i) {arm += std::fabs(p(humanoid_walk::getup_params::kNames[i].c_str()));}
    arm /= (kAll - 15);
    RCLCPP_INFO(get_logger(),
      "RL[%s] t=%.1f vzpriamenost=%.3f gx=%+.3f gy=%+.3f | driek y/r/p=%+.2f/%+.2f/%+.2f | ruky |q|=%.2f | kolena %.2f %.2f",
      tag, getup_rl_.time(), -g[2], g[0], g[1], p("waist_yaw_joint"), p("waist_roll_joint"),
      p("waist_pitch_joint"), arm, p("left_knee_joint"), p("right_knee_joint"));
  }

  void rl_switch_out(bool success)
  {
    rl_success_ = success;
    rl_state_ = RlState::kSwitchOut;
    rl_request_switch(false);
  }

  void run_getup_rl(const std::array<double, 3> & gravity)
  {
    using Status = humanoid_walk::GetupRl::Status;
    ++rl_ticks_;
    if (rl_state_ == RlState::kSwitchIn) {
      if (rl_switch_state_ == 2) {
        getup_rl_.start();
        rl_stand_t_ = -1.0;
        rl_hold_warned_ = false;
        rl_state_ = RlState::kRun;
        RCLCPP_INFO(get_logger(), "Vstavanie (RL): regulatory prepnute, vstavam");
      } else if (rl_switch_state_ == 3 || ++rl_switch_wait_ > 250) {
        RCLCPP_ERROR(get_logger(),
          "Vstavanie (RL): prepnutie regulatorov zlyhalo. Bezi getup_pid_controller? "
          "(skontroluj: ros2 control list_controllers; po add_getup_rl.py treba colcon build a restart simulacie)");
        rl_state_ = RlState::kIdle;
        return;
      } else {
        return;
      }
    }

    // ---- jeden krok politiky
    std::array<float, kAll> q{}, dq{};
    for (int i = 0; i < kAll; ++i) {
      const auto & name = humanoid_walk::getup_params::kNames[i];
      q[i] = static_cast<float>(all_pos_[name]);
      dq[i] = static_cast<float>(all_vel_[name]);
    }
    const std::array<double, 3> wv = {
      imu_.angular_velocity.x, imu_.angular_velocity.y, imu_.angular_velocity.z};
    const auto omega = mul(R_p_i_, wv);
    const Status st = getup_rl_.step(gravity, omega, q, dq);
    std::array<float, kAll> tgt = getup_rl_.targets();

    if (st == Status::kStanding && rl_stand_t_ < 0.0) {
      rl_stand_t_ = getup_rl_.time();
      RCLCPP_INFO(get_logger(), "Vstavanie (RL): robot stoji (po %.1f s), davam ruky dole", rl_stand_t_);
      rl_diag("stoji", gravity);
    }
    if (rl_stand_t_ >= 0.0) {
      // Politika chodze je citliva na polohu ruk, preto sa ruky pred odovzdanim plynule dostanu do 0
      // (nohy a driek ostavaju pod kontrolou politiky vstavania).
      const double u = std::min(1.0, (getup_rl_.time() - rl_stand_t_) / std::max(0.05, rl_arm_phase_));
      for (int i = 15; i < kAll; ++i) {tgt[i] *= static_cast<float>(1.0 - u);}
    }
    if (rl_ticks_ % 25 == 0 && rl_state_ != RlState::kSwitchOut) {rl_diag("vstava", gravity);}

    control_msgs::msg::MultiDOFCommand msg;
    msg.dof_names = humanoid_walk::getup_params::kNames;
    msg.values.assign(tgt.begin(), tgt.end());
    msg.values_dot.assign(kAll, 0.0);
    rl_ref_pub_->publish(msg);
    for (int i = 0; i < kN; ++i) {rl_leg_hold_[i] = tgt[i];}

    if (rl_state_ == RlState::kRun || rl_state_ == RlState::kHold) {
      if (st == Status::kFailed) {
        RCLCPP_WARN(get_logger(), "Vstavanie (RL) sa nepodarilo (po %.1f s), vraciam regulatory",
          getup_rl_.time());
        rl_switch_out(false);
        return;
      }
      if (rl_stand_t_ < 0.0 || (getup_rl_.time() - rl_stand_t_) < rl_arm_phase_) {return;}
      if (rl_state_ == RlState::kRun) {
        rl_state_ = RlState::kHold;
        rl_hold_t_ = getup_rl_.time();
      }
      // --- robot stoji a ma ruky dole: cakam na dobry okamih na odovzdanie chodzi
      if (-gravity[2] < 0.5) {
        RCLCPP_WARN(get_logger(), "Vstavanie (RL): robot spadol, vraciam regulatory");
        rl_switch_out(false);
        return;
      }
      const bool tilt_ok = std::fabs(gravity[0]) < rl_tilt_ && std::fabs(gravity[1]) < rl_tilt_;
      const bool waited = (getup_rl_.time() - rl_hold_t_) > rl_wait_;
      const bool auto_walk = get_parameter("getup_rl_auto_walk").as_bool();
      if (enable_requested_ || (auto_walk && tilt_ok)) {
        rl_diag("odovzdanie", gravity);
        rl_switch_out(true);
      } else if (!rl_hold_warned_ && (waited || !auto_walk)) {
        rl_hold_warned_ = true;
        if (auto_walk) {
          RCLCPP_WARN(get_logger(),
            "Vstavanie (RL): trup je naklonený (gx=%+.2f gy=%+.2f), drzim robota politikou vstavania; "
            "chodzu zapni cez /walk_policy/enable", gravity[0], gravity[1]);
        } else {
          RCLCPP_WARN(get_logger(),
            "Vstavanie (RL): robot stoji, drzim ho politikou vstavania; chodzu zapni cez /walk_policy/enable");
        }
      }
      return;
    }

    // kSwitchOut: politika dalej drzi robota, kym sa regulatory nevratia
    if (rl_switch_state_ == 2) {
      rl_state_ = RlState::kIdle;
      fall_ticks_ = 0;
      if (rl_success_) {
        enabled_ = true;
        enable_requested_ = false;
        have_t0_ = false;
        action_.fill(0.0f);
        policy_.reset();                    // nova pamat LSTM pre chodzu
        handoff_hold_ = rl_leg_hold_;
        handoff_left_ = std::max(1, static_cast<int>(rl_handoff_blend_ / 0.02 + 0.5));
        handoff_n_ = handoff_left_;
        rl_post_ = 150;                     // 3 s diagnostiky po odovzdani
        RCLCPP_INFO(get_logger(), "Vstavanie (RL): hotovo, odovzdavam politike chodze");
      } else {
        enabled_ = false;
        enable_requested_ = false;
        hold_ticks_ = 0;
      }
    } else if (rl_switch_state_ == 3 || ++rl_switch_wait_ > 250) {
      RCLCPP_ERROR(get_logger(), "Vstavanie (RL): vratenie regulatorov zlyhalo, skusam znova");
      rl_request_switch(false);
    }
  }

  void on_getup()
  {
    if (getup_active_) {return;}
    if (rl_mode_ == "rl") {
      if (on_getup_rl()) {return;}
      RCLCPP_WARN(get_logger(), "Vstavanie (RL) nejde, skusam starsie vstavanie podla klucovych poloh");
    }
    if (posture_ != "face_down") {
      RCLCPP_WARN(get_logger(),
        "Vstavanie mam pripravene iba z polohy na tvari (teraz: %s)", posture_.c_str());
      return;
    }
    if (getup_q_.empty() && !load_getup()) {return;}
    enabled_ = false;
    enable_requested_ = false;
    fall_ticks_ = 0;
    // vychodisko: namerana poloha klbov (plynuly nabeh do prveho kroku trajektorie)
    getup_start_.assign(getup_names_.size(), 0.0f);
    for (size_t j = 0; j < getup_names_.size(); ++j) {
      const auto it = all_pos_.find(getup_names_[j]);
      getup_start_[j] = (it != all_pos_.end()) ? static_cast<float>(it->second) : getup_q_[0][j];
    }
    getup_tick_ = 0;
    getup_active_ = true;
    publish_upper_trajectory();
    RCLCPP_INFO(get_logger(), "Vstavam sam: nabeh %.1f s + pohyb %.1f s", getup_blend_s_,
      getup_q_.size() * getup_dt_);
  }

  // cielova poloha klbu j v case t [s] od startu (nabeh + trajektoria)
  float getup_value(size_t j, double t) const
  {
    if (t < getup_blend_s_) {
      double u = t / getup_blend_s_;
      u = u * u * (3.0 - 2.0 * u);
      return static_cast<float>(getup_start_[j] * (1.0 - u) + getup_q_[0][j] * u);
    }
    size_t i = static_cast<size_t>((t - getup_blend_s_) / getup_dt_);
    if (i >= getup_q_.size()) {i = getup_q_.size() - 1;}
    return getup_q_[i][j];
  }

  void publish_upper_trajectory()
  {
    trajectory_msgs::msg::JointTrajectory traj;
    std::vector<size_t> up;
    for (size_t j = 0; j < getup_names_.size(); ++j) {
      if (std::find(kLegOrder.begin(), kLegOrder.end(), getup_names_[j]) == kLegOrder.end()) {
        up.push_back(j);
        traj.joint_names.push_back(getup_names_[j]);
      }
    }
    const double total = getup_blend_s_ + getup_q_.size() * getup_dt_;
    for (double t = 0.04; t <= total + 1e-9; t += 0.04) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      for (size_t j : up) {p.positions.push_back(getup_value(j, t));}
      p.time_from_start.sec = static_cast<int32_t>(t);
      p.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
      traj.points.push_back(p);
    }
    upper_pub_->publish(traj);
  }

  void run_getup(const std::array<double, 3> & g)
  {
    const double t = getup_tick_ * 0.02;
    const double total = getup_blend_s_ + getup_q_.size() * getup_dt_;
    std::array<float, kN> tgt = default_;
    for (int i = 0; i < kN; ++i) {
      if (getup_leg_idx_[i] >= 0) {tgt[i] = getup_value(static_cast<size_t>(getup_leg_idx_[i]), t);}
    }
    publish_targets(tgt);
    handoff_hold_ = tgt;
    ++getup_tick_;
    // Drep na nohach je hotovy. Politika chodze vie drzat rovnovahu lepsie nez pevna trajektoria,
    // preto ju zapnem v okamihu, ked je trup rovno (sklon ~0) - alebo na konci trajektorie.
    const bool window = (t - getup_blend_s_) >= handoff_t0_;
    const bool straight = std::fabs(g[0]) < handoff_gx_ && std::fabs(g[1]) < 0.1;
    if ((window && straight) || t >= total) {
      getup_active_ = false;
      enabled_ = true;
      enable_requested_ = false;
      have_t0_ = false;
      action_.fill(0.0f);
      policy_.reset();                      // nova pamat LSTM
      handoff_left_ = std::max(1, static_cast<int>(handoff_blend_s_ / 0.02 + 0.5));
      handoff_n_ = handoff_left_;
      std_msgs::msg::String pose;
      pose.data = "arms_down";              // ruky sa vratia do zakladnej polohy
      pose_pub_->publish(pose);
      RCLCPP_INFO(get_logger(), "Vstavanie: odovzdavam politike chodze (t=%.1f s, gx=%.2f)", t, g[0]);
    }
  }

  // ------------------------------------------------------------ vystup
  double now_s() {return get_clock()->now().seconds();}

  void gz_detach()
  {
    // uvolnenie priamo cez Gazebo (obchadza ros_gz_bridge)
    const std::string cmd =
      "gz topic -t " + detach_topic_ + " -m gz.msgs.Empty -p ' ' >/dev/null 2>&1 &";
    const int rc = std::system(cmd.c_str());
    (void)rc;
  }

  void publish_targets(const std::array<float, kN> & target)
  {
    std::array<float, kN> zero{};
    publish_ref(target, zero);
  }

  void publish_ref(const std::array<float, kN> & target, const std::array<float, kN> & dot)
  {
    last_target_ = target;
    control_msgs::msg::MultiDOFCommand msg;
    msg.dof_names = kLegOrder;
    msg.values.assign(target.begin(), target.end());
    msg.values_dot.assign(dot.begin(), dot.end());
    ref_pub_->publish(msg);
  }

  // ------------------------------------------------------------ WALK_HYBRID
  // Stavy: kStand (politika v8, stoji bez krokov) -> kRamp (nohy do vychodiskovej polohy starej chodze)
  //        -> kWalk (stara chodza, motion.pt) -> po case bez prikazu naspat kStand.

  // Politika stania ma ine zisky PD nez regulator nohy. Aby regulator vyvinul rovnaky moment:
  //   ref = q + (kp_politika / kp_regulator) * (ciel - q),   ref_dot = -((kd_politika - kd_reg) / kd_reg) * dq
  void step_stand(const std::array<double, 3> & gravity, const std::array<double, 3> & omega)
  {
    namespace wm = humanoid_walk::walk_mjlab;
    float obs[wm::kObs];
    for (int i = 0; i < 3; ++i) {
      obs[i] = static_cast<float>(omega[i]);
      obs[3 + i] = static_cast<float>(gravity[i]);
      obs[6 + i] = 0.0f;                           // prikaz 0 = stoj
    }
    for (int i = 0; i < kN; ++i) {
      obs[9 + i] = q_[i] - wm::kDefault[i];
      obs[9 + kN + i] = dq_[i];
      obs[9 + 2 * kN + i] = mj_action_[i];
    }
    float act[kN];
    walk_mjlab_.forward(obs, act);
    for (int i = 0; i < kN; ++i) {
      if (!std::isfinite(act[i])) {
        RCLCPP_WARN(get_logger(), "Politika stania vratila nekonecne hodnoty, preskakujem");
        return;
      }
    }
    std::array<float, kN> ref, dot;
    for (int i = 0; i < kN; ++i) {
      mj_action_[i] = act[i];
      const float qt = wm::kDefault[i] + wm::kScale[i] * act[i];
      ref[i] = q_[i] + stand_kratio_[i] * (qt - q_[i]);
      dot[i] = -stand_dratio_[i] * dq_[i];
    }
    if (handoff_left_ > 0) {   // plynuly prechod z predchadzajuceho cieľa
      const float a = static_cast<float>(handoff_n_ - handoff_left_) / handoff_n_;
      for (int i = 0; i < kN; ++i) {
        ref[i] = handoff_hold_[i] * (1.0f - a) + ref[i] * a;
        dot[i] *= a;
      }
      --handoff_left_;
    }
    publish_ref(ref, dot);
  }

  // true = tento krok vybavil hybrid (stanie / prechod); false = ide stara chodza (pokracuje step())
  bool hybrid_step(const std::array<double, 3> & gravity, const std::array<double, 3> & omega, double phase)
  {
    const double dt = 0.02;
    if (step_counter_ - hyb_last_step_ > 1) {          // politika predtym nebezala
      hyb_state_ = HybState::kStand;
      hyb_idle_ = 0.0;
      mj_action_.fill(0.0f);
    }
    hyb_last_step_ = step_counter_;
    const bool active = std::fabs(cmd_[0]) > cmd_dead_ || std::fabs(cmd_[1]) > cmd_dead_ ||
      std::fabs(cmd_[2]) > 2.0 * cmd_dead_;

    if (hyb_state_ == HybState::kStand && active) {
      hyb_state_ = HybState::kRamp;
      hyb_ramp_t_ = 0.0;
      ramp_from_ = q_;
      handoff_left_ = 0;
      RCLCPP_INFO(get_logger(), "Hybrid: prikaz prisiel, zacinam chodzu");
    } else if (hyb_state_ == HybState::kWalk) {
      hyb_idle_ = active ? 0.0 : hyb_idle_ + dt;
      if (hyb_idle_ >= stand_after_) {
        // obe nohy na zemi: faza chodze 0 az 0.05 a 0.5 az 0.55
        const bool both_down = std::fmod(phase, 0.5) < 0.06;
        if (both_down || hyb_idle_ >= stand_after_ + 0.8) {
          hyb_state_ = HybState::kStand;
          mj_action_.fill(0.0f);
          handoff_hold_ = last_target_;
          handoff_n_ = std::max(1, static_cast<int>(stand_blend_s_ / dt + 0.5));
          handoff_left_ = handoff_n_;
          RCLCPP_INFO(get_logger(), "Hybrid: bez prikazu, prepinam na stanie");
        }
      }
    }

    if (hyb_state_ == HybState::kRamp) {
      hyb_ramp_t_ += dt;
      const double u = std::min(1.0, hyb_ramp_t_ / std::max(0.05, ramp_s_));
      const float s = static_cast<float>(u * u * (3.0 - 2.0 * u));
      std::array<float, kN> t;
      for (int i = 0; i < kN; ++i) {t[i] = ramp_from_[i] * (1.0f - s) + default_[i] * s;}
      publish_targets(t);
      if (u >= 1.0) {
        hyb_state_ = HybState::kWalk;
        hyb_idle_ = 0.0;
        have_t0_ = false;                  // faza chodze od nuly
        action_.fill(0.0f);
        policy_.reset();                   // nova pamat LSTM
        handoff_left_ = 0;
      }
      return true;
    }
    if (hyb_state_ == HybState::kStand) {
      step_stand(gravity, omega);
      return true;
    }
    return false;
  }

  // ------------------------------------------------------------ hlavny krok
  void step()
  {
    // opakovane posielanie uvolnenia (sprava nic nepokazi, ak pride viackrat)
    if (detach_left_ > 0) {
      if (detach_left_ % 10 == 0) {detach_pub_->publish(std_msgs::msg::Empty());}
      if (detach_left_ == 100 || detach_left_ == 50) {gz_detach();}
      --detach_left_;
    }

    if (!have_imu_ || !have_joints_) {return;}
    ++step_counter_;

    // watchdog: ak prikazy z klavesnice prestanu chodit, robot sa zastavi
    if (cmd_timeout_ > 0.0 && have_cmd_stamp_ && (now_s() - cmd_stamp_) > cmd_timeout_) {
      cmd_ = {0.0, 0.0, 0.0};
    }

    // projekcia gravitacie do suradnic panvy: R_p_i * (R_w_i^T * [0, 0, -1])
    const double x = imu_.orientation.x, y = imu_.orientation.y;
    const double z = imu_.orientation.z, w = imu_.orientation.w;
    const std::array<double, 3> g_imu = {
      -2.0 * (x * z - y * w), -2.0 * (y * z + x * w), -(1.0 - 2.0 * (x * x + y * y))};
    const auto gravity = mul(R_p_i_, g_imu);

    update_posture(gravity);
    if (rl_post_ > 0) {
      --rl_post_;
      if (rl_post_ % 10 == 0) {rl_diag("CHODZA", gravity);}
    }
    if (rl_state_ != RlState::kIdle) {run_getup_rl(gravity); return;}
    if (getup_active_) {run_getup(gravity); return;}
    if (fall_ticks_ > 0) {run_fall(); return;}

    if (!enabled_) {
      publish_targets(hold_pose_);
      ++hold_ticks_;
      if (enable_requested_ && hold_ticks_ >= hold_ticks_max_) {
        enable_requested_ = false;
        enabled_ = true;
        have_t0_ = false;
        action_.fill(0.0f);
        policy_.reset();            // nova pamat LSTM pre novu chodzu
        detach_left_ = 100;
        RCLCPP_INFO(get_logger(), "Politika ZAPNUTA, uvolnujem robota zo zaveska");
      }
      return;
    }

    if (gravity[2] > fall_gravity_z_) {
      enabled_ = false;
      hold_ticks_ = 0;
      RCLCPP_WARN(get_logger(), "Robot sa naklana viac ako ~65 stupnov, politika vypnuta");
      publish_targets(hold_pose_);
      return;
    }

    const double now = now_s();
    if (!have_t0_) {t0_ = now; have_t0_ = true;}
    const double phase = std::fmod(now - t0_, period_) / period_;

    const std::array<double, 3> wv = {
      imu_.angular_velocity.x, imu_.angular_velocity.y, imu_.angular_velocity.z};
    const auto omega = mul(R_p_i_, wv);

    if (use_hybrid_ && hybrid_step(gravity, omega, phase)) {return;}      // WALK_HYBRID
    std::array<double, 3> cmd_eff = cmd_;
    hold_update(cmd_eff);

    float obs[kObs];
    for (int i = 0; i < 3; ++i) {
      obs[i] = static_cast<float>(omega[i] * ang_vel_scale_);
      obs[3 + i] = static_cast<float>(gravity[i]);
      obs[6 + i] = static_cast<float>(cmd_eff[i] * cmd_scale_[i]);
    }
    for (int i = 0; i < kN; ++i) {
      obs[9 + i] = static_cast<float>((q_[i] - default_[i]) * dof_pos_scale_);
      obs[9 + kN + i] = static_cast<float>(dq_[i] * dof_vel_scale_);
      obs[9 + 2 * kN + i] = action_[i];
    }
    obs[9 + 3 * kN] = static_cast<float>(std::sin(2.0 * M_PI * phase));
    obs[9 + 3 * kN + 1] = static_cast<float>(std::cos(2.0 * M_PI * phase));

    float act[kN];
    policy_.forward(obs, act);
    for (int i = 0; i < kN; ++i) {
      if (!std::isfinite(act[i])) {
        RCLCPP_WARN(get_logger(), "Politika vratila nekonecne hodnoty, preskakujem");
        return;
      }
    }
    std::array<float, kN> target;
    for (int i = 0; i < kN; ++i) {
      action_[i] = act[i];
      target[i] = static_cast<float>(act[i] * action_scale_ + default_[i]);
    }
    if (handoff_left_ > 0) {   // po vstavani: plynuly prechod z drepu na vystup politiky
      const float a = static_cast<float>(handoff_n_ - handoff_left_) / handoff_n_;
      for (int i = 0; i < kN; ++i) {target[i] = handoff_hold_[i] * (1.0f - a) + target[i] * a;}
      --handoff_left_;
    }
    publish_targets(target);

    // meranie pre porovnanie s MuJoCo
    float err_mean = 0.0f, dq_mean = 0.0f;
    for (int i = 0; i < kN; ++i) {
      const float e = std::fabs(target[i] - q_[i]);
      err_mean += e / kN;
      err_max_ = std::max(err_max_, e);
      dq_mean += std::fabs(dq_[i]) / kN;
      q_min_[i] = std::min(q_min_[i], q_[i]);
      q_max_[i] = std::max(q_max_[i], q_[i]);
      t_min_[i] = std::min(t_min_[i], target[i]);
      t_max_[i] = std::max(t_max_[i], target[i]);
    }
    err_sum_ += err_mean;
    dq_sum_ += dq_mean;
    if (have_last_t_) {dt_max_ = std::max(dt_max_, now - last_t_);}
    last_t_ = now;
    have_last_t_ = true;

    ++ticks_;
    if (ticks_ % 50 == 0) {
      const double imu_t = imu_.header.stamp.sec + imu_.header.stamp.nanosec * 1e-9;
      RCLCPP_DEBUG(get_logger(),
        "gravity_z=%+.2f cmd_x=%+.1f | chyba_poloh priem=%.3f max=%.2f | priem|dq|=%.2f "
        "| max_dt=%.0f ms | vek_imu=%.0f ms",
        gravity[2], cmd_[0], err_sum_ / 50, err_max_, dq_sum_ / 50, dt_max_ * 1000.0,
        imu_t > 0 ? (now - imu_t) * 1000.0 : 0.0);
      err_sum_ = 0.0; err_max_ = 0.0f; dq_sum_ = 0.0; dt_max_ = 0.0;
    }
    if (ticks_ % 200 == 0) {
      std::string a, b;
      char buf[16];
      for (int i = 0; i < kN; ++i) {
        std::snprintf(buf, sizeof(buf), "%.2f ", q_max_[i] - q_min_[i]); a += buf;
        std::snprintf(buf, sizeof(buf), "%.2f ", t_max_[i] - t_min_[i]); b += buf;
      }
      RCLCPP_DEBUG(get_logger(), "amplituda klbov [rad] L(HP HR HY K AP AR) P(HP HR HY K AP AR): %s",
        a.c_str());
      RCLCPP_DEBUG(get_logger(), "amplituda CIELOV  [rad] (cmd_x=%+.1f): %s", cmd_[0], b.c_str());
      q_min_.fill(9.0f); q_max_.fill(-9.0f); t_min_.fill(9.0f); t_max_.fill(-9.0f);
    }
  }

  humanoid_walk::G1Policy policy_;
  // WALK_HYBRID
  humanoid_walk::WalkMjlab walk_mjlab_;
  bool use_hybrid_{true};
  HybState hyb_state_{HybState::kStand};
  std::array<float, kN> mj_action_{}, hold_pose_{}, last_target_{}, ramp_from_{};
  std::array<float, kN> stand_kratio_{}, stand_dratio_{};
  double stand_after_{1.5}, ramp_s_{0.5}, stand_blend_s_{0.3}, cmd_dead_{0.05}, hyb_idle_{0.0}, hyb_ramp_t_{0.0};
  long step_counter_{0}, hyb_last_step_{-10};
  std::array<float, kN> default_{}, q_{}, dq_{}, action_{};
  std::array<float, kN> q_min_{}, q_max_{}, t_min_{}, t_max_{};
  // drzanie miesta
  bool hold_enabled_{true}, hold_anchor_{false}, hold_odom_anchor_{false}, have_odom_{false};
  double hold_kp_{1.0}, hold_kv_{1.0}, hold_max_{0.5}, hold_yaw_kp_{1.0};
  double hold_idle_{0.0}, hold_last_t_{0.0}, hold_yaw0_{0.0}, hold_x0_{0.0}, hold_y0_{0.0};
  double odom_x_{0.0}, odom_y_{0.0}, odom_vx_{0.0}, odom_vy_{0.0}, odom_yaw_{0.0};
  double odom_stamp_{0.0}, odom_wall_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::array<double, 3> cmd_scale_{}, cmd_{0.0, 0.0, 0.0};
  Mat3 R_p_i_{};
  double ang_vel_scale_{0.25}, dof_pos_scale_{1.0}, dof_vel_scale_{0.05}, action_scale_{0.25};
  double period_{0.8}, fall_gravity_z_{-0.4}, cmd_timeout_{0.0};
  int hold_ticks_max_{150};

  std::vector<size_t> joint_index_;
  sensor_msgs::msg::Imu imu_;
  bool have_joints_{false}, have_imu_{false};
  bool enabled_{false}, enable_requested_{false};
  int hold_ticks_{0}, detach_left_{0};
  bool have_t0_{false}, have_cmd_stamp_{false}, have_last_t_{false};
  double t0_{0.0}, cmd_stamp_{0.0}, last_t_{0.0};
  long ticks_{0};
  double err_sum_{0.0}, dq_sum_{0.0}, dt_max_{0.0};
  float err_max_{0.0f};
  std::string detach_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr ref_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  // padanie a vstavanie
  std::string getup_file_, posture_{"upright"}, candidate_posture_{"upright"};
  int fall_ticks_{0}, candidate_ticks_{0};
  std::vector<std::string> getup_names_;
  std::vector<std::vector<float>> getup_q_;
  std::vector<float> getup_start_;
  std::vector<int> getup_leg_idx_;
  std::map<std::string, double> all_pos_;
  double getup_dt_{0.02}, getup_blend_s_{1.0};
  long getup_tick_{0};
  bool getup_active_{false};
  double handoff_t0_{18.9}, handoff_gx_{0.05}, handoff_blend_s_{0.36};
  int handoff_left_{0}, handoff_n_{1};
  std::array<float, kN> handoff_hold_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fall_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr getup_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posture_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr upper_pub_;
  // vstavanie politikou (RL)
  enum class RlState {kIdle, kSwitchIn, kRun, kHold, kSwitchOut};
  RlState rl_state_{RlState::kIdle};
  humanoid_walk::GetupRl getup_rl_;
  std::string rl_mode_{"rl"}, rl_weights_, rl_ctrl_, rl_legs_ctrl_, rl_upper_ctrl_, rl_jsb_;
  double rl_arm_phase_{1.5}, rl_handoff_blend_{0.1}, rl_stand_t_{-1.0};
  int rl_switch_state_{0}, rl_switch_wait_{0};   // 0 nic, 1 caka, 2 hotovo, 3 chyba
  long rl_ticks_{0};
  int rl_post_{0};
  double rl_tilt_{0.10}, rl_wait_{4.0}, rl_hold_t_{0.0};
  bool rl_hold_warned_{false};
  bool rl_success_{false};
  std::array<float, kN> rl_leg_hold_{};
  std::map<std::string, double> all_vel_;
  rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr rl_ref_pub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr rl_switch_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WalkPolicyNode>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "walk_policy_node: %s\n", e.what());
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
