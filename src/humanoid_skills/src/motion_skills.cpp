// motion_skills: pohybove skills robota (C++), ROS2 action server /skills/execute.
//
//   ros2 action send_goal /skills/execute humanoid_interfaces/action/Skill "{skill: 'walk_forward', value: 1.0}" --feedback
//
// Zoznam skills (JSON, pre AI agenta): topic /skills/catalog (transient_local).
// Neplatny prikaz sa odmietne a vysledok obsahuje status + message.
// Pocas skillu uzol posiela /cmd_vel a nastavi /skills/active = true (teleop vtedy mlci).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <humanoid_interfaces/action/skill.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <std_msgs/msg/string.hpp>

#include "humanoid_skills/skill_core.hpp"

using namespace std::chrono_literals;
using SkillAction = humanoid_interfaces::action::Skill;
using GoalHandle = rclcpp_action::ServerGoalHandle<SkillAction>;

class MotionSkills : public rclcpp::Node
{
public:
  MotionSkills()
  : Node("motion_skills")
  {
    skills::Params p;
    p.walk_speed = declare_parameter<double>("walk_speed", p.walk_speed);
    p.back_speed = declare_parameter<double>("back_speed", p.back_speed);
    p.side_speed = declare_parameter<double>("side_speed", p.side_speed);
    p.turn_rate = declare_parameter<double>("turn_rate", p.turn_rate);
    p.accel = declare_parameter<double>("accel", p.accel);
    p.decel = declare_parameter<double>("decel", p.decel);
    p.lag = declare_parameter<double>("lag", p.lag);
    p.arm_s = declare_parameter<double>("arm_s", p.arm_s);
    p.camera_hfov_deg = declare_parameter<double>("camera_hfov_deg", p.camera_hfov_deg);      // TT_PATCH
    p.stop_dist = declare_parameter<double>("obstacle_stop", p.stop_dist);       // LIDAR_PATCH
    p.block_dist = declare_parameter<double>("obstacle_block", p.block_dist);
    p.lidar_guard = declare_parameter<bool>("lidar_guard", p.lidar_guard);
    lp_.floor_z = declare_parameter<double>("lidar_floor_z", lp_.floor_z);
    lp_.self_x = declare_parameter<double>("lidar_self_x", lp_.self_x);
    lp_.self_y = declare_parameter<double>("lidar_self_y", lp_.self_y);
    lp_.flip = declare_parameter<bool>("lidar_flip", true);          // SYM_PATCH: URDF ma roll 180 st.
    exec_ = std::make_unique<skills::SkillExecutor>(p);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pose_pub_ = create_publisher<std_msgs::msg::String>("/skills/pose", 10);
    // SKILLS2_PATCH: gesta (trajektoria rúk) a vstavanie zo zeme
    gesture_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      declare_parameter<std::string>("upper_topic", "/upper_body_controller/joint_trajectory"), 10);
    getup_pub_ = create_publisher<std_msgs::msg::Empty>("/chappie/getup", 10);
    hand_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      declare_parameter<std::string>("hand_topic", "/hand_controller/joint_trajectory"), 10);
    rest_on_start_ = declare_parameter<bool>("rest_on_start", true);
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {js_[m->name[i]] = m->position[i];}
        sens_.have_hands = js_.count("right_hand_thumb_0_joint") > 0;
        if (m->header.stamp.sec > 0 || m->header.stamp.nanosec > 0) {
          sens_.sim_t = m->header.stamp.sec + 1e-9 * m->header.stamp.nanosec;
        }
        maybe_rest_on_start();
      });
    active_pub_ = create_publisher<std_msgs::msg::Bool>("/skills/active", 10);
    catalog_pub_ = create_publisher<std_msgs::msg::String>(
      "/skills/catalog", rclcpp::QoS(1).transient_local().reliable());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/pelvis", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_imu(*m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/chappie/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(*m);});

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {on_cloud(*m);});
    lidar_pub_ = create_publisher<std_msgs::msg::String>("/lidar/distances", 10);
    lidar_timer_ = create_wall_timer(200ms, [this]() {publish_lidar();});

    server_ = rclcpp_action::create_server<SkillAction>(
      this, "/skills/execute",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const SkillAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandle> gh) {on_accepted(gh);});

    std_msgs::msg::String cat;
    cat.data = exec_->catalog_json();
    catalog_pub_->publish(cat);

    timer_ = create_wall_timer(20ms, [this]() {tick();});
    RCLCPP_INFO(get_logger(), "Skills pripravene: %zu (action /skills/execute, zoznam na /skills/catalog)",
      exec_->specs().size());
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const double x = m.orientation.x, y = m.orientation.y, z = m.orientation.z, w = m.orientation.w;
    sens_.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    sens_.gz = -(1.0 - 2.0 * (x * x + y * y));
    lp_.up_x = 2.0 * (x * z - w * y);       // "hore" v suradniciach panvy (pre lidar)
    lp_.up_y = 2.0 * (y * z + w * x);
    lp_.up_z = 1.0 - 2.0 * (x * x + y * y);
    sens_.have_imu = true;
  }

  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    sens_.ox = m.pose.pose.position.x;
    sens_.oy = m.pose.pose.position.y;
    const auto & q = m.pose.pose.orientation;          // NAV_PATCH: natocenie v suradniciach odometrie
    sens_.oyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    sens_.have_odom = true;
    exec_->observe(sens_);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & m)
  {
    int ox = -1, oy = -1, oz = -1;
    for (const auto & f : m.fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {continue;}
      if (f.name == "x") {ox = static_cast<int>(f.offset);}
      else if (f.name == "y") {oy = static_cast<int>(f.offset);}
      else if (f.name == "z") {oz = static_cast<int>(f.offset);}
    }
    if (ox < 0 || oy < 0 || oz < 0 || m.is_bigendian) {return;}
    const size_t n = static_cast<size_t>(m.width) * m.height;
    if (n == 0 || m.point_step < 12 || m.data.size() < n * m.point_step) {return;}
    lid_ = skills::lidar_from_cloud(m.data.data(), n, m.point_step, ox, oy, oz, lp_);
    lid_stamp_ = now();
    lid_got_ = true;
  }

  // prenesie posledne vzdialenosti z lidaru do sens_ (ak su cerstve)
  void refresh_lidar()
  {
    const bool fresh = lid_got_ && lid_.seen >= 50 && (now() - lid_stamp_).seconds() < 1.0;
    sens_.have_lidar = fresh;
    sens_.lid_front = fresh ? lid_.front : 99.0;
    sens_.lid_back = fresh ? lid_.back : 99.0;
    sens_.lid_left = fresh ? lid_.left : 99.0;
    sens_.lid_right = fresh ? lid_.right : 99.0;
  }

  void publish_lidar()
  {
    refresh_lidar();
    if (!sens_.have_lidar) {return;}
    char buf[240];
    std::snprintf(buf, sizeof(buf),
      "{\"front\":%.2f,\"back\":%.2f,\"left\":%.2f,\"right\":%.2f,\"arms_up\":%s,\"upright\":%s}",
      std::min(sens_.lid_front, 10.0), std::min(sens_.lid_back, 10.0),
      std::min(sens_.lid_left, 10.0), std::min(sens_.lid_right, 10.0), exec_->arms_up() ? "true" : "false",
      (!sens_.have_imu || sens_.gz < -0.7) ? "true" : "false");
    std_msgs::msg::String s;
    s.data = buf;
    lidar_pub_->publish(s);
  }

  void on_accepted(std::shared_ptr<GoalHandle> gh)
  {
    refresh_lidar();
    const auto goal = gh->get_goal();
    auto result = std::make_shared<SkillAction::Result>();
    skills::Result r;
    if (!exec_->start(goal->skill, goal->value, sens_, r)) {
      fill(*result, r);
      RCLCPP_WARN(get_logger(), "Skill '%s' odmietnuty: %s", goal->skill.c_str(), r.message.c_str());
      gh->abort(result);
      return;
    }
    handle_ = gh;
    last_tick_ = now();
    set_active(true);
    RCLCPP_INFO(get_logger(), "Skill '%s' (%.2f) spusteny", goal->skill.c_str(), goal->value);
  }

  void tick()
  {
    if (!handle_) {return;}
    refresh_lidar();
    const rclcpp::Time t = now();
    double dt = (t - last_tick_).seconds();
    last_tick_ = t;
    dt = std::min(std::max(dt, 0.001), 0.1);

    skills::Output out;
    bool canceled = false;
    if (handle_->is_canceling()) {
      out = exec_->cancel();
      canceled = true;
    } else {
      // odometria sa moze stratit -> neber stare data
      out = exec_->update(sens_, dt);
    }
    if (out.publish_cmd) {
      geometry_msgs::msg::Twist c;
      c.linear.x = out.vx;
      c.linear.y = out.vy;
      c.angular.z = out.wz;
      cmd_pub_->publish(c);
    }
    if (out.send_pose) {
      std_msgs::msg::String s;
      s.data = out.pose;
      pose_pub_->publish(s);
    }
    if (!out.gesture.empty()) {publish_gesture(out.gesture, out.gesture_arm, out.gesture_total);}                       // SKILLS2_PATCH
    if (out.send_getup) {getup_pub_->publish(std_msgs::msg::Empty());}
    if (out.finished) {
      auto result = std::make_shared<SkillAction::Result>();
      fill(*result, out.result);
      RCLCPP_INFO(get_logger(), "Skill hotovy: %s - %s", out.result.status.c_str(),
        out.result.message.c_str());
      auto gh = handle_;
      handle_.reset();
      set_active(false);
      if (canceled) {
        gh->canceled(result);
      } else if (out.result.success) {
        gh->succeed(result);
      } else {
        gh->abort(result);
      }
      return;
    }
    auto fb = std::make_shared<SkillAction::Feedback>();
    fb->state = out.state;
    fb->progress = out.progress;
    fb->value_done = out.value_done;
    handle_->publish_feedback(fb);
  }

  // ---- SKILLS2_PATCH: gesta rukami (a prstami). Trajektoria ide regulatoru hornej casti (upper_body_controller);
  // ruka startuje z NAMERANEJ polohy (/joint_states), plynule prejde do zakladnej polohy a na konci je presne v nej.
  // Pozicie sú v konvencii PRAVEJ ruky (lava = zrkadlo: roll, yaw, wrist_roll s opacnym znamienkom) a overene
  // doprednou kinematikou G1: elbow = 0 znamena predlaktie kolmo na ramenu, elbow ~1.57 = natiahnuta ruka.
  struct Pose5 {double p, r, y, e, w;};       // shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll
  // ZAKLADNA poloha PRAVEJ ruky (rovnaka ako vychodzia poloha pri trenovani politiky chodze); lava = zrkadlo.
  static constexpr double kRest[5] = {0.2, -0.2, 0.0, 0.6, 0.0};

  static double smooth(double u)
  {
    u = std::min(1.0, std::max(0.0, u));
    return u * u * (3.0 - 2.0 * u);
  }

  static double gesture_ramp(const std::string & g)           // ako dlho ruka nabieha / sa vracia [s]
  {
    if (g == "point_forward" || g == "thumbs_up" || g == "push") {return 2.0;}
    if (g == "salute") {return 1.8;}
    if (g == "clap" || g == "dance") {return 1.6;}
    if (g == "bow") {return 2.5;}
    return 1.4;
  }

  using JV = std::vector<std::pair<std::string, double>>;     // klb -> ciel

  static void add_arm(JV & o, const char * side, const Pose5 & p)
  {
    const char * parts[5] = {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"};
    const double sgn[5] = {1.0, -1.0, -1.0, 1.0, -1.0};      // zrkadlenie pre lavu ruku
    const double v[5] = {p.p, p.r, p.y, p.e, p.w};
    const bool left = (side[0] == 'l');
    for (int i = 0; i < 5; ++i) {o.emplace_back(std::string(side) + "_" + parts[i] + "_joint", left ? v[i] * sgn[i] : v[i]);}
  }

  // zakladna poloha klbu (ruky: kRest, zrkadlovo pre lavu; driek a ostatne: 0)
  static double canon_of(const std::string & name)
  {
    const char * parts[5] = {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"};
    const double sgn[5] = {1.0, -1.0, -1.0, 1.0, -1.0};
    for (const char * side : {"right", "left"}) {
      for (int i = 0; i < 5; ++i) {
        if (name == std::string(side) + "_" + parts[i] + "_joint") {
          return kRest[i] * ((side[0] == 'l') ? sgn[i] : 1.0);
        }
      }
    }
    return 0.0;
  }

  // ciele klbov gesta v case t (poradie klbov je stale rovnake)
  static JV gesture_pose(const std::string & g, int arm, double t, double T)
  {
    const double rp = gesture_ramp(g);
    const double tau = (t > rp && t < T - rp) ? (t - rp) : 0.0;          // cas vnutri aktivnej casti
    const bool act = (t > rp && t < T - rp);
    JV o;
    if (g == "wave") {
      const double wig = act ? std::sin(2.0 * M_PI * 1.0 * tau) : 0.0;
      const Pose5 p{-0.12, -1.57, -1.57, 0.3 + 0.55 * wig, 0.0};          // ruka do strany, predlaktie hore, mavanie v lakti
      if (arm == 1 || arm == 3) {add_arm(o, "right", p);}
      if (arm == 2 || arm == 3) {add_arm(o, "left", p);}
    } else if (g == "cheer") {
      const double wig = act ? std::sin(2.0 * M_PI * 1.4 * tau) : 0.0;     // ruky vysoko nad hlavou, striedavo pumpuju
      add_arm(o, "right", {-2.7 + 0.2 * wig, -0.9, 0.0, 1.0 + 0.5 * wig, 0.0});
      add_arm(o, "left", {-2.7 - 0.2 * wig, -0.9, 0.0, 1.0 - 0.5 * wig, 0.0});
    } else if (g == "point_forward") {
      add_arm(o, "right", {-1.62, -0.12, 0.0, 1.5, 0.0});                  // ruka natiahnuta rovno dopredu
    } else if (g == "thumbs_up") {
      add_arm(o, "right", {-0.9, -0.3, 0.0, 0.9, 0.6});                    // predlaktie dopredu, palec hore
    } else if (g == "push") {
      // NAV_PATCH: obe ruky dopredu vo vyske hrudnika (dlane/zapastia ~ 0,9 m nad zemou), overene FK modelu G1, bez kolizii
      add_arm(o, "right", {-1.0, -0.2, 0.0, 1.4, 0.0});
      add_arm(o, "left", {-1.0, -0.2, 0.0, 1.4, 0.0});
    } else if (g == "salute") {
      add_arm(o, "right", {0.0, -1.43, -1.0, -0.95, 0.0});                 // dlan k celu, lakeť do strany (overene FK)
    } else if (g == "clap") {
      // ruky sa striedavo otvaraju (f = 0) a spajaju (f = 1: dlane sa prave dotknu = 0.8 vzdialenosti k plne spojenej polohe);
      // polohy z FK modelu G1
      const double f = 0.8 * (act ? 0.5 - 0.5 * std::cos(2.0 * M_PI * 1.5 * tau) : 0.0);
      const double op[5] = {-0.21, -0.9, 0.1, -0.37, 0.0}, cl[5] = {-0.77, -0.78, 0.68, -0.29, 0.0};
      const Pose5 p{op[0] + (cl[0] - op[0]) * f, op[1] + (cl[1] - op[1]) * f, op[2] + (cl[2] - op[2]) * f,
        op[3] + (cl[3] - op[3]) * f, 0.0};
      add_arm(o, "right", p);
      add_arm(o, "left", p);
    } else if (g == "bow") {
      o.emplace_back("waist_pitch_joint", 0.35);                           // uklon dopredu (limit klbu 0.52)
    } else if (g == "dance") {
      const double s1 = act ? std::sin(2.0 * M_PI * 0.9 * tau) : 0.0;
      const double sw = act ? std::sin(2.0 * M_PI * 0.45 * tau) : 0.0;
      add_arm(o, "right", {-1.4 + 0.5 * s1, -0.7, 0.0, 1.0 + 0.4 * s1, 0.0});
      add_arm(o, "left", {-1.4 - 0.5 * s1, -0.7, 0.0, 1.0 - 0.4 * s1, 0.0});
      o.emplace_back("waist_yaw_joint", 0.35 * sw);                        // pohupovanie bokmi
    }
    return o;
  }

  // arm: 1 = prava, 2 = lava, 3 = obe (len pre wave); T = celkova dlzka vratane nabehu a navratu
  void publish_gesture(const std::string & g, int arm = 1, double T = 6.0)
  {
    const double rp = gesture_ramp(g);
    trajectory_msgs::msg::JointTrajectory tr;
    // VZDY sa posiela CELA horna cast (driek + obe ruky); klby mimo gesta idu do zakladnej polohy,
    // takze po kazdom geste je robot cely v zakladnej polohe (aj keby predchadzajuce gesto nieco nechalo)
    std::vector<std::string> names = {"waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"};
    for (const char * side : {"right", "left"}) {
      for (const char * part : {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"}) {
        names.push_back(std::string(side) + "_" + part + "_joint");
      }
    }
    std::vector<double> meas, canon;
    for (const auto & nme : names) {
      tr.joint_names.push_back(nme);
      canon.push_back(canon_of(nme));
      const auto it = js_.find(nme);
      meas.push_back((it != js_.end()) ? it->second : canon.back());
    }
    for (double t = 0.1; t <= T + 1e-9; t += 0.1) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      const double up = smooth(t / rp) * (1.0 - smooth((t - (T - rp)) / rp));
      const JV tg = gesture_pose(g, arm, t, T);
      for (size_t k = 0; k < names.size(); ++k) {
        double target = canon[k];
        for (const auto & kv : tg) {if (kv.first == names[k]) {target = kv.second;}}
        const double base = meas[k] + (canon[k] - meas[k]) * smooth(t / rp);   // plynule do zakladnej polohy
        pt.positions.push_back(base + (target - base) * up);
      }
      pt.time_from_start.sec = static_cast<int32_t>(t);
      pt.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
      tr.points.push_back(pt);
    }
    gesture_pub_->publish(tr);
    if (g == "point_forward" || g == "thumbs_up") {publish_fingers(g, T);}
    RCLCPP_INFO(get_logger(), "Gesto '%s' (%.1f s, ruka %d)", g.c_str(), T, arm);
  }

  // obe ruky plynule do zakladnej polohy (pri starte a po zmene)
  void publish_rest(double dur)
  {
    const char * parts[5] = {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"};
    const double sgn[5] = {1.0, -1.0, -1.0, 1.0, -1.0};
    trajectory_msgs::msg::JointTrajectory tr;
    double meas[10], canon[10];
    int n = 0;
    for (const char * side : {"right", "left"}) {
      for (int i = 0; i < 5; ++i, ++n) {
        const std::string name = std::string(side) + "_" + parts[i] + "_joint";
        tr.joint_names.push_back(name);
        canon[n] = kRest[i] * ((std::string(side) == "left") ? sgn[i] : 1.0);
        const auto it = js_.find(name);
        meas[n] = (it != js_.end()) ? it->second : canon[n];
      }
    }
    for (double t = 0.1; t <= dur + 1e-9; t += 0.1) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      for (int i = 0; i < 10; ++i) {pt.positions.push_back(meas[i] + (canon[i] - meas[i]) * smooth(t / dur));}
      pt.time_from_start.sec = static_cast<int32_t>(t);
      pt.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
      tr.points.push_back(pt);
    }
    gesture_pub_->publish(tr);
    RCLCPP_INFO(get_logger(), "Ruky: prechod do zakladnej polohy (%.1f s)", dur);
  }

  // jednorazovo po starte: ak ruky nie su v zakladnej polohe (po spawne mu visia lubovolne), daj ich tam
  void maybe_rest_on_start()
  {
    if (rest_done_ || !rest_on_start_) {return;}
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t_node0_).count() < 3.0) {return;}
    const char * parts[5] = {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"};
    double dev = 0.0;
    for (const char * side : {"right", "left"}) {
      for (int i = 0; i < 5; ++i) {
        const auto it = js_.find(std::string(side) + "_" + parts[i] + "_joint");
        if (it == js_.end()) {return;}                            // joint_states este nie su kompletne
        const double c = kRest[i] * ((std::string(side) == "left" && (i == 1 || i == 2 || i == 4)) ? -1.0 : 1.0);
        dev = std::max(dev, std::fabs(it->second - c));
      }
    }
    rest_done_ = true;
    if (dev > 0.12) {publish_rest(3.0);}
  }

  // prsty PRAVEJ ruky (hand_controller, funguje len pri spusteni simulacie s with_hands:=true)
  void publish_fingers(const std::string & g, double T)
  {
    const char * names[7] = {"thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1"};
    double f[7];
    if (g == "point_forward") {     // ukazovak natiahnuty, stredny stiahnuty, palec pritlaceny
      const double q[7] = {0.0, 0.6, -1.3, 0.0, 0.0, 1.5, 1.7};
      std::copy(q, q + 7, f);
    } else {                        // thumbs_up: pest (ukazovak, stredny stiahnute), palec vztycen hore
      const double q[7] = {-1.04, -1.04, 0.0, 1.5, 1.7, 1.5, 1.7};   // thumb_0/1 na limite: palec smeruje hore
      std::copy(q, q + 7, f);
    }
    trajectory_msgs::msg::JointTrajectory tr;
    for (const char * n : names) {tr.joint_names.push_back(std::string("right_hand_") + n + "_joint");}
    for (double t = 0.1; t <= T + 1e-9; t += 0.1) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      const double u = smooth((t - 0.8) / 1.2) * (1.0 - smooth((t - (T - 2.2)) / 1.2));
      for (int i = 0; i < 7; ++i) {pt.positions.push_back(f[i] * u);}
      pt.time_from_start.sec = static_cast<int32_t>(t);
      pt.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
      tr.points.push_back(pt);
    }
    hand_pub_->publish(tr);
  }

  static void fill(SkillAction::Result & dst, const skills::Result & src)
  {
    dst.success = src.success;
    dst.status = src.status;
    dst.message = src.message;
    dst.value_done = src.value_done;
  }

  void set_active(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    active_pub_->publish(m);
  }

  std::unique_ptr<skills::SkillExecutor> exec_;
  skills::Sensors sens_;
  std::shared_ptr<GoalHandle> handle_;
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_pub_, catalog_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gesture_pub_;   // SKILLS2_PATCH
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr getup_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr hand_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::map<std::string, double> js_;
  bool rest_done_ = false, rest_on_start_ = true;
  std::chrono::steady_clock::time_point t_node0_ = std::chrono::steady_clock::now();
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lidar_pub_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  skills::LidarParams lp_;
  skills::LidarSectors lid_;
  rclcpp::Time lid_stamp_{0, 0, RCL_ROS_TIME};
  bool lid_got_ = false;
  rclcpp_action::Server<SkillAction>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionSkills>());
  rclcpp::shutdown();
  return 0;
}
