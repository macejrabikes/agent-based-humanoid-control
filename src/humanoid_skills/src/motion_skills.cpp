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
    p.stop_dist = declare_parameter<double>("obstacle_stop", p.stop_dist);       // LIDAR_PATCH
    p.block_dist = declare_parameter<double>("obstacle_block", p.block_dist);
    p.lidar_guard = declare_parameter<bool>("lidar_guard", p.lidar_guard);
    lp_.floor_z = declare_parameter<double>("lidar_floor_z", lp_.floor_z);
    lp_.self_x = declare_parameter<double>("lidar_self_x", lp_.self_x);
    lp_.self_y = declare_parameter<double>("lidar_self_y", lp_.self_y);
    exec_ = std::make_unique<skills::SkillExecutor>(p);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pose_pub_ = create_publisher<std_msgs::msg::String>("/skills/pose", 10);
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
    sens_.have_odom = true;
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
    char buf[200];
    std::snprintf(buf, sizeof(buf),
      "{\"front\":%.2f,\"back\":%.2f,\"left\":%.2f,\"right\":%.2f,\"arms_up\":%s}",
      std::min(sens_.lid_front, 10.0), std::min(sens_.lid_back, 10.0),
      std::min(sens_.lid_left, 10.0), std::min(sens_.lid_right, 10.0), exec_->arms_up() ? "true" : "false");
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
