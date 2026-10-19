// pose_skills: prehra pomenovanu pozu z poses.yaml.
// Vstup:  std_msgs/String na /skills/pose, napr. "arms_forward" alebo "crouch 5.0"
// Vystup: JointTrajectory na /body_controller a /hand_controller
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>

using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

struct Pose
{
  double duration = 3.0;
  std::map<std::string, double> joints;
};

class PoseSkills : public rclcpp::Node
{
public:
  PoseSkills()
  : Node("pose_skills")
  {
    declare_parameter<std::string>("poses_file", "");
    std::string file = get_parameter("poses_file").as_string();
    if (file.empty()) {
      file = ament_index_cpp::get_package_share_directory("humanoid_skills") +
        "/config/poses.yaml";
    }
    load_poses(file);

    declare_parameter<std::string>("body_topic", "/body_controller/joint_trajectory");
    body_pub_ = create_publisher<JointTrajectory>(
      get_parameter("body_topic").as_string(), 10);
    hand_pub_ = create_publisher<JointTrajectory>("/hand_controller/joint_trajectory", 10);
    sub_ = create_subscription<std_msgs::msg::String>(
      "/skills/pose", 10,
      std::bind(&PoseSkills::on_command, this, std::placeholders::_1));

    std::string names;
    for (const auto & kv : poses_) {
      names += kv.first + " ";
    }
    RCLCPP_INFO(get_logger(), "Nacitanych póz: %zu (%s)", poses_.size(), names.c_str());
  }

private:
  void load_poses(const std::string & file)
  {
    try {
      const YAML::Node root = YAML::LoadFile(file);
      for (const auto & p : root["poses"]) {
        Pose pose;
        for (const auto & kv : p.second) {
          const auto key = kv.first.as<std::string>();
          if (key == "duration") {
            pose.duration = kv.second.as<double>();
          } else {
            pose.joints[key] = kv.second.as<double>();
          }
        }
        poses_[p.first.as<std::string>()] = pose;
      }
    } catch (const YAML::Exception & e) {
      throw std::runtime_error("Chyba pri citani " + file + ": " + e.what());
    }
  }

  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    std::istringstream in(msg->data);
    std::string name;
    double duration = -1.0;
    in >> name;
    if (!(in >> duration)) {
      duration = -1.0;
    }

    const auto it = poses_.find(name);
    if (it == poses_.end()) {
      RCLCPP_WARN(get_logger(), "Neznama poza '%s'", name.c_str());
      return;
    }
    go_to(it->second, duration > 0.0 ? duration : it->second.duration);
    RCLCPP_INFO(get_logger(), "Poza '%s' (%.1f s)", name.c_str(),
      duration > 0.0 ? duration : it->second.duration);
  }

  void go_to(const Pose & pose, double duration)
  {
    JointTrajectory body, hand;
    JointTrajectoryPoint bp, hp;

    for (const auto & kv : pose.joints) {
      if (kv.first.find("_hand_") != std::string::npos) {
        hand.joint_names.push_back(kv.first);
        hp.positions.push_back(kv.second);
      } else {
        body.joint_names.push_back(kv.first);
        bp.positions.push_back(kv.second);
      }
    }

    const int32_t sec = static_cast<int32_t>(duration);
    const uint32_t nanosec = static_cast<uint32_t>((duration - sec) * 1e9);
    bp.time_from_start.sec = hp.time_from_start.sec = sec;
    bp.time_from_start.nanosec = hp.time_from_start.nanosec = nanosec;

    if (!body.joint_names.empty()) {
      body.points.push_back(bp);
      body_pub_->publish(body);
    }
    if (!hand.joint_names.empty()) {
      hand.points.push_back(hp);
      hand_pub_->publish(hand);
    }
  }

  std::map<std::string, Pose> poses_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr body_pub_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr hand_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSkills>());
  rclcpp::shutdown();
  return 0;
}
