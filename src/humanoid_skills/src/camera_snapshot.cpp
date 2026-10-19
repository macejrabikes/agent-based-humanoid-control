// camera_snapshot: ulozi aktualny obraz z kamery robota do PNG.
//   ros2 service call /camera/snapshot std_srvs/srv/Trigger
// Subor: ~/chappie_snapshots/snap_<cas>.png (a vzdy aj latest.png). Cesta je v odpovedi sluzby.
// Tento obrazok sa neskor posiela AI agentovi.
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "humanoid_skills/png_writer.hpp"

class CameraSnapshot : public rclcpp::Node
{
public:
  CameraSnapshot()
  : Node("camera_snapshot")
  {
    const std::string topic = declare_parameter<std::string>("topic", "/chappie/camera/image");
    dir_ = declare_parameter<std::string>("dir", "");
    if (dir_.empty()) {
      const char * home = std::getenv("HOME");
      dir_ = std::string(home ? home : ".") + "/chappie_snapshots";
    }
    std::filesystem::create_directories(dir_);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {last_ = m;});
    srv_ = create_service<std_srvs::srv::Trigger>(
      "/camera/snapshot",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {on_snapshot(*res);});
    RCLCPP_INFO(get_logger(), "Snimky z %s ukladam do %s (sluzba /camera/snapshot)", topic.c_str(), dir_.c_str());
  }

private:
  void on_snapshot(std_srvs::srv::Trigger::Response & res)
  {
    if (!last_) {
      res.success = false;
      res.message = "Zatial neprisiel ziadny obraz z kamery (bezi simulacia a most?).";
      return;
    }
    const auto & im = *last_;
    std::vector<uint8_t> rgb(static_cast<size_t>(im.width) * im.height * 3);
    const bool bgr = (im.encoding == "bgr8");
    const bool rgba = (im.encoding == "rgba8" || im.encoding == "bgra8");
    if (!(im.encoding == "rgb8" || bgr || rgba)) {
      res.success = false;
      res.message = "Nepodporovany format obrazu: " + im.encoding;
      return;
    }
    const size_t px = rgba ? 4 : 3;
    for (uint32_t y = 0; y < im.height; ++y) {
      for (uint32_t x = 0; x < im.width; ++x) {
        const uint8_t * s = &im.data[static_cast<size_t>(y) * im.step + x * px];
        uint8_t * d = &rgb[(static_cast<size_t>(y) * im.width + x) * 3];
        const bool swap = (im.encoding == "bgr8" || im.encoding == "bgra8");
        d[0] = swap ? s[2] : s[0];
        d[1] = s[1];
        d[2] = swap ? s[0] : s[2];
      }
    }
    char name[64];
    const std::time_t t = std::time(nullptr);
    std::strftime(name, sizeof(name), "snap_%Y%m%d_%H%M%S.png", std::localtime(&t));
    const std::string path = dir_ + "/" + name;
    if (!pngw::write_rgb(path, rgb.data(), im.width, im.height, im.width * 3) ||
      !pngw::write_rgb(dir_ + "/latest.png", rgb.data(), im.width, im.height, im.width * 3))
    {
      res.success = false;
      res.message = "Nepodarilo sa zapisat subor do " + dir_;
      return;
    }
    res.success = true;
    res.message = path;
    RCLCPP_INFO(get_logger(), "Snimka ulozena: %s", path.c_str());
  }

  std::string dir_;
  sensor_msgs::msg::Image::ConstSharedPtr last_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSnapshot>());
  rclcpp::shutdown();
  return 0;
}
