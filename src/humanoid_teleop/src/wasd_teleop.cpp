// wasd_teleop: klavesnica -> /cmd_vel (+ zapnutie/vypnutie politiky chodze)
//
// Klaves (kazde stlacenie zmeni prikaz, ako v MuJoCo dema):
//   W / S      rychlost dopredu / dozadu
//   A / D      posun do strany vlavo / vpravo (ako krab)
//   Q / E      otacanie vlavo / vpravo
//   MEDZERA    stop
//   P / O      zapnut / vypnut politiku chodze (/walk_policy/enable)
//   X          koniec
// Sipky fungujuju ako W/S/A/D.
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class WasdTeleop : public rclcpp::Node
{
public:
  WasdTeleop()
  : Node("wasd_teleop")
  {
    step_vx_ = declare_parameter<double>("step_vx", 0.1);
    step_vy_ = declare_parameter<double>("step_vy", 0.1);
    step_wz_ = declare_parameter<double>("step_wz", 0.2);
    max_vx_ = declare_parameter<double>("max_vx", 1.0);
    min_vx_ = declare_parameter<double>("min_vx", -0.5);
    max_vy_ = declare_parameter<double>("max_vy", 0.5);
    max_wz_ = declare_parameter<double>("max_wz", 1.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    active_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/skills/active", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {skill_active_ = m->data;});
    enable_pub_ = create_publisher<std_msgs::msg::Bool>("/walk_policy/enable", 10);
    fall_pub_ = create_publisher<std_msgs::msg::String>("/chappie/fall", 10);
    getup_pub_ = create_publisher<std_msgs::msg::Empty>("/chappie/getup", 10);
    pose_pub_ = create_publisher<std_msgs::msg::String>("/skills/pose", 10);

    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("wasd_teleop musi bezat v terminali (stdin nie je tty)");
    }
    tcgetattr(STDIN_FILENO, &old_tio_);
    termios raw = old_tio_;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    tty_changed_ = true;

    key_timer_ = create_wall_timer(20ms, [this]() {read_keys();});
    pub_timer_ = create_wall_timer(100ms, [this]() {publish_cmd();});

    // BANNER_SIMPLE
    std::printf(
      "\nChappie\n"
      "  W/S  A/D  Q/E   - chodza, bokom, otacanie\n"
      "  MEDZERA stop   P/O politika   R/L/T/F ruky   G pad   U vstat   X koniec\n\n");
    print_status();
  }

  ~WasdTeleop() override
  {
    if (tty_changed_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }
    std::printf("\n");
  }

private:
  static double clean(double v)
  {
    v = std::round(v * 100.0) / 100.0;   // bez 0.30000000004
    return std::fabs(v) < 1e-9 ? 0.0 : v;
  }

  void print_status()
  {
    std::printf("\rvx=%+.1f  vy=%+.1f  wz=%+.1f      ", vx_, vy_, wz_);
    std::fflush(stdout);
  }

  void read_keys()
  {
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
      handle(c);
    }
  }

  void handle(char c)
  {
    // sipky prichadzaju ako ESC [ A/B/C/D
    if (esc_ == 0 && c == 27) {
      esc_ = 1;
      return;
    }
    if (esc_ == 1) {
      esc_ = (c == '[' || c == 'O') ? 2 : 0;
      return;
    }
    if (esc_ == 2) {
      esc_ = 0;
      switch (c) {
        case 'A': c = 'w'; break;
        case 'B': c = 's'; break;
        case 'C': c = 'e'; break;
        case 'D': c = 'q'; break;
        default: return;
      }
    }

    switch (std::tolower(static_cast<unsigned char>(c))) {
      case 'w': vx_ += step_vx_; break;
      case 's': vx_ -= step_vx_; break;
      case 'a': vy_ += step_vy_; break;
      case 'd': vy_ -= step_vy_; break;
      case 'q': wz_ += step_wz_; break;
      case 'e': wz_ -= step_wz_; break;
      case ' ': vx_ = vy_ = wz_ = 0.0; break;
      case 'r': send_pose("raise_right_arm"); return;
      case 'l': send_pose("raise_left_arm"); return;
      case 't': send_pose("raise_both_arms"); return;
      case 'f': send_pose("arms_down"); return;
      case 'g': {
        std_msgs::msg::String msg;
        msg.data = "forward";
        fall_pub_->publish(msg);
        return;
      }
      case 'u': getup_pub_->publish(std_msgs::msg::Empty()); return;
      case 'p': send_enable(true); return;
      case 'o': send_enable(false); return;
      case 'x':
        vx_ = vy_ = wz_ = 0.0;
        publish_cmd();
        rclcpp::shutdown();
        return;
      default: return;
    }
    vx_ = clean(std::clamp(vx_, min_vx_, max_vx_));
    vy_ = clean(std::clamp(vy_, -max_vy_, max_vy_));
    wz_ = clean(std::clamp(wz_, -max_wz_, max_wz_));
    publish_cmd();
    print_status();
  }

  void send_pose(const char * name)
  {
    std_msgs::msg::String msg;
    msg.data = name;
    pose_pub_->publish(msg);
    std::printf("\rpoza: %s                         \n", name);
    print_status();
  }

  void send_enable(bool on)
  {
    std_msgs::msg::Bool msg;
    msg.data = on;
    enable_pub_->publish(msg);
    std::printf("\rpolitika %s                         \n", on ? "ZAPNUTA (vyziadane)" : "VYPNUTA");
    print_status();
  }

  void publish_cmd()
  {
    if (skill_active_) {return;}   // bezi skill, /cmd_vel posiela on
    geometry_msgs::msg::Twist t;
    t.linear.x = vx_;
    t.linear.y = vy_;
    t.angular.z = wz_;
    cmd_pub_->publish(t);
  }

  double step_vx_{0.1}, step_vy_{0.1}, step_wz_{0.2};
  double max_vx_{1.0}, min_vx_{-0.5}, max_vy_{0.5}, max_wz_{1.0};
  double vx_{0.0}, vy_{0.0}, wz_{0.0};
  int esc_{0};
  bool tty_changed_{false};
  termios old_tio_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  bool skill_active_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fall_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr getup_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr key_timer_, pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<WasdTeleop>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "wasd_teleop: %s\n", e.what());
    rc = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
