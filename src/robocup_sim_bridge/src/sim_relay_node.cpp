// robocup_sim GT / kick-perception relay.
//
// 输入（域 10）：
//   /<robot>/imu/nav_all          基座位姿（仿真世界系 x/y/z + quat w,x,y,z）
//   /sim/ball_pose                球心世界坐标
//   /sim/goal_team_a|b            A/B 进攻球门世界坐标
//
// 输出（域 10）：
//   /soccer/ground_truth_pose/<robot>   Pose2D（brain GT；A 队 180° 翻转）
//   /soccer/ground_truth_ball/<robot>   Point（brain GT；A 队 180° 翻转）
//   /<robot>/perception/ball                    robot_interfaces/Ball（基座系）
//   /<robot>/perception/target_point            Point（球在基座系）
//   /<robot>/perception/target_destination      Point（进攻球门在基座系）
//   /<robot>/perception/target_goal_normal      Vector3（进攻方向在基座系）
//
// 设计约束：
//   - 只依赖 nav_all + /sim/*，不依赖旧 bridge；
//   - 任何输入超时/NaN 时跳过本周期输出，绝不发布 NaN；
//   - GT 的 A 队翻转（x=-x,y=-y,theta=yaw+pi）与旧 brain_zl_bridge 一致；
//   - 感知量用完整四元数 R(q)^T 转到基座系（与旧 kick_perception 同式）。
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/ball.hpp>
#include <yesense_interface/msg/nav_all.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

double WrapPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

struct Rot3 {
  std::array<double, 9> r{};  // row-major: v_world = R * v_base
};

Rot3 RotationFromQuat(double w, double x, double y, double z) {
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (n < 1e-9) return Rot3{};
  w /= n; x /= n; y /= n; z /= n;
  Rot3 out;
  out.r = {
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
      2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y),
  };
  return out;
}

double YawFromQuat(double w, double x, double y, double z) {
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (n < 1e-9) return 0.0;
  w /= n; x /= n; y /= n; z /= n;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// v_base = R^T * (v_world - p_world)
void WorldToBase(const Rot3& rot, const std::array<double, 3>& p_world,
                 double vx, double vy, double vz, double* bx, double* by, double* bz) {
  const double dx = vx - p_world[0];
  const double dy = vy - p_world[1];
  const double dz = vz - p_world[2];
  const auto& r = rot.r;
  *bx = r[0] * dx + r[3] * dy + r[6] * dz;
  *by = r[1] * dx + r[4] * dy + r[7] * dz;
  *bz = r[2] * dx + r[5] * dy + r[8] * dz;
}

// A 队 = robot_01..03，B 队 = robot_04..06（与 sim 的固定队伍映射一致）。
bool IsTeamA(const std::string& name) {
  if (name.size() < 2) return true;
  const std::string suffix = name.substr(name.size() - 2);
  try {
    const int idx = std::stoi(suffix);
    return idx >= 1 && idx <= 3;
  } catch (...) {
    return true;
  }
}

struct RobotEntry {
  std::string name;
  bool team_a{true};
  yesense_interface::msg::NavAll nav;
  bool has_nav{false};
  double nav_stamp_sec{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr gt_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr gt_ball_pub;
  rclcpp::Publisher<robot_interfaces::msg::Ball>::SharedPtr ball_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_point_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_dest_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr goal_normal_pub;
};

class SimRelayNode : public rclcpp::Node {
 public:
  SimRelayNode() : rclcpp::Node("sim_relay_node") {
    const std::string robots_param =
        this->declare_parameter<std::string>("robots", "robot_01 robot_02 robot_03");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 100.0);
    ball_hz_ = this->declare_parameter<double>("ball_publish_hz", 50.0);
    nav_timeout_sec_ = this->declare_parameter<double>("nav_timeout_sec", 0.5);
    ball_timeout_sec_ = this->declare_parameter<double>("ball_timeout_sec", 0.5);
    goal_timeout_sec_ = this->declare_parameter<double>("goal_timeout_sec", 1.0);

    std::istringstream iss(robots_param);
    std::string name;
    while (iss >> name) {
      auto entry = std::make_shared<RobotEntry>();
      entry->name = name;
      entry->team_a = IsTeamA(name);
      robots_.push_back(entry);
    }
    if (robots_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "robots 参数为空");
      throw std::runtime_error("robots 参数为空");
    }

    for (auto& entry : robots_) {
      entry->gt_pose_pub = this->create_publisher<geometry_msgs::msg::Pose2D>(
          "/soccer/ground_truth_pose/" + entry->name, 10);
      entry->gt_ball_pub = this->create_publisher<geometry_msgs::msg::Point>(
          "/soccer/ground_truth_ball/" + entry->name, 10);
      entry->ball_pub = this->create_publisher<robot_interfaces::msg::Ball>(
          "/" + entry->name + "/perception/ball", 10);
      entry->target_point_pub = this->create_publisher<geometry_msgs::msg::Point>(
          "/" + entry->name + "/perception/target_point", 10);
      entry->target_dest_pub = this->create_publisher<geometry_msgs::msg::Point>(
          "/" + entry->name + "/perception/target_destination", 10);
      entry->goal_normal_pub = this->create_publisher<geometry_msgs::msg::Vector3>(
          "/" + entry->name + "/perception/target_goal_normal", 10);
    }

    // nav_all：200 Hz，best_effort（与 sim / zl 契约一致）。
    auto nav_qos = rclcpp::SensorDataQoS();
    for (size_t i = 0; i < robots_.size(); ++i) {
      const std::string topic = "/" + robots_[i]->name + "/imu/nav_all";
      nav_subs_.push_back(this->create_subscription<yesense_interface::msg::NavAll>(
          topic, nav_qos,
          [this, i](const yesense_interface::msg::NavAll::SharedPtr msg) {
            auto& e = *robots_[i];
            e.nav = *msg;
            e.has_nav = true;
            e.nav_stamp_sec = this->now().seconds();
          }));
    }

    ball_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/sim/ball_pose", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          ball_ = *msg;
          has_ball_ = true;
          ball_stamp_sec_ = this->now().seconds();
        });
    goal_a_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/sim/goal_team_a", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          goal_a_ = *msg;
          has_goal_a_ = true;
          goal_a_stamp_sec_ = this->now().seconds();
        });
    goal_b_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/sim/goal_team_b", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          goal_b_ = *msg;
          has_goal_b_ = true;
          goal_b_stamp_sec_ = this->now().seconds();
        });

    const double hz = std::max(1.0, publish_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / hz), std::bind(&SimRelayNode::Tick, this));

    RCLCPP_INFO(this->get_logger(),
                "sim_relay ready: robots=%zu publish_hz=%.1f ball_hz=%.1f "
                "nav_timeout=%.2f ball_timeout=%.2f goal_timeout=%.2f",
                robots_.size(), publish_hz_, ball_hz_, nav_timeout_sec_, ball_timeout_sec_,
                goal_timeout_sec_);
  }

 private:
  void Tick() {
    const double now = this->now().seconds();
    const bool ball_ok = has_ball_ && (now - ball_stamp_sec_) <= ball_timeout_sec_ &&
                         std::isfinite(ball_.x) && std::isfinite(ball_.y) && std::isfinite(ball_.z);
    const bool goal_a_ok = has_goal_a_ && (now - goal_a_stamp_sec_) <= goal_timeout_sec_ &&
                           std::isfinite(goal_a_.x) && std::isfinite(goal_a_.y);
    const bool goal_b_ok = has_goal_b_ && (now - goal_b_stamp_sec_) <= goal_timeout_sec_ &&
                           std::isfinite(goal_b_.x) && std::isfinite(goal_b_.y);

    ++tick_count_;
    const double ratio = (ball_hz_ > 0.0) ? (publish_hz_ / ball_hz_) : 1.0;
    const bool publish_perception = (tick_count_ % std::max(1L, static_cast<long>(std::llround(ratio)))) == 0;

    for (auto& e : robots_) {
      if (!e->has_nav || (now - e->nav_stamp_sec) > nav_timeout_sec_) {
        continue;
      }
      const auto& nav = e->nav;
      const double px = nav.pos.longitude;
      const double py = nav.pos.latitude;
      const double pz = nav.pos.altitude;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }
      const double qw = nav.quat.q0;
      const double qx = nav.quat.q1;
      const double qy = nav.quat.q2;
      const double qz = nav.quat.q3;
      const double yaw = YawFromQuat(qw, qx, qy, qz);

      // ---- brain GT（A 队翻转 180°；与旧 bridge 一致）----
      geometry_msgs::msg::Pose2D gt_pose;
      gt_pose.x = e->team_a ? -px : px;
      gt_pose.y = e->team_a ? -py : py;
      gt_pose.theta = e->team_a ? WrapPi(yaw + kPi) : yaw;
      e->gt_pose_pub->publish(gt_pose);

      if (!ball_ok) continue;
      geometry_msgs::msg::Point gt_ball;
      gt_ball.x = e->team_a ? -ball_.x : ball_.x;
      gt_ball.y = e->team_a ? -ball_.y : ball_.y;
      gt_ball.z = ball_.z;
      e->gt_ball_pub->publish(gt_ball);

      if (!publish_perception) continue;
      const bool goal_ok = e->team_a ? goal_a_ok : goal_b_ok;
      if (!goal_ok) continue;
      const auto& goal = e->team_a ? goal_a_ : goal_b_;

      // ---- 感知量：完整姿态转到基座系 ----
      const Rot3 rot = RotationFromQuat(qw, qx, qy, qz);
      const std::array<double, 3> p_world{{px, py, pz}};
      double bx = 0.0, by = 0.0, bz = 0.0;

      WorldToBase(rot, p_world, ball_.x, ball_.y, ball_.z, &bx, &by, &bz);
      robot_interfaces::msg::Ball ball_msg;
      ball_msg.header.stamp = this->now();
      ball_msg.header.frame_id = e->name + "/base";
      ball_msg.x = bx;
      ball_msg.y = by;
      ball_msg.confidence = 100.0;
      e->ball_pub->publish(ball_msg);

      geometry_msgs::msg::Point target_point;
      target_point.x = bx;
      target_point.y = by;
      target_point.z = bz;
      e->target_point_pub->publish(target_point);

      WorldToBase(rot, p_world, goal.x, goal.y, goal.z, &bx, &by, &bz);
      geometry_msgs::msg::Point target_dest;
      target_dest.x = bx;
      target_dest.y = by;
      target_dest.z = bz;
      e->target_dest_pub->publish(target_dest);

      // 进攻方向（世界系）：A = -x，B = +x；转到基座系并归一化。
      const double dir_x = e->team_a ? -1.0 : 1.0;
      double nx = 0.0, ny = 0.0, nz = 0.0;
      WorldToBase(rot, std::array<double, 3>{{0.0, 0.0, 0.0}}, dir_x, 0.0, 0.0,
                  &nx, &ny, &nz);
      const double n = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (n > 1e-9) {
        nx /= n; ny /= n; nz /= n;
      }
      geometry_msgs::msg::Vector3 normal;
      normal.x = nx;
      normal.y = ny;
      normal.z = nz;
      e->goal_normal_pub->publish(normal);
    }
  }

  double publish_hz_{100.0};
  double ball_hz_{50.0};
  double nav_timeout_sec_{0.5};
  double ball_timeout_sec_{0.5};
  double goal_timeout_sec_{1.0};
  long tick_count_{0};

  std::vector<std::shared_ptr<RobotEntry>> robots_;
  std::vector<rclcpp::Subscription<yesense_interface::msg::NavAll>::SharedPtr> nav_subs_;

  geometry_msgs::msg::Point ball_;
  bool has_ball_{false};
  double ball_stamp_sec_{0.0};
  geometry_msgs::msg::Point goal_a_;
  geometry_msgs::msg::Point goal_b_;
  bool has_goal_a_{false};
  bool has_goal_b_{false};
  double goal_a_stamp_sec_{0.0};
  double goal_b_stamp_sec_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_a_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_b_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
