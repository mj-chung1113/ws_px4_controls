#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <memory>

using std::placeholders::_1;

class PID
{
public:
  PID(double Kp, double Ki, double Kd)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt)
  {
    integral_ += error * dt;
    double derivative = dt > 0.0 ? (error - prev_error_) / dt : 0.0;
    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    prev_error_ = error;
    return output;
  }

private:
  double Kp_, Ki_, Kd_;
  double prev_error_, integral_;
};

class PathFollower : public rclcpp::Node
{
public:
  enum class UGVState
  {
    WAITING_FOR_START,
    FOLLOWING_PATH,
    STOPPING_FOR_DRONE,        // 드론 이륙 위해 정지 + 2초 대기 시작
    WAITING_FOR_DRONE_TAKEOFF, // 2초 대기 완료 후 드론 이륙 신호 기다리는 중
    DRONE_TAKEOFF_COMPLETE,    // 드론 이륙 완료 후 다음 웨이포인트로 이동 직전
    DRONE_LANDING_WAITING
  };
  PathFollower()
      : Node("mission_follower_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
        pid_speed_(1.0, 0.0, 0.1), pid_steer_(2.0, 0.0, 0.1), current_target_index_(0), done_takeoff_(false)
  // 초기화
  {
    // Define UGV states

    // 파라미터 선언 및 로드
    declare_parameter<std::string>("path_file", "");
    declare_parameter<double>("arrival_threshold", 0.5);
    declare_parameter<double>("longitudinal_speed", 3.0);

    get_parameter("path_file", path_file_);
    get_parameter("arrival_threshold", arrival_thresh_);
    get_parameter("longitudinal_speed", max_speed_);

    RCLCPP_INFO(get_logger(), "[param check] path_file: %s", path_file_.c_str());

    // Publishers
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/model/X1_asp/cmd_vel", 10);
    takeoff_pub_ = create_publisher<std_msgs::msg::Bool>("/do_takeoff", 10);
    ugv_landing_arrival_pub_ = create_publisher<std_msgs::msg::Bool>("/ugv_landing_spot_arrived", 10);

    // Subscribers
    takeoff_sub_ = create_subscription<std_msgs::msg::Bool>("/done_takeoff", 10,
                                                            std::bind(&PathFollower::droneTakeoffCallback, this, _1));
    ready_to_go_sub = create_subscription<std_msgs::msg::Bool>("/ready_to_go", 10,
                                                               std::bind(&PathFollower::readyToGoCallback, this, _1));

    // Visualization publishers
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
    path_pub_ = create_publisher<visualization_msgs::msg::Marker>("path_marker", rclcpp::QoS(1).transient_local());

    // Load path and publish initial path marker
    load_path();
    publish_path_marker();

    // Control loop timer
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathFollower::control_loop, this));
  }

private:
  void readyToGoCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && current_ugv_state_ == UGVState::WAITING_FOR_START)
    {
      current_ugv_state_ = UGVState::FOLLOWING_PATH; // Transition to following path
      RCLCPP_INFO(get_logger(), "[UGV] /ready_to_go received. Transitioning to FOLLOWING_PATH state.");
      current_target_index_ = 0; // Reset waypoint index
      done_takeoff_ = false;     // Ensure drone takeoff state is false initially
    }
  }
  // drone_takeoff 토픽 수신
  void droneTakeoffCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(get_logger(), "[UGV] done_takeoff==true 수신");
      done_takeoff_ = true;
    }
  }

  // csv에서 (x,y,state) 읽어오기
  void load_path()
  {
    std::ifstream file(path_file_);
    if (!file.is_open())
    {
      RCLCPP_ERROR(get_logger(), "경로 파일 열기 실패: %s", path_file_.c_str());
      return;
    }

    std::string line;
    std::getline(file, line); // header 제거 = 1번째줄 제거
    while (std::getline(file, line))
    {
      std::stringstream ss(line);
      std::string x_str, y_str, state_str;
      if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, state_str))
      {
        double x = std::stod(x_str);
        double y = std::stod(y_str);
        int state = std::stoi(state_str);
        path_.emplace_back(x, y, state);
      }
    }
    RCLCPP_INFO(get_logger(), "%zu개의 waypoints 로드됨.", path_.size());
  }

  // 현재 위치 방향 갱신
  bool update_pose()
  {
    try
    {
      // Lookup transform from "map" to "X1_asp/base_link"
      auto transform = tf_buffer_.lookupTransform("map", "X1_asp/base_link", tf2::TimePointZero);
      current_x_ = transform.transform.translation.x;
      current_y_ = transform.transform.translation.y;

      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      current_yaw_ = yaw;
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "TF failed: %s", ex.what()); // Throttle warning
      return false;
    }
  }

  // Main control loop
  void control_loop()
  {
    geometry_msgs::msg::Twist cmd; // Initialize cmd_vel message

    // No pose update or waypoint processing if waiting for start
    if (current_ugv_state_ == UGVState::WAITING_FOR_START)
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd); // Ensure UGV is stopped
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "UGV waiting for mission start signal...");
      return; // Exit loop until ready_to_go is received
    }

    // For any state other than WAITING_FOR_START, we need pose and waypoints
    if (!update_pose() || current_target_index_ >= path_.size())
    {
      // If we lose pose or run out of waypoints, stop the UGV
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      RCLCPP_ERROR(get_logger(), "Failed to update pose or current_target_index_ out of bounds. Stopping UGV.");
      // Consider adding error handling or mission abort logic here
      return;
    }

    auto [goal_x, goal_y, goal_state] = path_[current_target_index_];
    double dx = goal_x - current_x_;
    double dy = goal_y - current_y_;
    double dist = std::hypot(dx, dy);

    // --- Mission Completion Check ---
    if (current_target_index_ == path_.size() - 1 && dist < arrival_thresh_)
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd); // UGV fully stops
      current_ugv_state_ = UGVState::DRONE_LANDING_WAITING;
      RCLCPP_INFO(get_logger(), "All waypoints completed. Shutting down node.");

      // Publish final arrival signal if needed, then shut down
      std_msgs::msg::Bool landing_ready_msg;
      landing_ready_msg.data = true;
      ugv_landing_arrival_pub_->publish(landing_ready_msg);
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    // --- State Machine Logic ---
    switch (current_ugv_state_)
    {
    case UGVState::FOLLOWING_PATH:
      // Check if UGV has arrived at a state 2 waypoint
      if (goal_state == 2 && dist < arrival_thresh_)
      {
        RCLCPP_INFO(get_logger(), "Arrived at state 2 waypoint. Transitioning to STOPPING_FOR_DRONE.");
        current_ugv_state_ = UGVState::STOPPING_FOR_DRONE;
        // Start the 2-second one-shot delay timer
        delay_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]()
            {
              RCLCPP_INFO(this->get_logger(), "2-second delay complete. Transitioning to WAITING_FOR_DRONE_TAKEOFF.");
              this->current_ugv_state_ = UGVState::WAITING_FOR_DRONE_TAKEOFF;
              this->delay_timer_->cancel(); // Cancel timer as it's one-shot
            },
            nullptr); // No specific callback group needed
      }
      else
      {
        // Normal path following PID control
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = std::atan2(std::sin(target_yaw - current_yaw_), std::cos(target_yaw - current_yaw_));
        double dt = 0.1; // Based on 100ms timer
        cmd.linear.x = std::clamp(pid_speed_.compute(dist, dt), -max_speed_, max_speed_);
        cmd.angular.z = std::clamp(pid_steer_.compute(yaw_error, dt), -1.0, 1.0);

        // Increment waypoint if arrived (for normal waypoints)
        if (dist < arrival_thresh_ && current_target_index_ + 1 < path_.size())
        {
          ++current_target_index_;
          RCLCPP_INFO(get_logger(), "Moving to next waypoint (%zu/%zu)", current_target_index_, path_.size());
        }
      }
      break;

    case UGVState::STOPPING_FOR_DRONE:
      // UGV remains stopped and sends initial takeoff signal
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      { // Scope for takeoff_signal
        std_msgs::msg::Bool takeoff_signal;
        takeoff_signal.data = true;
        takeoff_pub_->publish(takeoff_signal);
      }
      // No waypoint increment here; delay_timer_ will handle state transition
      break;

    case UGVState::WAITING_FOR_DRONE_TAKEOFF:
      // UGV remains stopped and continuously sends takeoff signal
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      { // Scope for takeoff_signal
        std_msgs::msg::Bool takeoff_signal;
        takeoff_signal.data = true;
        takeoff_pub_->publish(takeoff_signal);
      }

      // Check if drone takeoff is confirmed
      if (done_takeoff_)
      {
        RCLCPP_INFO(get_logger(), "Drone takeoff confirmed. Transitioning to DRONE_TAKEOFF_COMPLETE.");
        done_takeoff_ = false; // Reset for potential future state 2 waypoints
        current_ugv_state_ = UGVState::DRONE_TAKEOFF_COMPLETE;
      }
      break;

    case UGVState::DRONE_TAKEOFF_COMPLETE:
      // Drone takeoff is complete, UGV prepares to move to the next waypoint
      RCLCPP_INFO(get_logger(), "Drone mission phase complete. Moving to next waypoint.");
      ++current_target_index_;                       // Move to the actual next waypoint
      current_ugv_state_ = UGVState::FOLLOWING_PATH; // Revert to path following state
      // The loop will execute again, recalculating PID for the new target.
      // No 'break' needed here if you want to immediately compute the first command for the next segment
      // However, a 'break' and let the next control_loop iteration handle it is generally safer for state transitions.
      // For clarity, I'll add break and allow the next loop cycle to start from FOLLOWING_PATH.
      break;

    case UGVState::WAITING_FOR_START: // This state is handled at the very beginning of the function
      // This case should ideally not be reached if the initial check works
      // But included for completeness of the switch statement.
      break;
    }

    cmd_pub_->publish(cmd); // Publish the computed command velocity

    // Publish markers regardless of state (for current goal visualization)
    publish_marker(goal_x, goal_y);
    publish_path_marker();
  }

  // 목표 마커 시각화
  void publish_marker(double x, double y)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "goal";
    marker.id = 0;
    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.2;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker_pub_->publish(marker);
  }

  // 경로 시각화
  void publish_path_marker()
  {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = now();
    line.ns = "path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;

    line.pose.orientation.w = 1.0;

    line.scale.x = 0.08; // 선 두께
    line.color.g = 1.0;  // 초록
    line.color.a = 1.0;  // 불투명

    // 끝까지 남겨두고 싶으면 0초(=영구)로
    line.lifetime = rclcpp::Duration::from_seconds(0.0);

    for (auto &[x, y, state] : path_)
    {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = 0.1; // 살짝 띄워서 지면과 겹침 방지
      line.points.push_back(p);
    }
    path_pub_->publish(line);
  }

  // Member variables
  std::string path_file_;
  double arrival_thresh_, max_speed_;
  double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
  size_t current_target_index_;
  bool done_takeoff_;          // Flag set by droneTakeoffCallback
  UGVState current_ugv_state_; // Current state of the UGV

  std::vector<std::tuple<double, double, int>> path_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeoff_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_to_go_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr delay_timer_;                                  // For the 2-second delay
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ugv_landing_arrival_pub_; // 랑데뷰 준비 완료 신호
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 제어기
  PID pid_speed_, pid_steer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
