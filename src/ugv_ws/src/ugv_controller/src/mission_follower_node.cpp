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
  bool mission_start_ = false;
};

class PathFollower : public rclcpp::Node
{
public:
  PathFollower()
      : Node("mission_follower_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
        pid_speed_(1.0, 0.0, 0.1), pid_steer_(2.0, 0.0, 0.1), current_target_index_(0), drone_takeoff_(false)
  // 초기화
  {
    // 파라미터 선언 및 로드
    declare_parameter<std::string>("path_file", "");
    declare_parameter<double>("arrival_threshold", 0.5);
    declare_parameter<double>("longitudinal_speed", 3.0);

    get_parameter("path_file", path_file_);
    get_parameter("arrival_threshold", arrival_thresh_);
    get_parameter("longitudinal_speed", max_speed_);

    RCLCPP_INFO(get_logger(), "[param check] path_file: %s", path_file_.c_str());

    // 주행 publisher
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/model/X1_asp/cmd_vel", 10);
    takeoff_pub_ = create_publisher<std_msgs::msg::Bool>("/do_takeoff", 10);

    // 드론 이륙 완료 여부 subscriber(Boolean)
    takeoff_sub_ = create_subscription<std_msgs::msg::Bool>("/done_takeoff", 10,
                                                            std::bind(&PathFollower::droneTakeoffCallback, this, std::placeholders::_1));
    ready_to_go_sub = create_subscription<std_msgs::msg::Bool>(
        "ready_to_go", 10, std::bind(&PathFollower::startCallback, this, std::placeholders::_1));
    ugv_arrival_pub_ = create_publisher<std_msgs::msg::Bool>(
        "ugv/mission2_arrived", 10);

    // 시각화 publisher
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
    path_pub_ = create_publisher<visualization_msgs::msg::Marker>("path_marker", rclcpp::QoS(1).transient_local()); // latched QoS

    // 경로 rviz 시각화
    load_path();
    publish_path_marker();

    // 제어 루프 타이머
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathFollower::control_loop, this));
  }

private:
  void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if ((msg->data) && (mission_start_ == false))
    {
      mission_start_ = true; // 미션 시작 플래그 설정
      RCLCPP_INFO(get_logger(), "[UGV] ready_to_go==true 수신");
      current_target_index_ = 0; // 웨이포인트 인덱스 초기화
      done_takeoff_ = false;     // 드론 이륙 상태 초기화
    }
  }
  // drone_takeoff 토픽 수신
  void droneTakeoffCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(get_logger(), "[UGV] drone_takeoff==true 수신");
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
      RCLCPP_WARN(get_logger(), "TF 실패: %s", ex.what());
      return false;
    }
  }

  // 제어 루프
  void control_loop()
  {
    if (mission_start_ == false)
    {
      RCLCPP_INFO(get_logger(), "미션 시작 대기중...");
      return;
    }
    if (!update_pose() || current_target_index_ >= path_.size())
      return;

    auto [goal_x, goal_y, goal_state] = path_[current_target_index_];
    double dx = goal_x - current_x_;
    double dy = goal_y - current_y_;
    double dist = std::hypot(dx, dy);

    if (current_target_index_ == path_.size() - 1 && dist < arrival_thresh_)
    {
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop); // ① 차량 완전 정지
      RCLCPP_INFO(get_logger(),
                  "모든 웨이포인트 완료 → 노드 종료합니다.");
      timer_->cancel();   // ② 주행 타이머 끄기
      rclcpp::shutdown(); // ③ 노드 종료
      return;
    }

    double target_yaw = std::atan2(dy, dx);
    double yaw_error = std::atan2(std::sin(target_yaw - current_yaw_), std::cos(target_yaw - current_yaw_));

    // --- state == 2 처리 로직 ---
    if (goal_state == 2 && dist < arrival_thresh_)
    {
      // 1) 정지
      geometry_msgs::msg::Twist stop;
      std_msgs::msg::Bool msg;
      cmd_pub_->publish(stop);
      msg.data = true;
      takeoff_pub_->publish(msg); // 드론 이륙 신호 전송
      // 2) 드론 이륙 신호 대기
      if (!done_takeoff_)
      {
        RCLCPP_INFO(get_logger(), "state=2 지점 도착 → 드론 이륙 대기중…");
        return;
      }
      // 3) 이륙 신호 수신 시 다음 웨이포인트로 전진
      RCLCPP_INFO(get_logger(), "이륙 신호 수신 → 다음 웨이포인트로 이동");
      done_takeoff_ = true;
      ++current_target_index_;
    }

    // 주행 제어 PID
    double dt = 0.1;
    double speed = std::clamp(pid_speed_.compute(dist, dt), -max_speed_, max_speed_);
    double steer = std::clamp(pid_steer_.compute(yaw_error, dt), -1.0, 1.0);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = steer;
    cmd_pub_->publish(cmd);

    // 도착 판정 후 waypoint 인덱스 증가
    if (dist < arrival_thresh_ && current_target_index_ + 1 < path_.size())
    {
      ++current_target_index_;
      RCLCPP_INFO(get_logger(), "다음 waypoint로 이동 (%zu/%zu)", current_target_index_, path_.size());
    }

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

  // 멤버 변수
  std::string path_file_;
  double arrival_thresh_, max_speed_;
  double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
  size_t current_target_index_;
  bool drone_takeoff_;
  std::vector<std::tuple<double, double, int>> path_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeoff_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ugv_arrival_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

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
