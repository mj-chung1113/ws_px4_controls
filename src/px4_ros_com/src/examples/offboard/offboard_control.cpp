// ✅ [1] offboard_control_node.cpp
// 위치 제어나 속도 제어를 외부 명령에 따라 전환하며 동작

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using std::placeholders::_1;
using namespace px4_msgs::msg;

enum class ControlMode
{
  POSITION,
  VELOCITY
};

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("offboard_control_node")
  {
    offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // 생성자에서 구독 초기화
    vehicle_control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", rclcpp::QoS(1).best_effort(),
        std::bind(&OffboardControl::vehicle_control_mode_callback, this, _1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/command/pose", 10, std::bind(&OffboardControl::pose_callback, this, _1));
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>("/command/twist", 10, std::bind(&OffboardControl::twist_callback, this, _1));
    gimbal_pitch_sub_ = create_subscription<std_msgs::msg::Float32>("/gimbal_pitch_degree", 10, std::bind(&OffboardControl::gimbal_callback, this, std::placeholders::_1));
    takeoff_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/do_takeoff", 10,
        std::bind(&OffboardControl::takeoff_callback, this, _1) // <-- 이 부분을 수정
    );
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&OffboardControl::timer_callback, this));
  }

private:
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;

  ControlMode mode_ = ControlMode::POSITION;
  TrajectorySetpoint setpoint_{};
  int setpoint_counter_ = 0;

  const uint8_t MY_SYSID = 46;  // 비행 컨트롤러와 동일
  const uint8_t MY_COMPID = 47; // USER1
  const uint8_t TARGET_SYSID = 1;
  const uint8_t TARGET_COMPID = 1;
  const uint8_t FLAG_GIMBAL = 12;
  px4_msgs::msg::VehicleControlMode current_control_mode_{};
  bool target_command = false; // setpoint가 최소 한 번은 들어왔는가?
  bool armed_ = false;         // 이미 arm 했는가?
  bool takeoff_ok = false;     // takeoff 명령이 들어왔는가?
  uint64_t offboard_setpoint_counter_;
  void timer_callback()
  {
    // 1. Offboard Control Mode 메시지와 Trajectory Setpoint는 항상 발행
    //    PX4가 Offboard 모드로 진입하기 위한 기본 조건이므로,
    //    어떤 상태이든 지속적으로 이 두 메시지는 보내야 합니다.
    publish_offboard_control_mode();
    trajectory_setpoint_pub_->publish(setpoint_);

    // offboard_setpoint_counter_는 publish_offboard_control_mode()와 trajectory_setpoint_pub_가 호출될 때마다 증가
    // Setpoint가 유효하게 보내지고 있음을 PX4가 인식하는 카운터 역할
    // 최대값을 넘지 않도록 제한
    if (offboard_setpoint_counter_ < 40) // 50ms 주기라면 2초(40개)보다 충분히 많게 설정
    {
      offboard_setpoint_counter_++;
    }

    // 초기 Setpoint가 수신되었는지 확인 (uav_controller_pkg에서 보낸 명령)
    if (!target_command)
    {
      RCLCPP_INFO(get_logger(), "Waiting for initial target command (pose/twist) from uav_controller_pkg.");
      // 초기 명령 없으면 Arming 및 Offboard 진입 시도 안함
      return;
    }

    // 이 시점에서 target_command는 true, 즉 uav_controller_pkg에서 명령을 보냄

    // 2. Arming 상태가 아니라면 Offboard 진입 및 Arming 시도
    if (!armed_)
    {
      // takeoff_ok는 uav_controller_pkg에서 do_takeoff 명령을 받았는지 여부
      if (takeoff_ok) // do_takeoff 명령을 받아야만 Offboard 및 Arming 시도
      {
        // Setpoint 카운터가 충분히 쌓였는지 확인
        // 이 카운터는 PX4가 Offboard 모드 진입을 허용할 준비가 되었는지 판단하는 데 사용
        if (offboard_setpoint_counter_ >= 50) // 최소 50개 이상의 setpoint가 발행된 후 시도 (2.5초)
        {
          // Offboard 모드 요청 (계속 보냄, PX4가 실제로 전환할 때까지)
          publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Mode 6 is OFFBOARD
          RCLCPP_INFO(get_logger(), "Attempting to switch to OFFBOARD mode.");

          // Offboard 모드로 성공적으로 진입했는지 확인
          if (current_control_mode_.flag_control_offboard_enabled)
          {
            RCLCPP_INFO(get_logger(), "OFFBOARD mode ENABLED. Attempting to ARM.");
            // Arm 명령
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            armed_ = true; // Arm 명령을 보냈으므로 armed_ 플래그를 true로 설정
            RCLCPP_INFO(get_logger(), "Successfully entered OFFBOARD mode and ARMED.");
          }
          else
          {
            RCLCPP_INFO(get_logger(), "Waiting for OFFBOARD mode to be enabled (current: %d).",
                        current_control_mode_.flag_control_offboard_enabled);
          }
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Collecting sufficient offboard setpoints (%d/50)...", offboard_setpoint_counter_);
        }
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Waiting for 'takeoff_ok' command from uav_controller_pkg to arm.");
      }
    }
    // else if (armed_ && takeoff_ok) { // 이미 Arm 되고 Offboard라면, 다른 미션 로직으로 전환 가능
    //     // 예: Waypoint 비행, 착륙 등
    // }
  }
  // 콜백 함수
  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    current_control_mode_ = *msg;
  }
  /* Pose 콜백: setpoint 저장 + 플래그 ON + 카운터 초기화 */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    mode_ = ControlMode::POSITION;

    // ENU(uav_controller) -> NED(PX4) 변환
    // msg: x(E), y(N), z(U)  ->  setpoint: position[0](N), position[1](E), position[2](D)
    setpoint_.position[0] = msg->pose.position.y;  // NED North = ENU North (y)
    setpoint_.position[1] = msg->pose.position.x;  // NED East  = ENU East  (x)
    setpoint_.position[2] = -msg->pose.position.z; // NED Down  = -ENU Up   (-z)

    // Yaw도 ENU에서 NED로 변환 (ψ_ned = π/2 - ψ_enu)
    double yaw_enu = tf2::getYaw(msg->pose.orientation);
    setpoint_.yaw = static_cast<float>((M_PI / 2.0) - yaw_enu);

    // 로그를 추가하여 변환된 NED 값을 직접 확인하면 디버깅에 용이합니다.
    // RCLCPP_INFO(get_logger(), "Converted to NED -> N:%.2f, E:%.2f, D:%.2f, Yaw:%.2f",
    //                                 setpoint_.position[0], setpoint_.position[1], setpoint_.position[2], setpoint_.yaw);

    target_command = true;
  }
  void gimbal_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    send_gimbal_pitch(static_cast<float>(msg->data));
  }

  /* Twist 콜백도 동일하게 플래그·카운터 처리 */
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    mode_ = ControlMode::VELOCITY;
    setpoint_.position[0] = NAN;
    setpoint_.position[1] = NAN;
    setpoint_.position[2] = NAN;
    setpoint_.velocity[0] = msg->linear.x;
    setpoint_.velocity[1] = msg->linear.y;
    setpoint_.velocity[2] = msg->linear.z;
    setpoint_.yaw = NAN;
    setpoint_.yawspeed = msg->angular.z;

    target_command = true;
    setpoint_counter_ = 0;
  }
  void takeoff_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(get_logger(), "Takeoff command received.");
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
      armed_ = true;
      target_command = true;
      takeoff_ok = true;
      setpoint_.position[0] = 0.0f;  // NED North
      setpoint_.position[1] = 0.0f;  // NED East
      setpoint_.position[2] = -5.0f; // NED Down (5m Up in ENU)
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Takeoff command cancelled.");
      armed_ = false;
    }
  }
  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = (mode_ == ControlMode::POSITION);
    msg.velocity = (mode_ == ControlMode::VELOCITY);
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void send_gimbal_pitch(float pitch_deg)
  {

    take_gimbal_control();
    /* 2) 피치 명령 */
    auto now_us = this->get_clock()->now().nanoseconds() / 1000;

    VehicleCommand cmd{};
    cmd.timestamp = now_us;
    cmd.command = VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW; // 1000
    cmd.param1 = pitch_deg;                                               // Pitch (+위, -아래)
    cmd.param2 = 0.0f;                                                    // Yaw 유지
    cmd.param3 = NAN;                                                     // rate
    cmd.param4 = NAN;                                                     // rate
    cmd.param5 = FLAG_GIMBAL;                                             // follow(0) or lock(16)
    cmd.param7 = 1;                                                       // gimbal_device_id
    cmd.target_system = TARGET_SYSID;
    cmd.target_component = TARGET_COMPID;
    cmd.source_system = MY_SYSID;
    cmd.source_component = MY_COMPID; // ★ 반드시 동일
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
  }

  void take_gimbal_control()
  {
    VehicleCommand cmd{};
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command = VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE; // 1001

    /* ── 핵심: -2 를 넣으면 PX4가 msg 의 sys/comp id 그대로 채움 ── */
    cmd.param1 = MY_SYSID;
    cmd.param2 = MY_COMPID;
    cmd.param3 = cmd.param4 = 0; // secondary 없음
    cmd.param5 = FLAG_GIMBAL;    // flags
    cmd.param7 = 1;              // gimbal device id

    cmd.target_system = TARGET_SYSID;     // PX4
    cmd.target_component = TARGET_COMPID; // MAV_COMP_ID_AUTOPILOT
    cmd.source_system = MY_SYSID;
    cmd.source_component = MY_COMPID;
    cmd.from_external = true;

    vehicle_command_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), ">>> Sent CONFIGURE(1001) – take primary control");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}