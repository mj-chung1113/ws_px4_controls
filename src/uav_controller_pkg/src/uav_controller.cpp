#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h> // Matrix3x3을 사용하기 위해 필요 (getYaw가 정의된 곳 중 하나)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h> // Make sure this is included
#include <std_msgs/msg/int32.hpp>

#include <aruco_interfaces/msg/marker_pose_id.hpp> // 커스텀 메시지

#define TAKEOFFALTITUDE 10.0                                                   // ENU, 드론이 이 높이로 상승하면 착륙 지점으로 이동
#define WP_LOAD_PATH "/home/jmj/pro_asp_ws/ws_px4_controls/optimized_path.csv" // 웨이포인트 파일 경로
#define MARKER_SAVE_PATH "/home/jmj/pro_asp_ws/ws_px4_controls/marker_location.csv"

enum class MissionState
{
    IDLE,
    TAKEOFF,
    MOVING_TO_WAYPOINT,
    WAIT_BEFORE_SEARCHING,
    SEARCHING_FOR_MARKER,
    MISSION_COMPLETE,
    PRECISION_LANDING
};
enum class PrecisionLandingPhase
{
    XY_ALIGN,
    DESCENDING
};
class UavController : public rclcpp::Node
{
public:
    UavController()
        : Node("uav_controller"), state_(MissionState::IDLE), current_wp_idx_(0), takeoff_sent_(false)
    {
        load_waypoints(WP_LOAD_PATH);

        // QOS =================================================================================
        rclcpp::QoS latching_qos(1);
        latching_qos.transient_local();
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(3));
        auto sensor_qos = rclcpp::SensorDataQoS();

        // PUBLISHERS ==========================================================================
        pose_cmd_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose", 10);
        takeoff_pub_ = create_publisher<std_msgs::msg::Bool>("/done_takeoff", 10);
        gimbal_pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gimbal_pitch_degree", latching_qos);
        ready_to_go_pub_ = create_publisher<std_msgs::msg::Bool>("/ready_to_go", 10);
        marker_size_pub_ = create_publisher<std_msgs::msg::Float32>("/mk_size", 10);
        // SUBSCRIBERS ==========================================================================
        takeoff_sub_ = create_subscription<std_msgs::msg::Bool>("/do_takeoff", 10, std::bind(&UavController::takeoff_cb, this, std::placeholders::_1));
        ugv_arrived_sub_ = create_subscription<std_msgs::msg::Bool>("/ugv_landing_spot_arrived", 10, std::bind(&UavController::ugvLandingSpotCallback, this, std::placeholders::_1));
        marker_sub_ = create_subscription<aruco_interfaces::msg::MarkerPoseId>("/x500/target", pose_qos, std::bind(&UavController::aruco_marker_cb, this, std::placeholders::_1));
        local_pose_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos, std::bind(&UavController::local_pose_cb, this, std::placeholders::_1));
        // TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this); // This should be StaticTransformBroadcaster if it's a fixed frame

        // MAIN LOOP =============================================================================
        main_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UavController::main_loop, this));

        RCLCPP_INFO(get_logger(), "UavController initialised. Mission starts");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        state_ = MissionState::IDLE;
    }

private:
    // Pubs============================================================================
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeoff_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gimbal_pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_to_go_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr marker_size_pub_;
    // Subs============================================================================
    rclcpp::Subscription<aruco_interfaces::msg::MarkerPoseId>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ugv_arrived_sub_;

    // main timer =====================================================================
    rclcpp::TimerBase::SharedPtr main_timer_;

    // TF2 related members ============================================================
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_; // Declared as a member

    // Mission data ===================================================================
    MissionState state_;
    PrecisionLandingPhase precision_landing_phase_ = PrecisionLandingPhase::XY_ALIGN;
    size_t current_wp_idx_;
    std::vector<geometry_msgs::msg::Point> waypoints_enu_;
    std::vector<geometry_msgs::msg::Point> precise_markers_world_;
    std::vector<char> waypoint_labels_;
    px4_msgs::msg::VehicleLocalPosition::SharedPtr local_v_;
    geometry_msgs::msg::Pose current_pose_enu_{}; // Updated from px4_msgs/VehicleLocalPosition via TF
    geometry_msgs::msg::Point landing_spot_enu_{};
    geometry_msgs::msg::Point latest_marker_position_;
    bool takeoff_sent_;
    bool set_localstatic_ = false;       // 로컬 좌표계(NED) 설정 완료 여부
    bool ugv_landing_ready_{false};      // UGV 착륙지점 도착 여부
    float stopped_vel_threshold_ = 0.1f; // m/s, 드론이 정지했다고 판단하는 속도
    bool is_drone_stable_{false};
    float stable_timeout_ = 5.0f;      // seconds, 드론이 안정화되었다고 판단하는 시간 임계값
    float searching_timeout_ = 2.0f;   // seconds, 마커 검색 시간 임계값
    double target_landing_z_ = 0.0;    // 다음 하강 목표 고도
    const double landing_step_ = 0.5;  // 한 번에 몇 m씩 내릴지
    const double landing_min_z_ = 0.3; // 이하면 착륙 완료로 간주
    // Hovering time variables
    rclcpp::Time wait_start_time_;
    rclcpp::Time search_start_time_;

    // Marker duplication prevention
    std::unordered_set<int> seen_marker_ids_;
    float current_gimbal_pitch_ = 0.0f; // Initial pitch (facing downwards)

    const geometry_msgs::msg::Quaternion FIXED_HEADING_QUATERNION_{
        []
        {
            geometry_msgs::msg::Quaternion q;
            q.x = 0.0001510303616417948;
            q.y = -0.0004339473941201041;
            q.z = 0.999773449681471;
            q.w = -0.021279995101036512;
            return q;
        }()};

    void takeoff_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            if (takeoff_sent_)
            {
                RCLCPP_INFO(this->get_logger(), "[UAV] Takeoff command received again, but already taken off. Ignoring.");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "[UAV] Takeoff command received. Switching to TAKEOFF state.");
            state_ = MissionState::TAKEOFF;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[UAV] Received takeoff=false signal. Ignored.");
        }
    }

    void set_local_static()
    {
        // Ensure current_pose_enu_ has been updated at least once before setting local_static
        if (!update_pose())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current pose for setting local_static frame. Retrying...");
            return;
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now(); // Use this->get_clock()->now()
        t.header.frame_id = "map";
        t.child_frame_id = "local_static";

        // Use the current ENU pose for the origin of local_static
        t.transform.translation.x = current_pose_enu_.position.x;
        t.transform.translation.y = current_pose_enu_.position.y;
        t.transform.translation.z = current_pose_enu_.position.z;

        // This rotation (M_PI, 0, M_PI_2) means:
        // Roll 180 degrees (flips Z axis)
        // Pitch 0
        // Yaw 90 degrees (rotates X to Y, Y to -X)
        // This is a common way to convert ENU (map) to NED (local_static for PX4) assuming local_static is NED-like.
        // If local_static is *truly* NED (X=N, Y=E, Z=D), and map is ENU (X=E, Y=N, Z=U),
        // the map -> local_static transform should handle the coordinate system conversion implicitly.
        // For a fixed local_static frame, its orientation relative to map should be constant.
        // If current_pose_enu_.orientation is already the drone's orientation in map ENU,
        // you might want to use that for local_static if you want local_static to be aligned with drone's initial yaw.
        // For a pure ENU-to-NED conversion, consider a fixed transform.
        // tf2::Quaternion q_enu_to_ned;
        // Standard ENU to NED rotation: rotate -90 deg about X, then 180 deg about Z.
        // Or, a more common one: X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu (which corresponds to rotating about Z by -90, then X by 180)
        // Let's use a standard fixed transform that aligns 'map' ENU to 'local_static' NED for the frame itself,
        // and let TF2 handle the actual pose transformation later.

        // Simple ENU to NED rotation (e.g., X_NED=Y_ENU, Y_NED=X_ENU, Z_NED=-Z_ENU)
        // This corresponds to a 90 degree rotation around Z, then 180 degree rotation around X.
        // tf2::Quaternion q_map_to_local_static_rot;
        // q_map_to_local_static_rot.setRPY(M_PI, 0, M_PI_2); // This creates a transform that converts ENU to NED.
        // t.transform.rotation = tf2::toMsg(q_map_to_local_static_rot);

        // If local_static is simply an origin shift but maintains the same orientation as 'map' initially, use current_pose_enu_.orientation
        // For a true NED-like local_static frame (X forward, Y right, Z down relative to initial ENU orientation)
        // a fixed orientation for the *frame* itself is required.
        // However, if `current_pose_enu_.orientation` is the orientation of the drone's base_link,
        // and you want `local_static` to share that orientation at the starting point, you can use it.
        // Assuming 'local_static' is meant to be a fixed frame, let's define a standard ENU to NED rotation.
        tf2::Quaternion q_fixed_enu_to_ned;
        // Set fixed rotation (e.g., X_NED = Y_ENU, Y_NED = X_ENU, Z_NED = -Z_ENU)
        // This is a 90 degree rotation about Z (ENU) then 180 degree rotation about X (resulting intermediate axis)
        // This is a common way to align ENU to NED.
        q_fixed_enu_to_ned.setRPY(M_PI, 0, M_PI_2);
        t.transform.rotation = tf2::toMsg(q_fixed_enu_to_ned);

        static_broadcaster_->sendTransform(t); // Use the member broadcaster
        set_localstatic_ = true;               // Set flag after successful broadcast
        RCLCPP_INFO(this->get_logger(), "local_static frame broadcasted. Origin at map: x=%.2f, y=%.2f, z=%.2f",
                    current_pose_enu_.position.x, current_pose_enu_.position.y, current_pose_enu_.position.z);
    }

    bool update_pose()
    {
        try
        {
            // Lookup transform from "map" to "x500_gimbal_0/base_link"
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "map", "x500_gimbal_0/base_link", tf2::TimePointZero, tf2::durationFromSec(0.5));

            // Update current_pose_enu_ (ENU frame relative to map origin)
            current_pose_enu_.position.x = transform.transform.translation.x; // East
            current_pose_enu_.position.y = transform.transform.translation.y; // North
            current_pose_enu_.position.z = transform.transform.translation.z; // Up
            current_pose_enu_.orientation = transform.transform.rotation;

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookupTransform failed: %s", ex.what());
            return false;
        }
    }

    bool isHeadingLockWaypoint(size_t index)
    {
        if (index >= waypoint_labels_.size())
            return false;
        return waypoint_labels_[index] == 'h';
    }
    bool isArucoWaypoint(size_t index)
    {
        if (index >= waypoint_labels_.size())
            return false;
        return waypoint_labels_[index] == 'm' || waypoint_labels_[index] == 'p' || waypoint_labels_[index] == 'h';
    }

    void send_gimbal_target_pitch_degree(float gimbal_pitch_degree)
    {
        if (std::fabs(current_gimbal_pitch_ - gimbal_pitch_degree) < 0.1f)
        {
            // Avoid sending duplicate commands (with a small tolerance)
            return;
        }

        std_msgs::msg::Float32 msg;
        msg.data = gimbal_pitch_degree;
        gimbal_pitch_pub_->publish(msg);

        current_gimbal_pitch_ = gimbal_pitch_degree;

        RCLCPP_INFO(this->get_logger(), "Gimbal pitch set to %.1f degrees", gimbal_pitch_degree);
    }

    void main_loop()
    {
        update_pose(); // Update current drone pose from TF

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //                      "State: %d, enu_x=  %.2f , enu_y=%.2f, enu_z=%.2f, WP_Idx: %zu",
        //                      static_cast<int>(state_), current_pose_enu_.position.x, current_pose_enu_.position.y, current_pose_enu_.position.z, current_wp_idx_);
        switch (state_)
        {
        case MissionState::IDLE:
        {
            if (!set_localstatic_)
            {
                RCLCPP_INFO(get_logger(), "Attempting to set local static frame for NED coordinates.");
                set_local_static(); // Attempt to set the local_static frame
                if (set_localstatic_)
                { // Check if it was successfully set
                    std_msgs::msg::Bool ready_msg;
                    ready_msg.data = true;
                    ready_to_go_pub_->publish(ready_msg);
                    RCLCPP_INFO(get_logger(), "Ready to go. Waiting for takeoff command.");
                }
            }
            break;
        }

        case MissionState::TAKEOFF:
        {
            send_gimbal_target_pitch_degree(-90.0f);

            // Create a Pose from the current waypoint Point
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = current_pose_enu_.position.x; // Use current drone's position
            target_pose.position.y = current_pose_enu_.position.y; // Use current drone's position
            target_pose.position.z = TAKEOFFALTITUDE;              // Set desired takeoff altitude
            // For takeoff, use fixed heading or current heading. Let's use FIXED_HEADING_QUATERNION_ for consistency.
            target_pose.orientation = current_pose_enu_.orientation; // Use current drone's orientation

            send_setpoint_enu_to_ned(target_pose); // Pass a Pose object
            RCLCPP_INFO(get_logger(), "send_setpoint");
            if (current_pose_enu_.position.z > 0.5f * TAKEOFFALTITUDE - 0.5) // Reduced tolerance for reaching altitude
            {
                if (!takeoff_sent_)
                {
                    std_msgs::msg::Bool flag;
                    flag.data = true;
                    takeoff_pub_->publish(flag);
                    takeoff_sent_ = true;
                    RCLCPP_INFO(get_logger(), "[UAV] Takeoff complete: drone_takeoff=TRUE");
                }
                state_ = MissionState::MOVING_TO_WAYPOINT;
                RCLCPP_INFO(get_logger(), "State changed to MOVING_TO_WAYPOINT");
            }
            break;
        }
        case MissionState::MOVING_TO_WAYPOINT:
        {
            // Adjust gimbal pitch based on next waypoint label
            if (current_wp_idx_ < waypoint_labels_.size())
            {
                char next_label = waypoint_labels_[current_wp_idx_];

                if (next_label == 'h')
                {
                    // RCLCPP_INFO(get_logger(), "H DETECTED, setting gimbal pitch to 0 degrees.");
                    send_gimbal_target_pitch_degree(0.0f); // Forward
                }
                else if (next_label == 'm' || next_label == 'p')
                {
                    // RCLCPP_INFO(get_logger(), "M OR P DETECTED, setting gimbal pitch to -90 degrees.");
                    send_gimbal_target_pitch_degree(-90.0f); // Downward
                }
            }

            if (current_wp_idx_ >= waypoints_enu_.size())
            {
                RCLCPP_INFO(get_logger(), "Mission sequence complete. Hovering.");
                state_ = MissionState::MISSION_COMPLETE; // Transition to safe hovering
                break;
            }

            const auto &wp_point = waypoints_enu_[current_wp_idx_];
            geometry_msgs::msg::Pose target_pose;
            target_pose.position = wp_point;

            // If current waypoint requires heading lock ('h' label)
            if (isHeadingLockWaypoint(current_wp_idx_))
            {
                target_pose.orientation = FIXED_HEADING_QUATERNION_; // Use predefined fixed orientation
                send_setpoint_enu_to_ned(target_pose);
            }
            else
            {
                // For all other waypoints, use the current drone's orientation or a default (e.g., 0 yaw)
                // For position control, orientation is often ignored by PX4 if position-only commands are sent.
                // However, if you explicitly want to control yaw, you need to provide a valid quaternion.
                // For simplicity, let's use the drone's current yaw if not heading locked.
                // target_pose.orientation = current_pose_enu_.orientation; // Use current drone's orientation
                send_setpoint_enu_to_ned(target_pose);
            }

            if (is_close(current_pose_enu_.position, waypoints_enu_[current_wp_idx_], 0.5))
            {
                RCLCPP_INFO(get_logger(), "Arrived at waypoint %zu.", current_wp_idx_);
                if (isArucoWaypoint(current_wp_idx_))
                {
                    wait_start_time_ = this->now();
                    state_ = MissionState::WAIT_BEFORE_SEARCHING;
                }
                else
                {
                    ++current_wp_idx_;
                }
            }
            break;
        }
        case MissionState::WAIT_BEFORE_SEARCHING:
        {
            const auto &wp_point = waypoints_enu_[current_wp_idx_];
            geometry_msgs::msg::Pose target_pose;
            target_pose.position = wp_point;
            if (isHeadingLockWaypoint(current_wp_idx_))
            {
                target_pose.orientation = FIXED_HEADING_QUATERNION_;
            }
            else
            {
                target_pose.orientation = current_pose_enu_.orientation;
            }
            send_setpoint_enu_to_ned(target_pose);

            if (is_drone_stable_ || (this->now() - wait_start_time_).seconds() > stable_timeout_)
            {
                RCLCPP_INFO(get_logger(), "drone stabled");
                search_start_time_ = this->now();
                state_ = MissionState::SEARCHING_FOR_MARKER;
            }
            break;
        }

        case MissionState::SEARCHING_FOR_MARKER:
        {
            if (current_wp_idx_ < waypoints_enu_.size())
            {
                const auto &wp_point = waypoints_enu_[current_wp_idx_];
                geometry_msgs::msg::Pose target_pose;
                target_pose.position = wp_point;
                if (isHeadingLockWaypoint(current_wp_idx_))
                {
                    target_pose.orientation = FIXED_HEADING_QUATERNION_;
                }
                else
                {
                    target_pose.orientation = current_pose_enu_.orientation;
                }
                send_setpoint_enu_to_ned(target_pose);

                if ((this->now() - search_start_time_).seconds() > searching_timeout_)
                {
                    RCLCPP_WARN(get_logger(), "No marker found at WP %zu within 2s. Moving on.", current_wp_idx_);
                    ++current_wp_idx_; // Move to next waypoint
                    state_ = MissionState::MOVING_TO_WAYPOINT;
                }
            }
            break;
        }
        case MissionState::PRECISION_LANDING:
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                                 "[Landing] Phase: %s | Target [N: %.2f, E: %.2f, Z: %.2f]",
                                 precision_landing_phase_ == PrecisionLandingPhase::XY_ALIGN ? "XY_ALIGN" : "DESCENDING",
                                 landing_spot_enu_.x, landing_spot_enu_.y, target_landing_z_);

            geometry_msgs::msg::Pose landing_pose;
            landing_pose.orientation = current_pose_enu_.orientation; // Yaw 유지

            double dx = latest_marker_position_.x - current_pose_enu_.position.x;
            double dy = latest_marker_position_.y - current_pose_enu_.position.y;
            double dist_xy = std::hypot(dx, dy);

            switch (precision_landing_phase_)
            {
            case PrecisionLandingPhase::XY_ALIGN:
            {
                // Step 1: x, y 먼저 정렬
                landing_pose.position.x = latest_marker_position_.x;
                landing_pose.position.y = latest_marker_position_.y;
                landing_pose.position.z = current_pose_enu_.position.z; // z 유지

                if (is_drone_stable_ && dist_xy < 0.2) // xy 정렬 완료
                {
                    precision_landing_phase_ = PrecisionLandingPhase::DESCENDING;
                    target_landing_z_ = current_pose_enu_.position.z - landing_step_; // 하강 시작
                    RCLCPP_INFO(get_logger(), "[Landing] XY 정렬 완료. 하강 단계 시작 (목표 z: %.2f)", target_landing_z_);
                }
                break;
            }

            case PrecisionLandingPhase::DESCENDING:
            {
                landing_pose.position.x = latest_marker_position_.x;
                landing_pose.position.y = latest_marker_position_.y;
                landing_pose.position.z = target_landing_z_; // 이번에 목표할 z

                if (current_pose_enu_.position.z <= landing_min_z_)
                {
                    RCLCPP_INFO(get_logger(), "Precision landing complete. Mission finished.");
                    main_timer_->cancel();
                    break;
                }

                // 다음 단계로 내려가도 될지 확인
                if (is_drone_stable_ && dist_xy < 0.2 &&
                    std::abs(current_pose_enu_.position.z - target_landing_z_) < 0.2)
                {
                    // 다음 단계로 z 더 낮추기
                    target_landing_z_ -= landing_step_;
                    if (target_landing_z_ < landing_min_z_)
                        target_landing_z_ = landing_min_z_;
                    RCLCPP_INFO(get_logger(), "[Landing] 다음 하강 목표 z: %.2f", target_landing_z_);
                }

                break;
            }
            }

            send_setpoint_enu_to_ned(landing_pose);
            break;
        }

        case MissionState::MISSION_COMPLETE:
        {
            RCLCPP_INFO(get_logger(), "Mission complete. Hovering at final location.");
            if (ugv_landing_ready_) // UGV 도착
            {
                RCLCPP_INFO(get_logger(), "UGV READY FOR LANDING");
                state_ = MissionState::PRECISION_LANDING;
                std_msgs::msg::Float32 marker_size_msg;
                marker_size_msg.data = 0.5;
                marker_size_pub_->publish(marker_size_msg);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "UGV is not ready for landing. Proceeding to hover and prepare for landing.");
                break;
            }

            if (!waypoints_enu_.empty())
            {
                const auto &last_wp_point = waypoints_enu_.back();
                geometry_msgs::msg::Pose hover_pose;
                hover_pose.position.x = last_wp_point.x;
                hover_pose.position.y = last_wp_point.y;
                hover_pose.position.z = last_wp_point.z;                // Hover 5m above last waypoint
                hover_pose.orientation = current_pose_enu_.orientation; // Maintain current yaw for hovering

                send_setpoint_enu_to_ned(hover_pose); // Send the full Pose for hovering
            }
            break;
        }
        }
    }
    void local_pose_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {

        local_v_ = msg; // PX4 로컬 위치 업데이트

        // 3D 속도
        float speed = std::sqrt(msg->vx * msg->vx + msg->vy * msg->vy + msg->vz * msg->vz);

        // 안정화 상태 감지
        if (speed < stopped_vel_threshold_)
        {
            is_drone_stable_ = true;
        }
        else
        {
            is_drone_stable_ = false;
        }
    }

    void aruco_marker_cb(const aruco_interfaces::msg::MarkerPoseId::SharedPtr msg)
    {
        // Only process markers if in SEARCHING_FOR_MARKER state
        if (state_ != MissionState::SEARCHING_FOR_MARKER)
            return;

        latest_marker_position_ = msg->pose.position;

        // Record the new marker IDR
        seen_marker_ids_.insert(msg->id);

        precise_markers_world_.push_back(msg->pose.position);
        if (msg->id != 10) // ✅ ID가 10이 아닐 때만 저장
        {
            std::ofstream ofs(MARKER_SAVE_PATH, std::ios_base::app);
            if (ofs)
            {
                ofs.precision(15);
                ofs << msg->pose.position.x << ","
                    << msg->pose.position.y << ","
                    << msg->pose.position.z << ","
                    << msg->id << "\n";
                ofs.close(); // Close the file after writing
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Failed to open marker_location.csv for writing!");
            }
        }

        RCLCPP_INFO(get_logger(), "Marker #%d found! CSV saved. Proceeding to next waypoint.", msg->id);

        if (current_wp_idx_ == waypoints_enu_.size() - 1)
        {
            RCLCPP_INFO(get_logger(), "Final marker found! Initiating precision landing.");
            landing_spot_enu_ = msg->pose.position; // Store in ENU (map) coordinates
            state_ = MissionState::PRECISION_LANDING;
        }
        else
        {
            ++current_wp_idx_;
            state_ = MissionState::MOVING_TO_WAYPOINT;
        }
    }

    void send_setpoint_enu_to_ned(const geometry_msgs::msg::Pose &target_pose_map)
    {
        // RCLCPP_INFO(this->get_logger(), "send_setpoint_enu_to_ned called with target: x=%.2f, y=%.2f, z=%.2f",
        //             target_pose_map.position.x, target_pose_map.position.y, target_pose_map.position.z);

        geometry_msgs::msg::PoseStamped input_pose_stamped;
        input_pose_stamped.header.frame_id = "map";
        input_pose_stamped.header.stamp = this->get_clock()->now(); // Corrected clock access
        input_pose_stamped.pose = target_pose_map;

        try
        {
            // Transform from "map" to "local_static"
            auto transformed_pose_stamped = tf_buffer_->transform(input_pose_stamped, "local_static", tf2::durationFromSec(0.1));

            geometry_msgs::msg::PoseStamped final_target_pose = transformed_pose_stamped; // Declare here to be accessible for logging
            final_target_pose.header.stamp = input_pose_stamped.header.stamp;             // Update timestamp after transform
            pose_cmd_pub_->publish(final_target_pose);                                    // Publish the transformed pose

            // Yaw 값을 가져오는 새로운 방법: Matrix3x3을 통한 RPY 추출
            tf2::Quaternion q_tf2;
            tf2::fromMsg(transformed_pose_stamped.pose.orientation, q_tf2); // geometry_msgs::msg::Quaternion을 tf2::Quaternion으로 변환

            tf2::Matrix3x3 m(q_tf2); // tf2::Quaternion으로부터 Matrix3x3 생성
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw); // Matrix3x3으로부터 Roll, Pitch, Yaw 추출

            // RCLCPP_INFO(this->get_logger(), "local_setpoint: x=%.2f, y=%.2f, z=%.2f, Yaw: %.2f (rad)",
            //             transformed_pose_stamped.pose.position.x,
            //             transformed_pose_stamped.pose.position.y,
            //             transformed_pose_stamped.pose.position.z,
            //             yaw); // 추출된 yaw 값 사용
        } // tf2::Quaternion으로부터 Yaw를 추출

        catch (const tf2::TransformException &ex) // Correctly place catch block
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform 'map' to 'local_static': %s", ex.what());
        }
    }
    void ugvLandingSpotCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        ugv_landing_ready_ = msg->data;
    }
    // Helper to load waypoints from CSV
    void load_waypoints(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", filename.c_str());
            return;
        }

        std::string line;
        // Skip header line
        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> tokens;

            while (std::getline(ss, segment, ','))
            {
                tokens.push_back(segment);
            }

            if (tokens.size() >= 4) // Ensure at least x, y, z, and label are present
            {
                geometry_msgs::msg::Point wp;
                try
                {
                    wp.x = std::stod(tokens[0]);
                    wp.y = std::stod(tokens[1]);
                    wp.z = std::stod(tokens[2]);
                    waypoints_enu_.push_back(wp);
                    waypoint_labels_.push_back(tokens[3][0]); // Get the first char of the label
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing waypoint line '%s': %s", line.c_str(), e.what());
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints_enu_.size(), filename.c_str());
    }

    // Helper to check if two points are close
    bool is_close(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double tolerance)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) < tolerance;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavController>());
    rclcpp::shutdown();
    return 0;
}