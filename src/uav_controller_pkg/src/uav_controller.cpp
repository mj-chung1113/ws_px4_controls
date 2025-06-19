// uav_controller.cpp — 최종 버전 (내부 NED / 출력 ENU)
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/int32.hpp>

#include <aruco_interfaces/msg/marker_pose_id.hpp> // 커스텀 메시지 

#define TAKEOFFALTITUDE -5.0  // NED 기준, 드론이 이 높이로 상승하면 착륙 지점으로 이동
enum class MissionState {
    IDLE,
    TAKEOFF,
    MOVING_TO_WAYPOINT,
    SEARCHING_FOR_MARKER,
    MISSION_COMPLETE,
    PRECISION_LANDING
};

class UavController : public rclcpp::Node
{
public:
    UavController() : Node("uav_controller"), state_(MissionState::IDLE), current_wp_idx_(0)
    {
        load_and_convert_waypoints_to_ned("/home/jmj/pro_asp_ws/ws_px4_controls/optimized_path.csv");

        pose_cmd_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose", 10);
        
        // auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        // marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/x500/target_pose", pose_qos,
        //     std::bind(&UavController::marker_cb, this, std::placeholders::_1));

        
        // pose 및 id 바인딩 
        auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(3));
        marker_sub_ = create_subscription<aruco_interfaces::msg::MarkerPoseId>(
        "/x500/target", pose_qos, // pose_qos는 적절히 정의되어 있다고 가정
        std::bind(&UavController::aruco_marker_cb, this, std::placeholders::_1)); // 추가

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto sensor_qos = rclcpp::SensorDataQoS();
        odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", sensor_qos,
            std::bind(&UavController::odometry_cb, this, std::placeholders::_1));
            
        main_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UavController::main_loop, this));

        RCLCPP_INFO(get_logger(), "UavController initialised. Mission starts in 2 s …");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        state_ = MissionState::TAKEOFF;


    }

private:
    // --- 월드 좌표계(ENU)에서 드론의 시작 위치 (오프셋) ---
    static constexpr double DRONE_START_X = -134.74925610706298; // East
    static constexpr double DRONE_START_Y =  61.782506989510747; // North
    static constexpr double DRONE_START_Z =  -0.39702116898307343; // Up

    // Pubs/Subs
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_pub_;
    rclcpp::Subscription<aruco_interfaces::msg::MarkerPoseId>::SharedPtr marker_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::TimerBase::SharedPtr main_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Mission data (모두 NED 좌표계 기준)
    MissionState state_;
    size_t current_wp_idx_;
    std::vector<geometry_msgs::msg::Point> waypoints_ned_;
    std::vector<geometry_msgs::msg::Point> precise_markers_world_;
    geometry_msgs::msg::Pose current_pose_ned_{};
    geometry_msgs::msg::Point landing_spot_ned_{};

    bool isArucoWaypoint(size_t index)
    {
    if (index >= waypoint_labels_.size()) return false;
    return waypoint_labels_[index] == 'm' || waypoint_labels_[index] == 'p'  ;
    }

    void odometry_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // 위치와 방향 정보를 NED 좌표계 그대로 저장
        current_pose_ned_.position.x = msg->position[0];
        current_pose_ned_.position.y = msg->position[1];
        current_pose_ned_.position.z = msg->position[2];
        
        current_pose_ned_.orientation.w = msg->q[0];
        current_pose_ned_.orientation.x = msg->q[1];
        current_pose_ned_.orientation.y = msg->q[2];
        current_pose_ned_.orientation.z = msg->q[3];
    }

    void main_loop()
    {
        // 로그에는 고도(Up)를 표시하기 위해 NED의 z값에 -1을 곱함
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "State: %d, Altitude: %.2f m, WP_Idx: %zu",
            static_cast<int>(state_), -current_pose_ned_.position.z, current_wp_idx_);

        switch(state_)
        {
        case MissionState::IDLE: break;

        case MissionState::TAKEOFF:
            send_setpoint_as_enu(0, 0, TAKEOFFALTITUDE);
            if (-current_pose_ned_.position.z > TAKEOFFALTITUDE+0.5) {
                RCLCPP_INFO(get_logger(), "Takeoff complete. Moving to first waypoint.");
                state_ = MissionState::MOVING_TO_WAYPOINT;
            }
            break;

        case MissionState::MOVING_TO_WAYPOINT:
            // ▼▼▼ CHANGED: 이 블록의 로직을 수정합니다 ▼▼▼
            if (current_wp_idx_ >= waypoints_ned_.size()) {
                RCLCPP_INFO(get_logger(), "Mission sequence complete. Hovering.");
                state_ = MissionState::MISSION_COMPLETE; // 착륙 대신 안전한 호버링 상태로 전환
                break;
            }
            // ▲▲▲ CHANGED END ▲▲▲

            {
                const auto& wp = waypoints_ned_[current_wp_idx_];
                // 웨이포인트 고도보다 5m 위에서 접근하도록 수정 (안전을 위해)
                send_setpoint_as_enu(wp.x, wp.y, wp.z);
            }

            if (is_close(current_pose_ned_.position, waypoints_ned_[current_wp_idx_], 1.0)) {
                RCLCPP_INFO(get_logger(), "Arrived at waypoint %zu.", current_wp_idx_);
                if (isArucoWaypoint(current_wp_idx_)) {
                    state_ = MissionState::SEARCHING_FOR_MARKER;
                } else {
                    ++current_wp_idx_;
                }
            }
            break;

        case MissionState::SEARCHING_FOR_MARKER:
            if(current_wp_idx_ < waypoints_ned_.size()) {
                const auto& wp = waypoints_ned_[current_wp_idx_];
                // 마커 탐색을 위해 웨이포인트보다 2m 위에서 호버링
                // send_setpoint_as_enu(wp.x, wp.y, wp.z - 2.0);
                send_setpoint_as_enu(wp.x, wp.y, wp.z+1);
            }
            break;

        case MissionState::PRECISION_LANDING:
            RCLCPP_INFO(get_logger(), "Executing precision landing at [N: %.2f, E: %.2f]", landing_spot_ned_.x, landing_spot_ned_.y);
            send_setpoint_as_enu(landing_spot_ned_.x, landing_spot_ned_.y, 1.0); // Land mode

            if (-current_pose_ned_.position.z < 0.3) {
                RCLCPP_INFO(get_logger(), "Precision landing complete. Mission finished.");
                main_timer_->cancel();
            }
            break;

        case MissionState::MISSION_COMPLETE:
            RCLCPP_INFO(get_logger(), "Mission complete. Hovering at final location.");
            if (!waypoints_ned_.empty()) {
                const auto& last_wp = waypoints_ned_.back();
                // 마지막 웨이포인트 상공에서 호버링
                send_setpoint_as_enu(last_wp.x, last_wp.y, last_wp.z - 5.0);
            }
            break;
        }
    }

    void aruco_marker_cb(const aruco_interfaces::msg::MarkerPoseId::SharedPtr msg)
    {
        if (state_ != MissionState::SEARCHING_FOR_MARKER) return;

        // multi_tracker_node에서 이미 "x500_gimbal_0/base_link" 프레임으로 변환하여 발행했으므로
        // uav_controller에서는 TF 변환이 더 이상 필요 없습니다!
        // msg->pose는 이미 드론의 base_link 기준입니다.

        geometry_msgs::msg::Point marker_pos_ned;
        // base_link 기준 마커 위치를 현재 드론의 NED 위치에 더하여 월드 NED 좌표를 얻습니다.
        marker_pos_ned.x = current_pose_ned_.position.x + msg->pose.position.x;
        marker_pos_ned.y = current_pose_ned_.position.y + msg->pose.position.y;
        marker_pos_ned.z = current_pose_ned_.position.z + msg->pose.position.z;

        geometry_msgs::msg::Point marker_pos_world;
        marker_pos_world.x = marker_pos_ned.y + DRONE_START_X;
        marker_pos_world.y = marker_pos_ned.x + DRONE_START_Y;
        marker_pos_world.z = -marker_pos_ned.z + DRONE_START_Z;

        precise_markers_world_.push_back(marker_pos_world);

        // ✅ 마커 ID와 함께 저장
        std::ofstream ofs("/home/jmj/pro_asp_ws/ws_px4_controls/precise_marker_locations.csv", std::ios_base::app);
        if (ofs) {
            ofs.precision(15);
            ofs << marker_pos_world.x << "," << marker_pos_world.y << "," << marker_pos_world.z << "," << msg->id << "\n"; // msg->id 사용
        }

        RCLCPP_INFO(get_logger(), "Marker #%d found! CSV saved. Proceeding to next waypoint.", msg->id); // msg->id 사용

        if (current_wp_idx_ == waypoints_ned_.size() - 1) {
            RCLCPP_INFO(get_logger(), "Final marker found! Initiating precision landing.");
            landing_spot_ned_ = marker_pos_ned;
            state_ = MissionState::PRECISION_LANDING;
        } else {
            ++current_wp_idx_;
            state_ = MissionState::MOVING_TO_WAYPOINT;
        }
    }
    // void marker_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    // {
    //     if (state_ != MissionState::SEARCHING_FOR_MARKER) return;

    //     geometry_msgs::msg::PoseStamped marker_in_base;
    //     try {
    //         tf_buffer_->transform(*msg, marker_in_base, "x500_gimbal_0/base_link", tf2::durationFromSec(0.1));
    //     }
    //     catch (tf2::TransformException &ex) {
    //         RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
    //         return;
    //     }

    //     geometry_msgs::msg::Point marker_pos_ned;
    //     marker_pos_ned.x = current_pose_ned_.position.x + marker_in_base.pose.position.x;
    //     marker_pos_ned.y = current_pose_ned_.position.y + marker_in_base.pose.position.y;
    //     marker_pos_ned.z = current_pose_ned_.position.z + marker_in_base.pose.position.z;

    //     geometry_msgs::msg::Point marker_pos_world;
    //     marker_pos_world.x = marker_pos_ned.y + DRONE_START_X;
    //     marker_pos_world.y = marker_pos_ned.x + DRONE_START_Y;
    //     marker_pos_world.z = -marker_pos_ned.z + DRONE_START_Z;


    //     precise_markers_world_.push_back(marker_pos_world);
    //     save_precise_locations("/home/jmj/pro_asp_ws/ws_px4_controls/precise_marker_locations.csv");
        
    //     RCLCPP_INFO(get_logger(), "Marker found! CSV saved. Proceeding to next waypoint.");

    //     if (current_wp_idx_ == waypoints_ned_.size() - 1) {
    //         // 마지막 웨이포인트인 경우
    //         RCLCPP_INFO(get_logger(), "Final marker found! Initiating precision landing on marker.");
    //         landing_spot_ned_ = marker_pos_ned; // 착륙 지점을 '마커 위치'로 설정
    //         state_ = MissionState::PRECISION_LANDING;   // 정밀 착륙 상태로 전환
    //     } else {
    //         // 마지막이 아닌 다른 ArUco 웨이포인트인 경우
    //         RCLCPP_INFO(get_logger(), "Marker found! Proceeding to next waypoint.");
    //         ++current_wp_idx_; // 다음 웨이포인트로 인덱스 증가
    //         state_ = MissionState::MOVING_TO_WAYPOINT; // 다시 웨이포인트 이동 상태로 전환
    //     }
    // }

    void send_setpoint_as_enu(double north, double east, double down)
    {
        geometry_msgs::msg::PoseStamped sp;
        sp.header.frame_id = "map";
        sp.header.stamp = this->get_clock()->now();
        
        sp.pose.position.x = east;
        sp.pose.position.y = north;
        sp.pose.position.z = -down;
        
        sp.pose.orientation.w = 1.0;
        pose_cmd_pub_->publish(sp);
    }

    static bool is_close(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b, double tol)
    { 
        double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z; 
        return std::sqrt(dx*dx + dy*dy + dz*dz) < tol; 
    }
    // 헤더 파일에서 추가 필요

    std::vector<char> waypoint_labels_;   

    void load_and_convert_waypoints_to_ned(const std::string& file)
    {
        std::ifstream ifs(file);
        if (!ifs) {
            RCLCPP_ERROR(get_logger(), "Cannot open waypoint file: %s", file.c_str());
            rclcpp::shutdown();
            return;
        }

        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string value_str;

            geometry_msgs::msg::Point world_p_enu{};
            std::getline(ss, value_str, ' '); world_p_enu.x = std::stod(value_str);
            std::getline(ss, value_str, ' '); world_p_enu.y = std::stod(value_str);
            std::getline(ss, value_str, ' '); world_p_enu.z = std::stod(value_str);

            // 라벨이 있다면 읽기
            char label = ' ';  // default
            if (std::getline(ss, value_str, ' ')) {
                label = value_str.empty() ? ' ' : value_str[0];  // 'm', 'i', 'p'
            }

            // ENU → NED 변환
            double rel_enu_x = world_p_enu.x - DRONE_START_X;
            double rel_enu_y = world_p_enu.y - DRONE_START_Y;
            double rel_enu_z = world_p_enu.z - DRONE_START_Z;

            geometry_msgs::msg::Point p_ned{};
            p_ned.x = rel_enu_y;
            p_ned.y = rel_enu_x;
            p_ned.z = -rel_enu_z;

            waypoints_ned_.push_back(p_ned);
            waypoint_labels_.push_back(label);
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded and converted %zu waypoints to NED frame from %s",
            waypoints_ned_.size(),
            file.c_str()
        );
    }

    void save_precise_locations(const std::string& file)
    {
        std::ofstream ofs(file, std::ios_base::app);
        if (!ofs) {
            std::ofstream create_file(file);
            create_file.precision(15);
            create_file << "x,y,z\n";
            create_file.close();
            ofs.open(file, std::ios_base::app);
        }
        ofs.precision(15);
        
        if (!precise_markers_world_.empty()) {
            const auto& world_p = precise_markers_world_.back();
            ofs << world_p.x << ',' << world_p.y << ',' << world_p.z << '\n';
        }
    }
};

int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<UavController>());rclcpp::shutdown();return 0;}