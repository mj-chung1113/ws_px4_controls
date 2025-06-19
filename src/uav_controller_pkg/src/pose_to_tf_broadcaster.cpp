#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class PoseToTfBroadcaster : public rclcpp::Node
{
public:
    PoseToTfBroadcaster() : Node("pose_to_tf_broadcaster")
    {
        // TF 정보를 발행할 Broadcaster를 초기화합니다.
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // VehicleLocalPosition 토픽을 구독합니다.
        auto sensor_qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&PoseToTfBroadcaster::local_position_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ Pose to TF Broadcaster has been started.");
    }

private:
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;

        // TF 메시지의 헤더를 설정합니다.
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        // VehicleLocalPosition 메시지에서 위치 정보를 추출하여 ENU 좌표계로 변환합니다.
        t.transform.translation.x = msg->y;
        t.transform.translation.y = msg->x;
        t.transform.translation.z = -msg->z;

        // VehicleLocalPosition 메시지에서 방향(heading) 정보를 추출하여 쿼터니언으로 변환합니다.
        // PX4 heading (NED, 0=North) -> ENU yaw (0=East)
        tf2::Quaternion q;
        double enu_yaw = -msg->heading + (M_PI / 2.0);
        q.setRPY(0, 0, enu_yaw);

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // TF 정보를 발행합니다.
        tf_broadcaster_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseToTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}