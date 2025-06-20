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
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 드론 base_link 위치 브로드캐스트용
        auto sensor_qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&PoseToTfBroadcaster::local_position_callback, this, std::placeholders::_1));

        // 카메라 static transform 1회만 브로드캐스트
        broadcast_camera_static_transform();

        RCLCPP_INFO(this->get_logger(), "C++ Pose to TF Broadcaster has been started.");
    }

private:
    void broadcast_camera_static_transform()
    {
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.stamp = this->get_clock()->now();
        camera_tf.header.frame_id = "x500_gimbal_0/camera_link";  // 부모 프레임
        camera_tf.child_frame_id = "x500_gimbal_0/camera";        // 자식 프레임

        // 위치: -0.07, 0, -0.16
        camera_tf.transform.translation.x = -0.07;
        camera_tf.transform.translation.y = 0.0;
        camera_tf.transform.translation.z = -0.16;

        // 회전: RPY(0, 0, pi) → 180도 yaw
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI);
        camera_tf.transform.rotation.x = q.x();
        camera_tf.transform.rotation.y = q.y();
        camera_tf.transform.rotation.z = q.z();
        camera_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(camera_tf);
        RCLCPP_INFO(this->get_logger(), "Static transform camera_link → camera broadcasted.");
    }

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";       // 월드 기준 (ENU)
        t.child_frame_id = "base_link";  // 드론 기준

        // 위치: PX4는 NED → ENU로 변환
        t.transform.translation.x = msg->y;
        t.transform.translation.y = msg->x;
        t.transform.translation.z = -msg->z;

        // 방향 (heading → ENU yaw)
        tf2::Quaternion q;
        double enu_yaw = -msg->heading + (M_PI / 2.0);
        q.setRPY(0, 0, enu_yaw);

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

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
