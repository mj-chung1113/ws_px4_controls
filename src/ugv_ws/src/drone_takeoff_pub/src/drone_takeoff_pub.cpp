#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class TakeoffPublisher : public rclcpp::Node {
public:
  TakeoffPublisher()
  : Node("drone_takeoff_pub")
  {
    pub_ = create_publisher<std_msgs::msg::Bool>("drone_takeoff", 10);

    // 2 초 뒤 한 번만 발행하고 종료
    timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published drone_takeoff → true");
        rclcpp::shutdown();
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffPublisher>());
  return 0;
}
