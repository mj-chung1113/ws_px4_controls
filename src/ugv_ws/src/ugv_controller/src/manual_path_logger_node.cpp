#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <fstream>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

class ManualPathLogger : public rclcpp::Node {
public:
  ManualPathLogger()
  : Node("manual_path_logger"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    file_path_ = std::string(std::getenv("HOME")) + "/final_project/ugv_ws/src/ugv_controller/mission1.csv";
    
    // 🔁 기존 파일 덮어쓰기 (ios::trunc)
    file_.open(file_path_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "CSV 파일을 열 수 없습니다: %s", file_path_.c_str());
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "✅ CSV 저장 위치: %s", file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "↩️  Enter 키를 누르면 위치가 기록됩니다 (Ctrl+C로 종료)");
  }

  void spin() {
    while (rclcpp::ok()) {
      if (check_enter_pressed()) {
        log_current_pose();
      }
      rclcpp::spin_some(shared_from_this());
      usleep(100000);  // 0.1초 대기
    }
    file_.close();
    RCLCPP_INFO(this->get_logger(), "📁 CSV 저장 완료 후 종료");
  }

private:
  std::string file_path_;
  std::ofstream file_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void log_current_pose() {
    try {
      auto transform = tf_buffer_.lookupTransform("map", "X1_asp/base_link", tf2::TimePointZero);
      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;
      int state = 1;

      file_ << x << "," << y << "," << state << "\n";
      file_.flush();  // 실시간 저장
      RCLCPP_INFO(this->get_logger(), "저장됨: x=%.2f, y=%.2f, state=%d", x, y, state);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF 실패: %s", ex.what());
    }
  }

  bool check_enter_pressed() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);

    int result = select(1, &fds, NULL, NULL, &tv);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 터미널 설정 복원

    if (result > 0) {
      char ch;
      read(STDIN_FILENO, &ch, 1);
      return ch == '\n';
    }
    return false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualPathLogger>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
