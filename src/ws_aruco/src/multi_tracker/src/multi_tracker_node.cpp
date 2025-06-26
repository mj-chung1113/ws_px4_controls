#include "multi_tracker/multi_tracker_node.hpp"
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <aruco_interfaces/msg/marker_pose_id.hpp>
#include <std_msgs/msg/float32.hpp>

MultiTrackerNode::MultiTrackerNode()
    : Node("multi_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting MultiTrackerNode");

    loadParameters();

    RCLCPP_INFO(this->get_logger(), "Loaded vehicle_type: %s", _vehicle_type.c_str());

    // RMW QoS settings
    auto image_qos = rclcpp::QoS(1).best_effort();
    auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(3)).reliable();

    _detectorParams = cv::aruco::DetectorParameters::create();
    _dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

    _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        _image_topic, image_qos,
        std::bind(&MultiTrackerNode::image_callback, this, std::placeholders::_1));

    _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        _camera_info_topic, image_qos,
        std::bind(&MultiTrackerNode::camera_info_callback, this, std::placeholders::_1));

    marker_size_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/mk_size", image_qos,
        std::bind(&MultiTrackerNode::marker_size_callback, this, std::placeholders::_1));

    //_target_id_pub = this->create_publisher<std_msgs::msg::Int32>(_target_id_topic, pose_qos);
    _image_pub = this->create_publisher<sensor_msgs::msg::Image>(_image_proc_topic, image_qos);
    //_target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(_target_pose_topic, pose_qos);
    _aruco_marker_pub = this->create_publisher<aruco_interfaces::msg::MarkerPoseId>(_aruco_marker_topic, pose_qos); // 추가
    // TF Buffer 및 Listener 초기화
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void MultiTrackerNode::loadParameters()
{
    declare_parameter<std::string>("vehicle_type", "x500");
    declare_parameter<int>("aruco_id", 0);
    declare_parameter<int>("dictionary", 0); // DICT_4X4_50
    declare_parameter<double>("marker_size", 1.0);

    get_parameter("vehicle_type", _vehicle_type);
    get_parameter("aruco_id", _param_aruco_id);
    get_parameter("dictionary", _param_dictionary);
    get_parameter("marker_size", _param_marker_size);

    declare_parameter<std::string>("image_topic", "");
    declare_parameter<std::string>("camera_info_topic", "");
    // declare_parameter<std::string>("target_id_topic", "/target_id");
    declare_parameter<std::string>("image_proc_topic", "/image_proc");
    // declare_parameter<std::string>("target_pose_topic", "/target_pose");
    declare_parameter<std::string>("aruco_marker_topic", "/x500/target");

    get_parameter("image_topic", _image_topic);
    get_parameter("camera_info_topic", _camera_info_topic);
    // get_parameter("target_id_topic", _target_id_topic);
    get_parameter("image_proc_topic", _image_proc_topic);
    // get_parameter("target_pose_topic", _target_pose_topic);
    get_parameter("aruco_marker_topic", _aruco_marker_topic);
}
void MultiTrackerNode::marker_size_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    _param_marker_size = msg->data;

    RCLCPP_WARN(this->get_logger(), "[marker_size_callback] 마커 사이즈 업데이트됨: %f", _param_marker_size);
}

void MultiTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            if (msg->encoding == "rgb8")
            {
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, _dictionary, corners, ids, _detectorParams);
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        if (ids.empty())
        {
            // No markers detected, just publish the processed image and return
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = cv_ptr->image;
            _image_pub->publish(*out_msg.toImageMsg().get());
            return; // 마커가 없으면 여기서 함수 종료
        }
        double max_area = 0.0;
        int best_marker_idx = -1;

        // Iterate through detected markers to find the one with the largest bounding box area
        for (size_t i = 0; i < ids.size(); ++i)
        {
            // Calculate the bounding box area of the current marker
            // A simple way to approximate area for a quadrilateral is using the Shoelace formula
            // Or just use min/max x/y to get width/height, though not perfectly accurate for rotated markers
            // For simplicity, we can use cv::contourArea for general polygons
            double current_area = cv::contourArea(corners[i]);

            if (current_area > max_area)
            {
                max_area = current_area;
                best_marker_idx = i;
            }
        }
        if (best_marker_idx == -1)
        {
            // This should theoretically not happen if ids is not empty, but as a safeguard
            RCLCPP_WARN(this->get_logger(), "No best marker found despite detections.");
            // Still publish the image without a selected marker pose
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = cv_ptr->image;
            _image_pub->publish(*out_msg.toImageMsg().get());
            return;
        }
        int selected_id = ids[best_marker_idx];
        std::vector<std::vector<cv::Point2f>> selected_corners = {corners[best_marker_idx]}; // Single element vector

        if (!_camera_matrix.empty() && !_dist_coeffs.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs; // Use vectors for multiple markers
            cv::aruco::estimatePoseSingleMarkers(corners, _param_marker_size, _camera_matrix, _dist_coeffs, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); i++)
            {
                // Annotate image with axes
                cv::drawFrameAxes(cv_ptr->image, _camera_matrix, _dist_coeffs, rvecs[i], tvecs[i], _param_marker_size);
                cv::Mat R;
                cv::Rodrigues(rvecs[i], R);
                tf2::Matrix3x3 tf_rot(
                    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
                tf2::Quaternion q;
                tf_rot.getRotation(q);
                //====

                try
                {
                    geometry_msgs::msg::TransformStamped t_cam_to_base_stamped = tf_buffer_->lookupTransform(
                        "x500_gimbal_0/camera_link", "x500_gimbal_0/base_link", tf2::TimePointZero);
                    tf2::Transform t_cam_to_base;
                    tf2::fromMsg(t_cam_to_base_stamped.transform, t_cam_to_base);

                    tf2::Vector3 v_cam_to_base = t_cam_to_base.getOrigin();
                    tf2::Quaternion q_cam_to_base = t_cam_to_base.getRotation();

                    // OpenCV -> base_link 기준 변환
                    tf2::Vector3 v_base_to_marker_assumed(-tvecs[i][2], tvecs[i][0], -tvecs[i][1]);

                    // camera offset은 0
                    tf2::Vector3 v_camera_offset_in_base(0.05, 0.0, -0.16);
                    tf2::Vector3 v_rotated_offset = tf2::quatRotate(q_cam_to_base, v_camera_offset_in_base);

                    tf2::Vector3 final_pos_in_cam = v_cam_to_base + v_base_to_marker_assumed + v_rotated_offset;

                    // 방향 (rvec → quaternion)
                    cv::Mat rot_mat;
                    cv::Rodrigues(rvecs[i], rot_mat);
                    tf2::Matrix3x3 R_marker_in_cv(
                        rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
                        rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                        rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
                    tf2::Quaternion q_marker_in_cv;
                    R_marker_in_cv.getRotation(q_marker_in_cv);

                    tf2::Matrix3x3 R_cv_to_ros;
                    R_cv_to_ros.setValue(0, 1, 0,
                                         0, 0, -1,
                                         -1, 0, 0);
                    tf2::Quaternion q_cv_to_ros;
                    R_cv_to_ros.getRotation(q_cv_to_ros);

                    tf2::Quaternion q_final = q_cv_to_ros * q_marker_in_cv;

                    geometry_msgs::msg::Pose final_relative_pose;
                    final_relative_pose.position.x = final_pos_in_cam.x();
                    final_relative_pose.position.y = final_pos_in_cam.y();
                    final_relative_pose.position.z = final_pos_in_cam.z();
                    final_relative_pose.orientation = tf2::toMsg(q_final);

                    geometry_msgs::msg::PoseStamped marker_in_camera;
                    marker_in_camera.header.stamp = msg->header.stamp;
                    marker_in_camera.header.frame_id = "x500_gimbal_0/camera_link";
                    marker_in_camera.pose = final_relative_pose;

                    marker_in_camera.pose.orientation = tf2::toMsg(q_final);

                    geometry_msgs::msg::PoseStamped marker_in_map = tf_buffer_->transform(
                        marker_in_camera, "map", tf2::durationFromSec(0.3));

                    // publish
                    aruco_interfaces::msg::MarkerPoseId aruco_target;
                    aruco_target.header = marker_in_map.header;
                    aruco_target.pose = marker_in_map.pose;
                    aruco_target.relativepose.x = -tvecs[i][2];
                    aruco_target.relativepose.x = tvecs[i][0];
                    aruco_target.relativepose.x = -tvecs[i][1];
                    aruco_target.id = ids[i];
                    _aruco_marker_pub->publish(aruco_target);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                }
            }
        }
        else
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Camera intrinsics are not initialized yet.");
        }

        annotate_image(cv_ptr);

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = cv_ptr->image;
        _image_pub->publish(*out_msg.toImageMsg().get());
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MultiTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!_camera_matrix.empty() && !_dist_coeffs.empty())
    {
        return;
    }

    // Always update the camera matrix and distortion coefficients from the new message
    _camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();           // Use clone to ensure a deep copy
    _dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double *>(msg->d.data())).clone(); // Use clone to ensure a deep copy

    // Log the first row of the camera matrix to verify correct values
    RCLCPP_INFO(this->get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
                _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
                _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
                _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
    RCLCPP_INFO(this->get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
                _camera_matrix.at<double>(0, 0), // fx
                _camera_matrix.at<double>(1, 1), // fy
                _camera_matrix.at<double>(0, 2), // cx
                _camera_matrix.at<double>(1, 2)  // cy
    );

    // Check if focal length is zero after update
    if (_camera_matrix.at<double>(0, 0) == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Focal length is zero after update!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Updated camera intrinsics from camera_info topic.");
    }
}

// cv_bridge 사용
void MultiTrackerNode::annotate_image(cv_bridge::CvImagePtr image)
{
    // Annotate the image with the target position and marker size
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "X: " << _target[0] << " Y: " << _target[1] << " Z: " << _target[2];
    std::string text_xyz = stream.str();

    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1;
    int thickness = 2;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
    baseline += thickness;
    cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
    cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTrackerNode>());
    rclcpp::shutdown();
    return 0;
}