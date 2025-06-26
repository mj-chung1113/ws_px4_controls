from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    bridge_cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        # Clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # TF pose
        '/model/X1_asp/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/X1_asp/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/x500_gimbal_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/x500_gimbal_0/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        # # Odometry
        # '/model/X1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        # # Velocity command
        # '/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
        # # GPS sensor
        # '/world/default/model/X1_asp/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
    ]

    x500_camera_cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
    ]

    return LaunchDescription([
        ExecuteProcess(cmd=bridge_cmd, output='screen'),
        # ExecuteProcess(cmd=x1_camera_cmd, output='screen'),
        # ExecuteProcess(cmd=x1_lidar_cmd, output='screen'),
        ExecuteProcess(cmd=x500_camera_cmd, output='screen')
    ])
