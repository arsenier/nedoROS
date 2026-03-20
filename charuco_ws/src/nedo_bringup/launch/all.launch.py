from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    charuco_launch = os.path.join(
        get_package_share_directory('charuco_detector'),
        'launch',
        'hand_eye_calibration.launch.py'
    )

    usb_cam_params = os.path.join(
        os.path.expanduser('~'),
        'nedoROS/charuco_ws/src/usb_cam/params.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[usb_cam_params],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(charuco_launch),
        ),

        Node(
            package='apriltag_pose_detector',
            executable='apriltag_pose_detector_node',
            name='apriltag_pose_detector',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'camera_info_topic': '/camera_info',
                'marker_size_m': 0.120,
                'marker_id': 239,
                'dictionary_id': 2, 
                'publish_debug_image': True,
            }]
        ),

        Node(
            package='charuco_rectifier',
            executable='charuco_rectifier_node',
            name='charuco_rectifier',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'camera_info_topic': '/camera_info',
                'board_pose_topic': '/image_raw_charuco_pose',
            }]
        ),

        Node(
            package='pose_recalibrator',
            executable='pose_recalibrator',
            name='pose_recalibrator',
            output='screen',
        )
    ])