from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    isBlue_value = LaunchConfiguration('isBlue')
    use_sim_time_value = LaunchConfiguration('use_sim_time')

    isBlue_launch_arg = DeclareLaunchArgument(
        'isBlue',
        default_value='False'
    )

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True'
    )
    return LaunchDescription([
        isBlue_launch_arg,
        use_sim_time_launch_arg,
        Node(
            package='krabi_caisse_detector',
            namespace='krabi_ns',
            executable='caisse_detector_node',
            name='krabi_caisse_detector',
            parameters=[{
                "use_sim_time": use_sim_time_value,
                "is_blue": isBlue_value,
                "debug_image": True,
                # Tune these once the camera is in its final position
                "roi_leftmost_x":      150,
                "roi_leftmost_y":      70,
                "roi_leftmost_width":  50,
                "roi_leftmost_height": 50,
                "roi_leftcenter_x":      320,
                "roi_leftcenter_y":      70,
                "roi_leftcenter_width":  50,
                "roi_leftcenter_height": 50,
                "roi_rightcenter_x":      490,
                "roi_rightcenter_y":      70,
                "roi_rightcenter_width":  50,
                "roi_rightcenter_height": 50,
                "roi_rightmost_x":      650,
                "roi_rightmost_y":      70,
                "roi_rightmost_width":  50,
                "roi_rightmost_height": 50,
            }]
        )
    ])
