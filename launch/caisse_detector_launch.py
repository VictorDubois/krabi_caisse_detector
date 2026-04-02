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
        #launch_ros.actions.SetParameter(name='isBlue', value=isBlue_value),
        Node(
            #prefix=['gdbserver localhost:3000'],
            package='krabi_caisse_detector',
            namespace='krabi_ns',
            executable='caisse_detector_node',
            name='krabi_caisse_detector',
            parameters=[{"use_sim_time": use_sim_time_value, "is_blue": isBlue_value, "debug_image": True,
            "roi_x": 100,
            "roi_y": 0,
            "roi_width": 600,
            "roi_height": 350}]
        )
    ])
