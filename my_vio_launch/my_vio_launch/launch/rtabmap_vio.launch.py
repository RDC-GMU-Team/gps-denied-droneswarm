from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_base_topic',
            default_value='/openmv',
            description='Base topic for camera messages'
        ),
        
        # Your OpenMV node (publishes images and camera info)
        Node(
            package='openmv_listener',
            executable='openmv_listener_node',
            name='openmv_listener_node',
            output='screen'
        ),
        
    
])
