from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to your custom OpenVINS config
    config_path = os.path.join(
        get_package_share_directory('my_vio_launch'),  # Change this
        'config',
        'estimator_config.yaml'
    )

    # Your OpenMV listener node
    ifwater_camera_node = Node(
        package='ifwater_camera',
        executable='camera_publisher',
        name='ifwater_camera_publisher',
        output='screen',
        parameters=[]
    )
    
    # OpenVINS node with remapping
    ov_msckf_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        namespace='ov_msckf',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'verbosity': 'INFO',
            'use_stereo': False,
            'max_cameras': 1,
            'save_total_state': False,
        }],
        remappings=[
            # Remap OpenVINS expected topics to your OpenMV topics
            ('/cam0/image_raw', '/camera/image_raw'),
            ('/imu0', '/imu/data'),  # Change to your actual IMU topic
        ]
    )
    
    # Optional: RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('ov_msckf'),
                'launch',
                'display_ros2.rviz'
            )
        ],
        output='screen'
    )
    
    return LaunchDescription([
        ifwater_camera_node,
        ov_msckf_node,
        #rviz_node,  # Comment this out if you don't want RViz
    ])