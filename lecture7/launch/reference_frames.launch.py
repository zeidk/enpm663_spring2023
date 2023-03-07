from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    # SensorCamera
    sensor_camera = Node(
        package='enpm663_ref_frames',
        executable='sensor_camera',
        output='screen',
    )
    
    ld.add_action(sensor_camera)
    
    return ld