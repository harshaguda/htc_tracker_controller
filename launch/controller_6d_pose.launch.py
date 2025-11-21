from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    controller_name_arg = DeclareLaunchArgument(
        'controller_name',
        default_value='controller_1',
        description='Name of the HTC Vive controller'
    )
    
    tracker_name_arg = DeclareLaunchArgument(
        'tracker_name',
        default_value='tracker_1',
        description='Name of the HTC Vive tracker'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate in Hz'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output'
    )
    
    # Create node
    controller_6d_pose_node = Node(
        package='htc_tracker_controller',
        executable='controller_6d_pose_publisher',
        name='controller_6d_pose_publisher',
        output='screen',
        parameters=[{
            'controller_name': LaunchConfiguration('controller_name'),
            'tracker_name': LaunchConfiguration('tracker_name'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'verbose': LaunchConfiguration('verbose')
        }]
    )
    
    return LaunchDescription([
        controller_name_arg,
        tracker_name_arg,
        publish_rate_arg,
        verbose_arg,
        controller_6d_pose_node
    ])
