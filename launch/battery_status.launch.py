from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   battery_config= os.path.join(get_package_share_directory('demo_eps'),
       'config',
       'battery_config.yaml'
    )
   battery_manager = Node(
        package='demo_eps',
        executable='battery_manager_node',
        name='battery_manager',
        output='screen',
        parameters=[battery_config],
        emulate_tty=True
    )
   
   return LaunchDescription([
        battery_manager,

    ])
