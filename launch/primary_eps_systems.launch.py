from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   battery_config= os.path.join(get_package_share_directory('demo_eps'),
       'config',
       'battery_config.yaml'
    )
   bcdu_config = os.path.join(get_package_share_directory('demo_eps'),
       'config',
       'bcdu_config.yaml'
    )
   battery_manager = Node(
        package='demo_eps',
        executable='battery_manager_node',
        name='battery_manager',
        output='screen',
        parameters=[battery_config],
        emulate_tty=True
    )
   bcdu=Node(
       package='demo_eps',
       executable='bcdu_node',
       name='bcdu_node',
       output='screen',
       parameters=[bcdu_config],
       emulate_tty=True
   )
   
   ddcu= Node(
       package='demo_eps',
       executable='ddcu_device',
       name="ddcu_node",
       output='screen',
       emulate_tty=True
   )
    
   return LaunchDescription([
        battery_manager,
        bcdu,
        ddcu
    ])
