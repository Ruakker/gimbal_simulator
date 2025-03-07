from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
            
def generate_launch_description():
    t1 = Node(package = "simulator", executable = "chassis")
    t2 = Node(package = "simulator", executable = "armor")
    t3 = Node(package = "simulator", executable = "collisionChecker")
    rviz2 = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", get_package_share_directory("simulator") + "/config/simulator_Rviz2.rviz"]
    )
    return LaunchDescription([t1, t2, t3, rviz2])