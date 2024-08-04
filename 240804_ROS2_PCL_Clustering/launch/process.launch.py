from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pcl_segmentation_node = Node(
        package = 'point_cloud_processing',
        executable = 'pcl_segmentation',
        output = 'screen',
    )
    
    return LaunchDescription([
        pcl_segmentation_node,
    ])