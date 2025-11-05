
'''

Describer:  This script lauchs (spawn) the sphere in gazebo using a sdf file. 
            Just much easier using a SDF here.
            the sdf could be found in /src/my_sphere_pkg/models/sdf/sphere_goal

            The node_mark (coordinate_node.py)reads the position on the sphere and publishes on a topic /marker_position

            --> I will invoke this launch file later in my environment launch file <--

            Note: I remove rviz here in order to using an other rviz config later on my own environment 
                  Also, gazebo and the empty world are launched in my own environment later 

'''

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():


    pkg_dir = get_package_share_directory('my_sphere_pkg') 

    


    # SDF
    sdf_file_name = 'sdf/sphere_goal/model.sdf'
    sdf = os.path.join(pkg_dir, 'models', sdf_file_name)
    
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py', 
                        arguments=['-entity', 'my_sphere', '-file', sdf, '-x','0', '-y','-0.8', '-z','1'], 
                        output='screen')


    node_mark = Node(package ='my_sphere_pkg', executable ='reader_mark_node', output ='screen')
    
    return LaunchDescription([spawn_entity, node_mark])