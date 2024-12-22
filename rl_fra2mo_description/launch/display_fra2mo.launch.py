import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, OrSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_slam_view =  LaunchConfiguration('use_slam_view', default='false')
    use_explore = LaunchConfiguration('use_explore', default='false')
    use_standard = LaunchConfiguration('use_standard', default='false')
   

    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"
    
    rviz_config_file_slam = '/home/user/ros2_ws/src/rl_fra2mo_description/rviz_conf/slam_view.rviz'
    rviz_config_file_explore = '/home/user/ros2_ws/src/rl_fra2mo_description/rviz_conf/explore.rviz'
    rviz_config_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'conf', 'fra2mo_conf.rviz')
    
    xacro = os.path.join(get_package_share_directory('rl_fra2mo_description'), "urdf", xacro_file_name)
    
    # Configurazione per l'uso del tempo di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]),value_type=str)}
    
    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": True}
            ]
    )
    
    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    
    # Nodo RViz2 con config slam
    rviz_node_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_slam],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition = IfCondition(use_slam_view),
    )
    # Nodo RViz2 con config explore
    rviz_node_explore = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_explore],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition = IfCondition(use_explore),
    )
    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(OrSubstitution(use_slam_view,use_explore)),
    )



    # nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]
    nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node, rviz_node_slam, rviz_node_explore]

    return LaunchDescription(nodes_to_start)
