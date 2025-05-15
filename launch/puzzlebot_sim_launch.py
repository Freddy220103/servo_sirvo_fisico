import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    urdf_file_name = 'puzzlebot.urdf'

    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file_name
    )

    with open(urdf, 'r') as infp:
        robot_description = infp.read()


    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )


    use_sim_time_param = {'use_sim_time': True}

    my_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[use_sim_time_param]
    )

    df_1 = Node(
        package='ServoYSirvo_nav2_puzzlebot',
        executable='dynamic_tf',
        output='screen',
        parameters=[use_sim_time_param]
    )

    localisation = Node(
        package='ServoYSirvo_nav2_puzzlebot',
        executable='localisation',
        parameters=[use_sim_time_param]
    )

    joint_pub = Node(
        package='ServoYSirvo_nav2_puzzlebot',
        executable='joint_state_pub',
        parameters=[use_sim_time_param]
    )

    kinematic = Node(
        package='ServoYSirvo_nav2_puzzlebot',
        executable='puzzlebot_kinematic_model',
        parameters=[use_sim_time_param]
    )

    my_rqt_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        output='screen',
        parameters=[use_sim_time_param]
    )

    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[use_sim_time_param]
    )

    pruebita = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
        parameters=[use_sim_time_param]
    )


    return LaunchDescription([
        # pruebita, 
        static_tf_1,
        # df_1,
        my_rviz_node,
        my_rqt_node,
        localisation,
        kinematic,
        joint_pub,
        robot_state_pub_node
    ])
