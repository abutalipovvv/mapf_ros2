#!/usr/bin/env python3

import os
import yaml
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node, SetRemap


def _ns(namespace: str) -> str:
    namespace = namespace.strip('/')
    return f'/{namespace}' if namespace else ''


def _is_clock_topic(topic_name: str) -> bool:
    if not topic_name:
        return False
    return topic_name.strip('/') == 'clock'


def load_sdf_with_namespace(model_path: str, namespace: str) -> str:
    with open(model_path, 'r', encoding='utf-8') as f:
        sdf_text = f.read()

    ns = _ns(namespace)

    topic_map = {
        '<tf_topic>/tf</tf_topic>': f'<tf_topic>{ns}/tf</tf_topic>',
        '<topic>cmd_vel</topic>': f'<topic>{ns}/cmd_vel</topic>',
        '<odom_topic>odom</odom_topic>': f'<odom_topic>{ns}/odom</odom_topic>',
        '<topic>joint_states</topic>': f'<topic>{ns}/joint_states</topic>',
        '<topic>imu</topic>': f'<topic>{ns}/imu</topic>',
        '<topic>scan</topic>': f'<topic>{ns}/scan</topic>',
        #'<topic>camera/image_raw</topic>': f'<topic>{ns}/camera/image_raw</topic>',
        '<camera_info_topic>camera/camera_info</camera_info_topic>':
            f'<camera_info_topic>{ns}/camera/camera_info</camera_info_topic>',
    }

    for original, replacement in topic_map.items():
        sdf_text = sdf_text.replace(original, replacement)

    return sdf_text


def create_namespaced_bridge_yaml(base_yaml_path: str, namespace: str) -> str:
    with open(base_yaml_path, 'r', encoding='utf-8') as f:
        bridges = yaml.safe_load(f)

    ns = _ns(namespace)
    namespaced_bridges = []

    for bridge in bridges:
        new_bridge = dict(bridge)

        ros_topic = new_bridge.get('ros_topic_name', '')
        gz_topic = new_bridge.get('gz_topic_name', '')

        # Полностью выкидываем clock из per-robot bridge
        if _is_clock_topic(ros_topic) or _is_clock_topic(gz_topic):
            continue

        if ros_topic:
            ros_topic = ros_topic.lstrip('/')
            new_bridge['ros_topic_name'] = f'{ns}/{ros_topic}'

        if gz_topic:
            gz_topic = gz_topic.lstrip('/')
            new_bridge['gz_topic_name'] = f'{ns}/{gz_topic}'

        if new_bridge.get('ros_topic_name', '').endswith('/cmd_vel'):
            if new_bridge.get('ros_type_name') == 'geometry_msgs/msg/TwistStamped':
                new_bridge['ros_type_name'] = 'geometry_msgs/msg/Twist'

        namespaced_bridges.append(new_bridge)

    tmp_dir = Path(tempfile.gettempdir()) / 'tb3_multi_bridge'
    tmp_dir.mkdir(parents=True, exist_ok=True)

    output_path = tmp_dir / f'{namespace.strip("/")}_bridge.yaml'
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(namespaced_bridges, f, sort_keys=False)

    return str(output_path)


def create_namespaced_nav2_params_yaml(base_yaml_path: str, namespace: str) -> str:
    with open(base_yaml_path, 'r', encoding='utf-8') as f:
        params_text = f.read()

    ns = _ns(namespace)
    params_text = params_text.replace('<robot_namespace>', ns)

    tmp_dir = Path(tempfile.gettempdir()) / 'tb3_multi_nav2'
    tmp_dir.mkdir(parents=True, exist_ok=True)

    output_path = tmp_dir / f'{namespace.strip("/")}_nav2_params.yaml'
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(params_text)

    return str(output_path)


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')

    robots_file_path = os.path.join(pkg_path, 'config', 'robots.yaml')

    with open(robots_file_path, 'r', encoding='utf-8') as file:
        yaml_data = yaml.safe_load(file)

    robots = yaml_data['robots']

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Использовать симуляционное время'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='enable_rviz',
        default_value='true',
        description='Enable rviz launch'
    ))

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = f'turtlebot3_{tb3_model}'

    urdf_file_name = f'turtlebot3_{tb3_model}.urdf'
    urdf_path = os.path.join(tb3_pkg, 'urdf', urdf_file_name)
    sdf_path = os.path.join(tb3_pkg, 'models', model_folder, 'model.sdf')
    bridge_template = os.path.join(tb3_pkg, 'params', f'{model_folder}_bridge.yaml')

    with open(urdf_path, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_pkg, 'models')
    ))

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    ld.add_action(clock_bridge)

    last_action = None

    for robot in robots:
        namespace = robot['name']
        gz_entity_name = f'{namespace}_{tb3_model}'

        remappings = [
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/odom", "odom"),
            ("/scan", "scan"),
        ]

        patched_sdf = load_sdf_with_namespace(sdf_path, namespace)
        namespaced_bridge_yaml = create_namespaced_bridge_yaml(bridge_template, namespace)

        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
            }],
            remappings=remappings
        )

        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-name', gz_entity_name,
                '-string', patched_sdf,
                '-x', str(robot['x_pose']),
                '-y', str(robot['y_pose']),
                '-z', str(robot.get('z_pose', 0.01)),
            ],
            output='screen'
        )

        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{namespace}_ros_gz_bridge',
            output='screen',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={namespaced_bridge_yaml}',
            ]
        )

        image_bridge_cmd = None
        if tb3_model != 'burger':
            image_bridge_cmd = Node(
                package='ros_gz_image',
                executable='image_bridge',
                namespace=namespace,
                arguments=[f'/{namespace}/camera/image_raw'],
                output='screen',
            )

        nav2_launch_file = os.path.join(pkg_path, 'launch', 'nav2', 'bringup_launch.py')
        map_yaml_file = os.path.join(pkg_path, 'maps', 'cafe_world_map.yaml')
        params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
        namespaced_nav2_params = create_namespaced_nav2_params_yaml(params_file, namespace)

        message = (
            f"{{header: {{frame_id: map}}, pose: {{pose: {{position: "
            f"{{x: {robot['x_pose']}, y: {robot['y_pose']}, z: 0.1}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, }} }}"
        )

        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable',
                f'/{namespace}/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped',
                message
            ],
            output='screen'
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_yaml_file,
                'use_namespace': 'True',
                'namespace': namespace,
                'params_file': namespaced_nav2_params,
                'autostart': 'true',
                'use_sim_time': 'true',
                'log_level': 'warn',
                'map_server': 'True'
            }.items()
        )

        nav2_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            bringup_cmd,
            initial_pose_cmd,
        ])

        rviz_launch_file = os.path.join(pkg_path, 'launch', 'rviz_launch.py')
        rviz_config_file = os.path.join(pkg_path, 'rviz', 'nav2_default_view.rviz')

        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'true',
                'rviz_config': rviz_config_file,
            }.items(),
            condition=IfCondition(enable_rviz)
        )

        fake_bms = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub',
                f'/{namespace}/battery_state',
                'sensor_msgs/msg/BatteryState',
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 12.0, percentage: 0.8, capacity: 5.0}",
                '-r', '1'
            ],
            output='log'
        )

        robot_group_actions = [
            node_robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            fake_bms,
            nav2_actions,
            rviz,
        ]

        if image_bridge_cmd is not None:
            robot_group_actions.insert(3, image_bridge_cmd)

        robot_group = GroupAction(robot_group_actions)

        if last_action is None:
            ld.add_action(robot_group)
        else:
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[robot_group]
                )
            )
            ld.add_action(spawn_robot_event)

        last_action = spawn_entity

    return ld
