from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    fcu_bridge_001_node = Node(
        package='fcu_core',
        executable='fcu_bridge_001',
        name='fcu_bridge_001',
        output='screen',
        remappings=[
            ('~/odometry_001', '/vins_estimator/odometry'),
            ('~/imu_global_001', 'imu_global_001'),
            ('~/odom_global_001', 'odom_global_001'),
            ('~/path_global_001', 'path_global_001'),
            ('~/path_target_001', 'path_target_001'),
            ('~/goal_001', '/move_base_simple/goal'),
            ('~/motion_001', 'motion_001'),
            ('~/command', '/fcu_command/command'),
            ('~/mission_001', '/fcu_mission/mission_001'),
        ],
        parameters=[
            {
                'DRONE_IP': '192.168.0.201',
                'USB_PORT': '/dev/ttyACM0',
                'BANDRATE': 460800,
                'channel': 1,
                'offboard': False,
                'use_uwb': True,
                'set_goal': False,
                'simple_target': True,
            }
        ]
    )

    fcu_bridge_002_node = Node(
        package='fcu_core',
        executable='fcu_bridge_002',
        name='fcu_bridge_002',
        output='screen',
        remappings=[
            ('~/odometry_002', '/vins_estimator/odometry'),
            ('~/imu_global_002', 'imu_global_002'),
            ('~/odom_global_002', 'odom_global_002'),
            ('~/path_global_002', 'path_global_002'),
            ('~/path_target_002', 'path_target_002'),
            ('~/goal_002', '/move_base_simple/goal'),
            ('~/motion_002', 'motion_002'),
            ('~/command', '/fcu_command/command'),
            ('~/mission_002', '/fcu_mission/mission_002'),
        ],
        parameters=[
            {
                'DRONE_IP': '192.168.0.202',
                'USB_PORT': '/dev/ttyACM0',
                'BANDRATE': 460800,
                'channel': 1,
                'offboard': False,
                'use_uwb': True,
                'set_goal': False,
                'simple_target': True,
            }
        ]
    )

    fcu_bridge_003_node = Node(
        package='fcu_core',
        executable='fcu_bridge_003',
        name='fcu_bridge_003',
        output='screen',
        remappings=[
            ('~/odometry_003', '/vins_estimator/odometry'),
            ('~/imu_global_003', 'imu_global_003'),
            ('~/odom_global_003', 'odom_global_003'),
            ('~/path_global_003', 'path_global_003'),
            ('~/path_target_003', 'path_target_003'),
            ('~/goal_003', '/move_base_simple/goal'),
            ('~/motion_003', 'motion_003'),
            ('~/command', '/fcu_command/command'),
            ('~/mission_003', '/fcu_mission/mission_003'),
        ],
        parameters=[
            {
                'DRONE_IP': '192.168.0.203',
                'USB_PORT': '/dev/ttyACM0',
                'BANDRATE': 460800,
                'channel': 1,
                'offboard': False,
                'use_uwb': True,
                'set_goal': False,
                'simple_target': True,
            }
        ]
    )

    fcu_bridge_004_node = Node(
        package='fcu_core',
        executable='fcu_bridge_004',
        name='fcu_bridge_004',
        output='screen',
        remappings=[
            ('~/odometry_004', '/vins_estimator/odometry'),
            ('~/imu_global_004', 'imu_global_004'),
            ('~/odom_global_004', 'odom_global_004'),
            ('~/path_global_004', 'path_global_004'),
            ('~/path_target_004', 'path_target_004'),
            ('~/goal_004', '/move_base_simple/goal'),
            ('~/motion_004', 'motion_004'),
            ('~/command', '/fcu_command/command'),
            ('~/mission_004', '/fcu_mission/mission_004'),
        ],
        parameters=[
            {
                'DRONE_IP': '192.168.0.204',
                'USB_PORT': '/dev/ttyACM0',
                'BANDRATE': 460800,
                'channel': 1,
                'offboard': False,
                'use_uwb': True,
                'set_goal': False,
                'simple_target': True,
            }
        ]
    )

    fcu_bridge_005_node = Node(
        package='fcu_core',
        executable='fcu_bridge_005',
        name='fcu_bridge_005',
        output='screen',
        remappings=[
            ('~/odometry_005', '/vins_estimator/odometry'),
            ('~/imu_global_005', 'imu_global_005'),
            ('~/odom_global_005', 'odom_global_005'),
            ('~/path_global_005', 'path_global_005'),
            ('~/path_target_005', 'path_target_005'),
            ('~/goal_005', '/move_base_simple/goal'),
            ('~/motion_005', 'motion_005'),
            ('~/command', '/fcu_command/command'),
            ('~/mission_005', '/fcu_mission/mission_005'),
        ],
        parameters=[
            {
                'DRONE_IP': '192.168.0.205',
                'USB_PORT': '/dev/ttyACM0',
                'BANDRATE': 460800,
                'channel': 1,
                'offboard': False,
                'use_uwb': True,
                'set_goal': False,
                'simple_target': True,
            }
        ]
    )

    fcu_bridge_006_node = Node(
        package='fcu_core',
        executable='fcu_bridge_006',
        name='fcu_bridge_006',
        output='screen',
        remappings=[
            ('~/odometry_006', '/vins_estimator/odometry'),
            ('~/imu_global_006', 'imu_global_006'),
            ('~/odom_global_006', 'odom_global_006'),
            ('~/path_global_006', 'path_global_006'),
            ('~/path_target_006', 'path_target_006'),
            ('~/goal_006', '/move_base_simple/goal'),
            ('~/motion_006', 'motion_006'),
            ('~/command', '/fcu_command/command'),
            ('~/mission_006', '/fcu_mission/mission_006'),
        ],
        parameters=[
            {
                'DRONE_IP': '192.168.0.206',
                'USB_PORT': '/dev/ttyACM0',
                'BANDRATE': 460800,
                'channel': 1,
                'offboard': False,
                'use_uwb': True,
                'set_goal': False,
                'simple_target': True,
            }
        ]
    )


        
    # fcu_command_node = Node(
    #     package='fcu_core',
    #     executable='fcu_command',
    #     name='fcu_command',
    #     output='screen',
    #     emulate_tty=True
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [os.path.join(
            get_package_share_directory('fcu_core'),  # 替换'fcu_core'为你的包名
            'rviz',  # 配置文件夹名称
            'default.rviz'  # 你的 rviz 配置文件名
            )]
        ],
    )
    
    fcu_mission_node = Node(
        package='fcu_core',
        executable='fcu_mission',
        name='fcu_mission',
        output='screen',
    )
    
    # 创建并返回 launch 描述
    return LaunchDescription([
        fcu_bridge_001_node,
        fcu_bridge_002_node,
        fcu_bridge_003_node,
        fcu_bridge_004_node,
        fcu_bridge_005_node,
        fcu_bridge_006_node,
        rviz_node,
        # fcu_command_node,
        fcu_mission_node
    ])