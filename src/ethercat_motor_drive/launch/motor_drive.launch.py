#############################################如果要使用位置控制器，使用这个launch文件：#############################################

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, TimerAction
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():

#     # Declare arguments
#     declared_arguments = [
#         DeclareLaunchArgument(
#             'description_file',
#             default_value='motor_drive.config.xacro',
#             description='URDF/XACRO description file with the axis.',
#         )
#     ]

#     description_file = LaunchConfiguration('description_file')

#     # Get URDF via xacro
#     robot_description_content = Command([
#         PathJoinSubstitution([FindExecutable(name="xacro")]),
#         " ",
#         PathJoinSubstitution([
#             FindPackageShare("ethercat_motor_drive"),
#             "description/config",
#             description_file,
#         ]),
#     ])
#     robot_description = {"robot_description": robot_description_content}

#     robot_controllers = PathJoinSubstitution([
#         FindPackageShare("ethercat_motor_drive"),
#         "config",
#         "controllers.yaml",
#     ])

#     control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_description, robot_controllers],
#         output="both",
#     )

#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )

#     joint_state_broadcaster_spawner = TimerAction(
#         period=0.0,
#         actions=[
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
#             )
#         ]
#     )

#     joint_trajectory_controller_spawner = TimerAction(
#         period=10.0,
#         actions=[
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
#             )
#         ]
#     )

#     return LaunchDescription(
#         declared_arguments + [
#             control_node,
#             robot_state_pub_node,
#             joint_state_broadcaster_spawner,
#             joint_trajectory_controller_spawner,
#         ]
#     )

#########################################################如果要使用力矩控制器，使用这个launch文件：###########################################################
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, TimerAction
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():

#     declared_arguments = [
#         DeclareLaunchArgument(
#             'description_file',
#             default_value='motor_drive.config.xacro',
#             description='URDF/XACRO description file with the axis.',
#         )
#     ]

#     description_file = LaunchConfiguration('description_file')

#     robot_description_content = Command([
#         PathJoinSubstitution([FindExecutable(name="xacro")]),
#         " ",
#         PathJoinSubstitution([
#             FindPackageShare("ethercat_motor_drive"),
#             "description/config",
#             description_file,
#         ]),
#     ])
#     robot_description = {"robot_description": robot_description_content}

#     robot_controllers = PathJoinSubstitution([
#         FindPackageShare("ethercat_motor_drive"),
#         "config",
#         "controllers.yaml",
#     ])

#     control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_description, robot_controllers],
#         output="both",
#     )

#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )

#     # 先启动 joint_state_broadcaster
#     joint_state_broadcaster_spawner = TimerAction(
#         period=0.0,
#         actions=[
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
#             )
#         ]
#     )

#     # 再启动力矩控制器
#     joint_effort_controller_spawner = TimerAction(
#         period=20.0,  # 适当等待硬件接口就绪
#         actions=[
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["joint_effort_controller", "-c", "/controller_manager"],
#             )
#         ]
#     )

#     return LaunchDescription(
#         declared_arguments + [
#             control_node,
#             robot_state_pub_node,
#             joint_state_broadcaster_spawner,
#             joint_effort_controller_spawner,
#         ]
#     )



# 启动位置控制器（按你提供的 controllers.yaml 配置）
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1) 可选：URDF/Xacro 文件名参数
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='motor_drive.config.xacro',
            description='URDF/XACRO description file with the axis.',
        )
    ]

    description_file = LaunchConfiguration('description_file')

    # 2) 通过 xacro 生成 robot_description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ethercat_motor_drive"),
            "description/config",
            description_file,
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}

    # 3) controllers.yaml 路径
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ethercat_motor_drive"),
        "config",
        "controllers.yaml",
    ])

    # 4) ros2_control_node + robot_state_publisher
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 5) 依次加载控制器（先 JSB，再 left_arm，再 left_hand）
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,  # 给 RSP/CM 一点时间
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                output="screen",
            )
        ]
    )

    left_arm_controller_spawner = TimerAction(
        period=20.0,  # 在 JSB 之后
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_arm_controller", "-c", "/controller_manager"],
                output="screen",
            )
        ]
    )


    right_arm_controller_spawner = TimerAction(
        period=20.0,  # 在 JSB 之后
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_controller", "-c", "/controller_manager"],
                output="screen",
            )
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
        ]
    )
