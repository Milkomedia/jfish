import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnShutdown

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def validate_mode(context, *args, **kwargs):
    # Retrieve the mode value from the launch configuration
    mode_val = LaunchConfiguration('mode').perform(context)
    # Check if the mode is either 'sim' or 'real'
    if mode_val not in ['sim', 'real']:
        # Raise an error to abort the launch
        raise RuntimeError("Invalid mode provided: {}. Please use 'sim' or 'real'.".format(mode_val))
    # If valid, simply return an empty list (no additional actions needed)
    return []

def generate_launch_description():
    # Declare a launch argument 'mode' with default value 'sim'
    mode_arg = DeclareLaunchArgument(
        'mode',
        description='Launch mode: sim or real'
    )

    opti_node_config = os.path.join(
        get_package_share_directory('mocap'),
        'config',
        'cfg.yaml'
    )

    mode = LaunchConfiguration('mode')

    # microstrain launch
    microstrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('microstrain_inertial_driver'),
                'launch',
                'microstrain_launch.py'
            )
        ),
        launch_arguments={'mode': mode}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
    )

    nodes = [
        # MuJoCo Node (Run only when mode==sim)
        Node(
            package='mujoco_sim',
            executable='mujoco_node',
            name='mujoco_node',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
        ),

        # Watchdog Node
        Node(
            package='watchdog_manager',
            executable='watchdog_worker',
            name='watchdog_node',
        ),

        Node(
            package='mocap',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
            parameters=[opti_node_config],
        ),

        # Optitrack Node
        Node(
            package='mocap',
            executable='mocap_worker',
            name='optitrack_node',
            parameters=[{'mode': mode}],
        ),

        # IMU Node
        Node(
            package='imu_worker',
            executable='imu_worker',
            name='imu_node',
            parameters=[{'mode': mode}],
        ),

        # SBUS Worker Node
        Node(
            package='sbus',
            executable='sbus_worker',
            name='sbus_node',   
        ),

        # ARM Changer Node
        Node(
            package='arm_changer',
            executable='arm_changer',
            name='arm_changing_node',
        ),

        # Controller Node
        Node(
            package='controller_geom',
            executable='controller_worker_geom',
            name='controller_node',
        ),

        # Allocator Node
        Node(
            package='allocator',
            executable='allocator_worker',
            name='allocator_node',
        ),

        # Dynamixel Node
        Node(
            package='dynamixel_worker',
            executable='dynamixel_worker',
            name='dynamixel_node',
            parameters=[{'mode': mode}],
        ),

        # Teensy Node
        Node(
            package='teensy_sender',
            executable='teensy_worker',
            name='teensy_node',
            parameters=[{'mode': mode}],
        ),

        # GUI Node
        Node(
            package='gui',
            executable='gui',
            name='gui_node',
        ),
    ]

    # --- Event handler: print info log when last node starts ---
    on_start_info = RegisterEventHandler(
        OnProcessStart(
            target_action=nodes[-1],
            on_start=[
                LogInfo(msg="\n\n\n\n\n >>   'Let's roll.'  <<\n")
            ],
        )
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg="\n\n Closing all nodes..\n\n\n\n")
            ]
        )
    )

    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=validate_mode),
        microstrain_launch,
        *nodes,
        on_start_info,
        shutdown_handler,
    ])