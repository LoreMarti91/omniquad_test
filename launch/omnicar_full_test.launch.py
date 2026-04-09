from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from datetime import datetime

def generate_launch_description():

    # --- CONTROLLORI ---
    # load_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', 'state_broadcaster', '--set-state', 'active'],
    #     output='screen'
    # )
    # load_state_broadcaster_dist = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', 'distributor_state_broadcaster', '--set-state', 'active'],
    #     output='screen'
    # )

    load_omni_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'omni_controller', '--set-state', 'active'],
        output='screen'
    )

    # --- HOMING SERVICE ---
    homing_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/omni_controller/homing_srv', 'std_srvs/srv/SetBool', '{data: true}'],
        output='screen'
    )

    # --- NODI SENSORI ---

    # # LiDAR S2E incluso nel launch
    # s2e_lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('sllidar_ros2'),
    #             'launch',
    #             'sllidar_s2e_launch.py'
    #         ])
    #     )
    # )

    # # Realsense ridotta solo ai topic compressi
    # realsense_node = Node(
    #     package="realsense2_camera",
    #     executable="realsense2_camera_node",
    #     name="realsense_node",
    #     output="screen",
    #     parameters=[{
    #         "enable_color": True,
    #         "enable_depth": True,
    #         "enable_infra1": False,
    #         "enable_infra2": False,
    #         "enable_gyro": False,
    #         "enable_accel": False,
    #         "color_fps": 15,
    #         "depth_fps": 15,
    #         "color_width": 640,
    #         "color_height": 480,
    #         "depth_width": 640,
    #         "depth_height": 480
    #     }]
    # )

    # --- NODO Omniquad test ---
    omniquad_test_node = Node(
        package='omniquad_test',
        executable='omniquad_test_node',
        name='omniquad_test_node',
        output='screen',
        parameters=[{'test_name': 'softleg_jump_' + datetime.now().strftime("%m_%d_%Y_%H_%M_%S")}]
    )

    # --- ROS2 BAG (salva solo topic essenziali + state broadcaster) ---
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    bag_dir = os.path.expanduser("~/mulinex_ws/bag")
    os.makedirs(bag_dir, exist_ok=True)

    mcap_path = os.path.join(bag_dir, f"Test_Omniquad_{timestamp}")
     
    
    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/omni_controller/command',
            '/omni_controller/joints_reference',
            # '/scan',
            # '/camera/color/image_raw/compressed',
            # '/camera/depth/image_rect_raw/compressedDepth',
            '/state_broadcaster/joints_state',
            '/state_broadcaster/performance_indexes',
            '/distributor_state_broadcaster/distributors_state',
            '/distributor_state_broadcaster/transition_event',
            "mcap",'-o', mcap_path
        ],
        output='screen'
    )
    

    # --- SEQUENZA CONTROLLORI ---
    # first_controller_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=load_state_broadcaster,
    #         on_exit=[load_state_broadcaster_dist]
    #     )
    # )

    # second_controller_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=load_state_broadcaster_dist,
    #         on_exit=[load_omni_controller]
    #     )
    # )

    # third_controller_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=load_omni_controller,
    #         on_exit=[homing_service, s2e_lidar_launch, realsense_node]
    #     )
    # )

    # # Dopo sensori e homing -> avvia il nodo di test e rosbag
    # third_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=homing_service,
    #         on_exit=[
    #             TimerAction(
    #                 period=2.0,
    #                 actions=[record_process, omniquad_test_node]
    #             )
    #         ]
    #     )
    # )

    # return LaunchDescription([
    #     load_state_broadcaster,
    #     first_controller_handler,
    #     second_controller_handler,
    #     third_controller_handler,
    #     third_handler
    # ])



    first_controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=load_omni_controller,
            on_exit=[homing_service,]
        )
    )

    second_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=homing_service,
            on_exit=[
                TimerAction(
                    period=2.0,
                    
                    actions=[record_process, omniquad_test_node]
                )
            ]
        )
    )
    return LaunchDescription([
        load_omni_controller,
        first_controller_handler, 
        second_handler
    ])