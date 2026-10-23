# Main entry point for REAL UR5 (CB3 / PolyScope 3.15) arm control.
#
# Stack: ur_robot_driver (ur_control.launch.py, description through
# ur5_real_rsp.launch.py -> ur5_endo_flex_real.xacro) + the ported
# ur5_endo_flex control nodes + RViz (live robot-state visualization).
#
# The endo_flex chain is present in the URDF but unactuated on real
# hardware; its /flex/* command topics are inert (no Gazebo bridge).
# enable_flex:=false silences those topics.
#
# Real-robot flow (see ur5_real_ws/PLAN.md runbook):
#   1. ros2 launch ur5_endo_flex ur5_real.launch.py
#   2. Pendant: load + PLAY external_control.urp (driver first, then Play)
#   3. If the initial home trajectory fired before Play (controller
#      inactive, message dropped), press HOME on the control panel.

import os

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    launch_gui = LaunchConfiguration('launch_gui')
    enable_flex = LaunchConfiguration('enable_flex')
    headless_mode = LaunchConfiguration('headless_mode')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    home_duration = LaunchConfiguration('home_duration')
    launch_rviz = LaunchConfiguration('launch_rviz')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.10',
            description='IP of the UR5 controller (PolyScope).'),
        DeclareLaunchArgument(
            'launch_gui',
            default_value='true',
            description='Start the Tk control panel (true) or a headless '
                        'robot_node without keyboard input (false).'),
        DeclareLaunchArgument(
            'enable_flex',
            default_value='true',
            description='Keep publishing the (inert) endo_flex /flex/* '
                        'topics and synthetic flex joint states. '
                        'false silences them; the flex chain stays in the URDF.'),
        DeclareLaunchArgument(
            'headless_mode',
            default_value='false',
            description='ur_robot_driver headless mode (driver sends its own '
                        'program instead of external_control.urp). '
                        'Use exactly ONE start path: pendant Play OR headless.'),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Run against mock hardware (no robot needed); '
                        'commands mirror to states. For validation only.'),
        DeclareLaunchArgument(
            'home_duration',
            default_value='6.0',
            description='Duration [s] of the initial home-pose trajectory.'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start RViz (live robot-state visualization).'),
    ]

    # ------------------------------------------------------------------
    # ur_robot_driver bringup. Vendored copy of ur_control.launch.py with
    # description_launchfile defaulting to our rsp launch, which defaults
    # to ur5_endo_flex_real.xacro + config/ur5_calibration.yaml. (The
    # vendored copy exists because passing description_launchfile through
    # IncludeLaunchDescription did not take effect in this environment.)
    # ------------------------------------------------------------------
    rsp_launch = PathJoinSubstitution(
        [FindPackageShare('ur5_endo_flex'), 'launch', 'ur5_real_rsp.launch.py'])
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur5_endo_flex'), 'launch', 'ur5_real_control.launch.py'])),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': robot_ip,
            'launch_rviz': 'false',
            'launch_dashboard_client': 'true',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'headless_mode': headless_mode,
            'use_mock_hardware': use_mock_hardware,
        }.items(),
    )

    # ------------------------------------------------------------------
    # Control-node parameters (ported verbatim from ur5_sim_ws
    # ur5_sim.launch.py shared_node_params; use_sim_time removed)
    # ------------------------------------------------------------------
    shared_node_params = {
        'base_link': 'base_link',
        'tip_link': 'endoscope_link',
        'controller_topic':
            '/scaled_joint_trajectory_controller/joint_trajectory',
        'step_size_mm': 5.0,
        'tilt_step_deg': 5.0,
        'traj_duration_s': 1.0,
        'rcm_point_in_tip_frame': [0.0, 0.0, -0.30],
        'endoscope_tip_in_link_frame': [0.0, 0.0, 0.20195],
        'marker_topic': '/endoscope_markers',
        'marker_publish_period': 0.1,
        'markers_enabled': True,
        'rcm_cone_half_angle_deg': 20.0,
        'rcm_cone_length': 0.20,
        'rcm_sphere_diameter': 0.03,
        'endoscope_line_width': 0.006,
        'endoscope_line_overshoot': 0.05,
        # Flex kinematics
        'flex_N': 8,
        'flex_SEG_LEN': 0.008,
        'flex_ARC_MIN': 0.030,
        'flex_ARC_MAX': 0.050,
        'flex_TENDON_R': 0.004,
        # Continuous-control velocity limits
        'arm_linear_vel_m_s': 0.025,          # 25 mm/s
        'arm_axis_vel_m_s':  0.025,           # 25 mm/s along tube
        'arm_tilt_vel_deg_s': 4.0,
        'flex_yaw_vel_rad_s': 1.0,
        'flex_curv_vel_1pm_s': 10.0,
        'flex_arc_vel_m_s': 0.020,
        'control_tick_hz': 20.0,
        'tracking_standoff_m': 0.105,
        # endo_flex kill-switch (topics only; flex chain stays in the URDF)
        'enable_flex': enable_flex,
    }

    # Integrated GUI control panel (default); embeds the node in-process.
    gui_panel = Node(
        package='ur5_endo_flex',
        executable='gui_panel.py',
        name='ur5_endo_flex_control_panel',
        output='screen',
        parameters=[shared_node_params],
        condition=IfCondition(launch_gui),
    )

    # Fallback headless node when launch_gui:=false.
    robot_node_headless = Node(
        package='ur5_endo_flex',
        executable='robot_node.py',
        name='ur5_endo_flex_node_headless',
        output='screen',
        parameters=[dict(shared_node_params, enable_keyboard=False)],
        condition=UnlessCondition(launch_gui),
    )

    # One-shot home pose so /joint_states (and RViz) reflect HOME_DEG.
    # NOTE on real hardware: fires when /joint_states first appears (driver
    # up), which may be BEFORE pendant Play; while the motion controller is
    # inactive the trajectory is dropped. After Play, press HOME on the
    # panel. In mock mode it executes immediately.
    initial_pose = Node(
        package='ur5_endo_flex',
        executable='home_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'home_positions_deg': [0.0, -120.0, -120.0, -30.0, 90.0, 45.0],
            'trajectory_duration_s': home_duration,
            'wait_for_joint_states': True,
        }],
    )

    # Live robot-state visualization (real /joint_states + RSP TF)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('ur5_endo_flex'), 'config', 'view.rviz'])],
        condition=IfCondition(launch_rviz),
        additional_env={'QT_XCB_GL_INTEGRATION': 'glx'},
    )

    # Make script-side imports (e.g. 'from kinematics import ...') resolve:
    # the installed scripts live in <prefix>/lib/ur5_endo_flex.
    scripts_lib_dir = os.path.join(
        get_package_prefix('ur5_endo_flex'), 'lib', 'ur5_endo_flex')
    existing = os.environ.get('PYTHONPATH', '')
    python_path = (existing + ':' + scripts_lib_dir) if existing else scripts_lib_dir

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable(name='PYTHONPATH', value=python_path),
            driver,
            initial_pose,
            gui_panel,
            robot_node_headless,
            rviz_node,
        ]
    )
