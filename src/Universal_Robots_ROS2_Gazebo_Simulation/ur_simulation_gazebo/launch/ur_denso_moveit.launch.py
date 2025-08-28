# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl (Modified for UR + Denso MoveIt)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    
    # Denso specific arguments
    denso_model = LaunchConfiguration("denso_model")
    denso_namespace = LaunchConfiguration("denso_namespace")
    denso_moveit_config_package = LaunchConfiguration("denso_moveit_config_package")
    denso_moveit_config_file = LaunchConfiguration("denso_moveit_config_file")

    # Launch UR and Denso simulation control (without MoveIt)
    ur_denso_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "launch_rviz": "false",  
        }.items(),
    )

    # Launch UR MoveIt
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "false",  # Launch RViz for UR robot visualization (only ur not denso)
            "use_fake_hardware": "true",  # to change moveit default controller to joint_trajectory_controller
        }.items(),
    )

    # Launch Denso MoveIt components directly (without Gazebo spawn since it's already running from ur_sim_control)
    from launch_ros.actions import Node
    from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
    from launch_ros.parameter_descriptions import ParameterValue
    import os
    from ament_index_python.packages import get_package_share_directory
    
    def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        try:
            with open(absolute_file_path, 'r') as file:
                import yaml
                return yaml.safe_load(file)
        except EnvironmentError:
            return None
    
    # Denso robot description for MoveIt
    denso_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("denso_robot_descriptions"), "urdf", "denso_robot.urdf.xacro"]
            ),
            " ",
            "model:=",
            denso_model,
            " ",
            "sim:=true",
            " ",
            "namespace:=",
            denso_namespace,
        ]
    )
    denso_robot_description = {"denso_robot_description": ParameterValue(value=denso_robot_description_content, value_type=str)}
    
    # Denso semantic description for MoveIt
    denso_robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(denso_moveit_config_package), "srdf", denso_moveit_config_file]
            ),
            " ",
            "model:=",
            denso_model,
            " ",
            "namespace:=",
            denso_namespace,
        ]
    )
    denso_robot_description_semantic = {"denso_robot_description_semantic": ParameterValue(value=denso_robot_description_semantic_content, value_type=str)}
    
    # Load MoveIt configuration files
    denso_kinematics_yaml = load_yaml('denso_robot_moveit_config', 'config/kinematics.yaml')
    denso_robot_description_kinematics = {'denso_robot_description_kinematics': denso_kinematics_yaml}
    
    denso_ompl_planning_yaml = load_yaml('denso_robot_moveit_config', 'config/ompl_planning.yaml')
    denso_ompl_planning_pipeline_config = {
        'denso_move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    if denso_ompl_planning_yaml:
        denso_ompl_planning_pipeline_config['denso_move_group'].update(denso_ompl_planning_yaml)
    
    denso_moveit_controllers = {
        'denso_moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    
    denso_trajectory_execution = {
        'denso_moveit_manage_controllers': False,
        'denso_trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'denso_trajectory_execution.allowed_goal_duration_margin': 0.5,
        'denso_trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Denso MoveIt move_group node
    denso_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='denso_move_group',
        output='screen',
        parameters=[
            denso_robot_description,
            denso_robot_description_semantic,
            denso_robot_description_kinematics,
            denso_ompl_planning_pipeline_config,
            denso_trajectory_execution,
            denso_moveit_controllers,
            {'denso_use_sim_time': True},
            {'denso_planning_group': 'denso_arm'},
        ],
    )
    
    # Denso robot state publisher
    denso_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='denso_robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}, denso_robot_description],
    )
    
    denso_moveit_launch = [denso_move_group_node, denso_robot_state_publisher_node]

    nodes_to_launch = [
        ur_denso_control_launch,
        ur_moveit_launch,
    ] + denso_moveit_launch

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur12e",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_simulation_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    
    # Denso specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "denso_model",
            default_value="hsr065a1_n32",
            description="Type/series of used Denso robot.",
            choices=["hsr065a1_n32", "cobotta", "vs060", "vs087"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "denso_namespace",
            default_value="denso_",
            description="Namespace for Denso robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "denso_moveit_config_package",
            default_value="denso_robot_moveit_config",
            description="Denso MoveIt config package with robot SRDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "denso_moveit_config_file",
            default_value="denso_robot.srdf.xacro",
            description="Denso MoveIt SRDF/XACRO description file with the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)]) 