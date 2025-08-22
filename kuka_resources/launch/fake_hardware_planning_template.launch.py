# Copyright 2022 Aron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_urdf_folder = LaunchConfiguration("robot_urdf_folder")
    robot_urdf_filepath = LaunchConfiguration("robot_urdf_filepath")    
    dof = LaunchConfiguration("dof")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    rviz_config_file = (
        get_package_share_directory("kuka_resources")
        + f"/config/planning_{dof.perform(context)}_axis.rviz"
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(robot_urdf_folder.perform(context)),
                    robot_urdf_filepath.perform(context).split("/")[1],
                    robot_urdf_filepath.perform(context).split("/")[2],
                ]
            ),
            " ",
            "mode:=",
            "mock",
            " ",
            "x:=",
            x,
            " ",
            "y:=",
            y,
            " ",
            "z:=",
            z,
            " ",
            "roll:=",
            roll,
            " ",
            "pitch:=",
            pitch,
            " ",
            "yaw:=",
            yaw,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    controller_config = (
        get_package_share_directory("kuka_resources")
        + f"/config/fake_hardware_config_{dof.perform(context)}_axis.yaml"
    )

    controller_manager_node = "/controller_manager"

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
    )

    robot_description_kinematics = {
        "robot_description_kinematics": {
            "manipulator": {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}
        }
    }

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        parameters=[
            robot_description_kinematics,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn controllers
    def controller_spawner(controller_with_config):
        arg_list = [controller_with_config[0], "-c", controller_manager_node]
        if controller_with_config[1] is not None:
            arg_list.append("-p")
            arg_list.append(controller_with_config[1])
        return Node(package="controller_manager", executable="spawner", arguments=arg_list)

    controller_names_and_config = [
        ("joint_state_broadcaster", None),
        ("joint_trajectory_controller", controller_config),
    ]

    controller_spawners = [
        controller_spawner(controllers) for controllers in controller_names_and_config
    ]

    to_start = [control_node, robot_state_publisher, rviz] + controller_spawners

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_folder", default_value="kuka_lbr_iisy_support"))
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_filepath", default_value=f"/urdf/lbr_iisy3_r760.urdf.xacro"))
    launch_arguments.append(DeclareLaunchArgument("dof", default_value="6"))
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
