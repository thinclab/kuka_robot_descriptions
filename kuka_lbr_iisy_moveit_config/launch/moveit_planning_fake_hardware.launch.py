# Copyright 2022 Áron Svastits
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
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_urdf_folder = LaunchConfiguration("robot_urdf_folder")    
    robot_urdf_filepath = LaunchConfiguration("robot_urdf_filepath")    
    robot_srdf_folder = LaunchConfiguration("robot_srdf_folder")        
    robot_srdf_filepath = LaunchConfiguration("robot_srdf_filepath")     

    moveit_config = (
        MoveItConfigsBuilder("kuka_lbr_iisy")
        .robot_description(
            file_path=get_package_share_directory(robot_urdf_folder.perform(context))
            + robot_urdf_filepath.perform(context)
        )
        .robot_description_semantic(
            get_package_share_directory(robot_srdf_folder.perform(context))
            + robot_srdf_filepath.perform(context)
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(
            file_path=get_package_share_directory("kuka_lbr_iisy_support")
            + f"/config/{robot_model.perform(context)}_joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    fake_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("kuka_resources"),
                "/launch/fake_hardware_planning_template.launch.py",
            ]
        ),
        launch_arguments={
            "robot_family": "{}".format("lbr_iisy"),
            "dof": f"{6}",
        }.items(),
    )

    to_start = [fake_hardware_launch, move_group_server]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"))
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_folder", default_value="kuka_lbr_iisy_support"))
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_filepath", default_value=f"/urdf/lbr_iisy3_r760.urdf.xacro"))
    launch_arguments.append(DeclareLaunchArgument("robot_srdf_folder", default_value="kuka_lbr_iisy_moveit_config"))
    launch_arguments.append(DeclareLaunchArgument("robot_srdf_filepath", default_value=f"/urdf/lbr_iisy3_r760.srdf"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
