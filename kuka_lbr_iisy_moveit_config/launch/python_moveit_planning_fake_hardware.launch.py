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

import os
import yaml

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

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_urdf_folder = LaunchConfiguration("robot_urdf_folder")    
    robot_urdf_filepath = LaunchConfiguration("robot_urdf_filepath")    
    robot_srdf_folder = LaunchConfiguration("robot_srdf_folder")        
    robot_kinematics_folder = LaunchConfiguration("robot_kinematics_folder")        
    robot_ompl_folder = LaunchConfiguration("robot_ompl_folder")        
    robot_srdf_filepath = LaunchConfiguration("robot_srdf_filepath")

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
            "robot_urdf_folder": f"{robot_urdf_folder.perform(context)}",
            "robot_srdf_folder": f"{robot_srdf_folder.perform(context)}",
            "robot_urdf_filepath": f"{robot_urdf_filepath.perform(context)}",
            "robot_srdf_filepath": f"{robot_srdf_filepath.perform(context)}",           
            "robot_kinematics_folder": f"{robot_kinematics_folder.perform(context)}",
            "robot_ompl_folder": f"{robot_ompl_folder.perform(context)}",
        }.items(),
    )

    to_start = [fake_hardware_launch] #, move_group_server

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"))
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_folder", default_value="kuka_lbr_iisy_support"))
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_filepath", default_value=f"/urdf/lbr_iisy3_r760.urdf.xacro"))
    launch_arguments.append(DeclareLaunchArgument("robot_srdf_folder", default_value="kuka_lbr_iisy_moveit_config"))
    launch_arguments.append(DeclareLaunchArgument("robot_kinematics_folder", default_value="kuka_lbr_iisy_moveit_config"))
    launch_arguments.append(DeclareLaunchArgument("robot_ompl_folder", default_value="kuka_lbr_iisy_moveit_config"))
    launch_arguments.append(DeclareLaunchArgument("robot_srdf_filepath", default_value=f"/urdf/lbr_iisy3_r760.srdf"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])