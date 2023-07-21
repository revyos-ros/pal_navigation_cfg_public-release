# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import os
import yaml
from typing import Dict
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import SetRemap
from launch_pal.param_utils import parse_parametric_yaml


def loc_and_nav(context, *args, **kwargs):
    remappings_file = LaunchConfiguration("remappings_file").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    map_yaml_file = LaunchConfiguration("map")

    nav2_bringup_pkg = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    source_files = []

    with open(remappings_file, "r") as remappings_yaml:
        remappings = yaml.safe_load(remappings_yaml)

    with open(params_file, "r") as params_yaml:
        params: Dict = yaml.safe_load(params_yaml)
        for key, param in params.items():
            source_files.append(
                os.path.join(
                    get_package_share_directory("pal_navigation_cfg_params"),
                    key,
                    param + ".yaml",
                )
            )

    configured_params = parse_parametric_yaml(
        source_files=source_files, param_rewrites=remappings
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_pkg, "/localization_launch.py"]
        ),
        launch_arguments={
            "params_file": configured_params,
            "map": map_yaml_file,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('slam'))
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_pkg, "/slam_launch.py"]),
        launch_arguments={"params_file": configured_params}.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    cmd_vel_remap = SetRemap(src="cmd_vel", dst="nav_vel")

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_pkg, "/navigation_launch.py"]
        ),
        launch_arguments={"params_file": configured_params}.items(),
    )
    return [localization_launch, slam_toolbox_launch, cmd_vel_remap, nav2_bringup_launch]


def generate_launch_description():
    nav2_bringup_pkg = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )
    pmb2_maps_dir = get_package_share_directory("pmb2_maps")

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="false",
        description="Whether to start the SLAM or the Map Localization",
    )

    declare_params_file_launch_arg = DeclareLaunchArgument(
        "params_file",
        description=(
            "Full path to the ROS2 parameters file to use for AMCL"
            " configuration"
        ),
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pmb2_maps_dir, "config", "map.yaml"),
        description="Full path to map yaml file to load",
    )

    declare_remappings_file_launch_arg = DeclareLaunchArgument(
        "remappings_file",
        default_value=os.path.join(
            get_package_share_directory("pal_navigation_cfg_bringup"),
            "params",
            "default_remappings.yaml",
        ),
        description=(
            "Full path to the ROS2 parameters file to use for NAV2"
            " configuration"
        ),
    )

    loc_and_nav_launch = OpaqueFunction(function=loc_and_nav)

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_pkg, "/rviz_launch.py"])
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_params_file_launch_arg)
    ld.add_action(declare_remappings_file_launch_arg)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(loc_and_nav_launch)
    ld.add_action(rviz_launch)

    return ld
