#!/usr/bin/env python3
"""Launch the Room 315 Stäubli scene for the optional demonstration."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    demo_share = get_package_share_directory("mfja_staubli_manipulation_demos")
    launch_file = f"{demo_share}/launch/room_315_staubli_pick_place_sim.launch.py"
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={"gui": "true"}.items(),
    )
    return LaunchDescription([scene])


if __name__ == "__main__":
    service = LaunchService()
    service.include_launch_description(generate_launch_description())
    raise SystemExit(service.run())
