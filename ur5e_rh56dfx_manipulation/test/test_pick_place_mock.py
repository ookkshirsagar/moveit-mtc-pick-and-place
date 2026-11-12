"""End-to-end check: the full pick and place succeeds on mock hardware, headless."""
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

TIMEOUT = 240.0  # seconds, planning plus execution with the default velocity scaling


@pytest.mark.launch_test
def generate_test_description():
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ur5e_rh56dfx_manipulation"), "launch", "mtc_demo.launch.py"])),
        launch_arguments={"rviz": "false", "hardware": "mock"}.items(),
    )
    return LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestPickPlace(unittest.TestCase):

    def test_pick_and_place_completes(self, proc_output):
        proc_output.assertWaitFor("Pick and place complete!", timeout=TIMEOUT)
