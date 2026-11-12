"""End-to-end check: the full pick and place succeeds on mock hardware, headless."""
import re
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
# every way the run can end, so a failure is reported at once instead of after the timeout
OUTCOME = re.compile(r"Pick and place complete!|Planning failed|Execution failed|aborting MTC chain|"
                     r"Failed after \d+ attempts")


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
        proc_output.assertWaitFor(OUTCOME, timeout=TIMEOUT)
        outcome = [OUTCOME.search(o.text.decode(errors="replace")) for o in proc_output]
        outcome = next(m.group(0) for m in outcome if m)
        self.assertEqual(outcome, "Pick and place complete!", f"the pick and place ended with: {outcome}")
