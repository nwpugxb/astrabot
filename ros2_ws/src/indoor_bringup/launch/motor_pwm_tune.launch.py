"""GUI teleop for ESP32 deck robot PWM feedforward calibration.

Prerequisites:
  Terminal 1: scripts/run_microros_agent.sh /dev/ttyUSB0
  Firmware: OPEN_LOOP_MOTOR=false, STALL_PROTECTION_ENABLE=true

Then:
  ./run_motor_pwm_tune.sh
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="indoor_bringup",
                executable="motor_pwm_tune_gui_node",
                name="motor_pwm_tune_gui",
                output="screen",
            ),
        ]
    )
