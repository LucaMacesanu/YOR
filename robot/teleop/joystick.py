# joystick.py
"""
To run joystick on Jetson Nano, install xpad drivers, add user to the input group,
and set appropriate udev rules.

Controls (press Start to enable, Back to disable):
    Left stick      drive base (forward/back, sideways)
    Right stick X   turn base
    R1 / L1 (hold)  lift up / down; stops on release
    D-pad           neck: left/right = pan, up/down = tilt (held = keeps moving)
    Y               cycle base speed preset
"""

import time
import numpy as np
import pygame
from pygame.joystick import Joystick
from loop_rate_limiters import RateLimiter

from commlink import RPCClient

XBOX_CONTROLLER_MAP = {
    "start": 7,
    "back": 6,
    "y": 3,
    "l1": 4,
    "r1": 5,
    "left_horizontal_axis": 0,
    "left_vertical_axis": 1,
    "right_horizontal_axis": 3,
    "right_vertical_axis": 4,
}

PS4_CONTROLLER_MAP = {
    "start": 3,
    "back": 0,  # square
    "y": 2,  # triangle
    "l1": 4,
    "r1": 5,
    "left_horizontal_axis": 0,
    "left_vertical_axis": 1,
    "right_horizontal_axis": 3,
    "right_vertical_axis": 4,
    "pad_y": 7,
}

controller_map = XBOX_CONTROLLER_MAP

# Neck on the D-pad: degrees per second while held. Flip a sign if that
# joint moves the wrong way.
NECK_RATE_DEG_S = 60.0
NECK_PAN_SIGN = 1.0    # D-pad right -> +pan
NECK_TILT_SIGN = -1.0  # D-pad up -> -tilt (head up; verified on robot 2026-10-19)
NECK_SEND_HZ = 30.0


def apply_deadzone(arr, dz=0.05):
    return np.where(np.abs(arr) <= dz, 0.0, np.sign(arr) * (np.abs(arr) - dz) / (1 - dz))


class JoystickNode:
    def __init__(self):
        pygame.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No joystick detected")
        self.joystick = Joystick(0)

        # Three speed presets (m/s, m/s, rad/s) for Base.set_target_base_velocity
        self.max_vels = [
            np.array([0.50, 0.50, 1.57]),
            np.array([0.25, 0.25, 1.57]),
            np.array([0.75, 0.75, 1.57]),
        ]
        self.max_vel_setting = 0
        self.vel_alpha = 0.9  # low-pass on commanded vel

        self.control_loop_running = False

        # RPC to YOR (which wraps the new Base)
        self.yor = RPCClient(host="localhost", port=5557)
        self.yor.init()  # starts Base control loop on the server

        self.last_lift_cmd = 0  # +1 up, -1 down, 0 stopped
        self.last_y = 0
        self.warned_no_hat = False

        # Neck target (deg), integrated from the D-pad. None = no neck on the robot.
        self.neck_target = None
        self.neck_limits = None
        self.last_neck_send = 0.0
        self.neck_unsent = False
        try:
            self.neck_limits = self.yor.get_neck_limits()
            pos = self.yor.get_neck_position()
            if self.neck_limits and pos:
                self.neck_target = {"pan": float(pos["pan"]), "tilt": float(pos["tilt"])}
                print(f"Neck on D-pad, starting at {self.neck_target}")
            else:
                print("No neck on this robot; D-pad does nothing")
        except Exception as e:
            print(f"Neck unavailable ({e}); D-pad does nothing")

    def display_joystick_inputs(self):
        """Display current joystick input values"""
        print("\n" + "="*50)
        print("JOYSTICK INPUTS:")
        print("="*50)
        
        # Display axis values
        print("AXES:")
        for i in range(self.joystick.get_numaxes()):
            axis_value = self.joystick.get_axis(i)
            print(f"  Axis {i}: {axis_value:6.3f}")
        
        # Display button states
        print("BUTTONS:")
        for i in range(self.joystick.get_numbuttons()):
            button_state = self.joystick.get_button(i)
            print(f"  Button {i}: {'PRESSED' if button_state else 'RELEASED'}")
        
        # Display hat values
        num_hats = self.joystick.get_numhats()
        if num_hats > 0:
            print("HATS:")
        for i in range(num_hats):
                hat_value = self.joystick.get_hat(i)
                print(f"  Hat {i}: {hat_value}")
        else:
            print("HATS: None available")
        
        # Display current control state
        print(f"CONTROL STATE: {'RUNNING' if self.control_loop_running else 'STOPPED'}")
        print(f"MAX VEL SETTING: {self.max_vel_setting}")
        print("="*50)

    def control_loop(self):
        rate = RateLimiter(60, name="joystick")
        last_target_velocity = np.zeros(3, dtype=float)
        display_counter = 0  # Counter to control display frequency

        while True:
            pygame.event.pump()
            
            if not self.control_loop_running and self.joystick.get_button(controller_map["start"]):
                self.control_loop_running = True
                print("Control started")
                self.display_joystick_inputs()

            if self.control_loop_running and self.joystick.get_button(controller_map["back"]):
                self.control_loop_running = False
                if self.last_lift_cmd != 0:
                    self.yor.lift_stop()
                    self.last_lift_cmd = 0
                print("Control stopped")
                self.display_joystick_inputs()

            if self.control_loop_running:
                # Y: cycle speed preset (on press)
                y_btn = self.joystick.get_button(controller_map["y"])
                if y_btn and not self.last_y:
                    self.max_vel_setting = (self.max_vel_setting + 1) % len(self.max_vels)
                    print("Max velocity setting:", self.max_vel_setting)
                self.last_y = y_btn

                # Display inputs every 5 seconds (300 frames at 60Hz)
                display_counter += 1
                if display_counter >= 300:
                    self.display_joystick_inputs()
                    display_counter = 0

                # Left stick -> translation (vx, vy), right stick X -> yaw rate
                vy = -self.joystick.get_axis(controller_map["left_horizontal_axis"])
                vx = -self.joystick.get_axis(controller_map["left_vertical_axis"])
                w  = -self.joystick.get_axis(controller_map["right_horizontal_axis"])
                target_velocity = np.array([vx, vy, w], dtype=float)
                target_velocity = apply_deadzone(target_velocity)

                # scale & smooth
                target_velocity = self.max_vels[self.max_vel_setting] * target_velocity
                target_velocity = (1 - self.vel_alpha) * target_velocity + self.vel_alpha * last_target_velocity
                last_target_velocity = target_velocity

                # Send to Base (SparkFlex swerve)
                if np.linalg.norm(target_velocity, ord=1) > 1e-2:
                    self.yor.set_base_velocity(target_velocity)

                # Bumpers -> lift (hold R1 = up, hold L1 = down, both/neither = stop)
                up = self.joystick.get_button(controller_map["r1"])
                down = self.joystick.get_button(controller_map["l1"])
                lift_cmd = (1 if up else 0) - (1 if down else 0)
                if lift_cmd != self.last_lift_cmd:
                    if lift_cmd > 0:
                        print("RPC: lift_up()")
                        self.yor.lift_up()
                    elif lift_cmd < 0:
                        print("RPC: lift_down()")
                        self.yor.lift_down()
                    else:
                        print("RPC: lift_stop()")
                        self.yor.lift_stop()
                    self.last_lift_cmd = lift_cmd

                # D-pad -> neck (rate control: held = keeps moving, released = holds)
                if self.joystick.get_numhats() < 1:
                    if not self.warned_no_hat:
                        print("WARN: Controller reports 0 hats; D-pad unsupported by this device/driver")
                        self.warned_no_hat = True
                    pad_x, pad_y = 0, 0
                else:
                    pad_x, pad_y = self.joystick.get_hat(0)
                if self.neck_target is not None and (pad_x or pad_y):
                    step = NECK_RATE_DEG_S / 60.0  # loop runs at 60 Hz
                    for joint, d in (("pan", NECK_PAN_SIGN * pad_x), ("tilt", NECK_TILT_SIGN * pad_y)):
                        lo, hi = self.neck_limits[joint]
                        self.neck_target[joint] = float(np.clip(self.neck_target[joint] + d * step, lo, hi))
                    now = time.monotonic()
                    self.neck_unsent = True
                    if now - self.last_neck_send >= 1.0 / NECK_SEND_HZ:
                        self.yor.set_neck_position(self.neck_target["pan"], self.neck_target["tilt"])
                        self.last_neck_send = now
                        self.neck_unsent = False
                elif self.neck_target is not None and self.neck_unsent:
                    # D-pad released: send the final target the throttle skipped.
                    self.yor.set_neck_position(self.neck_target["pan"], self.neck_target["tilt"])
                    self.neck_unsent = False

            rate.sleep()


def main():
    node = JoystickNode()
    node.control_loop()


if __name__ == "__main__":
    main()
