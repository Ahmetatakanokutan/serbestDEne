from simple_pid import PID
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil
import time
from typing import Tuple
from math import radians


class CameraPID:
    def __init__(self, width, height, Kp=0.005, Ki=0.0, Kd=0.01):
        self.frame_width = width
        self.frame_height = height
        self.center_x = width // 2
        self.center_y = height // 2

        self.pid_x = PID(Kp, Ki, Kd, setpoint=self.center_x)
        self.pid_y = PID(Kp, Ki, Kd, setpoint=self.center_y)

        self.pid_x.output_limits = (-1, 1)
        self.pid_y.output_limits = (-1, 1)

    def compute(self, target_x, target_y) -> Tuple[float, float]:
        # x_offset = target_x - self.center_x
        # y_offset = target_y - self.center_y

        x_output = self.pid_x(target_x)
        y_output = self.pid_y(target_y)

        return x_output, y_output


class AileronController:
    def __init__(self, connection):
        self.conn: mavutil.mavudp = connection
        self.mav: ardupilotmega.MAVLink = connection.mav
        self.aileron_channel = 5  # Aileron channel number
        self.elevator_channel = 6
        self.aileron_val = 1500
        self.elevator_val = 1500

    def update_aileron_position_raw(self):
        # TODO compute the aileron command based on target_x and target_y
        channels = [0] * 18
        channels[self.aileron_channel] = self.aileron_val
        channels[self.elevator_channel] = self.elevator_val
        self.mav.rc_channels_override_send(1, 1, *channels[:8], force_mavlink1=False)

    def update_position(self, target_x, target_y):
        # TODO compute the aileron command based on target_x and target_y
        # This is a placeholder for the actual implementation
        # self.mav.set_position_target_local_ned_send()
        return


class VelocityController:
    def __init__(self, connection):
        self.conn: mavutil.mavudp = connection
        self.mav: ardupilotmega.MAVLink = connection.mav
        self.type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

    def set_target_velocity(self, vx, vy, vz):
        """
        Set the target velocity in local NED coordinates.
        """
        yaw_rate = 0.0

        self.mav.set_position_target_local_ned_send(
            self.conn.messages["SYSTEM_TIME"].time_boot_ms,
            1,
            1,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            self.type_mask,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0,
            0,
            0,
            0,
            yaw_rate,
        )


class UAVInterface:
    # TODO placeholder for UAV interface
    def __init__(self, mavlink_worker):
        self.mavlink_worker = mavlink_worker

    def send_aileron_command(self, deflection):
        # TODO Send command to UAV
        pass

    def get_current_roll(self):
        # TODO Retrieve current roll angle from UAV
        return 0.0
