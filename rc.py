import time
import imu
import log
from pymavlink import mavutil
import math

class RCManager:

    def __init__(self, logger=None, ac=None):
        try:
            if logger == None:
               logger = log.LogLib()

            self.log = logger

            self.master = mavutil.mavlink_connection(
                '/dev/ttyACM0',
                baud=115200)
            self.master.wait_heartbeat()

        except Exception as e:
            self.log.critical(e)


    def arm(self):
        self.master.arducopter_arm()
        # self.master.mav.command_long_send(
        #     self.master.target_system,
        #     self.master.target_component,
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        #     0,
        #     1, 0, 0, 0, 0, 0, 0)


    def disarm(self):
        self.master.arducopter_disarm()
        # self.master.mav.command_long_send(
        #     self.master.target_system,
        #     self.master.target_component,
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        #     0,
        #     0, 0, 0, 0, 0, 0, 0)


    def set_rc_channel_pwm(self, id, pwm=1500):
        
        if (id < 1) or (id > 6):
            self.log.error('Channel does not exist' % id)
            return

        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        if id < 9:
            
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,                # target_system
                self.master.target_component,             # target_component
                *rc_channel_values)

    def killall(self):
        self.set_rc_channel_pwm(1, 1500)
        self.set_rc_channel_pwm(2, 1500)
        self.set_rc_channel_pwm(3, 1500)
        self.set_rc_channel_pwm(4, 1500)
        self.set_rc_channel_pwm(5, 1500)
        self.set_rc_channel_pwm(6, 1500)

    def raw(self, id, pwm=1500):
        try:
            if (id == "pitch") :
                self.set_rc_channel_pwm(1, pwm)           
            if (id == "roll") :
                self.set_rc_channel_pwm(2, pwm)
            if (id == "throttle") :
                self.set_rc_channel_pwm(3, pwm)
            if (id == "yaw") :
                self.set_rc_channel_pwm(4, pwm)
            if (id == "forward") :
                self.set_rc_channel_pwm(5, pwm)
            if (id == "lateral") :
                self.set_rc_channel_pwm(6, pwm)
        except:
            pass
        
    # def move


if __name__ == "__main__":
    auv= RCManager()
    auv.arm()
    auv.killall()
    while True:
        time.sleep(0.2)
        auv.raw("lateral", 1450)

