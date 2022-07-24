import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)

master.wait_heartbeat()

# Arm
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

def set_rc_channel_pwm(id, pwm=1500):
    if id < 1:
        print("Channel does not exist.")
        return

    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm
        master.mav.rc_channels_override_send(master.target_system,master.target_component, *rc_channel_values)

while True:
    time.sleep(0.2)
    set_rc_channel_pwm(2, 1550)
time.sleep(5)
set_rc_channel_pwm(2, 1550)
time.sleep(3)