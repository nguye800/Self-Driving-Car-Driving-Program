from driving.manual_mode import Manual
from driving.dc_motor import Motor, DriveBase
from car_bluetooth.bt_pos import bt_main

SELF_DRIVE = False
def main():
    #init motor pins
    left = Motor(en_pwm=12, in_a=3, in_b=4, sleep_pin=5, fault_pin=2, enc_a=17, enc_b=18)
    right = Motor(en_pwm=13, in_a=7, in_b=8, sleep_pin=9, fault_pin=6, enc_a=19, enc_b=20)

    #define drive base
    drive = DriveBase(left, right)

    #wait until bluetooth gets called
    
    #while not 0.5 meters away: run self_drive

    #run manual_mode
    if SELF_DRIVE is False:
        pass