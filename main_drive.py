from manual_drive.manual_mode import Manual
from manual_drive.dc_motor import Motor
from initialization.init import get_distance_sensors, get_wheel_motors
from automatic_drive.automatic_drive import auto_drive

SELF_DRIVE = True
def main():
    #init motor pins
    print("initializing motors")
    left_motor, right_motor = get_wheel_motors()
    print("initializing sensors")
    sonar_left, sonar_right = get_distance_sensors()

    #wait until bluetooth gets called
    
    #while not 0.5 meters away: run self_drive
    if SELF_DRIVE:
        auto_drive(left_motor=left_motor, right_motor=right_motor, sonar_left=sonar_left, sonar_right=sonar_right)

    #run manual_mode
    if SELF_DRIVE is False:
        pass

if __name__ == "__main__":
    main()