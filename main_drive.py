from manual_drive.manual_mode import Manual, DriveBase
from initialization.init import get_distance_sensors, get_wheel_motors
from automatic_drive.automatic_drive import auto_drive
import asyncio
from car_bluetooth import bt_pos

DEFAULT = {"x": "0.0", "y": "0.0"}

async def main_loop():
    left_motor, right_motor = get_wheel_motors()
    # sonar_left, sonar_right = get_distance_sensors()
    drive = DriveBase(left_motor, right_motor)
    manual_mode = Manual(drive)

    # Start BLE server
    # asyncio.create_task(bt_pos.bt_main())
    asyncio.create_task(bt_pos.bt_main())
    await bt_pos.wait_for_connection()
    print("BLE connected; starting drive loop.")

    while True:
        print("mode:", bt_pos.SELF_DRIVE)
        if bt_pos.SELF_DRIVE:
            # auto_drive(left_motor, right_motor, sonar_left, sonar_right)
            await asyncio.sleep(0.1)
        else:
            if bt_pos.JOYSTICK == None:
                manual_mode.joystick(DEFAULT)
            else:
                print(bt_pos.JOYSTICK)
                manual_mode.joystick(bt_pos.JOYSTICK)
                await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(main_loop())
