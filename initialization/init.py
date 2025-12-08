from manual_drive.dc_motor import Motor
import time

import board
import busio
from gpiozero import DigitalOutputDevice
import adafruit_vl53l0x

SONAR_COLLISION_THRESHOLD = 0.25

# globals for sensors
sonar_left = None
sonar_right = None

class SonarWrapper:
    """
    Simple wrapper so check_collision() can use getValue() and getName()
    """
    def __init__(self, name, sensor):
        self._name = name
        self._sensor = sensor

    def getName(self):
        return self._name

    def getValue(self):
        """
        Return distance in meters.
        adafruit_vl53l0x.VL53L0X.range is in millimeters.
        """
        dist_mm = self._sensor.range
        # Sensor returns 0 or very large on error sometimes;
        return dist_mm / 1000.0

# ----------------------------
# Motor + Sonar setup
# ----------------------------
def check_collision(sensors, threshold=SONAR_COLLISION_THRESHOLD):
    """
    Returns True if any sonar in `sensors` sees an obstacle closer than `threshold` meters.
    Assumes sonar lookupTable is configured to return meters (which we do above).
    """
    for s in sensors:
        if s is None:
            continue
        v = s.getValue()
        # Debug if you want:
        # print(f"Sonar {s.getName()} reading: {v:.3f} m")
        if v < threshold:
            print(f"[SONAR COLLISION] {s.getName()} detects object at {v:.3f} m")
            return True
    return False

def get_distance_sensors(): 
    print("[DEBUG] Initializing ToF sensors")
    global sonar_left, sonar_right

    # If already initialized, just return them
    if sonar_left is not None and sonar_right is not None:
        print("[DEBUG] sensors already initialized")
        return sonar_left, sonar_right

    # I2C bus (SCL: GPIO 11, SDA: GPIO 10)
    print("[DEBUG] Initializing I2C bus, GPIO 10 (SDA) & GPIO11 (SCL)")
    i2c = busio.I2C(scl=board.D11, sda=board.D10)

    # XSHUT pins (BCM numbering)
    xshut_left = DigitalOutputDevice(22)   # wire to left VL53 XSHUT
    xshut_right = DigitalOutputDevice(27)  # wire to right VL53 XSHUT

    # Reset both sensors
    xshut_left.off()
    xshut_right.off()
    time.sleep(0.01)

    # Bring up LEFT sensor first and give it a new I2C address
    print("[DEBUG] Enabling LEFT sensor...")
    xshut_left.on()
    time.sleep(0.05)
    vl_left = adafruit_vl53l0x.VL53L0X(i2c)
    vl_left.set_address(0x30)   # any free address ≠ 0x29
    print("[DEBUG] Left sensor set to address 0x30")

    # Then bring up RIGHT sensor and assign a different address
    print("[DEBUG] Enabling RIGHT sensor...")
    xshut_right.on()
    time.sleep(0.05)
    vl_right = adafruit_vl53l0x.VL53L0X(i2c)
    vl_right.set_address(0x31)
    print("[DEBUG] Right sensor set to address 0x31")


    sonar_left = SonarWrapper("left_tof", vl_left)
    sonar_right = SonarWrapper("right_tof", vl_right)

    return sonar_left, sonar_right

def get_wheel_motors(): # fix
    left = Motor(
        en_pwm=12, in_a=3, in_b=4, sleep_pin=5, fault_pin=2, enc_a=17, enc_b=18
    )
    right = Motor(
        en_pwm=13, in_a=7, in_b=8, sleep_pin=9, fault_pin=6, enc_a=19, enc_b=20
    )
    
    return left, right