import time
import smbus
import board
import busio
from digitalio import DigitalInOut, Direction
import RPi.GPIO as GPIO
from VL53L0X import VL53L0X

# Initialize I2C bus and sensor.
i2c = busio.I2C(board.D23, board.D22)


xshut1 = 21
xshut2 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(xshut1, GPIO.OUT)
GPIO.setup(xshut2, GPIO.OUT)

GPIO.output(xshut1, GPIO.LOW)
GPIO.output(xshut2, GPIO.LOW)

time.sleep(0.01)

GPIO.output(xshut1, GPIO.HIGH)
time.sleep(0.01)

sensor_left = VL53L0X(i2c_bus = 3, i2c_address = 0x29)
sensor_left.set_address(0x30)

GPIO.output(xshut2, GPIO.HIGH)
time.sleep(0.01)

sensor_right = VL53L0X(i2c_bus = 3, i2c_address=0x29)

print("Sensor Left moved to 0x30, Right 0x29")
print("Reading distances...")

try:
    # Main loop will read the range and print it every second.
    while True:
        d1 = sensor_left.get_distance()
        d2 = sensor_right.get_distance()
        print(f"Sensor left: {d1} mm, Right: {d2} mm")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exit")  # Exit on CTRL+C

GPIO.cleanup()