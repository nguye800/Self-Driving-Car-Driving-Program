import time
import board
import busio
import digitalio
import adafruit_vl53l0x

# import RPi.GPIO as GPIO
# from VL53L0X import VL53L0X, Vl53l0xAccuracyMode

# Initialize I2C bus and sensor.
xshut1 = digitalio.DigitalInOut(board.D21)
xshut1.direction = digitalio.Direction.OUTPUT
print(xshut1)


xshut2 = digitalio.DigitalInOut(board.D24)
xshut2.direction = digitalio.Direction.OUTPUT

#shutfown both sensors
xshut1.value = False
xshut2.value = False
time.sleep(0.1)

#init i2c
i2c = busio.I2C(board.SCL, board.SDA)

xshut1.value = True
time.sleep(0.1)

sensor_left = adafruit_vl53l0x.VL53L0X(i2c)
sensor_left.set_address(0x30)
print("Left sensor active at 0x30")

xshut2.value = True
time.sleep(0.1)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c)
print("Right sensor active at 0x29")

# GPIO.setwarnings(False)

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(xshut1, GPIO.OUT)
# GPIO.setup(xshut2, GPIO.OUT)

# GPIO.output(xshut1, GPIO.LOW)
# GPIO.output(xshut2, GPIO.LOW)

# time.sleep(0.5)


# sensor_left = VL53L0X(i2c_bus = 3, i2c_address = 0x29)
# sensor_right = VL53L0X(i2c_bus = 3, i2c_address=0x30)

# sensor_left.open()
# sensor_right.open()

# GPIO.output(xshut1, GPIO.HIGH)
# time.sleep(0.5)
# sensor_left.start_ranging(Vl53l0xAccuracyMode.BETTER)

# GPIO.output(xshut2, GPIO.HIGH)
# time.sleep(0.5)
# sensor_right.start_ranging(Vl53l0xAccuracyMode.BETTER)

# print("Sensor Left moved to 0x30, Right 0x29")
print("Reading distances...")

try:
    # Main loop will read the range and print it every second.
    while True:
        d1 = sensor_left.distance
        d2 = sensor_right.distance
        if d1 > 0 or d2 > 0:
            print(f"Sensor left: {d1} mm, Right: {d2} mm")
        else: 
            print("Error: dist1 =", d1, "dist2 =", d2)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exit")  # Exit on CTRL+C
    # sensor_right.stop_ranging()
    # GPIO.output(xshut2, GPIO.LOW)
    # sensor_left.stop_ranging()
    # GPIO.output(xshut1,GPIO.LOW)

    # sensor_left
    # sensor_right.close()
    
# GPIO.cleanup()
