import RPi.GPIO as GPIO
import time

# Define the GPIO pins
TRIG = 7  # Trigger pin (example)
ECHO = 11  # Echo pin (example)

# Setup the GPIO mode and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(2)

    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(TRIG, GPIO.LOW)

    timeout = time.time() + 0.05
    while GPIO.input(ECHO) == 0:
        if time.time() > timeout:
            return None
        pulse_start = time.time()
    timeout = time.time() + 0.05
    while GPIO.input(ECHO) == 1:
        if time.time() > timeout:
            return None
        pulse_end = time.time()

    # Calculate the duration of the pulse
    pulse_duration = pulse_end - pulse_start

    # Calculate distance: (speed of sound * time) / 2
    distance = (pulse_duration / 2) * 34300  # in centimeters
    return distance

try:
    while True:
        dist = get_distance()
        if dist is not None:
            print(f"Distance: {dist:.2f} cm")
        else:
            print("sensor timeout")
except KeyboardInterrupt:
    print("Stopped by user")
    GPIO.cleanup()
