import RPi.GPIO as GPIO
import time

# Define the GPIO pins
TRIG = 23  # Trigger pin (example)
ECHO = 24  # Echo pin (example)

# Setup the GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # Trigger a short pulse to start measurement
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG, False)

    # Wait for the echo start
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Wait for the echo end
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate the duration of the pulse
    pulse_duration = pulse_end - pulse_start

    # Calculate distance: (speed of sound * time) / 2
    distance = (pulse_duration * 34300) / 2  # in centimeters
    return distance

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.2f} cm")
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped by user")
    GPIO.cleanup()
