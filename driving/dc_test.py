from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button, RotaryEncoder
import time



# Change these GPIO numbers to match your wiring
ENA = 12   # 
IN1 = 3    # 
IN2 = 4    # 

# Create the motor control pins
pwm = PWMOutputDevice(ENA, frequency=5000, initial_value=0.0)
in1 = DigitalOutputDevice(IN1)
in2 = DigitalOutputDevice(IN2)

print("Starting motor test...")

# Set direction: forward
in1.on()
in2.off()

# Set speed (50% duty cycle)
pwm.value = 0.5
print("Motor should be spinning now!")
time.sleep(5)

# Stop the motor
pwm.value = 0.0
in1.off()
in2.off()

print("Motor stopped.")
