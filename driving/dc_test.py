from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button, RotaryEncoder
import time


class Motor():
    def __init__(self, ena: int, ina: int, inb: int):
        ENA = ena
        INA = ina
        INB = inb

        #motor control
        self.pwm = PWMOutputDevice(ENA, frequency=5000, initial_value=0.0)
        self.in1 = DigitalOutputDevice(INA)
        self.in2 = DigitalOutputDevice(INB)

        print("Motor initialized!")

    def forward(self):
        self.in1.on()
        self.in2.off()

        self.pwm.value = 0.25
        print("Motor moving forward!")
        time.sleep(5)
    
    def backward(self):
        self.in1.off()
        self.in2.on()

        self.pwm.value = 0.25
        print("Motor moving backward!")
        time.sleep(5)

    def stop(self):
        self.in1.off()
        self.in2.off()

        self.pwm.value = 0.0
        print("Motor stopped!")

if __name__ == "__main__":
    ENA = 12   # 
    IN1 = 3    # 
    IN2 = 4   # 

    motor1 = Motor(ENA, IN1, IN2)
    
    print("starting demo")
    motor1.forward()
    motor1.stop()
    motor1.backward()
    motor1.stop()


# # Create the motor control pins
#     pwm = PWMOutputDevice(ENA, frequency=5000, initial_value=0.0)
#     in1 = DigitalOutputDevice(IN1)
#     in2 = DigitalOutputDevice(IN2)

#     print("Starting motor test...")

# # # Set direction: forward
#     in1.on()
#     in2.off()

# # # Set speed (50% duty cycle)
#     pwm.value = 0.5
#     print("Motor should be spinning now!")
#     time.sleep(50)

# # Stop the motor
    # pwm.value = 0.0
    # in1.off()
    # in2.off()

    # print("Motor stopped.")

    # in1.off()
    # in2.on()
    
    # pwm.value = 0.25
    # print("Motor should be spinning now!")
    # time.sleep(5)

    # # Stop the motor
    # pwm.value = 0.0
    # in1.off()
    # in2.off()
