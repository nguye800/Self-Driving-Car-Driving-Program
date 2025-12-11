import time
#FILE IMPORTS
from manual_drive.dc_motor import DriveBase, Motor

class Manual():
    def __init__(self, left_motor: Motor, right_motor: Motor):
        self.motor_l = left_motor
        self.motor_r = right_motor
    
    def clamp(self, v, lo=-1.0, hi=1.0):
        return max(lo, min(hi, v))

    def deadzone(self, v, dz=0.06):
        return 0.0 if abs(v) < dz else v

    def mix_cartesian(self, x, y):
        """
        Arcade/Tank mixing using Cartesian joystick:
        y = forward/back
        x = turn left/right
        Returns (left, right) in [-1..1]
        """
        left  = self.clamp(y + x)
        right = self.clamp(y - x)
        return left, right
    
    def apply_signed(self, l, r):
        """Send signed speeds into DriveBase class."""
        # left motor
        if abs(l) < 0.02:
            self.motor_l.stop(brake_mode=False)
        else:
            speed = abs(l)
            if l >= 0:
                self.motor_l.forward(speed)
                print(f"[DEBUG]: LEFT Motor turning forward with {speed}")
            else:
                self.motor_l.backward(speed)
                print(f"[DEBUG]: LEFT Motor turning backward with {speed}")

        # right motor
        if abs(r) < 0.02:
            self.motor_r.stop(brake_mode=False)
        else:
            speed = abs(r)
            if r >= 0:
                self.motor_r.forward(speed)
                print(f"[DEBUG]: RIGHT Motor turning forward with {speed}")
            else:
                self.motor_r.backward(speed)
                print(f"[DEBUG]: RIGHT Motor turning forward with {speed}")
        
    def joystick(self, pkt):
        try:
            self.motor_l.enable()
            self.motor_r.enable()
            x = float(pkt["x"])
            y = float(pkt["y"])
            # Apply deadzone & clamp
            # print(x, y)
            x = self.clamp(self.deadzone(x))
            y = self.clamp(self.deadzone(y))

            # Mix to tank drive
            l, r = self.mix_cartesian(x, y)

            # Drive motors
            self.apply_signed(l, r)
            time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.motor_l.stop(brake_mode=False)
            self.motor_r.stop(brake_mode=False)
            self.motor_l.disable()
            self.motor_r.disable()

if __name__ == "__main__":
    #init motor pins
    left = Motor(en_pwm=12, in_a=3, in_b=4, sleep_pin=5, fault_pin=2, enc_a=17, enc_b=18)
    right = Motor(en_pwm=13, in_a=7, in_b=8, sleep_pin=9, fault_pin=6, enc_a=19, enc_b=20)

    #define drive base
    drive = DriveBase(left, right)
    manual_mode = Manual(drive)
    pkt = {"x": "0.5", "y": "0.5"}

    for i in range(10):
        manual_mode.joystick(pkt)