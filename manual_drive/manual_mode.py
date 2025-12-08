import time
#FILE IMPORTS
from dc_motor import DriveBase, Motor

class Manual():
    def __init__(self, drivebase: DriveBase):
        self.drive = drivebase
    
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
        l_dir = "fwd" if l >= 0 else "rev"
        r_dir = "fwd" if r >= 0 else "rev"
        self.drive.set_speeds(abs(l), abs(r), (l_dir, r_dir))

    def joystick(self, pkt):
        try:
            x = float(pkt["x"])
            y = float(pkt["y"])
            # Apply deadzone & clamp
            print(x, y)
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
            self.drive.stop(brake=False)
            self.drive.disable()

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