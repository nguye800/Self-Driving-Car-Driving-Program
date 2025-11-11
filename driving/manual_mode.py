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

    def joystick(self):
        try:
            # Apply deadzone & clamp
            x = self.clamp(self.deadzone(x))
            y = self.clamp(self.deadzone(y))

            # Mix to tank drive
            l, r = self.mix_cartesian(x, y)

            # Drive motors
            self.apply_signed(l, r)
            time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            self.drive.stop(brake=False)
            self.drive.disable()

