from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button, RotaryEncoder
import time

class Motor:
    """
    One DC motor driven by an L293D channel + quadrature encoder.
    - enable_pwm: PWM pin driving ENA/ENB
    - in_a, in_b: direction pins to L293D INx
    - sleep_pin: optional (some carrier boards expose nSLEEP)
    - fault_pin: optional (active-low; wired as Button pull-up)
    - enc_a, enc_b: quadrature encoder pins (optional)
    """
    def __init__(
        self,
        enable_pwm: int,
        in_a: int,
        in_b: int,
        sleep_pin: int | None = None,
        fault_pin: int | None = None,
        enc_a: int | None = None,
        enc_b: int | None = None,
        pwm_freq: int = 5000
    ):
        self.pwm = PWMOutputDevice(enable_pwm, frequency=pwm_freq, initial_value=0.0)
        self.in_a = DigitalOutputDevice(in_a, initial_value=False)
        self.in_b = DigitalOutputDevice(in_b, initial_value=False)

        self.sleep = DigitalOutputDevice(sleep_pin, initial_value=True) if sleep_pin is not None else None
        self.fault = Button(fault_pin, pull_up=True) if fault_pin is not None else None

        self.encoder = RotaryEncoder(enc_a, enc_b, max_steps=0) if (enc_a is not None and enc_b is not None) else None

        # default: coast
        self.coast()

    def set_speed(self, duty: float):
        """Set PWM duty (0..1)."""
        self.pwm.value = max(0.0, min(1.0, float(duty)))

    def forward_dir(self):
        """Set H-bridge direction to forward (INa=1, INb=0)."""
        self.in_a.on(); self.in_b.off()

    def backward_dir(self):
        """Set H-bridge direction to backward (INa=0, INb=1)."""
        self.in_a.off(); self.in_b.on()

    def brake(self):
        """Active brake (INa=INb=1, PWM=0)."""
        self.in_a.on(); self.in_b.on()
        self.pwm.value = 0.0

    def coast(self):
        """Coast (both OUTs Hi-Z on L293D via PWM=0 with one input low)."""
        self.in_a.off(); self.in_b.off()
        self.pwm.value = 0.0

    def enable(self):
        if self.sleep: self.sleep.on()

    def disable(self):
        self.coast()
        if self.sleep: self.sleep.off()

    # -------- high-level helpers --------
    def forward(self, speed: float = 0.6):
        self.forward_dir()
        self.set_speed(speed)

    def backward(self, speed: float = 0.6):
        self.backward_dir()
        self.set_speed(speed)

    def stop(self, brake_mode: bool = False):
        self.brake() if brake_mode else self.coast()

    # -------- telemetry --------
    def steps(self) -> int | None:
        return self.encoder.steps if self.encoder else None

    def reset_encoder(self):
        if self.encoder: self.encoder.reset()

    def ok(self) -> bool:
        """True if no fault reported (or no fault pin present)."""
        return True if self.fault is None else self.fault.is_pressed  # pull-up => pressed means HIGH => OK


class DriveBase:
    """
    Two-motor tank drive (left/right or m1/m2).
    Offers forward/back/turn/stop primitives using the two Motor instances.
    """
    def __init__(self, m1: Motor, m2: Motor):
        self.m1 = m1
        self.m2 = m2
        self.enable()

    def enable(self):
        self.m1.enable(); self.m2.enable()

    def disable(self):
        self.m1.disable(); self.m2.disable()

    def set_speeds(self, s1: float, s2: float, dirs=("fwd","fwd")):
        # dirs: tuple of "fwd"/"rev" per motor
        self.m1.forward_dir() if dirs[0] == "fwd" else self.m1.backward_dir()
        self.m2.forward_dir() if dirs[1] == "fwd" else self.m2.backward_dir()
        self.m1.set_speed(s1)
        self.m2.set_speed(s2)

    # ----- primitives -----
    def forward(self, speed=0.6):
        self.set_speeds(speed, speed, ("fwd","fwd"))

    def backward(self, speed=0.6):
        self.set_speeds(speed, speed, ("rev","rev"))

    def turn_left(self, speed=0.6):    # tank turn
        self.set_speeds(speed, speed, ("rev","fwd"))

    def turn_right(self, speed=0.6):   # tank turn
        self.set_speeds(speed, speed, ("fwd","rev"))

    def stop(self, brake=False):
        self.m1.stop(brake)
        self.m2.stop(brake)

    # ----- telemetry -----
    def encoder_steps(self):
        return (self.m1.steps(), self.m2.steps())

    def ok(self):
        return self.m1.ok() and self.m2.ok()


# M1: ENA=12, IN1=3 (PH), IN2=4 (Disable), SLEEP=5, FAULT=2, ENC A/B = 17/18
# M2: ENB=13, IN3=7 (PH), IN4=8 (Disable), SLEEP=9, FAULT=6, ENC A/B = 19/20

def build_drivebase() -> DriveBase:
    m1 = Motor(
        enable_pwm=12, in_a=3, in_b=4, sleep_pin=5, fault_pin=2, enc_a=17, enc_b=18
    )
    m2 = Motor(
        enable_pwm=13, in_a=7, in_b=8, sleep_pin=9, fault_pin=6, enc_a=19, enc_b=20
    )
    return DriveBase(m1, m2)


if __name__ == "__main__":
    drive = build_drivebase()
    try:
        def status(tag):
            s1, s2 = drive.encoder_steps()
            print(f"{tag} | enc1={s1} enc2={s2} | ok={drive.ok()}")

        drive.forward(0.5);  status("Forward"); time.sleep(2)
        drive.turn_left(0.5); status("Left");    time.sleep(1.5)
        drive.turn_right(0.5);status("Right");   time.sleep(1.5)
        drive.backward(0.5); status("Back");     time.sleep(2)
        drive.stop(brake=False); status("Stop (coast)")
        print("Demo complete. Ctrl+C to exit or reuse DriveBase in your app.")
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        drive.stop(brake=False)
        drive.disable()
        print("Clean exit.")
