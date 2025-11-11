from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button, RotaryEncoder
import time

class Motor:
    """
    PH/EN mode:
      - enable_pwm -> EN/IN1 (PWM): 0 = brake (H,H), 1 = drive; duty controls speed
      - in_a       -> PH/IN2 (dir): 0 or 1 selects direction
      - in_b       -> DISABLE (active-HIGH): 1 = Hi-Z (coast), 0 = outputs enabled
      - sleep_pin  -> nSLEEP (active-HIGH)
      - fault_pin  -> nFAULT (active-LOW, open-drain; use pull-up)
      - enc_a/b    -> optional quadrature encoder

    stop(brake=False): brake=False => COAST (Hi-Z via DISABLE=1)
                       brake=True  => BRAKE (EN=0, DISABLE=0)
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
        pwm_freq: int = 20000  # keep above audible range
    ):
        # EN/IN1: PWM speed control
        self.pwm = PWMOutputDevice(enable_pwm, frequency=pwm_freq, initial_value=0.0)

        # PH/IN2: direction
        self.dir = DigitalOutputDevice(in_a, initial_value=False)

        # DISABLE: active-HIGH (1 = Hi-Z/coast). Start enabled (LOW).
        self.disable_pin = DigitalOutputDevice(in_b, initial_value=False)

        # nSLEEP: active-HIGH; default awake
        self.sleep = DigitalOutputDevice(sleep_pin, initial_value=True) if sleep_pin is not None else None

        # nFAULT: active-LOW; use pull-up. Pressed==True when LOW (fault).
        self.fault = Button(fault_pin, pull_up=True) if fault_pin is not None else None

        # optional encoder
        self.encoder = RotaryEncoder(enc_a, enc_b, max_steps=0) if (enc_a is not None and enc_b is not None) else None

        # default to COAST per request (Hi-Z via DISABLE=1)
        self.coast()

    # ---- helpers for output state ----
    def _enable_outputs(self):
        if self.sleep: self.sleep.on()        # nSLEEP=1
        self.disable_pin.off()                # DISABLE=0 (enabled)

    def _disable_outputs(self):
        self.disable_pin.on()                 # DISABLE=1 (Hi-Z)

    # ---- low-level controls ----
    def set_speed(self, duty: float = 0.5):
        """Set PWM duty (0..1) on EN/IN1. EN=0 brakes per truth table."""
        d = max(0.0, min(1.0, float(duty)))
        self._enable_outputs()
        self.pwm.value = d

    def forward_dir(self):
        """PH/IN2 = 1 (choose as 'forward')."""
        self._enable_outputs()
        self.dir.on()

    def backward_dir(self):
        """PH/IN2 = 0 (choose as 'backward')."""
        self._enable_outputs()
        self.dir.off()

    def brake(self):
        """Active brake: EN=0 while enabled (OUT1=H, OUT2=H)."""
        self._enable_outputs()
        self.pwm.value = 0.0

    def coast(self):
        """True Hi-Z: DISABLE=1 (independent of EN/PH)."""
        self.pwm.value = 0.0
        self._disable_outputs()

    def enable(self):
        self._enable_outputs()

    def disable(self):
        self.coast()
        if self.sleep: self.sleep.off()       # optional power save

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
        """True if no fault (nFAULT HIGH). With pull_up=True: is_pressed==True means LOW (fault)."""
        return True if self.fault is None else (not self.fault.is_pressed)


class DriveBase:
    """Two-motor tank drive using DRV8873 in PH/EN mode."""
    def __init__(self, m1: Motor, m2: Motor):
        self.m1 = m1
        self.m2 = m2
        self.enable()

    def enable(self):
        self.m1.enable(); self.m2.enable()

    def disable(self):
        self.m1.disable(); self.m2.disable()

    def set_speeds(self, s1: float, s2: float, dirs=("fwd","fwd")):
        (self.m1.forward_dir() if dirs[0] == "fwd" else self.m1.backward_dir())
        (self.m2.forward_dir() if dirs[1] == "fwd" else self.m2.backward_dir())
        self.m1.set_speed(s1)
        self.m2.set_speed(s2)

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

    def encoder_steps(self):
        return (self.m1.steps(), self.m2.steps())

    def ok(self):
        return self.m1.ok() and self.m2.ok()


# Example wiring (BCM):
# M1: EN/IN1(PWM)=12, PH/IN2=3, DISABLE=4, nSLEEP=5, nFAULT=2, ENC A/B = 17/18
# M2: EN/IN1(PWM)=13, PH/IN2=7, DISABLE=8, nSLEEP=9, nFAULT=6, ENC A/B = 19/20

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

        # Stop: choose coast (Hi-Z) or brake (H,H)
        drive.stop(brake=False); status("Stop (coast)")
        # drive.stop(brake=True);  status("Stop (brake)")

        print("Demo complete. Ctrl+C to exit or reuse DriveBase in your app.")
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        drive.stop(brake=False)
        drive.disable()
        print("Clean exit.")
