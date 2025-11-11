# import sys
# import threading
# from bluetooth import BluetoothSocket, RFCOMM, advertise_service, SERIAL_PORT_CLASS, SERIAL_PORT_PROFILE, PORT_ANY
from driving.dc_motor import DriveBase, Motor
# sudo apt install bluetooth bluez python3-pybluez
import bluetooth, json, math, time

def clamp(v, lo=-1.0, hi=1.0): return max(lo, min(hi, v))
def apply_signed(dr, l, r):
    l_dir = "fwd" if l >= 0 else "rev"
    r_dir = "fwd" if r >= 0 else "rev"
    dr.set_speeds(abs(l), abs(r), (l_dir, r_dir))

def polar_to_diff(r, theta_deg):
    """
    Smooth omni-like mix that works great for a single virtual stick:
    Convert (r,θ) to left/right [-1..1].
    """
    t = math.radians(theta_deg)
    fwd = r * math.sin(t)    # forward component
    yaw = r * math.cos(t)    # turn component
    l = clamp(fwd + yaw)
    r_ = clamp(fwd - yaw)
    return l, r_

def run_bt_loop(drive):
    srv = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    srv.bind(("", 1)); srv.listen(1)
    print("🔵 Waiting for Bluetooth…")
    sock, addr = srv.accept(); print("✅ Connected:", addr)
    try:
        while True:
            raw = sock.recv(256).decode(errors="ignore").strip()
            if not raw: continue
            try:
                pkt = json.loads(raw)
                r = float(pkt["r"]); theta = float(pkt["theta"])
            except Exception:
                # CSV fallback: "r,theta"
                try:
                    r, theta = map(float, raw.split(","))
                except Exception:
                    continue

            # deadzone + clamp
            if r < 0.05: r = 0.0
            r = clamp(r, 0.0, 1.0)

            l, r_ = polar_to_diff(r, theta)
            apply_signed(drive, l, r_)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close(); srv.close()
        drive.stop(brake=False); drive.disable()

if __name__ == "__main__":
    #init motor pins
    left = Motor(en_pwm=12, in_a=3, in_b=4, sleep_pin=5, fault_pin=2, enc_a=17, enc_b=18)
    right = Motor(en_pwm=13, in_a=7, in_b=8, sleep_pin=9, fault_pin=6, enc_a=19, enc_b=20)

    #define drive base
    drive = DriveBase(left, right)
    run_bt_loop(drive)
