from manual_drive.dc_motor import Motor

SONAR_COLLISION_THRESHOLD = 0.25
        
# ----------------------------
# Motor + Sonar setup
# ----------------------------
def check_collision(sensors, threshold=SONAR_COLLISION_THRESHOLD):
    """
    Returns True if any sonar in `sensors` sees an obstacle closer than `threshold` meters.
    Assumes sonar lookupTable is configured to return meters (which we do above).
    """
    for s in sensors:
        if s is None:
            continue
        v = s.getValue()
        # Debug if you want:
        # print(f"Sonar {s.getName()} reading: {v:.3f} m")
        if v < threshold:
            print(f"[SONAR COLLISION] {s.getName()} detects object at {v:.3f} m")
            return True
    return False

def get_distance_sensors(): # fix

    return sonar_left, sonar_right

def get_wheel_motors(): # fix
    left = Motor(
        enable_pwm=12, in_a=3, in_b=4, sleep_pin=5, fault_pin=2, enc_a=17, enc_b=18
    )
    right = Motor(
        enable_pwm=13, in_a=7, in_b=8, sleep_pin=9, fault_pin=6, enc_a=19, enc_b=20
    )
    
    return left, right