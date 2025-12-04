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
    
    
    return left, right