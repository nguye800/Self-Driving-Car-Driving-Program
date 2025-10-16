import bluetooth
import time
import numpy as np
# from gpiozero import Motors

# --- Constants for the Car ---
CAR_SPEED_MPS = 0.5  # Meters per second, must be calibrated
TURN_DURATION_S = 1.2 # Time needed to complete a 90-degree turn, must be calibrated
KNOWN_DEVICE_ADDR = "XX:XX:XX:XX:XX:XX" # MAC address of the user's computer
SERVICE_UUID = "94f39d29-7d6d-437d-973b-fba39e49d4ee"


class BluetoothPos:
    def __init__(self, device_address=KNOWN_DEVICE_ADDR, service_uuid=SERVICE_UUID):
        self.device_address = device_address
        self.service_uuid = service_uuid
        self.bt_connection = None
        # self.motors = Motors(forward=(17, 18), backward=(22, 23)) # Example GPIO pins


    # def connect_to_device(device_address, service_uuid):
    #     """
    #     Scans for and connects to the user's device via Bluetooth RFCOMM.

    #     Returns:
    #         BluetoothSocket: A connection object if successful, otherwise None.
    #     """
    #     print(f"Searching for device {device_address}...")
    #     service_matches = bluetooth.find_service(uuid=service_uuid, address=device_address)

    #     if len(service_matches) == 0:
    #         print("Error: Could not find the required Bluetooth service.")
    #         return None

    #     first_match = service_matches[0]
    #     port = first_match["port"]
    #     host = first_match["host"]

    #     try:
    #         print(f"Connecting to host {host} on port {port}...")
    #         sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    #         sock.connect((host, port))
    #         print("Connection successful.")
    #         return sock
    #     except bluetooth.btcommon.BluetoothError as e:
    #         print(f"Bluetooth Error: {e}")
    #         return None

    def measure_distance(bt_connection):
        """
        Performs a single "ping-pong" distance measurement over Bluetooth.
        This assumes the connected device is running a responder script.
        """
        try:
            start_time = time.monotonic_ns()
            bt_connection.send(b'PING')
            response = bt_connection.recv(1024) # With a timeout
            end_time = time.monotonic_ns()

            if response == b'PONG':
                round_trip_ns = end_time - start_time
                # This calculation is theoretical and highly prone to latency error
                # A calibration step to find and subtract system latency is required.
                round_trip_s = round_trip_ns / 1_000_000_000
                distance = (299792458 * round_trip_s) / 2
                return distance
            return None
        except IOError:
            print("Error during distance measurement.")
            return None


    def solve_triangulation(p0, p1, p2, d0, d1, d2):
        """
        Calculates the location of a target using three distance measurements
        from three non-collinear points. (Adapted from your code).
        """
        dist_p0_p1 = np.linalg.norm(p1 - p0)

        if dist_p0_p1 > d0 + d1 or dist_p0_p1 < abs(d0 - d1):
            print("Error: Circles do not intersect. Cannot solve.")
            return None, None

        a = (d0**2 - d1**2 + dist_p0_p1**2) / (2 * dist_p0_p1)
        h_sq = d0**2 - a**2
        h = 0 if h_sq < 0 else np.sqrt(h_sq)

        p_intersect = p0 + a * (p1 - p0) / dist_p0_p1
        perp_vector = np.array([-(p1[1] - p0[1]), (p1[0] - p0[0])]) / dist_p0_p1

        possible_point1 = p_intersect + h * perp_vector
        possible_point2 = p_intersect - h * perp_vector

        dist_to_point1 = np.linalg.norm(possible_point1 - p2)
        dist_to_point2 = np.linalg.norm(possible_point2 - p2)

        calculated_location = possible_point1 if abs(dist_to_point1 - d2) < abs(dist_to_point2 - d2) else possible_point2

        direction_vector = calculated_location - p2
        world_angle_rad = np.arctan2(direction_vector[1], direction_vector[0])
        world_angle_deg = np.degrees(world_angle_rad)

        car_final_heading_deg = -90.0 # Assumes final move was along negative Y-axis
        required_turn_deg = world_angle_deg - car_final_heading_deg
        required_turn_deg = (required_turn_deg + 180) % 360 - 180

        return calculated_location, required_turn_deg

    # def run_initial_calibration(bt_connection):
    #     """
    #     Executes the L-shaped movement pattern to get the first triangulation.
    #     """
    #     print("--- Starting Initial Calibration ---")
        
    #     # Define the car's path based on its actions
    #     p0 = np.array([0.0, 0.0])
        
    #     # 1. First measurement at the starting point
    #     d0 = measure_distance(bt_connection)
    #     if d0 is None: return None, None
    #     print(f"Measurement at P0: {d0:.2f} m")

    #     # 2. Drive forward
    #     control_motors("forward", 2.0)
    #     p1 = p0 + np.array([CAR_SPEED_MPS * 2, 0.0])
    #     d1 = measure_distance(bt_connection)
    #     if d1 is None: return None, None
    #     print(f"Measurement at P1: {d1:.2f} m")
        
    #     # 3. Turn right and drive forward again
    #     control_motors("right", TURN_DURATION_S)
    #     control_motors("forward", 2.0)
    #     p2 = p1 + np.array([0.0, -CAR_SPEED_MPS * 2])
    #     d2 = measure_distance(bt_connection)
    #     if d2 is None: return None, None
    #     print(f"Measurement at P2: {d2:.2f} m")

    #     # 4. Solve for the initial location and required turn
    #     target_loc, initial_turn = solve_triangulation(p0, p1, p2, d0, d1, d2)
        
    #     if target_loc is not None:
    #         print(f"Calibration complete. Target estimated at {target_loc}.")
    #         print(f"Initial turn required: {initial_turn:.1f} degrees.")
    #         # Execute the initial turn to face the target
    #         # turn_duration = abs(initial_turn / 90.0) * TURN_DURATION_S
    #         # turn_direction = "left" if initial_turn > 0 else "right"
    #         # control_motors(turn_direction, turn_duration)
        
    #     return target_loc, p2 # Return target location and car's final position
