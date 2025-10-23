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
        self.server_sock = None
        self.client_sock = None
        self.client_info = None
        self.service_uuid = service_uuid
        # self.motors = Motors(forward=(17, 18), backward=(22, 23)) # Example GPIO pins


    # def bt_server_start(self):
    #     """
    #     Initializes the server, advertises the service,
    #     and waits for a single client (laptop/phone) to connect.
        
    #     Returns:
    #         True if connection is successful, False otherwise.
    #     """
    #     try:
    #         self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    #         self.server_sock.bind(("", bluetooth.PORT_ANY))
    #         self.server_sock.listen(1)
    #         port = self.server_sock.getsockname()[1]

    #         # Advertise the service using the UUID
    #         bluetooth.advertise_service(self.server_sock, "RaspiBtSrv",
    #                            service_id=self.service_uuid,
    #                            service_classes=[self.service_uuid, bluetooth.SERIAL_PORT_CLASS],
    #                            profiles=[bluetooth.SERIAL_PORT_PROFILE],
    #                            )
                               
    #         print(f"Waiting for connection on RFCOMM channel {port}...")
    #         print("Ensure Pi is discoverable (`bluetoothctl discoverable on`)")
            
    #         # This is a blocking call. The script will pause here until
    #         # your computer/client successfully connects.
    #         self.client_sock, self.client_info = self.server_sock.accept()
    #         print(f"Accepted connection from {self.client_info}")
    #         return True
            
    #     except Exception as e:
    #         print(f"Error starting server or accepting connection: {e}")
    #         self.close()
    #         return False

    # def handle_connection_loop(self):
    #     """
    #     Handles the "ping-pong" logic for an established connection.
    #     This loop will run as long as the client is connected.
    #     """
    #     if not self.client_sock:
    #         print("Error: No client connected. Call start_server_and_wait() first.")
    #         return

    #     print("Connection active. Listening for 'PING' requests...")
    #     try:
    #         while True:
    #             data = self.client_sock.recv(1024)
    #             if not data:
    #                 print("Client disconnected.")
    #                 break
                
    #             # This is the core responder logic for distance measurement
    #             if data == b'PING':
    #                 self.client_sock.send(b'PONG')
    #             else:
    #                 # You can handle other commands here
    #                 print(f"Received command: {data.decode('utf-8')}")
    #                 self.client_sock.send(b'CMD_ACK') # Acknowledge command

    #     except IOError:
    #         print("Connection lost.")
    #     finally:
    #         self.close()

    # def get_connection(self):
    #     """
    #     A helper getter to pass the connection object to other functions
    #     (like your triangulation/navigation logic).
    #     """
    #     return self.client_sock

    # def close(self):
    #     """
    #     Closes all open sockets to clean up.
    #     """
    #     if self.client_sock:
    #         self.client_sock.close()
    #         self.client_sock = None
    #     if self.server_sock:
    #         self.server_sock.close()
    #         self.server_sock = None
    #     print("Sockets closed.")

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
