import time
import asyncio
from bluez_peripheral.peripheral import Peripheral
from bluez_peripheral.service import Service
from bluez_peripheral.characteristic import Characteristic
# from gpiozero import Motors

# --- UUIDs from your TSX file ---
SERVICE_UUID = 'b07498ca-ad5b-474e-940d-16f1a71141e0'
PING_CHAR_UUID    = 'c1ff12bb-3ed8-46e5-b4f9-a6ca6092d345'
PONG_CHAR_UUID    = 'd7add780-b042-4876-aae1-112855353cc1'

ping_start: int = 0
ping_char_global: Characteristic = None
subscribed: bool = False

# --- Constants for the Car ---
CAR_SPEED_MPS = 0.5  # Meters per second, must be calibrated
TURN_DURATION = 1.2 # Time needed to complete a 90-degree turn, must be calibrated
KNOWN_DEVICE_ADDR = "XX:XX:XX:XX:XX:XX" # MAC address of the user's computer

class PongCharacteristic(Characteristic):
    """
    This is the "PONG" characteristic from the web app
    """
    def __init__(self, service):
        super().__init__(
            PONG_CHAR_UUID,
            ["write-without-response"],
            service
        )

    def write_value(self, value, options):
        """
        This function is called when the web app writes data to us.
        """
        global ping_start
        
        end_time_ns = time.perf_counter_ns()
        rtt_ns = end_time_ns - ping_start
        rtt_ms = rtt_ns / 1_000_000.0
        
        print(f"Pi-measured RTT: {rtt_ms:.2f} ms")
    
class PingCharacteristic(Characteristic):
    """
    This is the "PING" characteristic sent to the web app
    """
    def __init__(self, service):
        super().__init__(
            PING_CHAR_UUID,
            ["notify"],
            service
        )
    def notify_change(self, notifying):
        """
        This function is called when the client subscribes/unsubscribes.
        """
        global subscribed
        subscribed = notifying
        if notifying:
            print("Client subscribed")
        else:
            print("Client unsubscribed")

async def ping_loop(peripheral: Peripheral):
    """
    This is a separate task that pings the client every 2 seconds.
    """
    global ping_start, ping_char_global, subscribed
    
    while True:
        await asyncio.sleep(2) # Wait 2 seconds
        
        if peripheral.is_connected and subscribed and ping_char_global is not None:
            try:
                pi_ping_start_time_ns = time.perf_counter_ns()
                ping_char_global.notify(b'PING')
                             
            except Exception as e:
                print(f"Error sending PING: {e}")

async def main():
    global ping_char_global

    periph = Peripheral("PiPingerTest")
    
    service = Service(SERVICE_UUID, periph)

    pong_char = PongCharacteristic(service)
    ping_char = PingCharacteristic(service)
    ping_char_global = ping_char # Store global reference
    
    service.add_characteristic(ping_char)
    service.add_characteristic(pong_char)    
    periph.add_service(service)

    #advertising
    print("Starting BLE Server... Advertising as 'PiPingerTest'")
    
    try:
        # Start advertising and run the main loop.
        await periph.start()
        
        # Start the background pinging task.
        ping_task = asyncio.create_task(ping_loop(periph))
        
        # Wait for the peripheral to be stopped (e.g., by Ctrl+C)
        await periph.wait_for_disconnect()
        
    except KeyboardInterrupt:
        print("Stopping server...")
    finally:
        if 'ping_task' in locals():
            ping_task.cancel()
        await periph.stop()

if __name__ == "__main__":
    # Example: sudo python3 ble_pi_pinger_server.py
    asyncio.run(main())

# class BluetoothPos:
#     def __init__(self, device_address=KNOWN_DEVICE_ADDR, service_uuid=SERVICE_UUID):
#         self.server_sock = None
#         self.client_sock = None
#         self.client_info = None
#         self.service_uuid = service_uuid
#         # self.motors = Motors(forward=(17, 18), backward=(22, 23)) # Example GPIO pins



#     def measure_distance(bt_connection):
#         """
#         Performs a single "ping-pong" distance measurement over Bluetooth.
#         This assumes the connected device is running a responder script.
#         """
#         try:
#             start_time = time.monotonic_ns()
#             bt_connection.send(b'PING')
#             response = bt_connection.recv(1024) # With a timeout
#             end_time = time.monotonic_ns()

#             if response == b'PONG':
#                 round_trip_ns = end_time - start_time
#                 # This calculation is theoretical and highly prone to latency error
#                 # A calibration step to find and subtract system latency is required.
#                 round_trip_s = round_trip_ns / 1_000_000_000
#                 distance = (299792458 * round_trip_s) / 2
#                 return distance
#             return None
#         except IOError:
#             print("Error during distance measurement.")
#             return None


#     def solve_triangulation(p0, p1, p2, d0, d1, d2):
#         """
#         Calculates the location of a target using three distance measurements
#         from three non-collinear points. (Adapted from your code).
#         """
#         dist_p0_p1 = np.linalg.norm(p1 - p0)

#         if dist_p0_p1 > d0 + d1 or dist_p0_p1 < abs(d0 - d1):
#             print("Error: Circles do not intersect. Cannot solve.")
#             return None, None

#         a = (d0**2 - d1**2 + dist_p0_p1**2) / (2 * dist_p0_p1)
#         h_sq = d0**2 - a**2
#         h = 0 if h_sq < 0 else np.sqrt(h_sq)

#         p_intersect = p0 + a * (p1 - p0) / dist_p0_p1
#         perp_vector = np.array([-(p1[1] - p0[1]), (p1[0] - p0[0])]) / dist_p0_p1

#         possible_point1 = p_intersect + h * perp_vector
#         possible_point2 = p_intersect - h * perp_vector

#         dist_to_point1 = np.linalg.norm(possible_point1 - p2)
#         dist_to_point2 = np.linalg.norm(possible_point2 - p2)

#         calculated_location = possible_point1 if abs(dist_to_point1 - d2) < abs(dist_to_point2 - d2) else possible_point2

#         direction_vector = calculated_location - p2
#         world_angle_rad = np.arctan2(direction_vector[1], direction_vector[0])
#         world_angle_deg = np.degrees(world_angle_rad)

#         car_final_heading_deg = -90.0 # Assumes final move was along negative Y-axis
#         required_turn_deg = world_angle_deg - car_final_heading_deg
#         required_turn_deg = (required_turn_deg + 180) % 360 - 180

#         return calculated_location, required_turn_deg

#     # def run_initial_calibration(bt_connection):
#     #     """
#     #     Executes the L-shaped movement pattern to get the first triangulation.
#     #     """
#     #     print("--- Starting Initial Calibration ---")
        
#     #     # Define the car's path based on its actions
#     #     p0 = np.array([0.0, 0.0])
        
#     #     # 1. First measurement at the starting point
#     #     d0 = measure_distance(bt_connection)
#     #     if d0 is None: return None, None
#     #     print(f"Measurement at P0: {d0:.2f} m")

#     #     # 2. Drive forward
#     #     control_motors("forward", 2.0)
#     #     p1 = p0 + np.array([CAR_SPEED_MPS * 2, 0.0])
#     #     d1 = measure_distance(bt_connection)
#     #     if d1 is None: return None, None
#     #     print(f"Measurement at P1: {d1:.2f} m")
        
#     #     # 3. Turn right and drive forward again
#     #     control_motors("right", TURN_DURATION_S)
#     #     control_motors("forward", 2.0)
#     #     p2 = p1 + np.array([0.0, -CAR_SPEED_MPS * 2])
#     #     d2 = measure_distance(bt_connection)
#     #     if d2 is None: return None, None
#     #     print(f"Measurement at P2: {d2:.2f} m")

#     #     # 4. Solve for the initial location and required turn
#     #     target_loc, initial_turn = solve_triangulation(p0, p1, p2, d0, d1, d2)
        
#     #     if target_loc is not None:
#     #         print(f"Calibration complete. Target estimated at {target_loc}.")
#     #         print(f"Initial turn required: {initial_turn:.1f} degrees.")
#     #         # Execute the initial turn to face the target
#     #         # turn_duration = abs(initial_turn / 90.0) * TURN_DURATION_S
#     #         # turn_direction = "left" if initial_turn > 0 else "right"
#     #         # control_motors(turn_direction, turn_duration)
        
#     #     return target_loc, p2 # Return target location and car's final position
