import time
import asyncio
import inspect
from typing import Any
from bless import (
    BlessServer,
    GATTCharacteristicProperties as GcProps,
    GATTAttributePermissions as GaPerms
)
import json
import numpy as np
# from gpiozero import Motors

# --- UUIDs from your TSX file ---
SERVICE_UUID = 'b07498ca-ad5b-474e-940d-16f1a71141e0'
PING_CHAR_UUID    = 'c1ff12bb-3ed8-46e5-b4f9-a6ca6092d345'
PONG_CHAR_UUID    = 'd7add780-b042-4876-aae1-112855353cc1'
COMMAND_CHAR_UUID = 'e2f3c4d5-6789-4abc-def0-1234567890ab'
DATA_CHAR_UUID    = 'f1e2d3c4-b5a6-4789-8abc-0def12345678'

server_instance: BlessServer = None
ping_start: float = 0.0
calc_list = []
event_loop: asyncio.AbstractEventLoop = None

SELF_DRIVE = False
curr_position = np.array([0.0, 0.0])
curr_deg = 0.0
target = np.array([0.0, 0.0])
target_deg = 0.0
target_dist = 0.0

# --- Constants for the Car ---
CAR_SPEED_MPS = 0.5  # Meters per second, must be calibrated
TURN_DURATION_90 = 1.2 # Time needed to complete a 90-degree turn, must be calibrated
TURN_SPEED = 90 / TURN_DURATION_90
SOFTWARE_LATENCY_MS = 80.0  

def is_client_connected() -> bool:
    """
    Bless exposes `is_connected` differently across versions (property vs method).
    Normalize the result so the rest of the code can treat it as a bool.
    """
    global server_instance
    if not server_instance:
        return False
    status = getattr(server_instance, "is_connected", False)
    try:
        return bool(status())
    except TypeError:
        return bool(status)


async def update_characteristic(service_uuid: str, char_uuid: str):
    """
    Bless versions before 0.2 returned a bool from update_value while newer
    releases return an awaitable. Support both so we never `await` a bool.
    """
    if not server_instance:
        return
    result = server_instance.update_value(service_uuid, char_uuid)
    if inspect.isawaitable(result):
        await result


async def get_average_rtt(samples=5):
    """
    Measures and averages a number of RTT samples.
    """
    global calc_list, event_loop, target_dist
    
    while True:
        calc_list.clear() # Clear old samples
        
        # Wait for enough RTT samples while connected
        while len(calc_list) < samples:
            if not is_client_connected():
                await asyncio.sleep(1.0)
                break
            await asyncio.sleep(0.2) # Wait for pings (ping interval is ~0.1s)
        else:
            # Get a copy of the samples and clear the list for the next run
            current_samples = calc_list.copy()
            calc_list.clear()
            
            avg_rtt = sum(current_samples) / len(current_samples)
            print(f"Average RTT: {avg_rtt:.2f} ms")
            rtt = avg_rtt - SOFTWARE_LATENCY_MS
            target_dist = rtt / 1_000.0 * 299792458 / 2
            print(f"Estimated Distance: {target_dist:.2f} m")
        
        await asyncio.sleep(0.5)


async def next_ping():
    global server_instance, ping_start
    
    while True:
        if is_client_connected():
            try:
                ping_char = server_instance.get_characteristic(PING_CHAR_UUID)
                ping_char.value = b'\x01'

                ping_start = time.monotonic()

                await update_characteristic(SERVICE_UUID, PING_CHAR_UUID)
            except Exception as e:
                print(f"Error: {e}")
            await asyncio.sleep(0.1)
        else:
            await asyncio.sleep(1.0)

def write_recv(characteristic: Any, value: bytearray, **kwargs):
    """
    Called when the client writes to a characteristic.
    This is our "PING" handler.
    """
    global ping_start, calc_list, SELF_DRIVE
    
    if characteristic.uuid == PONG_CHAR_UUID:
        # Received a PING from the app.
        
        rtt = (time.monotonic() - ping_start) * 1000
        if ping_start > 0:
            print(f"RTT: {rtt:.2f} ms")
            calc_list.append(rtt)
    elif characteristic.uuid == COMMAND_CHAR_UUID:
        # Received a command from the app.

        try:
            command = value.decode('utf-8')
            if command == "START":
                SELF_DRIVE = True
            elif command == "MANUAL":
                SELF_DRIVE = False
            else:
                pkt = json.loads(command)
                      
            print(f"Received command: {command}")
        except UnicodeDecodeError:
            print(f"Received invalid command data: {value}")
        
async def send_data():
    global server_instance, target_dist, target_deg

    while True:
        if is_client_connected():
            try:
                # Send distance and direction in regualar intervals
                if target_dist < 2.0:
                    data_char = server_instance.get_characteristic(DATA_CHAR_UUID)
                    message = "MANUAL"
                    data_char.value = message.encode('utf-8')
                else:
                    data_char = server_instance.get_characteristic(DATA_CHAR_UUID)
                    message = {
                        "distance": round(target_dist, 2),
                        "direction": round(target_deg, 2)
                    }
                    message = json.dumps(message)
                    data_char.value = message.encode('utf-8')
                
                await update_characteristic(SERVICE_UUID, DATA_CHAR_UUID)
            except Exception as e:
                print(f"Error sending data: {e}")
        await asyncio.sleep(1.0)

async def on_connect():
    global server_instance
    while not is_client_connected():
        await asyncio.sleep(1.0)
    print("Client connected!")
    await asyncio.sleep(2.0)
    print("next ping")
    event_loop.create_task(next_ping())
    print("send data")
    event_loop.create_task(send_data())
    print("avg rtt")
    event_loop.create_task(get_average_rtt())

async def main():
    global server_instance, event_loop

    event_loop = asyncio.get_running_loop()

    print("Setting up BLE Peripheral...")
    server_instance = BlessServer(name="PiTest", loop=event_loop)

    server_instance.write_request_func = write_recv

    #advertising
    print("Starting BLE Server... Advertising as 'PiTest'")
    
    try:
        # Start advertising and run the main loop.
        await server_instance.add_new_service(SERVICE_UUID)
        
        await server_instance.add_new_characteristic(
            SERVICE_UUID,
            PING_CHAR_UUID,
            GcProps.notify,
            None,
            GaPerms.readable,
        )

        await server_instance.add_new_characteristic(
            SERVICE_UUID,
            PONG_CHAR_UUID,
            GcProps.write,
            None,
            GaPerms.writeable,
        )

        await server_instance.add_new_characteristic(
            SERVICE_UUID,
            COMMAND_CHAR_UUID,
            GcProps.write,
            None,
            GaPerms.writeable,
        )

        await server_instance.add_new_characteristic(
            SERVICE_UUID,
            DATA_CHAR_UUID,
            GcProps.notify,
            None,
            GaPerms.readable,
        )

        await server_instance.start()

        asyncio.create_task(on_connect())

        await asyncio.Event().wait() 
        
    except KeyboardInterrupt:
        print("Stopping server...")
    finally:
        if server_instance:
            await server_instance.stop()

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
