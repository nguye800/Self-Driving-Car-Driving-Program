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
import math
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

JOYSTICK = None
SELF_DRIVE = False
move_history = [] #from car navigation: [Movement, time]
curr_position = np.array([0.0, 0.0])
positions = [[0.0,0.0]]
curr_deg = 0.0
target = np.array([0.0, 0.0])
target_deg = 0.0
target_dist = 0.0

# --- Constants for the Car ---
CAR_SPEED_MPS = 0.5  # Meters per second, must be calibrated
TURN_DURATION_90 = 1.2 # Time needed to complete a 90-degree turn, must be calibrated
TURN_SPEED = 90 / TURN_DURATION_90
SOFTWARE_LATENCY_MS = 50.0  

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

async def update_position():
    global curr_position, curr_deg, positions, SELF_DRIVE
    
    while True:
        await asyncio.sleep(2)
        move_history.clear()
        for i in move_history:
            movement, duration = i
            if movement == "FORWARD":
                rad = np.deg2rad(curr_deg)
                direction_vector = np.array([np.cos(rad), np.sin(rad)])
                distance = CAR_SPEED_MPS * duration
                curr_position += direction_vector * distance
                positions.append(curr_position.tolist())
            elif movement == "BACKWARD":
                rad = np.deg2rad(curr_deg)
                direction_vector = np.array([np.cos(rad), np.sin(rad)])
                distance = CAR_SPEED_MPS * duration
                curr_position -= direction_vector * distance
                positions.append(curr_position.tolist())
            elif movement == "LEFT":
                angle_change = TURN_SPEED * duration
                curr_deg = (curr_deg - angle_change) % 360
            elif movement == "RIGHT":
                angle_change = TURN_SPEED * duration
                curr_deg = (curr_deg + angle_change) % 360

async def calculate_target():
    global curr_position, positions, curr_deg, target, target_deg, target_dist

    while True:
        await asyncio.sleep(2)
        if len(positions) >= 3:
            p1 = positions[-1] 
            p2 = positions[-3] # Use -3 to get some physical separation
            
            x1, y1, r1 = p1
            x2, y2, r2 = p2
            
            # Distance between the two car positions
            d = math.sqrt((x1-x2)**2 + (y1-y2)**2)
            
            # Trilateration logic:
            # If d > r1 + r2: Circles don't touch (too far)
            # If d < |r1 - r2|: One circle inside other (error)
            # If d == 0: Car hasn't moved
            
            if d > 0.1 and d <= (r1 + r2) and d >= abs(r1 - r2):
                # Calculate intersection points
                a = (r1**2 - r2**2 + d**2) / (2*d)
                h = math.sqrt(max(0, r1**2 - a**2))
                
                x2_rel = x2 - x1
                y2_rel = y2 - y1
                
                # Point 2 coordinates relative to Point 1
                x3 = x1 + a * (x2_rel / d)
                y3 = y1 + a * (y2_rel / d)
                
                # Two possible intersection points
                target_x_1 = x3 + h * (y2_rel / d)
                target_y_1 = y3 - h * (x2_rel / d)
                
                target_x_2 = x3 - h * (y2_rel / d)
                target_y_2 = y3 + h * (x2_rel / d)
                
                # Disambiguation:
                # We assume the target is roughly in front of us or use a 3rd point.
                # For simplicity: Use the point closer to our current heading.
                
                # Let's just pick one for now (or average them if they are close)
                target_coords = np.array([target_x_1, target_y_1])
                
                # Calculate Angle to Target relative to Car's Heading
                # Absolute angle to target
                abs_angle = math.atan2(target_coords[1] - curr_position[1], 
                                       target_coords[0] - curr_position[0])
                abs_deg = np.degrees(abs_angle)
                
                # Relative angle (Steering Error)
                # If Car is 90 deg, Target is 100 deg -> Rel is +10 (Right)
                rel_deg = abs_deg - curr_deg
                
                # Normalize to -180 to 180
                target_deg = (rel_deg + 180) % 360 - 180
                
                print(f"TRIANGULATED: RelAngle: {target_deg:.2f}, Coords: {target_coords}")

        await asyncio.sleep(1)

        

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
    global ping_start, calc_list, SELF_DRIVE, JOYSTICK
    
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
                JOYSTICK = None
                print("\nSelf-driving mode activated.\n")
            elif command == "MANUAL":
                SELF_DRIVE = False
                JOYSTICK = None
                print("\nManual mode activated.\n")
            else:
                if SELF_DRIVE == False:
                    JOYSTICK = json.loads(command)
                print(f"\njoystick: {JOYSTICK}\n")
                      
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
                    SELF_DRIVE = False
                
                data_char = server_instance.get_characteristic(DATA_CHAR_UUID)
                if SELF_DRIVE:
                    mode = "AUTO"
                else:
                    mode = "MANUAL"
                message = {
                    "mode": mode,
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
    event_loop.create_task(next_ping())
    event_loop.create_task(send_data())
    event_loop.create_task(get_average_rtt())
    event_loop.create_task(update_position())
    event_loop.create_task(calculate_target())

async def bt_main():
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
    asyncio.run(bt_main())


