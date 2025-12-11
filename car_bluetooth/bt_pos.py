import time
import asyncio
import inspect
from typing import Any, Optional
from bless import (
    BlessServer,
    GATTCharacteristicProperties as GcProps,
    GATTAttributePermissions as GaPerms
)
import json
import numpy as np
import math
from manual_drive.manual_mode import Manual, DriveBase
from initialization.init import get_distance_sensors, get_wheel_motors
from automatic_drive.automatic_drive import auto_drive
import asyncio
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
connection_ready_event: Optional[asyncio.Event] = None

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
SOFTWARE_LATENCY_MS = 53.0  

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


async def wait_for_connection(poll_interval: float = 0.1) -> None:
    """
    Await until the BLE client has connected so callers can safely proceed.
    """
    global connection_ready_event
    while connection_ready_event is None:
        # bt_main hasn't initialized yet; yield so it can start up.
        await asyncio.sleep(poll_interval)
    if is_client_connected():
        connection_ready_event.set()
    await connection_ready_event.wait()


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
    global calc_list, event_loop, target_dist, SOFTWARE_LATENCY_MS, SELF_DRIVE
    
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
            print(f"Average RTT: {avg_rtt:.2f} ms, mode: {SELF_DRIVE}")
            rtt = abs(avg_rtt - SOFTWARE_LATENCY_MS)
            target_dist = rtt / 1_000.0 * 299792458 / 2
            print(f"Estimated Distance: {target_dist:.2f} m")
        
        await asyncio.sleep(0.5)

def get_intersections(p1, p2):
    """ Returns the two intersection points of circles p1 and p2 """
    x1, y1, r1 = p1
    x2, y2, r2 = p2
    d = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    # 1. Check for non-intersecting circles (Too far apart or one inside other)
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        return []
    
    a = (r1**2 - r2**2 + d**2) / (2*d)
    h = math.sqrt(max(0, r1**2 - a**2))
    
    x2_rel = x2 - x1
    y2_rel = y2 - y1
    
    x3 = x1 + a * (x2_rel / d)
    y3 = y1 + a * (y2_rel / d)
    
    pt1 = np.array([x3 + h * (y2_rel / d), y3 - h * (x2_rel / d)])
    pt2 = np.array([x3 - h * (y2_rel / d), y3 + h * (x2_rel / d)])
    
    return [pt1, pt2]

async def update_position():
    global curr_position, curr_deg, positions, target_dist, SELF_DRIVE, CAR_SPEED_MPS, TURN_SPEED
    
    while True:
        # Wait for the next "Move" command batch
        if SELF_DRIVE == True:
            r0 = target_dist
            await asyncio.sleep(2)
            r1 = target_dist
            await asyncio.sleep(2)
            r2 = target_dist
            await asyncio.sleep(2)
            
            move_history_copy = [("FORWARD", 2.0, r0), ("RIGHT", 2.0, r1), ("FORWARD", 2.0, r2)]
            
            print(f"--- Starting Movement Sequence ---")

            for i in move_history_copy:
                movement, duration, dist = i
                
                # 1. Update the Car's Mathematical Position
                if movement == "FORWARD":
                    rad = np.deg2rad(curr_deg)
                    direction_vector = np.array([np.cos(rad), np.sin(rad)])
                    distance = CAR_SPEED_MPS * duration
                    curr_position += direction_vector * distance
                elif movement == "BACKWARD":
                    rad = np.deg2rad(curr_deg)
                    direction_vector = np.array([np.cos(rad), np.sin(rad)])
                    distance = CAR_SPEED_MPS * duration
                    curr_position -= direction_vector * distance
                elif movement == "LEFT":
                    angle_change = TURN_SPEED * duration
                    curr_deg = (curr_deg - angle_change) % 360
                elif movement == "RIGHT":
                    angle_change = TURN_SPEED * duration
                    curr_deg = (curr_deg + angle_change) % 360
                
                positions.append([curr_position[0], curr_position[1], dist])
                
                print(f"Recorded: Pos={curr_position}, Dist={dist:.2f}")
                await asyncio.sleep(0.1)

async def calculate_target():
    global positions, curr_position, curr_deg, target_deg, target
    
    while True:
        await asyncio.sleep(1)
        
        if len(positions) >= 3:
            # Get 3 diverse points
            c1 = positions[-1] 
            c2 = positions[-2] 
            c3 = positions[-3]
            
            triangle_points = []
            
            # --- 1. Intersect C1 & C2 ---
            pts_12 = get_intersections(c1, c2)
            if pts_12:
                # Disambiguate: Which point is closer to the edge of C3?
                # We calculate |distance_to_center_C3 - radius_C3|
                err0 = abs(math.hypot(pts_12[0][0]-c3[0], pts_12[0][1]-c3[1]) - c3[2])
                err1 = abs(math.hypot(pts_12[1][0]-c3[0], pts_12[1][1]-c3[1]) - c3[2])
                # Keep the one with lower error
                triangle_points.append(pts_12[0] if err0 < err1 else pts_12[1])

            # --- 2. Intersect C2 & C3 ---
            pts_23 = get_intersections(c2, c3)
            if pts_23:
                # Disambiguate using C1
                err0 = abs(math.hypot(pts_23[0][0]-c1[0], pts_23[0][1]-c1[1]) - c1[2])
                err1 = abs(math.hypot(pts_23[1][0]-c1[0], pts_23[1][1]-c1[1]) - c1[2])
                triangle_points.append(pts_23[0] if err0 < err1 else pts_23[1])

            # --- 3. Intersect C1 & C3 ---
            pts_13 = get_intersections(c1, c3)
            if pts_13:
                # Disambiguate using C2
                err0 = abs(math.hypot(pts_13[0][0]-c2[0], pts_13[0][1]-c2[1]) - c2[2])
                err1 = abs(math.hypot(pts_13[1][0]-c2[0], pts_13[1][1]-c2[1]) - c2[2])
                triangle_points.append(pts_13[0] if err0 < err1 else pts_13[1])
            
            # --- 4. Average the Triangle ---
            if len(triangle_points) > 0:
                # Calculate Centroid
                sum_x = sum(p[0] for p in triangle_points)
                sum_y = sum(p[1] for p in triangle_points)
                
                avg_x = sum_x / len(triangle_points)
                avg_y = sum_y / len(triangle_points)
                
                target = np.array([avg_x, avg_y])
                
                # Calc Angle
                abs_angle = math.atan2(avg_y - curr_position[1], avg_x - curr_position[0])
                abs_deg = np.degrees(abs_angle)
                rel_deg = abs_deg - curr_deg
                target_deg = (rel_deg + 180) % 360 - 180
                
                print(f"TRIANGLE CENTROID: {target}")
            else:
                print("Triangulation failed: No circles intersected.")

async def next_ping():
    global server_instance, ping_start, SERVICE_UUID, PING_CHAR_UUID
    
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
    global ping_start, calc_list, SELF_DRIVE, JOYSTICK, PONG_CHAR_UUID, COMMAND_CHAR_UUID
    
    if characteristic.uuid == PONG_CHAR_UUID:
        # Received a PING from the app.
        
        rtt = (time.monotonic() - ping_start) * 1000
        if ping_start > 0:
            # print(f"RTT: {rtt:.2f} ms")
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
    global server_instance, target_dist, target_deg, SELF_DRIVE, SERVICE_UUID, DATA_CHAR_UUID

    while True:
        if is_client_connected():
            try:
                # Send distance and direction in regualar intervals
                if target_dist < 2.0 and target_dist != 0:
                    SELF_DRIVE = False
                    print("\nTarget too close, switching to MANUAL mode.\n")
                
                data_char = server_instance.get_characteristic(DATA_CHAR_UUID)
                if SELF_DRIVE == True:
                    mode = "AUTO"
                else:
                    mode = "MANUAL"

                message = {
                    "mode": mode,
                    "distance": round(target_dist, 2),
                    "direction": round(target_deg, 2),
                    "target": target.tolist()
                }
                message = json.dumps(message)
                data_char.value = message.encode('utf-8')
                
                await update_characteristic(SERVICE_UUID, DATA_CHAR_UUID)
            except Exception as e:
                print(f"Error sending data: {e}")
        await asyncio.sleep(1.0)

async def on_connect():
    global server_instance, connection_ready_event
    while not is_client_connected():
        await asyncio.sleep(1.0)
    print("Client connected!")
    if connection_ready_event and not connection_ready_event.is_set():
        connection_ready_event.set()
    await asyncio.sleep(2.0)
    event_loop.create_task(next_ping())
    event_loop.create_task(send_data())
    event_loop.create_task(get_average_rtt())
    event_loop.create_task(update_position())
    event_loop.create_task(calculate_target())
    # event_loop.create_task(main_loop())

DEFAULT = {"x": "0.0", "y": "0.0"}

async def main_loop():
    global DEFAULT, JOYSTICK, SELF_DRIVE
    left_motor, right_motor = get_wheel_motors()
    # sonar_left, sonar_right = get_distance_sensors()
    manual_mode = Manual(left_motor, right_motor)

    while True:
        # print("mode:", SELF_DRIVE)
        if SELF_DRIVE:
            # auto_drive(left_motor, right_motor, sonar_left, sonar_right)
            await asyncio.sleep(0.1)
        else:
            if JOYSTICK == None:
                manual_mode.joystick(DEFAULT)
            else:
                print("joystick input", JOYSTICK)
                manual_mode.joystick(JOYSTICK)
                await asyncio.sleep(0.05)
        await asyncio.sleep(0.1)

async def bt_main():
    global server_instance, event_loop, connection_ready_event, SERVICE_UUID, PING_CHAR_UUID, PONG_CHAR_UUID, COMMAND_CHAR_UUID, DATA_CHAR_UUID

    event_loop = asyncio.get_running_loop()
    connection_ready_event = asyncio.Event()

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

