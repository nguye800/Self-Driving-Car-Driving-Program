import time
import json
import bluetooth  # pip install pybluez

from dc_motor import Motor  


class Manual:
    """
    Manual drive that listens for joystick coordinates (x, y)
    over Bluetooth and drives two Motor instances (left/right).

    Expected packet formats from the remote side:
      - "0.3,-0.8\n"
      - '{"x": 0.3, "y": -0.8}\n'
      - "x=0.3,y=-0.8\n"
    """

    def __init__(self, left_motor: Motor, right_motor: Motor, bt_uuid=None):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.bt_uuid = bt_uuid or "94f39d29-7d6d-437d-973b-fba39e49d4ee"
        self.bt_server_sock = None
        self.bt_client_sock = None

    # ------------- Motor helpers -------------

    def clamp(self, v, lo=-1.0, hi=1.0):
        return max(lo, min(hi, v))

    def deadzone(self, v, dz=0.06):
        return 0.0 if abs(v) < dz else v

    def mix_cartesian(self, x, y):
        """
        Arcade/Tank mixing using Cartesian joystick:
        y = forward/back
        x = turn left/right
        Returns (left, right) in [-1..1]
        """
        left = self.clamp(y + x)
        right = self.clamp(y - x)
        return left, right

    def apply_signed(self, l, r, stop_thresh=0.02):
        """
        Apply signed speeds in [-1,1] directly to Motor objects.
        If magnitude is very small, we coast that motor.
        """
        # LEFT MOTOR
        if abs(l) < stop_thresh:
            self.left_motor.stop(brake_mode=False)
        else:
            speed = abs(l)
            if l >= 0:
                self.left_motor.forward(speed)
            else:
                self.left_motor.backward(speed)

        # RIGHT MOTOR
        if abs(r) < stop_thresh:
            self.right_motor.stop(brake_mode=False)
        else:
            speed = abs(r)
            if r >= 0:
                self.right_motor.forward(speed)
            else:
                self.right_motor.backward(speed)

    def joystick_step(self, x, y):
        """Single control step using one pair of joystick values."""
        # Apply deadzone & clamp
        x = self.clamp(self.deadzone(x))
        y = self.clamp(self.deadzone(y))

        # Mix to tank drive
        l, r = self.mix_cartesian(x, y)

        # Drive motors
        self.apply_signed(l, r)

    # ------------- Bluetooth helpers -------------

    def start_bluetooth_server(self, service_name="RobotDrive"):
        """
        Start an RFCOMM Bluetooth server and wait for a single client
        (e.g., your backend that gets data from the website).
        """
        self.bt_server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.bt_server_sock.bind(("", bluetooth.PORT_ANY))
        self.bt_server_sock.listen(1)

        port = self.bt_server_sock.getsockname()[1]

        bluetooth.advertise_service(
            self.bt_server_sock,
            service_name,
            service_id=self.bt_uuid,
            service_classes=[self.bt_uuid, bluetooth.SERIAL_PORT_CLASS],
            profiles=[bluetooth.SERIAL_PORT_PROFILE],
        )

        print(f"[BT] Waiting for Bluetooth connection on RFCOMM channel {port}...")
        self.bt_client_sock, client_info = self.bt_server_sock.accept()
        print(f"[BT] Accepted connection from {client_info}")

    def parse_coordinates(self, text):
        """
        Parse coordinate strings into floats (x, y).

        Supported formats:
          - "0.3,-0.8"
          - '{"x": 0.3, "y": -0.8}'
          - "x=0.3,y=-0.8"
        """
        text = text.strip()
        if not text:
            raise ValueError("Empty packet")

        # JSON format
        if text.startswith("{"):
            obj = json.loads(text)
            return float(obj["x"]), float(obj["y"])

        # Simple "x,y"
        if "," in text and "=" not in text:
            xs, ys = text.split(",", 1)
            return float(xs), float(ys)

        # "x=0.3,y=-0.8" or similar
        if "x" in text and "y" in text:
            parts = text.replace(" ", "").split(",")
            vals = {}
            for p in parts:
                if "=" in p:
                    k, v = p.split("=", 1)
                    vals[k] = float(v)
            if "x" in vals and "y" in vals:
                return vals["x"], vals["y"]

        # Fallback: try just one float, treat as forward, no turning
        try:
            y = float(text)
            return 0.0, y
        except Exception:
            raise ValueError(f"Unrecognized coordinate format: {text!r}")

    def recv_coordinates(self):
        """
        Receive one packet from the Bluetooth client and parse it
        into (x, y). Returns None if the connection is closed.
        """
        if self.bt_client_sock is None:
            return None

        try:
            data = self.bt_client_sock.recv(1024)
        except OSError as e:
            print(f"[BT] recv error: {e}")
            return None

        if not data:
            # Connection closed
            print("[BT] Client disconnected")
            return None

        text = data.decode("utf-8", errors="ignore").strip()
        # You might receive multiple lines in one packet; handle line by line
        lines = text.splitlines()
        last_xy = None
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                x, y = self.parse_coordinates(line)
                last_xy = (x, y)
            except ValueError as e:
                print(f"[BT] Failed to parse '{line}': {e}")

        return last_xy

    def close_bluetooth(self):
        """Clean up Bluetooth sockets."""
        if self.bt_client_sock:
            try:
                self.bt_client_sock.close()
            except OSError:
                pass
            self.bt_client_sock = None

        if self.bt_server_sock:
            try:
                self.bt_server_sock.close()
            except OSError:
                pass
            self.bt_server_sock = None

    # ------------- Main loop -------------

    def run_from_bluetooth(self):
        """
        Main control loop: waits for BT client, then continuously
        reads (x, y) from Bluetooth and drives the motors.
        """
        try:
            self.left_motor.enable()
            self.right_motor.enable()

            if self.bt_client_sock is None:
                self.start_bluetooth_server()

            while True:
                coords = self.recv_coordinates()
                if coords is None:
                    # Client disconnected or error
                    break

                x, y = coords
                # Debug print: comment out if too noisy
                print(f"[BT] Received coords: x={x:.3f}, y={y:.3f}")

                self.joystick_step(x, y)
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n[BT] Manual control interrupted by user")

        finally:
            # Stop and disable motors
            self.left_motor.stop(brake_mode=False)
            self.right_motor.stop(brake_mode=False)
            self.left_motor.disable()
            self.right_motor.disable()
            self.close_bluetooth()
            print("[BT] Cleaned up and stopped")
