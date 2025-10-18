import sys
import threading
from bluetooth import BluetoothSocket, RFCOMM, advertise_service, SERIAL_PORT_CLASS, SERIAL_PORT_PROFILE, PORT_ANY

SERVICE_NAME = "Pi-SPP-Bridge"
# Standard SPP UUID (don’t change unless you know why)
SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"

class SPPServer:
    def __init__(self):
        self.server_sock = BluetoothSocket(RFCOMM)
        self.client_sock = None
        self.client_info = None
        self._rx_thread = None
        self._stop = threading.Event()

    def start(self):
        # Bind to any RFCOMM port and start listening
        self.server_sock.bind(("", PORT_ANY))
        self.server_sock.listen(1)
        port = self.server_sock.getsockname()[1]

        # Advertise so phones see “Serial/TTY” capability
        advertise_service(
            self.server_sock, SERVICE_NAME,
            service_id=SPP_UUID,
            service_classes=[SPP_UUID, SERIAL_PORT_CLASS],
            profiles=[SERIAL_PORT_PROFILE],
        )

        print(f"[SPP] Listening on RFCOMM channel {port}. "
              f"Look for service '{SERVICE_NAME}' on your phone.")
        self.accept_loop()

    def accept_loop(self):
        try:
            while not self._stop.is_set():
                print("[SPP] Waiting for phone to connect…")
                self.client_sock, self.client_info = self.server_sock.accept()
                print(f"[SPP] Connected to {self.client_info}")

                # Start a background thread to read incoming data
                self._rx_thread = threading.Thread(target=self._reader, daemon=True)
                self._rx_thread.start()

                # Simple REPL: anything you type here is sent to the phone
                print("[SPP] Type messages and press ENTER to send. Ctrl+C to quit.")
                while self.client_sock and not self._stop.is_set():
                    try:
                        line = sys.stdin.readline()
                        if not line:
                            break
                        data = line.encode("utf-8")
                        self.client_sock.send(data)
                    except (OSError, BrokenPipeError):
                        print("[SPP] Send failed (client disconnected).")
                        break

                self._cleanup_client()
        except KeyboardInterrupt:
            print("\n[SPP] Stopping…")
        finally:
            self.stop()

    def _reader(self):
        try:
            while not self._stop.is_set():
                data = self.client_sock.recv(1024)
                if not data:
                    print("[SPP] Client closed connection.")
                    break
                # Try to print as UTF-8, fallback to hex
                try:
                    print(f"[PHONE → PI] {data.decode('utf-8').rstrip()}")
                except UnicodeDecodeError:
                    print(f"[PHONE → PI] {data.hex()}")
        except OSError:
            pass  # socket closed
        finally:
            self._cleanup_client()

    def _cleanup_client(self):
        if self.client_sock:
            try:
                self.client_sock.close()
            except OSError:
                pass
        self.client_sock = None
        self.client_info = None
        print("[SPP] Client disconnected. Ready for a new connection.")

    def stop(self):
        self._stop.set()
        self._cleanup_client()
        try:
            self.server_sock.close()
        except OSError:
            pass
        print("[SPP] Server stopped.")

if __name__ == "__main__":
    SPPServer().start()
