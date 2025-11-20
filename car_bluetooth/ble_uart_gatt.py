import sys, array, threading, time
from gi.repository import GLib
import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service

BLUEZ_SERVICE_NAME = 'org.bluez'
ADAPTER_IFACE      = 'org.bluez.Adapter1'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
LE_ADVERTISING_MGR = 'org.bluez.LEAdvertisingManager1'

GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE    = 'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE    = 'org.bluez.GattDescriptor1'
LE_ADV_IFACE       = 'org.bluez.LEAdvertisement1'

NUS_SERVICE_UUID   = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX_UUID        = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'  # Write
NUS_TX_UUID        = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'  # Notify

MAIN_LOOP = None

def find_adapter(bus):
    obj = bus.get_object(BLUEZ_SERVICE_NAME, '/')
    mgr = dbus.Interface(obj, 'org.freedesktop.DBus.ObjectManager')
    for path, ifaces in mgr.GetManagedObjects().items():
        if ADAPTER_IFACE in ifaces:
            return path
    raise RuntimeError("Bluetooth adapter not found")

class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = '/org/example/app'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self): return dbus.ObjectPath(self.path)
    def add_service(self, service): self.services.append(service)

    @dbus.service.method('org.freedesktop.DBus.ObjectManager', out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()
        return response

class Service(dbus.service.Object):
    PATH_BASE = '/org/example/service'
    def __init__(self, bus, index, uuid, primary=True):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                'UUID': self.uuid,
                'Primary': self.primary,
                'Includes': dbus.Array([], signature='o')
            }
        }

    def get_path(self): return dbus.ObjectPath(self.path)
    def add_characteristic(self, chrc): self.characteristics.append(chrc)
    def get_characteristics(self): return self.characteristics

class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.get_path() + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.service = service
        self.descriptors = []
        self.notifying = False
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': dbus.Array(self.flags, signature='s'),
                'Descriptors': dbus.Array([d.get_path() for d in self.descriptors], signature='o')
            }
        }

    def get_path(self): return dbus.ObjectPath(self.path)
    def add_descriptor(self, desc): self.descriptors.append(desc)
    def get_descriptors(self): return self.descriptors

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):  # optional
        return dbus.Array([], signature='y')

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='a{sv}')
    def StartNotify(self, options):
        self.notifying = True

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='a{sv}')
    def StopNotify(self, options):
        self.notifying = False

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        # Default: print what we received
        try:
            data = bytes(bytearray(value)).decode('utf-8', errors='replace')
            print(f"[iPhone → Pi] {data}")
        except Exception as e:
            print(f"WriteValue decode error: {e}")

class NUS(Service):
    def __init__(self, bus, index=0):
        super().__init__(bus, index, NUS_SERVICE_UUID, primary=True)
        self.tx = NUS_TX_Char(bus, 0, self)
        self.rx = NUS_RX_Char(bus, 1, self)
        self.add_characteristic(self.tx)
        self.add_characteristic(self.rx)

class NUS_TX_Char(Characteristic):
    # Notify characteristic (Pi -> phone)
    def __init__(self, bus, index, service):
        super().__init__(bus, index, NUS_TX_UUID, ['notify'], service)

    def send(self, text: str):
        if not self.notifying:
            return
        payload = dbus.Array([dbus.Byte(b) for b in text.encode('utf-8')], signature='y')
        # Emit PropertiesChanged to push a notification
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': payload}, [])

    @dbus.service.signal(dbus_interface='org.freedesktop.DBus.Properties', signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated): pass

class NUS_RX_Char(Characteristic):
    # Write characteristic (phone -> Pi)
    def __init__(self, bus, index, service):
        super().__init__(bus, index, NUS_RX_UUID, ['write-without-response','write'], service)

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        msg = bytes(bytearray(value)).decode('utf-8', errors='replace')
        print(f"[iPhone → Pi] {msg}")
        # Echo back via TX (optional demo)
        self.service.tx.send(f"Echo: {msg}")

class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/example/advertisement'
    def __init__(self, bus, index, ad_type, svc_uuids):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = ad_type
        self.service_uuids = svc_uuids
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            LE_ADV_IFACE: {
                'Type': self.ad_type,
                'ServiceUUIDs': dbus.Array(self.service_uuids, signature='s'),
                'LocalName': dbus.String('Pi-BLE-UART'),
                'Includes': dbus.Array(['tx-power'], signature='s'),
                'Appearance': dbus.UInt16(0),
                'Discoverable': dbus.Boolean(True),
            }
        }

    def get_path(self): return dbus.ObjectPath(self.path)

    @dbus.service.method('org.freedesktop.DBus.Properties', in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface): return self.get_properties()[interface]

    @dbus.service.method(LE_ADV_IFACE)
    def Release(self): pass

def main():
    global MAIN_LOOP
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_path = find_adapter(bus)
    adapter = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), ADAPTER_IFACE)
    # Make sure adapter is powered
    adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), 'org.freedesktop.DBus.Properties')
    if not adapter_props.Get(ADAPTER_IFACE, 'Powered'):
        adapter_props.Set(ADAPTER_IFACE, 'Powered', dbus.Boolean(True))

    # Register GATT application
    app = Application(bus)
    nus = NUS(bus)
    app.add_service(nus)

    gatt_mgr = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), GATT_MANAGER_IFACE)
    adv_mgr  = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), LE_ADVERTISING_MGR)

    adv = Advertisement(bus, 0, 'peripheral', [NUS_SERVICE_UUID])

    print("[BLE] Registering GATT app…")
    gatt_mgr.RegisterApplication(app.get_path(), {},
        reply_handler=lambda: print("[BLE] GATT app registered"),
        error_handler=lambda e: print(f"[BLE] GATT register error: {e}")
    )

    print("[BLE] Registering advertisement…")
    adv_mgr.RegisterAdvertisement(adv.get_path(), {},
        reply_handler=lambda: print("[BLE] Advertising started"),
        error_handler=lambda e: print(f"[BLE] Adv register error: {e}")
    )

    # Periodically send a heartbeat to connected clients (if notifications enabled)
    def heartbeat():
        try:
            nus.tx.send("hello from Pi\n")
        except Exception:
            pass
        return True  # keep timer

    GLib.timeout_add_seconds(10, heartbeat)

    MAIN_LOOP = GLib.MainLoop()
    try:
        print("[BLE] Ready. Open nRF Connect on iPhone, find 'Pi-BLE-UART', connect.")
        print("      In the service, enable Notify on TX, and Write to RX.")
        MAIN_LOOP.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            adv_mgr.UnregisterAdvertisement(adv.get_path())
        except Exception:
            pass
        print("\n[BLE] Stopped.")

if __name__ == '__main__':
    main()
