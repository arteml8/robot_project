import asyncio
from bleak import BleakClient, BleakScanner

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
MAC = "64:E8:33:B7:71:56"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

class RobotBLEClient:
    def __init__(self, device_name="RobotBridge"):
        self.device_name = device_name
        self.client = None
        self.tx_char = None
        self.on_encoder_update = None
        self._write_lock = asyncio.Lock()

    async def connect(self):
        devices = await BleakScanner.discover()
        for d in devices:
            if MAC in d.address:
                print(f"🔗 Connecting to {d.name} ({d.address})")
                self.client = BleakClient(d.address)
                await self.client.connect()
                await self._discover_services()
                return True
        print("❌ Device not found.")
        return False

    async def _discover_services(self):
        for service in self.client.services:
            if UART_SERVICE_UUID in str(service.uuid):
                for char in service.characteristics:
                    if char.uuid == TX_UUID:
                        await self.client.start_notify(char.uuid, self._notification_handler)
                    if char.uuid == RX_UUID:
                        self.tx_char = char

    def _notification_handler(self, sender, data):
        message = data.decode(errors="ignore").strip()
        if message.startswith("ENC:"):
            try:
                tick_values = list(map(int, message[4:].split(",")))
                if len(tick_values) == 4 and self.on_encoder_update:
                    self.on_encoder_update(tick_values)
            except ValueError:
                print("⚠️ Invalid encoder data")

    async def poll_encoders(self, interval_ms=200):
        while self.client and self.client.is_connected:
            await self.send("CMD:GET_ENCODERS\n")
            await asyncio.sleep(interval_ms / 1000)

    async def send(self, command: str):
        if self.client and self.client.is_connected and self.tx_char:
            async with self._write_lock:
                await self.client.write_gatt_char(self.tx_char, command.encode())
        else:
            print("⚠️ BLE not connected; cannot send command.")

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("🔌 Disconnected.")