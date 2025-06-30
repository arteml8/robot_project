import asyncio
from bleak import BleakClient, BleakScanner

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32 → Jetson
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Jetson → ESP32

class RobotBLEClient:
    def __init__(self, device_name="RobotBridge"):
        self.device_name = device_name
        self.client = None
        self.tx_char = None

    async def connect(self):
        devices = await BleakScanner.discover()
        for d in devices:
            if UART_SERVICE_UUID in d.metadata.get('uuids'):
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
        print(f"🔄 Received: {data.decode().strip()}")

    async def send(self, command: str):
        if self.client and self.client.is_connected and self.tx_char:
            await self.client.write_gatt_char(self.tx_char, command.encode())
            print(f"➡️ Sent: {command.strip()}")

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("🔌 Disconnected.")