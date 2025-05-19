from i2c_sensor import I2CSensor

class GyroscopeITG3200(I2CSensor):
    def __init__(self, bus_id=1, address=0x68):
        super().__init__(bus_id, address, name = 'ITG3200', msb_first=True)
        self._initialize()

    def _initialize(self):
        self.bus.write_byte_data(self.address, 0x3E, 0x00)
        self.bus.write_byte_data(self.address, 0x15, 0x07)
        self.bus.write_byte_data(self.address, 0x16, 0x18)

        # self.write_register(0x3E, 0x00)  # Wake up
        # self.write_register(0x15, 0x07)  # Sample rate divider
        # self.write_register(0x16, 0x18)  # ±2000°/s, 256Hz low-pass

    def read_raw(self):
        return self.read_raw_data(self.address, 0x1D)

    def read_dps(self):
        raw = self.read_raw()
        return tuple(r / 14.375 for r in raw)  # Convert to deg/s