from i2c_sensor import I2CSensor

class GyroscopeITG3200(I2CSensor):
    def __init__(self, bus_id=1, address=0x68):
        scale_factor = 14.375
        super().__init__(bus_id, address, name = 'ITG3200', msb_first=True, scale_factor)
        self._initialize()

    def _initialize(self):
        self.bus.write_byte_data(self.address, 0x3E, 0x00)  # Wake up
        self.bus.write_byte_data(self.address, 0x15, 0x07)  # Sample rate divider
        self.bus.write_byte_data(self.address, 0x16, 0x18)  # ±2000°/s, 256Hz low-pass

    def read_raw(self):
        return self.read_raw_data(0x1D)
