# fusion/gnss_receiver.py

import serial
import pynmea2
import time

class GNSSReceiver:
    def __init__(self, port="/dev/ttyTHS1", baudrate=9600, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.last_data = {
            "timestamp": None,
            "latitude": None,
            "longitude": None,
            "speed": None,
            "course": None,
            "num_sats": None,
            "fix_quality": None
        }

    def read(self):
        while True:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line.startswith("$"):
                    continue

                msg = pynmea2.parse(line)

                if isinstance(msg, pynmea2.GGA):
                    self.last_data.update({
                        "timestamp": msg.timestamp,
                        "latitude": self._convert_to_decimal(msg.lat, msg.lat_dir),
                        "longitude": self._convert_to_decimal(msg.lon, msg.lon_dir),
                        "fix_quality": msg.gps_qual,
                        "num_sats": msg.num_sats
                    })

                elif isinstance(msg, pynmea2.RMC):
                    self.last_data.update({
                        "timestamp": msg.timestamp,
                        "latitude": self._convert_to_decimal(msg.lat, msg.lat_dir),
                        "longitude": self._convert_to_decimal(msg.lon, msg.lon_dir),
                        "speed": float(msg.spd_over_grnd) * 0.51444,  # knots to m/s
                        "course": float(msg.true_course) if msg.true_course else None
                    })

                return self.last_data

            except pynmea2.ParseError:
                continue
            except UnicodeDecodeError:
                continue

    def _convert_to_decimal(self, raw_value, direction):
        if not raw_value:
            return None
        degrees = float(raw_value[:2])
        minutes = float(raw_value[2:])
        decimal = degrees + minutes / 60
        if direction in ["S", "W"]:
            decimal = -decimal
        return decimal