import serial

# Adjust if you're using a different port (e.g., /dev/ttyUSB0)
ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)

while True:
    line = ser.readline().decode('utf-8', errors='ignore')
    if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
        print(line.strip())