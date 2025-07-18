import serial
import time

# Update with your serial port
port = "/dev/tty.RNBT-0A0B"
baud = 115200

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)  # Allow time for connection

# Send a drive time command
ser.write(b"CMD:DRIVE_TIME:0.2,0.0,0.0,2.0\n")
print("Command sent!")

# Optional: read response
while ser.in_waiting:
    print(ser.readline().decode().strip())

ser.close()