from rplidar import RPLidar

PORT_NAME = '/dev/tty.usbserial-2140'  # Change if needed

lidar = RPLidar(PORT_NAME)
info = lidar.get_info()
print('LIDAR info:', info)

health = lidar.get_health()
print('LIDAR health:', health)

for i, scan in enumerate(lidar.iter_scans()):
    print(scan)
    if i > 10:
        break

lidar.stop()
lidar.disconnect()