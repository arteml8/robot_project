# fusion/test_gnss.py

from gnss_receiver import GNSSReceiver
import time

def main():
    gps = GNSSReceiver()

    while True:
        data = gps.read()
        print(f"Time: {data['timestamp']}, Lat: {data['latitude']}, Lon: {data['longitude']}, "
              f"Speed: {data['speed']:.2f} m/s, Heading: {data['course']}, "
              f"Sats: {data['num_sats']}, Fix: {data['fix_quality']}")
        time.sleep(1)

if __name__ == "__main__":
    main()