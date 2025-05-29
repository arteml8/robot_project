# fusion/test_gnss.py

from fusion.gnss_receiver import GNSSReceiver
import time

def fmt(val, precision=2, default="N/A"):
    """Format float with fixed precision or return default if None."""
    try:
        return f"{val:.{precision}f}"
    except (TypeError, ValueError):
        return default

def main():
    gps = GNSSReceiver()

    while True:
        data = gps.read()
        print(f"Time: {data['timestamp']}, "
              f"Lat: {fmt(data['latitude'])}, "
              f"Lon: {fmt(data['longitude'])}, "
              f"Speed: {fmt(data['speed'])} m/s, "
              f"Heading: {fmt(data['course'])}, "
              f"Sats: {data['num_sats']}, "
              f"Fix: {data['fix_quality']}")
        time.sleep(1)

if __name__ == "__main__":
    main()