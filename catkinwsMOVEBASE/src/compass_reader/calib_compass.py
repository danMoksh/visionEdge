#!/usr/bin/env python

import time
import math
from smbus2 import SMBus
import sys

# HMC5883L I2C address
HMC5883L_ADDR = 0x1E
I2C_BUS = 1 # Change if your I2C bus is different

def init_compass(bus):
    """Initializes the compass for continuous measurement."""
    try:
        bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00) # Continuous-Measurement Mode
        return True
    except IOError as e:
        print "I/O Error: unable to write to I2C bus. Check connection."
        print e
        return False

def read_raw_data(bus):
    """Reads raw x, y, z data from the compass."""
    try:
        data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
        
        x_raw = (data[0] << 8) | data[1]
        z_raw = (data[2] << 8) | data[3] # Z is read between X and Y
        y_raw = (data[4] << 8) | data[5]

        # Apply two's complement
        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw

        return x, y
    except IOError:
        return None, None

if __name__ == '__main__':
    try:
        bus = SMBus(I2C_BUS)
    except IOError:
        print "I2C Bus not found. Exiting."
        sys.exit(1)

    if not init_compass(bus):
        sys.exit(1)

    # Initialize min/max variables to capture the range of sensor values
    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')

    print "--- Compass Calibration Utility ---"
    print "Slowly rotate your rover in a full circle (360 degrees) on a flat surface."
    print "Try to keep it level. The script will find the magnetic field boundaries."
    print "Press Ctrl+C when you are finished."
    print "------------------------------------"
    
    try:
        while True:
            x, y = read_raw_data(bus)
            if x is None or y is None:
                print "\rCould not read sensor data. Check connection.",
                time.sleep(1)
                continue

            # Update the min and max values
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)

            # Display live data on a single line
            sys.stdout.write("\rRaw X: %-6d Y: %-6d | Min/Max X: %-6d/%-6d Y: %-6d/%-6d" % (x, y, min_x, max_x, min_y, max_y))
            sys.stdout.flush()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print "\n\n--- Calibration Finished ---"
        # Calculate the offsets
        offset_x = (max_x + min_x) / 2.0
        offset_y = (max_y + min_y) / 2.0
        
        print "Calculated Offsets:"
        print "X offset: %.2f" % offset_x
        print "Y offset: %.2f" % offset_y
        print "\nUse these values to run your compass node."
        print "------------------------------------"
        print "Example command to copy:"
        print "rosrun your_package_name sensor_hub.py compass _x_offset:=%.2f _y_offset:=%.2f" % (offset_x, offset_y)
        print "------------------------------------"
        
    finally:
        bus.close()
