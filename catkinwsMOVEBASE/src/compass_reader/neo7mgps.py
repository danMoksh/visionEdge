#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from smbus2 import SMBus
import serial
import pynmea2
import math

# Constants
HMC5883L_ADDR = 0x1E
I2C_BUS = 1  # Jetson Nano usually uses I2C bus 1

def twos_comp(val):
    return val - 65536 if val > 32767 else val

def init_compass(bus):
    # Initialize HMC5883L registers
    bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)  # Configuration Register A
    bus.write_byte_data(HMC5883L_ADDR, 0x01, 0x20)  # Configuration Register B
    bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)  # Mode Register: continuous

def read_compass(bus):
    data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
    x = twos_comp((data[0] << 8) | data[1])
    z = twos_comp((data[2] << 8) | data[3])
    y = twos_comp((data[4] << 8) | data[5])
    heading_rad = math.atan2(y, x)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

def neo7m_node():
    rospy.init_node('neo7m_node')
    gps_pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
    compass_pub = rospy.Publisher('/heading', Float32, queue_size=10)

    port = rospy.get_param('~port', '/dev/ttyTHS1')
    baud = rospy.get_param('~baudrate', 9600)

    try:
        ser = serial.Serial(port, baud, timeout=1)
        rospy.loginfo(f"GPS Serial port {port} opened at {baud} baud.")
    except serial.SerialException as e:
        rospy.logerr("Serial port error: %s", e)
        return

    try:
        bus = SMBus(I2C_BUS)
        init_compass(bus)
        rospy.loginfo("Compass initialized on I2C bus %d", I2C_BUS)
    except Exception as e:
        rospy.logerr("I2C error: %s", e)
        return

    rate = rospy.Rate(5)
    fix_msg = NavSatFix()

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                fix_msg.latitude = msg.latitude or 0.0
                fix_msg.longitude = msg.longitude or 0.0
                fix_msg.altitude = float(msg.altitude) if msg.altitude else 0.0
                fix_msg.header.stamp = rospy.Time.now()
                fix_msg.header.frame_id = "gps"
                gps_pub.publish(fix_msg)

            heading = read_compass(bus)
            compass_pub.publish(heading)

        except pynmea2.nmea.ChecksumError:
            continue
        except Exception as e:
            rospy.logwarn("Read error: %s", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        neo7m_node()
    except rospy.ROSInterruptException:
        pass

