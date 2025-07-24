#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from smbus2 import SMBus
import math

HMC5883L_ADDR = 0x1E
I2C_BUS = 1

def init_compass(bus):
    try:
        bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)  # Config A
        bus.write_byte_data(HMC5883L_ADDR, 0x01, 0x20)  # Config B
        bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)  # Continuous mode
    except Exception as e:
        rospy.logerr(f"Compass init failed: {e}")

def twos_comp(val):
    return val - 65536 if val > 32767 else val

def read_compass(bus):
    try:
        data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
        x = twos_comp((data[0] << 8) | data[1])
        z = twos_comp((data[2] << 8) | data[3])
        y = twos_comp((data[4] << 8) | data[5])
        return x, y, z
    except Exception as e:
        rospy.logwarn(f"Compass read failed: {e}")
        return 0, 0, 0

def heading(x, y):
    angle = math.atan2(y, x)
    angle = math.degrees(angle)
    return angle + 360 if angle < 0 else angle

def compass_node():
    rospy.init_node('hmc5883l_node')
    pub = rospy.Publisher('heading', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    with SMBus(I2C_BUS) as bus:
        init_compass(bus)
        rospy.loginfo("Compass node running...")
        while not rospy.is_shutdown():
            x, y, z = read_compass(bus)
            angle = heading(x, y)
            rospy.loginfo(f"Heading: {angle:.2f}°")
            pub.publish(angle)
            rate.sleep()

if __name__ == '__main__':
    try:
        compass_node()
    except rospy.ROSInterruptException:
        pass

