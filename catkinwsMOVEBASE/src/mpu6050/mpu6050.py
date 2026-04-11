#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import smbus2
import math

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# I2C bus and device address
I2C_BUS = 0
DEVICE_ADDRESS = 0x68

# Sensitivity factors
ACCEL_SCALE = 16384.0    
GYRO_SCALE = 131.0          

def read_raw_data(bus, addr):
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def imu_node():
    rospy.init_node('mpu6050_imu_node', anonymous=False)
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize I2C bus
    bus = smbus2.SMBus(I2C_BUS)
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0)  # Wake up sensor

    rospy.loginfo("MPU6050 IMU publishing to /imu/data_raw")

    while not rospy.is_shutdown():
        # Read and convert data
        accel_x = read_raw_data(bus, ACCEL_XOUT_H) / ACCEL_SCALE * 9.80665
        accel_y = read_raw_data(bus, ACCEL_YOUT_H) / ACCEL_SCALE * 9.80665
        accel_z = read_raw_data(bus, ACCEL_ZOUT_H) / ACCEL_SCALE * 9.80665

        gyro_x = math.radians(read_raw_data(bus, GYRO_XOUT_H) / GYRO_SCALE)
        gyro_y = math.radians(read_raw_data(bus, GYRO_YOUT_H) / GYRO_SCALE)
        gyro_z = math.radians(read_raw_data(bus, GYRO_ZOUT_H) / GYRO_SCALE)

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # Fill linear acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        # Fill angular velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # (Optionally leave orientation fields as 0.0)
        imu_msg.orientation_covariance[0] = -1  # Indicates orientation not provided

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_node()
    except rospy.ROSInterruptException:
        pass

