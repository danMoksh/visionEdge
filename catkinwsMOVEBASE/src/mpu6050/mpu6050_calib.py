#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import smbus2
import math
import time

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# I2C bus and device address
I2C_BUS = 0  # Change to 1 if you are using I2C bus 1
DEVICE_ADDRESS = 0x68

# Sensitivity factors from datasheet
ACCEL_SCALE = 16384.0  # LSB/g for +/- 2g range
GYRO_SCALE = 131.0     # LSB/dps for +/- 250 dps range

def read_raw_data(bus, addr):
    """Reads a 16-bit value from the I2C bus."""
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
    value = (high << 8) | low
    # Apply two's complement
    if value > 32767:
        value -= 65536
    return value

def calibrate_imu(bus, samples=200):
    """
    Reads the sensor for a few seconds while it is stationary
    to determine the gyro and accelerometer biases.
    """
    rospy.loginfo("Calibrating IMU... Do not move the rover!")
    
    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0
    gyro_x_offset = 0
    gyro_y_offset = 0
    gyro_z_offset = 0

    for _ in range(samples):
        accel_x_offset += read_raw_data(bus, ACCEL_XOUT_H)
        accel_y_offset += read_raw_data(bus, ACCEL_YOUT_H)
        accel_z_offset += read_raw_data(bus, ACCEL_ZOUT_H)
        gyro_x_offset += read_raw_data(bus, GYRO_XOUT_H)
        gyro_y_offset += read_raw_data(bus, GYRO_YOUT_H)
        gyro_z_offset += read_raw_data(bus, GYRO_ZOUT_H)
        time.sleep(0.01)

    # Calculate average offsets
    offsets = {
        'accel_x': accel_x_offset / float(samples),
        'accel_y': accel_y_offset / float(samples),
        'accel_z': accel_z_offset / float(samples) - ACCEL_SCALE, # Z should be 1g (16384) when stationary
        'gyro_x': gyro_x_offset / float(samples),
        'gyro_y': gyro_y_offset / float(samples),
        'gyro_z': gyro_z_offset / float(samples)
    }
    
    rospy.loginfo("Calibration complete.")
    rospy.loginfo("Offsets: %s", offsets)
    return offsets

def imu_node():
    rospy.init_node('mpu6050_imu_node')
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz is a good rate for an IMU

    # Initialize I2C bus
    try:
        bus = smbus2.SMBus(I2C_BUS)
        bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0)  # Wake up sensor
    except IOError as e:
        rospy.logerr("Failed to initialize I2C bus %d: %s", I2C_BUS, e)
        return

    # --- Run Calibration ---
    offsets = calibrate_imu(bus)

    rospy.loginfo("MPU6050 IMU publishing to /imu/data_raw")

    while not rospy.is_shutdown():
        # Read raw data
        raw_ax = read_raw_data(bus, ACCEL_XOUT_H)
        raw_ay = read_raw_data(bus, ACCEL_YOUT_H)
        raw_az = read_raw_data(bus, ACCEL_ZOUT_H)
        raw_gx = read_raw_data(bus, GYRO_XOUT_H)
        raw_gy = read_raw_data(bus, GYRO_YOUT_H)
        raw_gz = read_raw_data(bus, GYRO_ZOUT_H)

        # Apply calibration offsets and scale to proper units
        # Acceleration in m/s^2
        accel_x = (raw_ax - offsets['accel_x']) / ACCEL_SCALE * 9.80665
        accel_y = (raw_ay - offsets['accel_y']) / ACCEL_SCALE * 9.80665
        accel_z = (raw_az - offsets['accel_z']) / ACCEL_SCALE * 9.80665

        # Angular velocity in rad/s
        gyro_x = math.radians((raw_gx - offsets['gyro_x']) / GYRO_SCALE)
        gyro_y = math.radians((raw_gy - offsets['gyro_y']) / GYRO_SCALE)
        gyro_z = math.radians((raw_gz - offsets['gyro_z']) / GYRO_SCALE)

        # Create and populate IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # --- Add Covariance ---
        # These values are examples and may need tuning. They represent the
        # uncertainty of the measurement.
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.angular_velocity_covariance = [0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005]
        
        # Mark orientation as not provided
        imu_msg.orientation_covariance[0] = -1

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_node()
    except rospy.ROSInterruptException:
        pass
