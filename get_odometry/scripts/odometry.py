#!/usr/bin/python

import rospy
import smbus
import math

#Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(reg, bus, address):
    return bus.read_byte_data(address, reg)

def read_word(reg, bus, address):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

def read_word_2c(reg, bus, address):
    val = read_word(reg, bus, address)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a) + (b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
 
def get_z_rotation(x,y,z):
    radians = math.atan2(z, dist(x,y))
    return math.degrees(radians)

def read_odom():

    bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
    address = 0x68       # via i2cdetect
 
    bus.write_byte_data(address, power_mgmt_1, 0)
 
    while not rospy.is_shutdown():
        #print("Gyroscope")
        #print("--------")
 
        gyroscope_xout = read_word_2c(0x43, bus, address)
        gyroscope_yout = read_word_2c(0x45, bus, address)
        gyroscope_zout = read_word_2c(0x47, bus, address)
 
        #print("gyroscope_xout: ", ("%5d" % gyroscope_xout), " scaled: ", (gyroscope_xout / 131))
        #print("gyroscope_yout: ", ("%5d" % gyroscope_yout), " scaled: ", (gyroscope_yout / 131))
        print("gyroscope_zout: ", ("%5d" % gyroscope_zout), " scaled: ", (gyroscope_zout / 131))
 
        #print("Accelerometer")
        #print("---------------------")
 
        accel_xout = read_word_2c(0x3b, bus, address)
        accel_yout = read_word_2c(0x3d, bus, address)
        accel_zout = read_word_2c(0x3f, bus, address)
 
        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0
 
        #print("accel_xout: ", ("%6d" % accel_xout), " scaled: ", accel_xout_scaled)
        #print("accel_yout: ", ("%6d" % accel_yout), " scaled: ", accel_yout_scaled)
        #print("accel_zout: ", ("%6d" % accel_zout), " scaled: ", accel_zout_scaled)
        #print("X Rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        #print("Y Rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        #print("Z rotation: ", get_z_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))


if __name__ == '__main__':
    try:
        read_odom()
    except rospy.ROSInterruptException:
        pass
