#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import adafruit_bno055 as BNO055
import board

i2c = board.I2C()
bno = BNO055.BNO055_I2C(i2c)

pwm_offset_ms = 0
HEADING_OFFSET = 0

class BNO_Publisher(Node):
    def __init__(self):
        super().__init__('BNO055')
        self.SysCalPub=self.create_publisher(Float64, 'IMU/SysCalibration')
        self.timer = self.create_timer(0.25,self.timer_callback)
    
    def timer_callback(self):
        sys_cal, gyro_cal, accel_cal, mag_cal = bno.calibration_status
        yaw, pitch, roll = bno.euler

        if cal is True and yaw != None:
            HEADING_OFFSET = -yaw
            cal = False
        
        yaw += HEADING_OFFSET
        yaw %= 360
        # if necessary
        #m_x, m_y, m_z = bno.read_magnetometer()
        #g_x, g_y, g_z = bno.read_gyroscope()
        #a_x, a_y, a_z = bno.read_accelerometer()
        #                bno.read_linear_acceleration()
        #                bno.read_gravity()

        self.SysCalPub.publish(sys_cal)
        #pub_cal.publish(gyro_cal, accel_cal, mag_cal)
        #pub_angle.publish(roll, pitch, yaw)

        # if necessary
        #pub_gyro_raw.publish(g_x, g_y, g_z)
        #pub_accel_raw.publish(a_x, a_y, a_z)
        #pub_mag_raw.publish(m_x, m_y, m_z)

def main(args=None):
    rclpy.init(args=args)
    global cal 
    cal = True

    BNO055_Publisher = BNO_Publisher()

    rclpy.spin(BNO055_Publisher)


    # define custom 3-variable message type for this
    #pub_cal = rospy.Publisher("IMU/SensorCalibrations", Vector3, queue_size=10)

    #pub_angle = rospy.Publisher("IMU/FusedAngle", Vector3, queue_size=10)

    #pub_gyro_raw = rospy.Publisher("IMU/RawGyro", Vector3, queue_size=10)
    #pub_accel_raw = rospy.Publisher("IMU/RawAccel", Vector3, queue_size=10)
    #pub_mag_raw = rospy.Publisher("IMU/RawMag", Vector3, queue_size=10)
    BNO055_Publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass
