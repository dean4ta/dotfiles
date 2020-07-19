#!/usr/bin/env python

import rospy


from sensor_msgs.msg import Imu

class JoyStick_Teleop:
	
	def __init__(self):
		# imu subscription subscriptions
		self.accel_sub = rospy.Subscriber('/camera/accel/sample', Imu, self.accel_cb)
		self.gyro_sub = rospy.Subscriber('/camera/gyro/sample', Imu, self.gyro_cb)
		
		# Cmd_vel publication
		self.imu_msg_pub = rospy.Publisher('/imu0', Imu, queue_size=10)

		# cmd_vel publication var
		self.imu_msg = Imu()

		self.accel_ready = False
		self.gyro_ready = False
		
	# TODO: This implementation of combining these two sensors is bad because the timing of the accelerometer may differ from the gyroscope. Needs real fusion
	def accel_cb(self, accel_msg):
		self.imu_msg.header = accel_msg.header
		self.imu_msg.orientation = accel_msg.orientation
		self.imu_msg.orientation_covariance = accel_msg.orientation_covariance
		self.imu_msg.linear_acceleration = accel_msg.linear_acceleration
		self.imu_msg.linear_acceleration_covariance = accel_msg.linear_acceleration_covariance
		self.accel_ready = True
		self.pub_imu_msg()

	def gyro_cb(self, gyro_msg):
		self.imu_msg.angular_velocity = gyro_msg.angular_velocity
		self.imu_msg.angular_velocity_covariance = gyro_msg.angular_velocity_covariance
		self.gyro_ready = True
		self.pub_imu_msg()
	
	def pub_imu_msg(self):
		if self.accel_ready and self.gyro_ready:
			self.imu_msg_pub.publish(self.imu_msg)
			self.gyro_ready = False
			self.accel_ready = False

if __name__ == '__main__':
	rospy.init_node('imu_combine', anonymous=True)
	myTeleop = JoyStick_Teleop()
	rospy.spin()