#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, Twist, Vector3
import tf
import math

class ImuToOdom:
    def __init__(self):
        rospy.init_node('imu_to_odom')

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        self.last_time = rospy.Time.now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.rate = rospy.Rate(50)  # 50 Hz update rate

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Assuming IMU gives us linear acceleration and angular velocity
        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        az = data.linear_acceleration.z

        # Angular velocity
        self.vth = data.angular_velocity.z

        # Integrate acceleration to get velocity
        self.vx += ax * dt
        self.vy += ay * dt

        # Integrate velocity to get position
        delta_x = self.vx * dt
        delta_y = self.vy * dt

        self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
        self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)

        # Update orientation
        self.th += self.vth * dt

        # Publish the odometry message
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # Publish the message
        self.odom_pub.publish(odom)

        self.last_time = current_time

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        imu_to_odom = ImuToOdom()
        imu_to_odom.spin()
    except rospy.ROSInterruptException:
        pass

