#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from math import tan

TWIST_COVARIANCE = [0.1, 0, 0, 0, 0, 0,
                    0, 0.1, 0, 0, 0, 0,
                    0, 0, 0.1, 0, 0, 0,
                    0, 0, 0, 0.1, 0, 0,
                    0, 0, 0, 0, 0.1, 0,
                    0, 0, 0, 0, 0, 0.1]

class OdomPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('odom_from_joint_states', anonymous=True)

        namespace = rospy.get_namespace().strip("/")
        if namespace:
            self.prefix = namespace + '/'
        else:
            self.prefix = ''
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.prefix + "odom"
        self.odom_msg.child_frame_id = self.prefix + "base_link"
        self.odom_msg.twist.covariance = TWIST_COVARIANCE

        # Subscriber to the joint_states topic
        self.subscriber = rospy.Subscriber('joint_states', JointState, self.callback)
        
        # Publisher for the odom topic
        self.odom_pub = rospy.Publisher('vesc/odom', Odometry, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0/50.0), self.publish_odom)
        
        # Define the wheelbase of the vehicle (in meters)
        self.wheelbase = 0.325  # Adjust this value based on your actual vehicle's wheelbase

    def callback(self, data):
        # Extract wheel and steering joint velocities and positions
        try:
            left_front_vel = data.velocity[data.name.index('racecar/left_front_wheel_joint')]
            left_rear_vel = data.velocity[data.name.index('racecar/left_rear_wheel_joint')]
            right_front_vel = data.velocity[data.name.index('racecar/right_front_wheel_joint')]
            right_rear_vel = data.velocity[data.name.index('racecar/right_rear_wheel_joint')]
            
            left_steering_angle = data.position[data.name.index('racecar/left_steering_hinge_joint')]
            right_steering_angle = data.position[data.name.index('racecar/right_steering_hinge_joint')]
        except ValueError:
            rospy.logerr("Required joint not found in JointState message.")
            return

        # Calculate the average velocity (simple average of the front and rear wheels)
        avg_velocity = (left_rear_vel + right_rear_vel + left_front_vel + right_front_vel) / 4.0 * 0.05  # 0.05 is the radius of the wheel

        # Calculate the average steering angle (assuming symmetric steering)
        avg_steering_angle = (left_steering_angle + right_steering_angle) / 2.0

        # Compute the angular velocity using the bicycle model
        angular_velocity_z = avg_velocity * tan(avg_steering_angle) / self.wheelbase

        # Create a new Odometry message
        self.odom_msg.header.stamp = rospy.Time.now()

        # Populate twist information
        self.odom_msg.twist.twist.linear.x = avg_velocity
        self.odom_msg.twist.twist.angular.z = angular_velocity_z

    def publish_odom(self, event):
        self.odom_pub.publish(self.odom_msg)

if __name__ == '__main__':
    try:
        odom_publisher = OdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

