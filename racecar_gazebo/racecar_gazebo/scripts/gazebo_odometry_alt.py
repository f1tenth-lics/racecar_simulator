#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry


def odometry_callback(msg):
    # Get the namespace of the node, strip any leading or trailing slashes for clean usage
    namespace = rospy.get_namespace().strip("/")
    if namespace:
        prefix = namespace + '/'
    else:
        prefix = ''

    br = tf.TransformBroadcaster()
    # Use the namespace as prefix for frame names
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     prefix + "base_link",
                     prefix + "odom")


if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    rospy.Subscriber("odom", Odometry, odometry_callback)
    rospy.spin()
