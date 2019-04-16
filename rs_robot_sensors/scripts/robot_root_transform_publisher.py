#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg

def publish_transform(transform, base_frame, parent_frame, child_frame):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame

    broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('robot_root_transform_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base_frame = rospy.get_param('~base_frame', 'odom')
    parent_frame = rospy.get_param('~parent_frame', 'parent_frame')
    child_frame = rospy.get_param('~child_frame', 'child_frame')

    rospy.loginfo("base_frame: " + base_frame)
    rospy.loginfo("parent_frame: " + parent_frame)
    rospy.loginfo("child_frame: " + child_frame)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(parent_frame, base_frame, rospy.Time())
            print(transform)
            publish_transform(transform, base_frame, parent_frame, child_frame)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

    listener.unregister()

    while not rospy.is_shutdown():
        rate.sleep()
