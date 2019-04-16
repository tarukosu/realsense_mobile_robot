#!/usr/bin/env python  
import rospy

import tf2_ros
import geometry_msgs.msg

def publish_transform(parent_frame, child_frame, offset_z):
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = offset_z
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('t265_ground_offset_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    default_offset = rospy.get_param('~default_offset', 0.0)
    base_frame = rospy.get_param('~base_frame', 'odom')
    footprint_frame = rospy.get_param('~footprint_frame', 'base_footprint')
    child_frame = rospy.get_param('~child_frame', 'child_frame')
    update_rate = rospy.get_param('~update_rate', 0.1) # Hz

    rospy.loginfo("default_offset: " + str(base_frame))
    
    rospy.loginfo("base_frame: " + base_frame)
    rospy.loginfo("footprint_frame: " + footprint_frame)
    rospy.loginfo("child_frame: " + child_frame)

    publish_transform(base_frame, child_frame, default_offset)
    
    rospy.sleep(10.0)

    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(child_frame, footprint_frame, rospy.Time())
            offset_z = - transform.transform.translation.z
            publish_transform(base_frame, child_frame, offset_z)
            rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

