#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def publish_transform(transform, base_frame, parent_frame, child_frame):
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # t = geometry_msgs.msg.TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame

    print(transform)
    broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('robot_root_transform_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # turtle_name = rospy.get_param('turtle', 'turtle2')
    # spawner(4, 2, 0, turtle_name)

    # turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)
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

    """
        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        turtle_vel.publish(msg)

        rate.sleep()
    """

