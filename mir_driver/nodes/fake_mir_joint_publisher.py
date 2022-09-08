#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState


def fake_mir_joint_publisher():
    rospy.init_node('fake_mir_joint_publisher')
    prefix = rospy.get_param('~prefix', '')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    publish_wheel_joints = rospy.get_param('~publish_wheel_joints', True)
    r = rospy.Rate(50.0)
    t = None
    while not rospy.is_shutdown():
        js = JointState()
        js.header.stamp = rospy.Time.now()
        if js.header.stamp == t:
            rospy.loginfo("Timestamp identical: %s, %s", js.header.stamp, t)
            r.sleep()
            continue

        t = js.header.stamp
        js.name = [prefix + 'fl_caster_rotation_joint',
                   prefix + 'fl_caster_wheel_joint',
                   prefix + 'fr_caster_rotation_joint',
                   prefix + 'fr_caster_wheel_joint',
                   prefix + 'bl_caster_rotation_joint',
                   prefix + 'bl_caster_wheel_joint',
                   prefix + 'br_caster_rotation_joint',
                   prefix + 'br_caster_wheel_joint',
                   ]
        if (publish_wheel_joints):
            js.name.extend([
                prefix + 'left_wheel_joint',
                prefix + 'right_wheel_joint',
            ])
        js.position = [0.0 for _ in js.name]
        js.velocity = [0.0 for _ in js.name]
        js.effort = [0.0 for _ in js.name]
        pub.publish(js)
        r.sleep()


if __name__ == '__main__':
    try:
        fake_mir_joint_publisher()
    except rospy.ROSInterruptException:
        pass
