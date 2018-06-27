#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def fake_mir_joint_publisher():
    rospy.init_node('fake_mir_joint_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name =  ['left_wheel_joint', 'right_wheel_joint',
                    'fl_caster_rotation_joint', 'fl_caster_wheel_joint',
                    'fr_caster_rotation_joint', 'fr_caster_wheel_joint',
                    'bl_caster_rotation_joint', 'bl_caster_wheel_joint',
                    'br_caster_rotation_joint', 'br_caster_wheel_joint']
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
