#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_system_default


class fake_mir_joint_publisher(Node):

    def __init__(self):
        super().__init__('fake_mir_joint_publisher')

        self.declare_parameter('tf_prefix', '')
        tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value.strip('/')
        if tf_prefix != "":
            tf_prefix = tf_prefix + '/'

        pub = self.create_publisher(
                msg_type=JointState,
                topic='joint_states',  # no prefix to joint states, just namespace
                qos_profile=qos_profile_system_default  # TODO Check QoS Settings
        )

        pub_rate = 1

        while(rclpy.ok()):
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [tf_prefix + 'left_wheel_joint', tf_prefix + 'right_wheel_joint',
                       tf_prefix + 'fl_caster_rotation_joint', tf_prefix + 'fl_caster_wheel_joint',
                       tf_prefix + 'fr_caster_rotation_joint', tf_prefix + 'fr_caster_wheel_joint',
                       tf_prefix + 'bl_caster_rotation_joint', tf_prefix + 'bl_caster_wheel_joint',
                       tf_prefix + 'br_caster_rotation_joint', tf_prefix + 'br_caster_wheel_joint']
            js.position = [0.0 for _ in js.name]
            js.velocity = [0.0 for _ in js.name]
            js.effort = [0.0 for _ in js.name]
            pub.publish(js)
            time.sleep(pub_rate)


def main(args=None):
    rclpy.init(args=args)
    node = fake_mir_joint_publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
