# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# # Copyright 2016 The Cartographer Authors
# # Copyright 2018 DFKI GmbH
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #    http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, qos_profile_system_default


class remove_child_frames_node(Node):

    def __init__(self):
        super().__init__('tf_remove_child_frames')
        remove_frames = self.declare_parameter('remove_frames', []).value
        tf_pub = self.create_publisher(
            msg_type=TFMessage,
            topic='tf_out',
            qos_profile=QoSProfile(depth=1)
        )

        def tf_cb(msg):
            msg.transforms = [t for t in msg.transforms
                              if t.child_frame_id.lstrip('/') not in remove_frames]
            if len(msg.transforms) > 0:
                tf_pub.publish(msg)

        self.create_subscription(
            msg_type=TFMessage,
            topic="tf",
            callback=tf_cb,
            qos_profile=qos_profile_system_default
        )

        tf_static_pub = self.create_publisher(
            msg_type=TFMessage,
            topic='tf_static_out',
            qos_profile=QoSProfile(depth=1)
            # latch
        )

        def tf_static_cb(msg):
            msg.transforms = [t for t in msg.transforms
                              if t.child_frame_id.lstrip('/') not in remove_frames]
            if len(msg.transforms) > 0:
                tf_static_pub.publish(msg)

        self.create_subscription(
            msg_type=TFMessage,
            topic="tf_static",
            callback=tf_static_cb,
            qos_profile=qos_profile_system_default
        )


def main(args=None):
    rclpy.init(args=args)
    node = remove_child_frames_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
