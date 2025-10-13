#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class GlobalPlanDebugViz:
    def __init__(self):
        self._plan_counter = 0

        default_plan_topic = rospy.get_param('~default_global_plan_topic', 'move_base_node/SBPLLatticePlanner/plan')
        plan_topic = rospy.get_param('~global_plan_topic', default_plan_topic)

        pose_array_topic = rospy.get_param('~pose_array_topic', 'global_plan_pose_array')
        marker_topic = rospy.get_param('~marker_topic', 'global_plan_marker')

        self._marker_frame = rospy.get_param('~marker_frame', 'base_link')
        self._marker_height = rospy.get_param('~marker_height', 0.8)
        self._marker_scale = rospy.get_param('~marker_scale', 0.2)
        self._marker_ns = rospy.get_param('~marker_ns', 'global_plan_counter')
        self._marker_id = rospy.get_param('~marker_id', 0)
        self._marker_color = {
            'r': rospy.get_param('~marker_color_r', 1.0),
            'g': rospy.get_param('~marker_color_g', 1.0),
            'b': rospy.get_param('~marker_color_b', 1.0),
            'a': rospy.get_param('~marker_color_a', 1.0),
        }

        skip_param = rospy.get_param('~number_of_poses_to_skip', 7)
        try:
            skip_param = int(skip_param)
        except (TypeError, ValueError):
            rospy.logwarn("number_of_poses_to_skip must be an integer, got %s. Using 0 (no skipping).", skip_param)
            skip_param = 0

        if skip_param < 0:
            rospy.logwarn("number_of_poses_to_skip cannot be negative. Using 0 (no skipping).")
            skip_param = 0

        self._poses_to_skip = skip_param

        self._pose_array_pub = rospy.Publisher(pose_array_topic, PoseArray, queue_size=1, latch=True)
        self._marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1, latch=True)
        self._plan_sub = rospy.Subscriber(plan_topic, Path, self._plan_callback, queue_size=1)

        rospy.loginfo('global_plan_debug_viz listening to %s', plan_topic)

    def _plan_callback(self, msg: Path):
        self._plan_counter += 1
        self._publish_pose_array(msg)
        self._publish_counter_marker()

    def _publish_pose_array(self, path_msg: Path):
        pose_array = PoseArray()
        pose_array.header = path_msg.header

        if self._poses_to_skip <= 0:
            pose_array.poses = [pose.pose for pose in path_msg.poses]
        else:
            pose_array.poses = [pose.pose for idx, pose in enumerate(path_msg.poses) if idx % self._poses_to_skip == 0]

        self._pose_array_pub.publish(pose_array)

    def _publish_counter_marker(self):
        marker = Marker()
        marker.header.frame_id = self._marker_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = self._marker_ns
        marker.id = self._marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self._marker_height
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._marker_scale
        marker.color.r = self._marker_color['r']
        marker.color.g = self._marker_color['g']
        marker.color.b = self._marker_color['b']
        marker.color.a = self._marker_color['a']
        marker.text = str(self._plan_counter)
        marker.lifetime = rospy.Duration(0.0)

        self._marker_pub.publish(marker)


def main():
    rospy.init_node('global_plan_debug_viz')
    GlobalPlanDebugViz()
    rospy.spin()


if __name__ == '__main__':
    main()
