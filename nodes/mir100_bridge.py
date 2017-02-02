#!/usr/bin/env python
import rospy

import copy
import sys

from mir100_robot import rosbridge
from rospy_message_converter import message_converter

from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import Pose, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseFeedback, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.msg import tfMessage

class TopicConfig(object):
    def __init__(self, topic, topic_type, latch=False, dict_filter=None):
        self.topic = topic
        self.topic_type = topic_type
        self.latch = latch
        self.dict_filter = dict_filter

# remap mir_actions/MirMoveBaseAction => move_base_msgs/MoveBaseAction
def _move_base_feedback_dict_filter(msg_dict):
    # filter out slots from the dict that are not in our message definition
    # e.g., MiRMoveBaseFeedback has the field "state", which MoveBaseFeedback doesn't
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['feedback'] = {key: msg_dict['feedback'][key] for key in MoveBaseFeedback.__slots__}
    return filtered_msg_dict

def _move_base_result_dict_filter(msg_dict):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['result'] = {key: msg_dict['result'][key] for key in MoveBaseResult.__slots__}
    return filtered_msg_dict

# topics we want to publish to ROS (and subscribe to from the MiR)
PUB_TOPICS = [TopicConfig('imu_data',           Imu),                     # not available in simulation
              TopicConfig('map',                OccupancyGrid,          latch=True),
              TopicConfig('move_base/feedback', MoveBaseActionFeedback, dict_filter=_move_base_feedback_dict_filter),  # really mir_actions/MirMoveBaseActionFeedback
              TopicConfig('move_base/result',   MoveBaseActionResult,   dict_filter=_move_base_result_dict_filter),    # really mir_actions/MirMoveBaseActionResult
              TopicConfig('move_base/status',   GoalStatusArray),
              TopicConfig('odom',               Odometry),                # odom_comb on real robot?
              TopicConfig('robot_pose',         Pose),
              TopicConfig('scan',               LaserScan),
              TopicConfig('tf',                 tfMessage)]

# topics we want to subscribe to from ROS (and publish to the MiR)
SUB_TOPICS = [TopicConfig('cmd_vel',               Twist),
              TopicConfig('move_base/cancel',      GoalID),
              TopicConfig('move_base/goal',        MoveBaseActionGoal),  # really mir_actions/MirMoveBaseActionGoal
              TopicConfig('move_base_simple/goal', PoseStamped)]

class PublisherWrapper(object):
    def __init__(self, topic_config, robot):
        self.topic_config = topic_config
        self.pub = rospy.Publisher(name=topic_config.topic,
                                   data_class=topic_config.topic_type,
                                   latch=topic_config.latch,
                                   queue_size=10)
        robot.subscribe(topic=('/' + topic_config.topic), callback=self.callback)
        rospy.loginfo("[%s] publishing topic '%s' [%s]",
                      rospy.get_name(), topic_config.topic, topic_config.topic_type._type)

    def callback(self, msg_dict):
        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict)
        msg = message_converter.convert_dictionary_to_ros_message(self.topic_config.topic_type._type, msg_dict)
        self.pub.publish(msg)

class SubscriberWrapper(object):
    def __init__(self, topic_config, robot):
        self.topic_config = topic_config
        self.robot = robot
        self.sub = rospy.Subscriber(name=topic_config.topic,
                                    data_class=topic_config.topic_type,
                                    callback=self.callback,
                                    queue_size=10)
        rospy.loginfo("[%s] subscribing to topic '%s' [%s]",
                      rospy.get_name(), topic_config.topic, topic_config.topic_type._type)

    def callback(self, msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict)
        self.robot.publish('/' + self.topic_config.topic, msg_dict)

class MiR100Bridge(object):
    def __init__(self):
        try:
            hostname = rospy.get_param('~hostname')
        except KeyError:
            rospy.logfatal('[%s] parameter "hostname" is not set!', rospy.get_name())
            sys.exit(-1)
        port = rospy.get_param('~port', 9090)

        rospy.loginfo('[%s] trying to connect to %s:%i...', rospy.get_name(), hostname, port)
        self.robot = rosbridge.RosbridgeSetup(hostname, port)

        r = rospy.Rate(10)
        i = 1
        while not self.robot.is_connected():
            if rospy.is_shutdown():
                sys.exit(0)
            if self.robot.is_errored():
                rospy.logfatal('[%s] connection error to %s:%i, giving up!', rospy.get_name(), hostname, port)
                sys.exit(-1)
            if i % 10 == 0:
                rospy.logwarn('[%s] still waiting for connection to %s:%i...', rospy.get_name(), hostname, port)
            i += 1
            r.sleep()

        rospy.loginfo('[%s] ... connected.', rospy.get_name())

        self.topics, self.topic_types = self.get_topics()

        for pub_topic in PUB_TOPICS:
            PublisherWrapper(pub_topic, self.robot)
            if ('/' + pub_topic.topic) not in self.topics:
                rospy.logwarn("[%s] topic '%s' is not published by the MiR!", rospy.get_name(), pub_topic.topic)

        for sub_topic in SUB_TOPICS:
            SubscriberWrapper(sub_topic, self.robot)
            if ('/' + sub_topic.topic) not in self.topics:
                rospy.logwarn("[%s] topic '%s' is not yet subscribed to by the MiR!", rospy.get_name(), sub_topic.topic)

    def get_topics(self):
        print 'Available topics:'

        srv_response = self.robot.callService('/rosapi/topics', msg={})
        topics = sorted(srv_response['topics'])
        topic_types = []

        for topic in topics:
            srv_response = self.robot.callService("/rosapi/topic_type", msg={'topic': topic})
            topic_types.append(srv_response['type'])

        for topic, topic_type in zip(topics, topic_types):
            print ' * %s [%s]' % (topic, topic_type)

        return topics, topic_types


def main():
    rospy.init_node('mir100_bridge')
    MiR100Bridge()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
