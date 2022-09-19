#!/usr/bin/env python3
# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Martin Günther

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LIN_MAX = 1.0
ANG_MAX = 1.5  # adjust this value to the rough maximum angular velocity

state = 'stopped'
start = rospy.Time(0)


def odom_cb(msg):
    global state

    twist = msg.twist.twist
    t = (rospy.Time.now() - start).to_sec()

    if state == 'wait_for_stop':
        if -0.05 < twist.linear.x < 0.05 and -0.1 < twist.angular.z < 0.1:
            state = 'stopped'
            rospy.loginfo('state transition --> %s', state)
        return

    if state == 'backward' and twist.linear.x < -0.9 * LIN_MAX:
        rospy.loginfo('backward from 0 to %f m/s in %f sec', twist.linear.x, t)
    elif state == 'forward' and twist.linear.x > 0.9 * LIN_MAX:
        rospy.loginfo('forward from 0 to %f m/s in %f sec', twist.linear.x, t)
    elif state == 'turning_clockwise' and twist.angular.z < -0.9 * ANG_MAX:
        rospy.loginfo('turning_clockwise from 0 to %f rad/s in %f sec', twist.angular.z, t)
    elif state == 'turning_counter_clockwise' and twist.angular.z > 0.9 * ANG_MAX:
        rospy.loginfo('turning_counter_clockwise from 0 to %f rad/s in %f sec', twist.angular.z, t)
    else:
        return

    state = 'wait_for_stop'
    rospy.loginfo('state transition --> %s', state)


def cmd_vel_cb(msg):
    global state, start

    if state != 'stopped':
        return

    if msg.linear.x <= -LIN_MAX:
        start = rospy.Time.now()
        state = 'backward'
    elif msg.linear.x >= LIN_MAX:
        start = rospy.Time.now()
        state = 'forward'
    elif msg.angular.z <= -ANG_MAX:
        start = rospy.Time.now()
        state = 'turning_clockwise'
    elif msg.angular.z >= ANG_MAX:
        start = rospy.Time.now()
        state = 'turning_counter_clockwise'
    else:
        return

    rospy.loginfo('state transition --> %s', state)


def main():
    rospy.init_node('acc_finder', anonymous=True)
    rospy.Subscriber('odom', Odometry, odom_cb)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)
    rospy.loginfo('acc_finder node ready and listening. now use teleop to move your robot to the limits!')
    rospy.spin()


if __name__ == '__main__':
    main()
