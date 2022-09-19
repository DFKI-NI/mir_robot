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

from nav_msgs.msg import Odometry

lin_min = 0.0
lin_max = 0.0
ang_min = 0.0
ang_max = 0.0


def odom_cb(msg):
    global lin_min, lin_max, ang_min, ang_max
    if lin_min > msg.twist.twist.linear.x:
        lin_min = msg.twist.twist.linear.x
    if lin_max < msg.twist.twist.linear.x:
        lin_max = msg.twist.twist.linear.x
    if ang_min > msg.twist.twist.angular.z:
        ang_min = msg.twist.twist.angular.z
    if ang_max < msg.twist.twist.angular.z:
        ang_max = msg.twist.twist.angular.z

    rospy.loginfo('linear: [%f, %f]   angular: [%f, %f]', lin_min, lin_max, ang_min, ang_max)


def main():
    rospy.init_node('min_max_finder', anonymous=True)
    rospy.Subscriber('odom', Odometry, odom_cb)
    rospy.loginfo('min_max_finde node ready and listening. now use teleop to move your robot to the limits!')
    rospy.spin()


if __name__ == '__main__':
    main()
