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

from dwb_msgs.msg import LocalPlanEvaluation


def eval_cb(msg):
    print('\n\n=========================================================\n\n')
    for heading, i in zip(['best trajectory', 'worst trajectory'], [msg.best_index, msg.worst_index]):
        print('### {}\n'.format(heading))
        print('Name                 |       Raw |   Scale | Scaled Score')
        print('---------------------|-----------|---------|-------------')
        for s in msg.twists[i].scores:
            print('{:20} | {:9.4f} | {:7.4f} | {:12.4f}'.format(s.name, s.raw_score, s.scale, s.raw_score * s.scale))
        print('---------------------------------------- total: {:9.4f}'.format(msg.twists[i].total))
        print()


def main():
    rospy.init_node('print_dwb_scores', anonymous=True)
    rospy.Subscriber('move_base_node/DWBLocalPlanner/evaluation', LocalPlanEvaluation, eval_cb)
    rospy.loginfo('print_dwb_scores ready.')
    rospy.spin()


if __name__ == '__main__':
    main()
