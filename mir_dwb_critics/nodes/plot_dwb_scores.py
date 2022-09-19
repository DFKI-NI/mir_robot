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

import os
import tkinter
import matplotlib.pyplot as plt
import numpy as np

import rospy
from dwb_msgs.msg import LocalPlanEvaluation

fig = None
rects = None
max_score = 0.0


def eval_cb(msg):
    global fig, rects, max_score
    labels = [s.name for s in msg.twists[msg.best_index].scores]
    scaled_scores = [s.scale * s.raw_score for s in msg.twists[msg.best_index].scores]

    # reverse labels + scaled_scores to show the critics in the correct order top to bottom
    labels.reverse()
    scaled_scores.reverse()

    if not fig:
        # init
        y = np.arange(len(labels))  # the label locations
        height = 0.35  # the height of the bars

        fig, ax = plt.subplots()
        rects = ax.barh(y - height / 2, scaled_scores, height, label='scaled score')

        ax.set_ylabel('DWB Critics')
        ax.set_title('Scaled scores')
        ax.set_yticks(y)
        ax.set_yticklabels(labels)

        fig.tight_layout()
        fig.canvas.set_window_title('DWB Scores')

    # update axis limit
    if max_score < max(scaled_scores):
        max_score = max(scaled_scores)
        plt.xlim(0.0, max_score)

    for rect, h in zip(rects, scaled_scores):
        rect.set_width(h)
    try:
        fig.canvas.draw()
        fig.canvas.flush_events()
    except tkinter.TclError:
        rospy.loginfo('Window was closed, exiting.')
        os._exit(0)


def main():
    rospy.init_node('plot_dwb_scores')
    rospy.Subscriber('move_base_node/DWBLocalPlanner/evaluation', LocalPlanEvaluation, eval_cb)
    rospy.loginfo('plot_dwb_scores ready.')
    plt.ion()
    rospy.spin()


if __name__ == '__main__':
    main()
