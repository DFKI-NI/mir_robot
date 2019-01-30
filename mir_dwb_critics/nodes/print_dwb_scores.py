#!/usr/bin/env python
import rospy

from dwb_msgs.msg import LocalPlanEvaluation

def eval_cb(msg):
    print '\n\n=====================================================\n\n'
    for i in [msg.best_index, msg.worst_index]:
        print 'Name                 |    Raw |  Scale | Scaled Score'
        print '---------------------|--------|--------|-------------'
        for s in msg.twists[i].scores:
            print '{:20} | {:6.2f} | {:6.2f} | {:12.2f}'.format(s.name, s.raw_score, s.scale, s.raw_score * s.scale)
        print '--------------------------------------- total: {:6.2f}'.format(msg.twists[i].total)
        print


def main():
    rospy.init_node('print_dwb_scores', anonymous=True)
    rospy.Subscriber('move_base_node/DWBLocalPlanner/evaluation', LocalPlanEvaluation, eval_cb)
    rospy.loginfo('print_dwb_scores ready.')
    rospy.spin()


if __name__ == '__main__':
    main()
