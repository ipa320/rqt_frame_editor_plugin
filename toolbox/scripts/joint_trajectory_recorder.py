#!/usr/bin/env python

## Python ##
##
import signal
import sys

## ROS ##
##
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


trajectory = JointTrajectory()


def run():

    subscriber = rospy.Subscriber("joint_states", JointState, joint_states_callback)

    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = "base_link"


def joint_states_callback(msg):

    ## WARNING: this doesn't check for changes
    trajectory.joint_names = msg.name

    ## Append point
    p = JointTrajectoryPoint()
    p.positions = msg.position
    p.time_from_start = rospy.Time.now() - trajectory.header.stamp
    trajectory.points.append(p)


def handler(signum, frame):
    ## WARNING: no mutex here, could be killing the append() function
    print 'Wrap-up'
    print '#################################'
    print trajectory

    sys.exit()


if __name__ == "__main__":

    rospy.init_node('joint_trajectory_recorder')

    signal.signal(signal.SIGINT, handler)

    print "Running. Press Ctrl+C to finish."
    run()

    rospy.spin()

# eof
