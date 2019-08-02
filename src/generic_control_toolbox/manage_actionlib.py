#!/usr/bin/env python
import sys

import actionlib
import rospy
import time
"""
    @package manage_actionlib
    This module provides utility methods for setting up actionlib calls through an overarching actionlib server.
"""


def monitor_action_goal(action_server,
                        action_client,
                        action_goal,
                        action_name="current action",
                        time_limit=float("inf"),
                        abort_on_fail=False):
    """Send and monitor an action goal to a given action client.

       The monitor will return in case of the client reporting success, preemption or
       abortion, and will also pass through any incoming preemptions to the action server.

       @param action_server Action server that is calling the action client. If the action server is preempted, this is passed to the action client.
       @param action_client The action client that is called by the action server. If the action server is preempted, the goal sent to this client is preempted as well.
       @param action_goal The action goal to be sent to the action client.
       @param action_name Optional name for the action. Will be displayed in the logs.
       @param time_limit An optional time limit for the action. If exceeded, the action client is preempted
    """

    success = False
    rospy.loginfo("Sending goal to " + action_name)
    action_client.send_goal(action_goal)
    init_time = rospy.Time(0)

    while init_time == rospy.Time(0):
        init_time = rospy.Time.now()
        time.sleep(0.01)

    while action_server.is_active():
        if (rospy.Time.now() - init_time).to_sec() > time_limit:
            rospy.logwarn("Timeout of request")
            action_client.cancel_goal()
            success = False
            break

        if action_server.is_preempt_requested():
            rospy.logwarn("Preempting " + action_name)
            action_client.cancel_goal()
            finished = action_client.wait_for_result(
                timeout=rospy.Duration(1.0))

            if not finished:
                rospy.logerr(action_name +
                             " failed to preempt! This should never happen")
                action_server.set_aborted(text="Aborted due to action " +
                                          action_name + " failing to preempt")
            else:
                action_server.set_preempted(
                    text="Preempted while running " + action_name)
                rospy.loginfo("Preempted while running")
            break

        if action_client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.logerr(action_name + " aborted!")
            success = False
            if abort_on_fail:
                action_server.set_aborted(text=action_name + " aborted")
            break

        if action_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            rospy.logerr(action_name + " preempted!")
            success = False
            if abort_on_fail:
                action_server.set_aborted(text=action_name + " was preempted")
            break

        if action_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(action_name + " succeeded!")
            success = True
            break

        time.sleep(0.1)

    return success
