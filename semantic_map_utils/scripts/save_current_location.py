#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf
from nav_msgs. msg import Odometry
from geometry_msgs.msg import PoseStamped
from mongodb_store.message_store import MessageStoreProxy


class InsertWaypoint(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.listener = tf.TransformListener()
        self.msg_store = MessageStoreProxy(
            database=rospy.get_param("~db_name", "semantic_map"), 
            collection=rospy.get_param("~collection_name", "waypoints")
        )
        self.meta = {"waypoint_name": rospy.get_param("~waypoint_name")}
        target_frame = rospy.get_param("~target_frame", "semantic_map")
        rospy.loginfo("Waiting to receive odometry")
        odom = rospy.wait_for_message("/naoqi_driver_node/odom", Odometry)
        p = PoseStamped(header=odom.header, pose=odom.pose.pose)
        p.pose.position.z = 0.
        rospy.loginfo("Transforming pose")
        new = self.transform_pose(target_frame, p)
        rospy.loginfo("Saving waypoint '%s'." % self.meta["waypoint_name"])
        self.msg_store.insert(new, self.meta)
        rospy.loginfo("... done")
        
    def transform_pose(self, target_frame, pose):
        while not rospy.is_shutdown():
            try:
                t = self.listener.getLatestCommonTime(target_frame, pose.header.frame_id)
                pose.header.stamp = t
                return self.listener.transformPose(target_frame, pose)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
                rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("save_current_location")
    InsertWaypoint(rospy.get_name())
