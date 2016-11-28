#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf
from std_srvs.srv import Empty, EmptyResponse
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose


class TransformPublisher(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.rate = rospy.get_param("~rate", 30)
        self.msg_store = MessageStoreProxy(
            database=rospy.get_param("~db_name", "semantic_map"), 
            collection=rospy.get_param("~collection_name", "config")
        )
        self.meta = {"transform": "semantic_map"}
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.trans = tuple()
        self.rot = tuple()
        try:        
            self.pose, self._id = self.load()
        except Exception:
            rospy.loginfo("No pose found")
            self.calibrate()
        else:
            rospy.loginfo("Found saved pose")
        rospy.Service("~calibrate", Empty, self.calibrate)
        rospy.loginfo("... done")
        
    def calibrate(self, *args):
        rospy.loginfo("Calibrating transform publisher")
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform('/base_link', '/odom', rospy.Time(0), timeout=rospy.Duration(5))
                t = self.listener.getLatestCommonTime('/base_link', '/odom')
                (trans, rot) = self.listener.lookupTransform('/odom', '/base_link', t)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
            else:
                break
        self.pose = Pose()
        self.pose.position.x = trans[0]
        self.pose.position.y = trans[1]
        self.pose.position.z = trans[2]
        self.pose.orientation.x = rot[0]
        self.pose.orientation.y = rot[1]
        self.pose.orientation.z = rot[2]
        self.pose.orientation.w = rot[3]
        self.store(self.pose)
        rospy.loginfo("Calibrated.")
        return EmptyResponse()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(
                (self.pose.position.x, self.pose.position.y, 0.0),
                (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w),
                rospy.Time.now(),
                "/semantic_map",
                "/odom"
            )
            r.sleep()
            
    def store(self, pose):
        try:
            _, _id = self.load()
        except Exception:
            pass
        else:
            self.msg_store.delete(str(_id))

        self.msg_store.insert(pose, self.meta)
        
    def load(self):
        message = self.msg_store.query(Pose._type, {}, self.meta)
        if len(message) == 0:
            raise Exception("Desired data set 'transform: semantic_map' not in datacentre.")
        else:
            return message[0][0], message[0][1]["_id"]

if __name__ == "__main__":
    rospy.init_node("semantic_map_tf")
    t = TransformPublisher(rospy.get_name())
    t.spin()

