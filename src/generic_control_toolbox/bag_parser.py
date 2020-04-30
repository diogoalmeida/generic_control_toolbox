#!/usr/bin/env python
import glob
import os
import sys
import rosbag
import numpy as np
import rospkg

class BagParser():
    '''
        Provides functionality to parse a ROS bag which holds 
        an actionlib feedback message.
    '''
    def __init__(self, pkg, rel_path, prefix):
        rospack = rospkg.RosPack()
        self._dir = rospack.get_path(pkg) + "/" + rel_path
        self._prefix = prefix


    def getAllBags(self, elements):
        '''
            Get all data from bags in the directory.

            @param elements attributes to extract from each bag (list)
            @returns list of bag data dictionaries.
        '''
        bag_data = []
        for num in range(len(glob.glob(self._dir + "/" + self._prefix + "*.bag"))):    
            bag = rosbag.Bag(self._dir + "/" + self._prefix + "_" +  str(num + 1) + ".bag")
            bag_data.append(self.getBagContents(elements, bag))

        return bag_data

    
    def getBagContents(self, elements, bag):
        '''
            Get all messages in the bag in the format msg.feedback.elements[i], plus the respective time.

            @param elements elements to extract from each bag (list)
            @param bag a rosbag.Bag instance.
            @returns A dictionary with the data mapping {elements[i]: value}
        '''
        data = {}
        data['t'] = []

        for element in elements:
            data[element] = []

        for topic, msg, time in bag.read_messages(): 
            data['t'].append(time.to_sec())

            for element in elements:
                msg_element = getattr(msg, element)
                list_data = self.msgToList(msg_element)
                if list_data is None:
                    raise RuntimeError("Got unsupported msg type " + msg_element._type)
                data[element].append(list_data)

        for key in data:
            data[key] = np.asarray(data[key])

        return data

    
    def msgToList(self, msg):
        '''
            Convert the given message to a list.
        '''
        if type(msg) is float or type(msg) is int:
            return msg
        if msg._type == 'geometry_msgs/WrenchStamped':
            return self.wrenchMsgToList(msg.wrench)
        elif msg._type == 'geometry_msgs/Wrench':
            return self.wrenchMsgToList(msg)
        elif msg._type == 'geometry_msgs/PoseStamped':
            return self.poseMsgToList(msg.pose)
        elif msg._type == 'geometry_msgs/Pose':
            return self.poseMsgToList(msg)
        else:
            return None

    
    def wrenchMsgToList(self, msg):
        '''
            Convert a wrench message to a list
        '''
        w = [msg.force.x, msg.force.y, msg.force.y, 
             msg.torque.x, msg.torque.y, msg.torque.z]
        return w

    def poseMsgToList(self, msg):
        '''
            Convert a pose message to list
        '''
        p = [msg.position.x, msg.position.y, msg.position.z,
             msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        return p