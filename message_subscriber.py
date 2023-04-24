#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将订阅/person_info话题，自定义消息类型learning_topic::Person

import rospy
from event_detection_topic.msg import Message

def messageInfoCallback(message_info):
    # pyload_data = bytes(message_msg.payload)
    # string_data = pyload_data.decode('utf-8')
    rospy.loginfo("Subcribe Message  Info: %d, %d, %d, %d, %d, %d, %s, %d, %s, %d",
                  message_info.sof, message_info.version, message_info.appId, message_info.type,
                  message_info.codec, message_info.sequence, message_info.timestamp, message_info.length,
                  message_info.payload, message_info.crc)


# def messageStateInfoCallback(message_state):
#     # pyload_data = bytes(message_msg.payload)
#     # string_data = pyload_data.decode('utf-8')
#     rospy.loginfo("Subcribe Message State Info: %d, %d, %d, %d, %d, %d, %s, %d, %s, %d",
#                   message_state.sof, message_state.version, message_state.appId, message_state.type,
#                   message_state.codec, message_state.sequence, message_state.timestamp, message_state.length,
#                   message_state.payload, message_state.crc)

# def messageDataInfoCallback(message_data):
#     # pyload_data = bytes(message_msg.payload)
#     # string_data = pyload_data.decode('utf-8')
#     rospy.loginfo("Subcribe Message Data Info: %d, %d, %d, %d, %d, %d, %s, %d, %s, %d",
#                   message_data.sof, message_data.version, message_data.appId, message_data.type,
#                   message_data.codec, message_data.sequence, message_data.timestamp, message_data.length,
#                   message_data.payload, message_data.crc)

def message_subscriber():
	# ROS节点初始化
    rospy.init_node('message_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/message_info的topic，注册回调函数messageInfoCallback
    rospy.Subscriber("/message_info", Message, messageInfoCallback)

    # rospy.Subscriber("/message_data", Message, messageDataInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    message_subscriber()
