#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 该例程将发布/message_info话题，自定义消息类型event_detection_topic::Message
import datetime
import time
import json
import rospy
from event_detection_topic.msg import Message

def message_publisher():
	# ROS节点初始化
    rospy.init_node('message_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/message_info的topic，消息类型为event_detection_topic::Message，队列长度10
    message_info_pub = rospy.Publisher('/message_info', Message, queue_size=10)

    # message_data_pub = rospy.Publisher('/message_data', Message, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        now_python = datetime.datetime.now()
        time_str = now_python.strftime("%Y-%m-%d %H:%M:%S")

        # Get the current year
        current_year = datetime.datetime.now().year
        # Calculate the total number of minutes since the start of the year
        start_of_year = datetime.datetime(current_year, 1, 1)
        total_minutes = (now_python - start_of_year).total_seconds() // 60
        # Calculate the number of minutes that have passed so far this year
        current_minutes = now_python.hour * 60 + now_python.minute
        # Calculate the number of minutes that have already passed this year
        minutes_passed = total_minutes - current_minutes

        minute_offset = now_python.minute # 计算当前时间在当天中已经过去的分钟数，即分钟内的偏移量
        # 计算当前时间的毫秒数，并添加到分钟偏移量中得到毫秒级时刻
        millisecond_offset = now_python.microsecond // 1000
        ms_time = minute_offset * 60 * 1000 + millisecond_offset
        # 将毫秒级时刻格式化为字符串
        ms_time_str = "{:03d}".format(ms_time)
        event_id, event_type, event_obj_id, event_confid= 0, 0, 0, 0
        event_pos, event_radius, ref_path_list, ref_link_list = [], [], [], []
        priority = "B00000000"
        detected_region, obstacle_list = [], []


        events_list = {
            "event_id": 0,
            "event_type": 1,
            "description": "None",
            "event_pos": event_pos,
            "event_obj_id": event_obj_id,
            "event_radius": event_radius,
            "priority": priority,
            "ref_path_list": ref_path_list,
            "ref_link_list": ref_link_list,
            "event_source": 5,
            "event_confid": event_confid,
            "time_details": int(minutes_passed)
        }

        data = {
            "type": 1,
            "ver": '1',
            "msg_cnt": 1,
            "min_of_year": int(minutes_passed),
            "second": ms_time_str,
            "fus_dev_id": "1",
            "ref_pos": [111, 111],
            "scene_type": 0,
            "objects_list": None,
            "events_list": events_list,
            "detected_region": detected_region,
            "obstacle_list": obstacle_list
        }
        
        state_data_result = json.dumps(data)
        state_data_result = state_data_result.encode('utf-8')

        # data_frame = {
        #     "head": '0xDA0xDB0xDC0xDD',
        #     "frame_type" : 0x01,
        #     "perception_type": 0x03,
        #     "length": 1,
        #     "data": data
        # }

        application_state = {
            "name": "message_detect",
            "version": "1",
            "publishDate":time_str,
            "installType":0x02,
            "installLocation":"/home/../ros/catkin_ws/src/..",
            "logLocation":"/home/../ros/catkin_ws/src/../log/",
            "status":0x02,
            "description":"****test message****"
        }

        application_data = {
            "type":  0x01,
            "codec": 0x02,
            "data" : state_data_result.hex()
        }


        state_json_result = json.dumps(application_state)
        state_json_result = bytes(state_json_result, encoding = "utf8")
        data_json_result = json.dumps(application_data)
        data_json_result = bytes(data_json_result, encoding = "utf8")

        now = rospy.get_rostime()
        state_message_msg = Message()
        state_message_msg.sof = 0x5A
        state_message_msg.version  = 0x01
        state_message_msg.appId  = 0x01
        state_message_msg.type = 0x01
        state_message_msg.codec = 0x01
        state_message_msg.sequence = 100
        state_message_msg.timestamp = rospy.Time.now().to_nsec() // 1000000
        state_message_msg.length = 100
        state_message_msg.payload = state_json_result
        # state_message_msg.payload.append(data_json_result)
        state_message_msg.crc = 0xFFFF

        data_message_msg = Message()
        data_message_msg.sof = 0x5A
        data_message_msg.version = 0x01
        data_message_msg.appId = 0x01
        data_message_msg.type = 0x04
        data_message_msg.codec = 0x01
        data_message_msg.sequence = 100
        data_message_msg.timestamp = now.to_sec()
        data_message_msg.length = 100
        data_message_msg.payload = data_json_result
        # state_message_msg.payload.append(data_json_result)
        data_message_msg.crc = 0xFFFF

        # 发布消息
        message_info_pub.publish(state_message_msg)
        rospy.loginfo("Publsh state_message_msg[%d, %d, %d, %d, %d, %d, %s, %d, %s, %d ]",
				state_message_msg.sof, state_message_msg.version, state_message_msg.appId, state_message_msg.type,
                state_message_msg.codec, state_message_msg.sequence, state_message_msg.timestamp, state_message_msg.length,
                state_message_msg.payload, state_message_msg.crc)

        message_info_pub.publish((data_message_msg))
        rospy.loginfo("Publsh data_message_msg[%d, %d, %d, %d, %d, %d, %s, %d, %s, %d ]",
                      data_message_msg.sof, data_message_msg.version, data_message_msg.appId, data_message_msg.type,
                      data_message_msg.codec, data_message_msg.sequence, data_message_msg.timestamp,
                      data_message_msg.length,
                      data_message_msg.payload, data_message_msg.crc)

        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        message_publisher()


    except rospy.ROSInterruptException:
        pass
