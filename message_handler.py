
import datetime
import threading
import time
import json
import rospy
from event_detection_topic.msg import Message
from configs import *



class MessageHandler:
    def __init__(self):
        self.events_list = []
        self.lock = threading.Lock()
        rospy.init_node('event_message', anonymous=True)
        rospy.Subscriber("2d_detection", Message, self.callback_detection)
        rospy.Subscriber("perception_fusion", Message, self.callback_fusion)
        self.event_pub = rospy.Publisher('event_topic', Message, queue_size=10)
        self.data_message_msg = DataMessage()
        # 给运维模块发送的消息计数，报文序列号，范围是0x0001~0xFFFF
        self.data_message_msg.sequence = 0x0000
        # Type1 消息计数 每次往外发布消息，自增1，范围是0-255
        self.msg_count = 0
        # event_id 事件计数 每次触发事件，自增1，范围是0-65535
        self.event_count = 0

    def callback_detection(self, data):
        # some logic to process the 2D detection data
        with self.lock:
            self.events_list.append(MESSAGE.event_data())

    def callback_fusion(self, data):
        # some logic to process the perception fusion data
        with self.lock:
            self.events_list.append(MESSAGE.event_data())

    def publish_events(self):
        if len(self.events_list) > 0:
            application_data = MESSAGE.application_data

            with self.lock:
                application_data["data"]["msg_cnt"] = (self.msg_count+1) % 256
                application_data["data"]["events_list"] = self.events_list
                # some logic to publish the events

                # and clear the events_list
                self.events_list = []

            data_json_result = json.dumps(application_data)
            data_json_result = bytes(data_json_result, encoding="utf8")

            self.data_message_msg.sof = 0x5A
            self.data_message_msg.version = 0x01
            self.data_message_msg.appId = 0x01
            self.data_message_msg.type = 0x04
            self.data_message_msg.codec = 0x01
            self.data_message_msg.sequence = (
                self.data_message_msg.sequence+1) & 0xFFFF
            self.data_message_msg.timestamp = rospy.get_rostime().to_sec()
            self.data_message_msg.length = len(data_json_result)
            self.data_message_msg.payload = data_json_result
            self.data_message_msg.crc = 0xFFFF

            self.event_pub.publish(data_json_result)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_events()
            rate = rospy.Rate(10)
            rate.sleep()
            # rospy.sleep(0.1)


if __name__ == '__main__':
    event_handler = MessageHandler()
    event_handler.run()
