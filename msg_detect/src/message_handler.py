import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
sys.path.append(str(ROOT))
# sys.path.append(str(ROOT / '..'))
from datetime import datetime
import threading
from threading import Timer
import json
import rospy
from utils import *
from message_msgs.msg import Message
from xiangji_msgs.msg import results as xj_results
from lidar_msgs.msg import results as lidar_results
from configs import *
import crcmod.predefined

class MessageHandler:
    def __init__(self):
        self.events_list = []
        self.lock = threading.Lock()

        rospy.init_node('event_message', anonymous=True)
        rospy.Subscriber("/vision_result", xj_results, self.callback_detection)
        # rospy.Subscriber("perception_fusion", Message, self.callback_fusion)
        self.event_pub = rospy.Publisher('/event_topic', Message, queue_size=1)
        # self.event_pub_rate = rospy.Rate(5)
        # self.state_pub_rate = rospy.Rate(1)
        self.state_pub = rospy.Publisher('/state_topic', Message, queue_size=1)
        self.send_time1 = rospy.Time.now() + rospy.Duration(1.0)
        self.send_time2 = rospy.Time.now() + rospy.Duration(5.0)
        self.crc16  = crcmod.predefined.mkPredefinedCrcFun('crc16')

        self.state_message_msg = Message()
        self.data_message_msg = DataMessage()

        # 给运维模块发送的消息计数，报文序列号，范围是0x0001~0xFFFF
        self.data_message_msg.sequence = 0x0001
        # Type1 消息计数 每次往外发布消息，自增1，范围是0-255
        self.msg_count = 0
        # event_id 事件计数 每次触发事件，自增1，范围是0-65535
        self.event_count = 0

        self.lanes_num = LaneInfo.laneNums
        # 逆行检测对象字典
        # self.rd_checkers = {lane_id: ReverseDrivingDetector(
        #     lane_angle) for lane_id, lane_angle in zip(LaneInfo.laneIdList, LaneInfo.laneAngles)}
        #
        # # 拥堵检测对象字典 同时开启车辆通过总耗时计算线程
        # self.cg_checkers = {lane_id: CongestionDetector()
        #                     for lane_id in LaneInfo.laneIdList}
        # for cg_checker in self.cg_checkers.values():
        #     thread = threading.Thread(target=cg_checker.calTotalTime)
        #     thread.start()

    # def vs_handler(self,n_xyxycs):
    #     '''
    #     车流量统计
    #     '''
    #     vehicle_sort_count = SortCount()
    #     vehicle_sort_count.sortcount(n_xyxycs, Vehicle_sort_list.list)
    #     up_count, down_count = vehicle_sort_count.getSortCountResult()
    #     print(up_count, down_count)
    #
    #
    # def ps_handler(self, n_xyxycs):
    #     '''
    #     人流量统计
    #     '''
    #     people_sort = PeSortCount()
    #     people_sort_count = people_sort.pesortcount(n_xyxycs, person_sort_list.list)
    #     print(people_sort_count)
    #         # if current_timestamp - last_timestamp > MESSAGE.publishTime:
    #         #     print(people_sort_count)
    #             # 发布消息
    #             # eventData = event_data()
    #             # eventData.event_id = self.event_count % 65536
    #             # self.event_count += 1
    #             # eventData.event_type = 904
    def ad_handler(self, n_xyxycs):
        '''
       危险区域
        '''
        regional_sort = RegionalJudgmentSort()
        regional_sort.PersonJudgment(n_xyxycs, PersonDangerArea.roi)
        people_sorts = regional_sort.getPersonResult()
        if len(people_sorts['bbox'])>0:
            logger_danger_area_detect.info("Hazardous area detection results:{}".format(people_sorts))
            eventData = event_data()
            eventData.event_id = self.event_count % 65536
            self.event_count += 1
            # 行人闯入 405
            eventData.event_type = 405
            # 行人ID,经度,纬度,参与ID
            event_des = people_sorts["track_id"]
            eventData.description = '{' + str(event_des) + '}'
            eventData.event_pos.lat = 0
            eventData.event_pos.lon = 0
            eventData.event_pos.ele = 0
            eventData.event_obj_id = 3
            eventData.event_source = 5
            with self.lock:
                self.events_list.append(eventData.__dict__)
        else:
            logger_danger_area_detect.info("Currently no detection results,detection area:{}"
                                           .format(PersonDangerArea.roi))
    # def nl_handler(self, n_xyxycs):
    #     '''
    #     车辆排队数量，排队长度统计
    #     '''
    #     regional_sort = RegionalJudgmentSort()
    #     regional_sort.regional_judgment_length(n_xyxycs, PersonDangerArea.list)
    #     vehicle_sorts = regional_sort.getVehicleResult()
    #     if vehicle_sorts is not None:
    #         print(vehicle_sorts)
    #         # eventData = event_data()
    #         # eventData.event_id = self.event_count % 65536
    #         # self.event_count += 1
    #         # # 车辆逆行 405
    #         # eventData.event_type = 405
    #         # # 行人ID,经度,纬度,参与ID
    #         # event_des = vehicle_sorts["track_id"]
    #         # eventData.description = '{' + str(event_des) + '}'
    #         # eventData.event_pos.lat = 0
    #         # eventData.event_pos.lon = 0
    #         # eventData.event_pos.ele = 0
    #         # eventData.event_obj_id = 3
    #         # eventData.event_source = 5
    #         # with self.lock:
    #         #     self.events_list.append(eventData.__dict__)

    def callback_detection(self, results):
        # some logic to process the 2D detection data
        new_results = ros2bbox_2d(results)
        if new_results is not None:
            self.ad_handler(new_results)

    # def rd_handler(self, lane_id, vehicle_info):
    #     """
    #     逆行处理函数，收到对应id车道目标物数据后，传递给对应的逆行检测对象，
    #     逆行逻辑触发时，将事件结果添加到eventList中
    #     """
    #     result = self.rd_checkers[lane_id].checkForReverseDriving(
    #         vehicle_info.id, vehicle_info.carDir, vehicle_info.speed)
    #     if result == -1:
    #         pass
    #     else:
    #         eventData = event_data()
    #         eventData.event_id = self.event_count % 65536
    #         self.event_count += 1
    #         # 车辆逆行 904
    #         eventData.event_type = 904
    #         # 车辆ID,车道ID,经度,纬度,速度
    #         event_des = [result, lane_id, 0, 0, vehicle_info.speed]
    #         eventData.description = '{' + str(event_des)[1:-1] + '}'
    #         eventData.event_pos.lat = 0
    #         eventData.event_pos.lon = 0
    #         eventData.event_pos.ele = 0
    #         eventData.event_obj_id = result
    #         eventData.event_source = 5
    #         with self.lock:
    #             self.events_list.append(eventData.__dict__)

    # def cg_handler(self, lane_id, vehicle_info):
    #     """
    #     拥堵处理函数，将收到的车辆数据传给对应的拥堵检测对象，根据计算所得拥堵结果
    #     当拥堵程度大于流畅等级时，上报事件，同时将车辆计数和平均速度填充上报
    #     """
    #     # add obj
    #     self.cg_checkers[lane_id].addObject(vehicle_info.id)
    #     logger_congestion.info(
    #         "cg_handler recv "+str(lane_id)+" obj_id "+str(vehicle_info.id))
    #     # cal cg_level
    #     result = self.cg_checkers[lane_id].check_congestion_level()
    #     result_speed = self.cg_checkers[lane_id].get_average_speed()
    #     result_vc_count = self.cg_checkers[lane_id].getVehicleCounts()
    #     if result == CongestionLevel.LIGHT or result == CongestionLevel.MODERATE or result == CongestionLevel.SERIOUS:
    #         eventData = event_data()
    #         eventData.event_id = self.event_count % 65536
    #         self.event_count += 1
    #         # 交通拥堵 707
    #         eventData.event_type = 707
    #         # 拥堵程度(1轻-2中-3重), 排队长度, 车道ID、平均速度, 总车流量(单位时间流过检测横截面的车辆数), 车间距, 时间占有率, 空间占有率
    #         event_des = [result, 0, lane_id,
    #                      result_speed, result_vc_count, 0, 0, 0]
    #         eventData.description = '{' + str(event_des)[1:-1] + '}'
    #         eventData.event_pos.lat = 0
    #         eventData.event_pos.lon = 0
    #         eventData.event_pos.ele = 0
    #         eventData.event_obj_id = vehicle_info.id
    #         eventData.event_source = 5
    #         with self.lock:
    #             self.events_list.append(eventData.__dict__)

    # def callback_fusion(self, data):
    #     # parse ros fusion data
    #     for obj in data.objects:
    #         if obj.lane_id in LaneInfo.laneIdList:
    #             self.rd_handler(obj.lane_id, obj)
    #             self.cg_handler(obj.lane_id, obj)

        # some logic to process the perception fusion data
        # with self.lock:
        #     self.events_list.append(MESSAGE.event_data())

    def publish_states(self):
        now_python = datetime.now()
        publishDate = now_python.strftime("%Y-%m-%d %H:%M:%S")
        application_state = MESSAGE.application_state
        application_state['publishDate'] = publishDate
        application_state['installLocation'] = METAINFO.installLocation
        application_state['logLocation'] = METAINFO.logLocation
        application_state['description'] = METAINFO.description

        state_json_result = json.dumps(application_state)
        state_json_result = bytes(state_json_result, encoding="utf8")
        crc = self.crc16(state_json_result)

        self.state_message_msg.sof = 0x5A
        self.state_message_msg.version = 0x01
        self.state_message_msg.appId = 0x01
        self.state_message_msg.type = 0x01
        self.state_message_msg.codec = 0x01
        self.state_message_msg.sequence = 100
        self.state_message_msg.timestamp = rospy.Time.now().to_nsec() // 1000000
        self.state_message_msg.length = 100
        self.state_message_msg.payload = state_json_result
        self.state_message_msg.crc = crc & 0xFFFF

        self.state_pub.publish(self.state_message_msg)
        logger_state.info("The current status is normal")
        # self.state_pub_rate.sleep()
        # rospy.loginfo("Publsh state_message_msg[%d, %d, %d, %d, %d, %d, %s, %d, %s, %d ]",
        #               self.state_message_msg.sof, self.state_message_msg.version, self.state_message_msg.appId,
        #               self.state_message_msg.type,self.state_message_msg.codec,self.state_message_msg.sequence,
        #               self.state_message_msg.timestamp,self.state_message_msg.length,
        #               self.state_message_msg.payload, self.state_message_msg.crc)


    def publish_events(self):
        if len(self.events_list) > 0:
            application_data = MESSAGE.application_data
            now_python = datetime.datetime.now()
            # Get the current year
            current_year = datetime.datetime.now().year
            # Calculate the total number of minutes since the start of the year
            start_of_year = datetime.datetime(current_year, 1, 1)
            total_minutes = (now_python - start_of_year).total_seconds() // 60
            # Calculate the number of minutes that have passed so far this year
            current_minutes = now_python.hour * 60 + now_python.minute
            # Calculate the number of minutes that have already passed this year
            minutes_passed = total_minutes - current_minutes
            minute_offset = now_python.minute  # 计算当前时间在当天中已经过去的分钟数，即分钟内的偏移量
            # 计算当前时间的毫秒数，并添加到分钟偏移量中得到毫秒级时刻
            millisecond_offset = now_python.microsecond // 1000
            ms_time = minute_offset * 60 * 1000 + millisecond_offset
            # 将毫秒级时刻格式化为字符串
            ms_time_str = "{:03d}".format(ms_time)

            with self.lock:
                application_data["data"]["msg_cnt"] = (self.msg_count) % 256
                self.msg_count += 1
                application_data["data"]["min_of_year"] = minutes_passed
                application_data["data"]["second"] = ms_time_str
                application_data["data"]["fus_dev_id"] = "mec_esn"
                application_data["data"]["ref_pos"]["lat"] = 0
                application_data["data"]["ref_pos"]["lon"] = 0

                application_data["data"]["events_list"] = self.events_list
                # some logic to publish the events

                # and clear the events_list
                self.events_list = []

            data_json_result = json.dumps(application_data)
            data_json_result = bytes(data_json_result, encoding="utf8")
            crc = self.crc16(data_json_result)

            # 运维服务与应用交互报文 crc未做
            self.data_message_msg.sof = 0x5A
            self.data_message_msg.version = 0x01
            self.data_message_msg.appId = 0x01
            self.data_message_msg.type = 0x04
            self.data_message_msg.codec = 0x01
            self.data_message_msg.sequence = (
                self.data_message_msg.sequence) & 0xFFFF
            self.data_message_msg.sequence += 1
            self.data_message_msg.timestamp = rospy.get_rostime().to_sec()
            self.data_message_msg.length = len(data_json_result)
            self.data_message_msg.payload = data_json_result
            self.data_message_msg.crc = crc & 0xFFFF

            self.event_pub.publish(self.data_message_msg)
            # self.event_pub_rate.sleep()

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if current_time >= self.send_time1:
                self.publish_states()
                self.send_time1 = rospy.Time.now() + rospy.Duration(120.0)  # 设置下一次发送时间
            if current_time >= self.send_time2:
                self.publish_events()
                self.send_time2 = rospy.Time.now() + rospy.Duration(60.0)
            # rate = rospy.Rate(1)
            # rate.sleep()
            rospy.sleep(0.1)  # 控制循环的频率和响应时间

if __name__ == '__main__':
    event_handler = MessageHandler()
    event_handler.run()
