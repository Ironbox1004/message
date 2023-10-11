import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
sys.path.append(str(ROOT))
import datetime
import threading
from threading import Timer
import json
import rospy
from utils import *
from message_msgs.msg import Message
from camera_msgs.msg import results as xj_results
from lidar_msgs.msg import results as lidar_results
from configs import *
import crcmod.predefined


class MessageHandler:
    def __init__(self):

        self.vehicle_sort_label = [0, 1, 4, 5]
        self.person_sort_label = [3]

        # self.is_set_PublishTime = False if METAINFO.publishTime is None else True
        self.vehicle_last_timestamp = time.time()
        self.person_last_timestamp = time.time()

        self.events_list = []
        self.report_list = [[], []]
        self.events_lock = threading.Lock()
        self.report_lock = threading.Lock()

        self.report_list_vehicle = [[], [], []]
        self.report_lock_vehicle = threading.Lock()
        self.old_vehicle_data = [0] * 6
        self.old_vehicle_class_counter = [0] * 6

        self.report_list_person = [[], []]
        self.report_lock_person = threading.Lock()
        self.old_person_data = [0] * 6

        rospy.init_node('event_message', anonymous=True)

        # 结果消息初始化
        rospy.Subscriber("/vision_result", xj_results, self.callback_2Ddetection)
        rospy.Subscriber("/fusion/output/AlgVis", lidar_results, self.callback_3Ddetection)

        # 事件触发类消息上报，包含设备状态
        self.event_pub = rospy.Publisher('/event_topic', Message, queue_size=1)
        self.state_pub = rospy.Publisher('/event_topic', Message, queue_size=1)

        # 事件统计类消息上报
        self.report_pub = rospy.Publisher('/report_topic', Message, queue_size=1)
        self.report_pub_withtime = rospy.Publisher('/report_topic', Message, queue_size=1)

        # 事件触发类消息初始化
        self.state_message_msg = Message()
        self.data_message_msg = Message()
        # 业务数据上报
        self.data_report_msg = Message()

        self.send_time_states = rospy.Time.now() + rospy.Duration(1.0)
        self.send_time_report_vehicle_data = rospy.Time.now() + rospy.Duration(1.0)
        self.send_time_report_person_data = rospy.Time.now() + rospy.Duration(1.0)

        # 检测函数初始化
        # self.people_sort = VeSortCount()
        self.vehicle_sort_count = SortCount()
        self.person_sort_count = SortCount()

        self.crc16 = crcmod.predefined.mkPredefinedCrcFun('crc16')
        # 给运维模块发送的消息计数，报文序列号，范围是0x0001~0xFFFF
        self.state_message_count = 0x0001
        # Type1 消息计数 每次往外发布消息，自增1，范围是0-255
        self.msg_count = 0
        # event_id 事件计数 每次触发事件，自增1，范围是0-65535
        self.event_count = 0
        # 统计类消息计数 每次往外发布消息，自增1，范围是0-32
        self.report_count = 0

        self.lanes_num = LaneInfo.laneNums
        self.rd_checkers = {lane_id: ReverseDrivingDetector(
            lane_angle) for lane_id, lane_angle in zip(LaneInfo.laneIdList, LaneInfo.laneAngles)}

        # 拥堵检测对象字典 同时开启车辆通过总耗时计算线程
        self.cg_checkers = {lane_id: CongestionDetector()
                            for lane_id in LaneInfo.laneIdList}
        for cg_checker in self.cg_checkers.values():
            thread = threading.Thread(target=cg_checker.calTotalTime)
            thread.start()

    def vs_handler(self, results_2d):
        '''
        车流量统计
        '''
        bboxes = results_2d[0]["bboxes_results"]
        reportData = report_data()
        reportData.AllCounts = []
        reportData.ClassCounter = []

        self.vehicle_sort_count.SortCountCal(bboxes, VehicleSortList.list, self.vehicle_sort_label)
        class_counter, all_counts = self.vehicle_sort_count.getSortCountResults()

        reportData.ClassCounter.append(class_counter)
        reportData.AllCounts.append(all_counts)

        logger_vehicle_sort.info("当前车流量为:{}".
                                 format(all_counts))
        with self.report_lock_vehicle:
            self.report_list_vehicle[0] = ["0x05"]
            self.report_list_vehicle[1] = reportData.AllCounts
            self.report_list_vehicle[2] = reportData.ClassCounter

    def ps_handler(self, results_2d):
        '''
        人流量统计
        '''
        bboxes = results_2d[0]["bboxes_results"]
        reportData = report_data()
        reportData.AllCounts = []
        self.person_sort_count.SortCountCal(bboxes, PersonSortList.list, self.person_sort_label)
        class_counter, all_counts = self.person_sort_count.getSortCountResults()

        reportData.AllCounts.append(all_counts)
        logger_person_sort.info("当前人流量为:{}".
                                format(all_counts))

        with self.report_lock_person:
            self.report_list_person[0] = ["0x04"]
            self.report_list_person[1] = reportData.AllCounts

    def ad_handler(self, results_2d):
        '''
       危险区域
        '''
        bboxes = results_2d[0]["bboxes_results"]
        frame_id = results_2d[0]["frame_id"]
        frame_name = results_2d[0]["frame_name"]

        regional_sort = RegionalJudgmentSort()

        for key, value in PersonDangerArea.camera.items():
            if key == frame_id[0]:
                detect_roi = value

                regional_sort.PersonJudgment(bboxes, detect_roi)
                people_sorts = regional_sort.getPersonResult()
                if len(people_sorts['bbox']) > 0:
                    logger_danger_area_detect.info("检测区域中出现行人:{},摄像头ID为：{},摄像头名：{}".
                                                   format(people_sorts, frame_id[0], frame_name[0]))
                    eventData = event_data()
                    eventData.event_id = self.event_count % 65536
                    self.event_count += 1
                    # 行人闯入 405
                    eventData.event_type = 405
                    eventData.even_frame_id = frame_id[0]
                    eventData.even_frame_name = frame_name[0]
                    # 行人ID,经度,纬度,参与ID
                    # TODO: 上报结果结构待定
                    event_des = people_sorts
                    eventData.description = '{' + str(event_des) + '}'
                    eventData.event_pos.lat = 0
                    eventData.event_pos.lon = 0
                    eventData.event_pos.ele = 0
                    eventData.event_obj_id = 3
                    eventData.event_source = 5
                    with self.events_lock:
                        self.events_list.append(eventData.__dict__)

                else:
                    pass

            else:
                pass

            # else:
            #     logger_danger_area_detect.info("检测区域中未出现行人:{},摄像头ID为：{},摄像头名：{}".
            #                                    format(detect_roi,frame_id[0],frame_name[0]))

    def nl_handler(self, results_3d):
        '''
        车辆排队数量，排队长度统计
        '''
        reporData = report_data()
        vechicle_cal = RegionalJudgmentSort()
        vechicle_cal.VehicleJudgment(results_3d)
        vechicle_lens, vechicle_nums = vechicle_cal.getVehicleResult()

        for key1, key2 in zip(vechicle_lens, vechicle_nums):
            reporData.queueLength = []
            value1 = vechicle_lens[key1]
            value2 = vechicle_nums[key2]
            if len(value1) == 0 or key1 == -1:
                continue
            else:
                logger_vehicles.info(
                    "当前:{},车道的车辆数量为:{},长度为:{}".format(key1, max(value2), max(value1))
                )
                results = {
                    "lane_id": key1,
                    "queueLenth": max(value1) if value1 else 0,
                    "queueNum": max(value2) if value2 else 0
                }  # 行人ID,经度,纬度,参与ID
            reporData.queueLength.append(results)
        with self.report_lock:
            self.report_list[0] = ["0x03"]
            self.report_list[1] = reporData.queueLength
        # print(self.report_list)

    def rd_handler(self, lane_id, vehicle_info):
        """
        逆行处理函数,收到对应id车道目标物数据后,传递给对应的逆行检测对象,
        逆行逻辑触发时,将事件结果添加到eventList中
        """
        result = self.rd_checkers[lane_id].checkForReverseDriving(
            vehicle_info.track_id, vehicle_info.heading, vehicle_info.speed)
        logger_reverse_driving.info("rd_handler ")
        if result == -1:
            pass
        else:
            eventData = event_data()
            eventData.event_id = self.event_count % 65536
            self.event_count += 1
            # 车辆逆行 904
            eventData.event_type = 904
            # 车辆ID,车道ID,经度,纬度,速度
            event_des = [result, lane_id, 0, 0, vehicle_info.speed]
            eventData.description = '{' + str(event_des)[1:-1] + '}'
            eventData.event_pos.lat = 0
            eventData.event_pos.lon = 0
            eventData.event_pos.ele = 0
            eventData.event_obj_id = result
            eventData.event_source = 5
            with self.events_lock:
                self.events_list.append(eventData.__dict__)

    def cg_handler(self, lane_id, vehicle_info):
        """
        拥堵处理函数，将收到的车辆数据传给对应的拥堵检测对象，根据计算所得拥堵结果
        当拥堵程度大于流畅等级时，上报事件，同时将车辆计数和平均速度填充上报
        """
        # add obj
        self.cg_checkers[lane_id].addObject(vehicle_info.track_id)
        # logger.info(
        #     "cg_handler recv laneid"+str(lane_id)+" obj_id "+str(vehicle_info.track_id))

        result_speed = self.cg_checkers[lane_id].get_average_speed()
        # logger.info(
        #     "cg_handler recv laneid"+str(lane_id)+" obj_id average_speed "+str(result_speed))

        # # cal cg_level
        result = self.cg_checkers[lane_id].check_congestion_level()
        # logger.info(
        #     "cg_handler recv laneid"+str(lane_id)+" obj_id congestion_level"+str(result))

        # # cal cg_level
        result_time = self.cg_checkers[lane_id].getTotalTime()
        # logger.info(
        #     "cg_handler recv laneid"+str(lane_id)+" obj_id total_time "+str(result_time))

        result_vc_count = self.cg_checkers[lane_id].getVehicleCounts()
        logger_congestion.info(
            "cg_handler recv laneid" +
            str(lane_id) + " obj_id " + str(vehicle_info.track_id)
            + "  average_speed " + str(result_speed) +
            "  congestion_level" + str(result)
            + "  total_time " + str(result_time) + "  VehicleCounts " + str(result_vc_count))
        if result == CongestionLevel.LIGHT or result == CongestionLevel.MODERATE or result == CongestionLevel.SERIOUS:
            logger_congestion.info("congest event 707!!!")
            eventData = event_data()
            eventData.event_id = self.event_count % 65536
            self.event_count += 1
            # 交通拥堵 707
            eventData.event_type = 707
            # 拥堵程度(1轻-2中-3重), 排队长度, 车道ID、平均速度, 总车流量(单位时间流过检测横截面的车辆数), 车间距, 时间占有率, 空间占有率
            event_des = [result, 0, lane_id,
                         result_speed, result_vc_count, 0, 0, 0]
            eventData.description = '{' + str(event_des)[1:-1] + '}'
            eventData.event_pos.lat = 0
            eventData.event_pos.lon = 0
            eventData.event_pos.ele = 0
            eventData.event_obj_id = vehicle_info.track_id
            eventData.event_source = 5
            with self.events_lock:
                self.events_list.append(eventData.__dict__)

    def callback_2Ddetection(self, results_2d):
        # some logic to process the 2D detection data
        new_results_2d = ros2bbox_2d(results_2d)
        if new_results_2d is not None:
            # TODO: 是否需要实现线程池化
            self.ps_handler(new_results_2d)
            self.vs_handler(new_results_2d)
            self.ad_handler(new_results_2d)

    def callback_3Ddetection(self, results_3d):
        # some logic to process the 3D detection data
        new_results_3d = ros2bbox_3d(results_3d)
        if new_results_3d is not None:
            self.nl_handler(new_results_3d)

    def callback_3Ddetection(self, results_3d):
        # some logic to process the 3D detection data
        new_results_3d = ros2bbox_3d(results_3d)
        if new_results_3d is not None:
            self.nl_handler(new_results_3d)

        for obj in results_3d.result3D:
            # 0汽车 1自行车 2行人 3公交车  4卡车八百 5工程车 6三轮车 7无人车 8障碍物 9无人骑行
            if obj.cls == 0 or obj.cls == 3 or obj.cls == 4 or obj.cls == 5:
                road_id = int(obj.road_id) if obj.road_id != '' else int(-1)

                speed = math.sqrt(
                    pow(obj.velocity[0], 2) + pow(obj.velocity[1], 2))
                heading = obj.rotation * 180 / math.pi
                ve = Vehicle(obj.track_id, 0, heading, speed,
                             obj.longitude, obj.latitude)
                self.cg_handler(road_id, ve)
                self.rd_handler(road_id, ve)

    def publish_states(self):

        now_python = datetime.datetime.now()
        publishDate = now_python.strftime("%Y-%m-%d %H:%M:%S")
        application_state = MESSAGE.application_state
        application_state['publishDate'] = publishDate
        application_state['installLocation'] = str(METAINFO.installLocation)
        application_state['logLocation'] = str(METAINFO.logLocation)
        application_state['description'] = METAINFO.App_description

        state_json_result = json.dumps(application_state)
        state_json_result = bytes(state_json_result, encoding="utf8")
        crc = self.crc16(state_json_result)

        self.state_message_msg.sof = 0x5A
        self.state_message_msg.version = 0x01
        self.state_message_msg.appId = 0x01
        self.state_message_msg.type = 0x01
        self.state_message_msg.codec = 0x01
        self.state_message_msg.sequence = self.state_message_count % 65535
        self.state_message_count += 1
        self.state_message_msg.timestamp = rospy.Time.now().to_nsec() // 1000000
        self.state_message_msg.length = len(state_json_result)
        self.state_message_msg.payload = state_json_result
        self.state_message_msg.crc = crc & 0xFFFF

        self.state_pub.publish(self.state_message_msg)
        logger_state.info("当前事件检测服务运行状态良好！")
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
            minute_offset = now_python.minute  # 计算当前时间在当天中已经过去的分钟数，即分钟内的偏移量
            minutes_passed = now_python.strftime("%Y-%m-%d %H:%M:%S")
            # 计算当前时间的毫秒数，并添加到分钟偏移量中得到毫秒级时刻
            millisecond_offset = now_python.microsecond // 1000
            ms_time = minute_offset * 60 * 1000 + millisecond_offset
            # 将毫秒级时刻格式化为字符串
            ms_time_str = "{:03d}".format(ms_time)

            with self.events_lock:
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
            self.data_message_msg.sequence = (self.data_message_msg.sequence) & 0xFFFF
            self.data_message_msg.sequence += 1
            self.data_message_msg.timestamp = rospy.get_rostime().to_sec()
            self.data_message_msg.length = len(data_json_result)
            self.data_message_msg.payload = data_json_result
            self.data_message_msg.crc = crc & 0xFFFF

            self.event_pub.publish(self.data_message_msg)
            # self.event_pub_rate.sleep()

    def publish_report_data(self):
        # 发送不需设置发送时间的报文
        if len(self.report_list[1]) > 0:
            # print(self.report_list)
            report_data = MESSAGE.report_data
            now_python = datetime.datetime.now()
            minute_offset = now_python.minute  # 计算当前时间在当天中已经过去的分钟数，即分钟内的偏移量
            # 计算当前时间的毫秒数，并添加到分钟偏移量中得到毫秒级时刻
            millisecond_offset = now_python.microsecond // 1000
            ms_time = minute_offset * 60 * 1000 + millisecond_offset
            # 将毫秒级时刻格式化为字符串
            ms_time_str = "{:03d}".format(ms_time)

            with self.report_lock:
                # print(self.report_list)
                self.report_count += 1
                report_data["type"] = int(self.report_list[0][0], 16)
                report_data["data"] = self.report_list[1]
                report_data["timestamp"] = ms_time_str
                self.report_list = [[], []]

            data_json_result = json.dumps(report_data)
            data_json_result = bytes(data_json_result, encoding="utf8")
            crc = self.crc16(data_json_result)

            # 交通统计信息上报
            self.data_report_msg.sof = 0x5A
            self.data_report_msg.version = 0x01
            self.data_report_msg.appId = 0x01
            self.data_report_msg.type = 0x04
            self.data_report_msg.codec = 0x01
            self.data_report_msg.sequence = self.report_count % 0xFFFF
            self.data_report_msg.timestamp = rospy.get_rostime().to_sec()
            self.data_report_msg.length = len(data_json_result)
            self.data_report_msg.payload = data_json_result
            self.data_report_msg.crc = crc & 0xFFFF

            self.report_pub.publish(self.data_report_msg)

    def publish_report_vehicle_data(self):
        if len(self.report_list_vehicle[1]) > 0:
            # print(self.report_list)
            report_data = MESSAGE.report_data
            now_python = datetime.datetime.now()
            minute_offset = now_python.minute  # 计算当前时间在当天中已经过去的分钟数，即分钟内的偏移量
            # 计算当前时间的毫秒数，并添加到分钟偏移量中得到毫秒级时刻
            millisecond_offset = now_python.microsecond // 1000
            ms_time = minute_offset * 60 * 1000 + millisecond_offset
            # 将毫秒级时刻格式化为字符串
            ms_time_str = "{:03d}".format(ms_time)

            with self.report_lock_vehicle:
                self.report_count += 1
                NowClassList = [self.report_list_vehicle[2][0].get(item, 0) for item in self.vehicle_sort_label]
                report_data["type"] = int(self.report_list_vehicle[0][0], 16)
                TrafficFlow = [x - y for x, y in zip(self.report_list_vehicle[1][0], self.old_vehicle_data)]
                report_data["Flow"] = {index: value for index, value in enumerate(TrafficFlow)}
                ClassFlow = [x - y for x, y in zip(NowClassList, self.old_vehicle_class_counter)]

                logger_vehicle_sort.info("当前车流量差值为:{},路口通行车辆类别为:{}".
                                         format(TrafficFlow, ClassFlow))

                report_data["timestamp"] = ms_time_str
                self.old_vehicle_data = self.report_list_vehicle[1][0]
                self.old_vehicle_class_counter = [self.report_list_vehicle[2][0].get(item, 0)
                                                  for item in self.vehicle_sort_label]
                self.report_list_vehicle = [[], [], []]

            data_json_result = json.dumps(report_data)
            data_json_result = bytes(data_json_result, encoding="utf8")
            crc = self.crc16(data_json_result)

            # 交通统计信息上报
            self.data_report_msg.sof = 0x5A
            self.data_report_msg.version = 0x01
            self.data_report_msg.appId = 0x01
            self.data_report_msg.type = 0x04
            self.data_report_msg.codec = 0x01
            self.data_report_msg.sequence = self.report_count % 0xFFFF
            self.data_report_msg.timestamp = rospy.get_rostime().to_sec()
            self.data_report_msg.length = len(data_json_result)
            self.data_report_msg.payload = data_json_result
            self.data_report_msg.crc = crc & 0xFFFF

            self.report_pub_withtime.publish(self.data_report_msg)

    def publish_report_person_data(self):
        if len(self.report_list_person[1]) > 0:
            # print(self.report_list)
            report_data = MESSAGE.report_data
            now_python = datetime.datetime.now()
            minute_offset = now_python.minute  # 计算当前时间在当天中已经过去的分钟数，即分钟内的偏移量
            # 计算当前时间的毫秒数，并添加到分钟偏移量中得到毫秒级时刻
            millisecond_offset = now_python.microsecond // 1000
            ms_time = minute_offset * 60 * 1000 + millisecond_offset
            # 将毫秒级时刻格式化为字符串
            ms_time_str = "{:03d}".format(ms_time)

            with self.report_lock_person:
                # print(self.report_list)
                self.report_count += 1
                # print("oldperson_data", self.old_person_data)
                report_data["type"] = int(self.report_list_person[0][0], 16)
                PersonFlow = [x - y for x, y in zip(self.report_list_person[1][0], self.old_person_data)]
                report_data["Flow"] = {index: value for index, value in enumerate(PersonFlow)}
                logger_person_sort.info("当前人流量差值为:{}".
                                        format(PersonFlow))
                report_data["timestamp"] = ms_time_str
                self.old_person_data = self.report_list_person[1][0]
                self.report_list_person = [[], []]

            data_json_result = json.dumps(report_data)
            data_json_result = bytes(data_json_result, encoding="utf8")
            crc = self.crc16(data_json_result)
            # 交通统计信息上报
            self.data_report_msg.sof = 0x5A
            self.data_report_msg.version = 0x01
            self.data_report_msg.appId = 0x01
            self.data_report_msg.type = 0x04
            self.data_report_msg.codec = 0x01
            self.data_report_msg.sequence = self.report_count % 0xFFFF
            self.data_report_msg.timestamp = rospy.get_rostime().to_sec()
            self.data_report_msg.length = len(data_json_result)
            self.data_report_msg.payload = data_json_result
            self.data_report_msg.crc = crc & 0xFFFF

            self.report_pub_withtime.publish(self.data_report_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_events()
            self.publish_report_data()
            current_time = rospy.Time.now()
            if current_time >= self.send_time_states:
                self.publish_states()
                self.send_time_states = rospy.Time.now() + rospy.Duration(METAINFO.state_publishTime)  # 设置下一次发送时间
            if current_time >= self.send_time_report_vehicle_data:
                self.publish_report_vehicle_data()
                self.send_time_report_vehicle_data = rospy.Time.now() + rospy.Duration(
                    METAINFO.statistics_publishTime)  # 设置下一次发送时间
            # if current_time >= self.send_time_report_person_data:
            #     self.publish_report_person_data()
            #     self.send_time_report_person_data = rospy.Time.now() + rospy.Duration(
            #         METAINFO.statistics_publishTime)  # 设置下一次发送时间
            # rate = rospy.Rate(1)
            # rate.sleep()
            rospy.sleep(0.1)  # 控制循环的频率和响应时间


if __name__ == '__main__':
    event_handler = MessageHandler()
    event_handler.run()
