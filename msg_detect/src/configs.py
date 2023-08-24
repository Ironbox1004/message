from nb_log import LogManager
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

class METAINFO:
    installLocation = ROOT
    publishTime = 50
    logLocation = ROOT / 'msg_log'
    App_description = "*Application status information reporting*"

# 车道信息，车道数，车道id，对应行驶角度
class LaneInfo:
    laneNums = 4
    laneIdList = [2, 1, 5, 7]
    laneAngles = [90, 180, 160, 70]

class ReverseDriving:
    COUNT = 3 # 当前id车辆逆行次数超过这个值时，并且满足下面的SPEED_THRESHOLD行驶速度，就会上报逆行事件
    IS_ROAD_DIR = True
    # ROAD_DIR = 180
    SCOPE = 30.0 # 判断逆行的角度范围，如果车辆航向角在道路指定范围的反向+-SCOPE范围内，即可认为是逆行方向
    REMOVE_TIME = 60 # 当记录逆行车辆未达逆行检测次数，并后续时间内没有捕捉到车辆逆行，就将该id的逆行记录去掉
    VECTOR_SIZE = 100
    SPEED_THRESHOLD = 5


class Congestion:
    ROAD_LENGTH = 15  # meters  监控路段长度
    DEPARTURE_TIME = 5  # seconds  目标指定时间未出现后，认为目标离开监控路段
    TIME_INTERVAL = 15  # seconds  拥堵判断统计周期
    FREE_VELOCITY = 50  # km/h  道路的自由流速度
    CAL_INTERVAL = 10  # ms cal vector  定时计算总耗时的间隔


class VehicleSortList:  # 最多8条同时存在

    list = [
        [[476, 373], [567, 374]],
        [[648, 271], [760, 273]],
        [[742, 332], [893, 333]],
        [[912, 265], [1060, 266]],
        [[1067, 309], [1244, 310]],
        [[1217, 242], [1335, 244]]
    ]


class PersonDangerArea:
    roi_1 = [[1508, 99], [1394, 105], [1572, 293], [1848, 267], [1856, 133]]
    roi_2 = [[850, 23], [850, 179], [1128, 179], [1128, 23]]
    roi_3 = [[1445, 80], [1445, 328], [1842, 328], [1842, 80]]
    roi = [roi_1, roi_2, roi_3]


class PersonSortList:
    list = [
        [[800, 352], [700, 668]],
        [[960, 0], [954, 150]]
    ]


class MESSAGE:
    application_state = {
        "name": "message_detect",
        "version": "1",
        "publishDate": None,
        "installType": 0x02,
        "installLocation": None,
        "logLocation": None,
        "status": 0x01,
        "description": "****test message****"
    }
    report_data = {
        "type": None,
        "codec": 0x02,
        "data": [],
        "timestamp": None
    }

    PositionObject = {
        "lat": 0,
        "lon": 0,
        "ele": 0
    }

    # type1 业务消息
    data = {
        "type": 1,
        "ver": '01',
        "msg_cnt": 1,
        "min_of_year": None,
        "second": None,
        "fus_dev_id": "1",
        "ref_pos": PositionObject,
        "scene_type": 0,
        "objects_list": None,
        "events_list": [],
        "detected_region": None,
        "obstacle_list": None
    }
    application_data = {
        "type": 0x01,
        "codec": 0x02,
        "data": data
    }
    event_data = {
        "event_id": 0,
        "event_type": 1,
        "description": "None",
        "event_pos": None,
        "event_obj_id": None,
        "event_radius": None,
        "priority": None,
        "ref_path_list": None,
        "ref_link_list": None,
        "event_source": 5,
        "event_confid": None,
        "time_details": None
    }
    objects_list = {
        "object_id": None,
        "object_type": None,
        "data_source": 3,
        "object_pos": None,
        "pos_confid": None,
        "speed": None,
        "heading": None,
        "speed_confid": None,
        "heading_confid": None,
        "acceleration": None,
        "object_size": None,
        "vehicle_type": None,
        "lane_id": None,
        "plate_num": None,
        "plate_type": None,
        "plate_color": None,
        "vehicle_color": None,
        "obj_size_confid": None,
        "object_type_ext": None,
        "object_type_ext_confid": None,
        "acceleration_confid": None,
        "status_duration": None,
        "path_history": None,
        "path_prediction": None,
        "tracking": None,
        "polygon": None
    }


class DataMessage:
    sof = 0x5A
    version = 0x01
    appId = 0x01
    type = 0x04
    codec = 0x01
    sequence = 0x0001
    timestamp = None
    length = 100
    payload = None
    crc = 0xFFFF


class PositionObject:
    lat = 0
    lon = 0
    ele = 0


class event_data:
    event_id = 0
    event_type = 1
    description = "None"
    event_pos = PositionObject()
    event_obj_id = None
    event_radius = None
    priority = None
    ref_path_list = None
    ref_link_list = None
    event_source = 5
    event_confid = None
    time_details = None


class report_data:
    queueLength = []
    # timestamp = None




class Vehicle:
     def __init__(self,track_id,lane_id,heading,speed,lon,lat) -> None:
        self.track_id=track_id
        self.lane_id=lane_id
        self.heading=heading
        self.speed=speed
        self.lon=lon
        self.lat=lat

logger_danger_area_detect = \
    LogManager('PersonDangerArea').get_logger_and_add_handlers(10,
                                                               log_path=METAINFO.logLocation,
                                                               log_filename='danger_area_detect.log')

logger_vehicle_sort = \
    LogManager('vehicle_sort').get_logger_and_add_handlers(10,
                                                           log_path=METAINFO.logLocation,
                                                           log_filename='vehicle_sort.log')
logger_person_sort = \
    LogManager('person_sort').get_logger_and_add_handlers(10,
                                                           log_path=METAINFO.logLocation,
                                                           log_filename='person_sort.log')


logger_reverse_driving = \
    LogManager('ReverseDrivingDetector').get_logger_and_add_handlers(10,
                                                                     log_path=METAINFO.logLocation,
                                                                     log_filename='reverse_driving.log')

logger_congestion = \
    LogManager('CongestionDetector').get_logger_and_add_handlers(10,
                                                                 log_path=METAINFO.logLocation,
                                                                 log_filename='Congestion.log')
logger_vehicles = \
    LogManager('Vehicles').get_logger_and_add_handlers(10,
                                                       log_path=METAINFO.logLocation,
                                                       log_filename='vehicles.log')

logger_state = \
    LogManager('MsgState').get_logger_and_add_handlers(10,
                                                       log_path=METAINFO.logLocation,
                                                       log_filename='msg_state.log')
