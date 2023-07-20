from nb_log import LogManager


class METAINFO:
    installLocation = ""
    logLocation = "/home/chenzhen/ros_work/catkin_ws/log"
    description = "****test message****"

class ReverseDriving:
    COUNT = 3
    IS_ROAD_DIR = True
    ROAD_DIR = 180
    SCOPE = 30.0
    REMOVE_TIME = 60
    VECTOR_SIZE = 100
    SPEED_THRESHOLD = 5


class Congestion:
    ROAD_LENGTH = 15  # meters
    DEPARTURE_TIME = 5  # seconds
    TIME_INTERVAL = 15  # seconds
    FREE_VELOCITY = 50  # km/h
    CAL_INTERVAL = 10  # ms cal vector


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


class LaneInfo:
    laneNums = 4
    laneIdList = [2, 1, 5, 7]
    laneAngles = [90, 180, 160, 70]


logger_danger_area_detect = LogManager('PersonDangerArea').get_logger_and_add_handlers(10,
                                                                        log_path=METAINFO.logLocation,
                                                                        log_filename='danger_area_detect.log')

logger_vehicle_sort = LogManager('vehicle_sort').get_logger_and_add_handlers(10,
                                                                        log_path=METAINFO.logLocation,
                                                                        log_filename='vehicle_sort.log')

logger_reverse_driving = LogManager('ReverseDrivingDetector').get_logger_and_add_handlers(10,
                                                                        log_path=METAINFO.logLocation,
                                                                        log_filename='reverse_driving.log')

logger_congestion = LogManager('CongestionDetector').get_logger_and_add_handlers(10,
                                                                        log_path=METAINFO.logLocation,
                                                                        log_filename='Congestion.log')
logger_vehicles = LogManager('Vehicles').get_logger_and_add_handlers(10,
                                                                    log_path=METAINFO.logLocation,
                                                                    log_filename='vehicles.log')


logger_state = LogManager('MsgState').get_logger_and_add_handlers(10,
                                                                    log_path=METAINFO.logLocation,
                                                                    log_filename='msg_state.log')
