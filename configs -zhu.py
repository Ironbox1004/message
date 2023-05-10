import numpy as np

CLASSES2D = ('Car', 'Bus', 'Cycling', 'Pedestrian', 'Special_Car', 'Truck',
              'Obstacle', 'Special_Target', 'Other_Objects')
class_to_idx = {cls: idx for idx, cls in enumerate(CLASSES2D)}
idx_list = [class_to_idx[cls] for cls in ["Car", "Bus", "Truck", "Pedestrian"]]


class METAINFO:

    engine2d_ckpt = "D://event_det//TensorRT-8.6.0.12.Windows10.x86_64.cuda-11.8//TensorRT-8.6.0.12//bin//yolox.engine"
    video_reader =  "D://event_det//video//2022-11-15-09-16-54_sensor_smgs_camera.mp4"

class COLORS:

    blue =  [255, 0, 0]
    yellow = [0, 255, 255]
    red = [0, 0, 255]
    green = [0, 255, 0]

class MESSAGE:

    application_state = {
        "name": "message_detect",
        "version": "1",
        "publishDate": None,
        "installType": 0x02,
        "installLocation": "/home/../ros/catkin_ws/src/..",
        "logLocation": "/home/../ros/catkin_ws/src/../log/",
        "status": 0x01,
        "description": "****test message****"
    }

    application_data = {
        "type": 0x01,
        "codec": 0x02,
        "data": None
    }

    data = {
        "type": 1,
        "ver": '01',
        "msg_cnt": 1,
        "min_of_year": None,
        "second": None,
        "fus_dev_id": "1",
        "ref_pos": None,
        "scene_type": 0,
        "objects_list": None,
        "events_list": None,
        "detected_region": None,
        "obstacle_list": None
    }

    events_list = {
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

class Vehicle_sort_list:
    list = [
        [[476,373],[567,374]],
        [[648,271],[760,273]],
        [[742,332],[893,333]],
        [[912,265],[1060,266]],
        [[1067,309],[1244,310]],
        [[1217,242],[1335,244]]

    ]
     
class Vehicle_area_list:
  list_pts_1 = [[482, 366], [257,588], [5, 933], [2, 1073], [95, 1076], [315, 706],[528, 437], [580, 352]]
  list_pts_2 = [[581, 346], [740, 332], [537, 702], [373, 1080], [96, 1072], [323, 697], [513, 458]]
  list_pts_3 = [[747, 330], [539, 716], [383, 1072], [756, 1075], [843, 604], [904, 316]]
  list_pts_4 = [[904, 316], [822, 746], [762, 1072], [1217, 1072], [1134, 609], [1060, 294]]
  list_pts_6 = [[1220, 281], [1456, 667], [1664, 1078], [1872, 1073], [1875, 906], [1749, 703],[1508,432], [1355, 259]]
  list_pts_5 = [[1067, 294], [1160, 708], [1228, 1070], [1654, 1072], [1445, 638], [1223, 291]]
  list = [list_pts_1,list_pts_2,list_pts_3,list_pts_4,list_pts_5,list_pts_6]

class Vehicle_line_head:
   p1=[520,360]
   p2=[650,340]
   p3=[820,320]
   p4=[1000,300]
   p5=[1150,286]
   p6=[1310,280]
   list=[p1,p2,p3,p4,p5,p6]
   
class danger_area:
  roi_1 = [[223, 128], [223,299], [447, 299], [447, 128]]
  roi_2 = [[850, 23],[850,179],[1128, 179],[1128, 23]]
  roi_3 = [[1445, 80], [1445,328],[1842,328],[1842, 80]]
  roi = [roi_1, roi_2, roi_3]


class person_sort_list:
  list = [
      [[925, 0],[1080, 925]]
          ]


class ReverseDriving:
    COUNT= 3
    IS_ROAD_DIR= True
    ROAD_DIR= 180
    SCOPE= 30.0
    REMOVE_TIME= 60
    VECTOR_SIZE= 100
    SPEED_THRESHOLD= 5

class Congestion:
    ROAD_LENGTH= 15  # meters
    DEPARTURE_TIME= 5  # seconds
    TIME_INTERVAL= 15  # seconds
    FREE_VELOCITY= 50  # km/h
    CAL_INTERVAL= 10  # ms cal vector
