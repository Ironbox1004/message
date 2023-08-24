from collections import Counter, deque
import math
from configs import *
import time
from enum import Enum
import datetime
from typing import Dict
import threading


def merge_list(*args,
               fill_value=None
               ):
    max_length = max([len(lst)
                      for lst in args])
    result = []
    for i in range(max_length):
        result.append([
            args[k][i]
            if i < len(args[k])
            else fill_value for k in range(len(args))
        ])

    return result


def ros2bbox_2d(ros_result
                ):
    top_lefts_x, top_lefts_y = [], []
    bottom_rights_x, bottom_rights_y = [], []
    track_ids = []
    labels = []

    for obs in ros_result.obs:

        for bbox_result in obs.obs_box:
            top_lefts_x.append(int(bbox_result.top_left.x))
            top_lefts_y.append(int(bbox_result.top_left.y))
            bottom_rights_x.append(int(bbox_result.bottom_right.x))
            bottom_rights_y.append(int(bbox_result.bottom_right.y))
            track_ids.append(int(bbox_result.track_id))
            labels.append(int(bbox_result.box_type))

        new_results_2d = merge_list(top_lefts_x,
                                    top_lefts_y,
                                    bottom_rights_x,
                                    bottom_rights_y,
                                    track_ids,
                                    labels, fill_value='_')

    return new_results_2d


def ros2bbox_3d(ros_result
                ):
    road_ids = []
    points_x = []
    points_y = []

    for result in ros_result.result3D:
        result.road_id = int(
            result.road_id) if result.road_id != '' else int(-1)

        road_ids.append(result.road_id)
        points_x.append(result.center[0])
        points_y.append(result.center[1])

    new_results_3d = merge_list(road_ids,
                                points_x,
                                points_y,
                                fill_value='_')

    return new_results_3d


class RegionalJudgmentSort:

    def __init__(self):
        self.in_bbox = {}
        self.result_now = {}
        self.bbox = []
        self.track_id = []
        self.classified_lists = {}
        self.distances = {}
        self.nums = {}

    def _is_poi_in_poly(self,
                        pt,
                        poly
                        ):
        """
        judge whether the point is in the polygon
        :param pt: point [x,y]
        :param poly: poly point [[x1,y1],[x2,y2],...]
        :return: point in polygon -> True, point not in polygon -> False
        """
        nvert = len(poly)
        vertx = []
        verty = []
        testx = pt[0]
        testy = pt[1]
        for item in poly:
            vertx.append(item[0])
            verty.append(item[1])
        j = nvert - 1
        res = False
        for i in range(nvert):
            if (verty[j] - verty[i]) == 0:
                j = i
                continue
            x = (vertx[j] - vertx[i]) * (testy - verty[i]) / \
                (verty[j] - verty[i]) + vertx[i]
            if ((verty[i] > testy) != (verty[j] > testy)) and (testx < x):
                res = not res
            j = i
        return res

    def _in_poly_area_dangerous(self,
                                xyxy,
                                area_poly
                                ):
        """
        judge whether the object is in the dangerous area
        :param xyxy: bbox of the object
        :return: True -> in danger，False -> Not in danger
        """
        # print(area_poly)
        if not area_poly:  # 为空
            return False
        # 求物体框的中点
        object_x1 = int(xyxy[0])
        object_y1 = int(xyxy[1])
        object_x2 = int(xyxy[2])
        object_y2 = int(xyxy[3])

        object_w = object_x2 - object_x1
        object_h = object_y2 - object_y1
        object_cx = object_x1 + (object_w / 2)
        object_cy = object_y1 + (object_h / 2)

        return self._is_poi_in_poly(
            [object_cx, object_cy],
            area_poly
        )

    def _euclidean_distance(self,
                            point1: int,
                            point2: int
                            ):

        x1, y1 = point1
        x2, y2 = point2

        return round(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2), 2)

    def PersonJudgment(self,
                       sort_results: list,
                       roi: list
                       ):
        """
        :param sort_results: sort results
        :param roi: dangerous area
        :return: bbox and track_id in dangerous area
        """
        for track in sort_results:
            bbox = track[:4]
            label = track[-1]
            track_id = track[4]

            if label == 3:
                for i in range(len(roi)):

                    if self._in_poly_area_dangerous(bbox,
                                                    roi[i]) == True:
                        self.bbox.append(bbox)

                        self.track_id.append(track_id)
        self.in_bbox = {
            "bbox": self.bbox,
            "track_id": self.track_id
        }

    def VehicleJudgment(self,
                        lists):
        self.classified_lists.clear()
        self.distances.clear()
        self.nums.clear()
        for sublist in lists:
            key = sublist[0]
            if key in self.classified_lists:
                self.classified_lists[key].append(sublist)
            else:
                self.classified_lists[key] = [sublist]

        for key, sublist in self.classified_lists.items():
            self.nums[key] = []
            self.distances[key] = []
            for i in range(len(sublist)):
                self.nums[key].append(len(sublist))

                for j in range(i + 1, len(sublist)):
                    distance = self._euclidean_distance(sublist[i][1:3],
                                                        sublist[j][1:3])
                    self.distances[key].append(distance)

    def getPersonResult(self):
        return self.in_bbox

    def getVehicleResult(self):

        return self.distances, self.nums


class VeSortCount:
    def __init__(self,
                 up_count=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 down_count=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 already_counted=deque(maxlen=500),
                 class_counter=Counter(),
                 paths={},
                 total_counter=0,
                 track_cls=0,
                 total_track=0,
                 last_track_id=-1,
                 angle=-1
                 ):
        self.already_counted = already_counted
        self.class_counter = class_counter
        self.paths = paths
        self.total_counter, self.track_cls, self.up_count, self.down_count, self.total_track = \
            total_counter, track_cls, up_count, down_count, total_track
        self.last_track_id = last_track_id
        self.angle = angle

    def _tlbr_midpoint(self,
                       box: list):
        """
        :param box: [x1, y1, x2, y2]
        :return: midpoint of box
        """
        minX, minY, maxX, maxY = box
        # minus y coordinates to get proper xy format
        midpoint = (int((minX + maxX) / 2), int((minY + maxY) / 2))
        return midpoint

    def _ccw(self,
             A, B, C):
        # 判断三个点A、B、C是否组成了一个逆时针方向的三角形
        """
        :param A: point A
        :param B: point B
        :param C: point C
        :return: True if A, B, C are in counter-clockwise order
        """
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def _intersect(self,
                   A, B, C, D):
        # 判断两条线段AB和CD是否相交
        """
        :param A: point A
        :param B: point B
        :param C: point C
        :param D: point D
        :return: True if line segments AB and CD intersect
        """
        return self._ccw(A, C, D) != self._ccw(B, C, D) and self._ccw(A, B, C) != self._ccw(A, B, D)

    def _vector_angle(self,
                      midpoint,
                      previous_midpoint):
        # 计算两个点构成的向量与x轴正方向之间的夹角
        """
        :param midpoint: current midpoint
        :param previous_midpoint: previous midpoint
        :return: angle between midpoint and previous_midpoint
        """
        x = midpoint[0] - previous_midpoint[0]
        y = midpoint[1] - previous_midpoint[1]
        return math.degrees(math.atan2(y, x))

    def veSortCount(self,
                    sort_results,
                    line: list):
        if len(sort_results) > 0:
            for track in sort_results:
                bbox = track[:4]
                track_id = int(track[4])
                label = track[-1]
                midpoint = self._tlbr_midpoint(bbox)
                # 1080==im.shape[0]
                origin_midpoint = (midpoint[0], 1920 - midpoint[1])

                if track_id not in self.paths and label in [0, 1, 4, 5]:
                    self.paths[track_id] = deque(maxlen=2)
                    self.total_track = track_id

                self.paths[track_id].append(midpoint)
                previous_midpoint = self.paths[track_id][0]
                origin_previous_midpoint = (
                    previous_midpoint[0], 1920 - previous_midpoint[1])  # 1080==im.shape[0]

                for i in range(len(line)):

                    if self._intersect(midpoint, previous_midpoint, line[i][0], line[i][1]) \
                            and track_id not in self.already_counted:

                        self.class_counter[self.track_cls] += 1
                        self.total_counter += 1

                        # Set already counted for ID to true.
                        self.already_counted.append(track_id)

                        self.angle = self._vector_angle(
                            origin_midpoint, origin_previous_midpoint)

                        if self.angle > 0:
                            self.up_count[i] += 1

                        if self.angle < 0:
                            self.down_count[i] += 1

    def getVeSortCountResult(self):
        return self.up_count, self.down_count


class PeSortCount:
    def __init__(self,
                 boxes=[],
                 label=[],
                 indexIDs=[],
                 memory={},
                 count=[0, 0, 0, 0]):
        super().__init__()
        self.boxes = boxes
        self.label = label
        self.indexIDs = indexIDs
        self.memory = memory
        self.count = count

    def _ccw(self, A, B, C):
        """
        :param A: point A
        :param B: point B
        :param C: point C
        :return: True if A, B, C are in counter-clockwise order
        """
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def _intersect(self, A, B, C, D):
        """
        :param A: point A
        :param B: point B
        :param C: point C
        :param D: point D
        :return: True if line segments AB and CD intersect
        """
        return self._ccw(A, C, D) != self._ccw(B, C, D) and self._ccw(A, B, C) != self._ccw(A, B, D)

    def peSortCount(self, sort_results, roi):
        self.previous = self.memory.copy()
        self.indexIDs.clear()
        self.boxes.clear()
        self.label.clear()
        for track in sort_results:
            self.boxes.append([track[0], track[1], track[2], track[3]])
            self.indexIDs.append(int(track[4]))
            self.memory[self.indexIDs[-1]] = self.boxes[-1]
            self.label.append(int(track[-1]))
            i = int(0)
            for box, label in zip(self.boxes, self.label):
                (x, y) = (int(box[0]), int(box[1]))
                (w, h) = (int(box[2]), int(box[3]))

                if self.indexIDs[i] != -1 and self.indexIDs[i] in self.previous and label == 3:
                    previous_box = self.previous[self.indexIDs[i]]
                    (x2, y2) = (int(previous_box[0]), int(previous_box[1]))
                    (w2, h2) = (int(previous_box[2]), int(previous_box[3]))
                    p1 = (int(x2 + (w2 - x2) / 2), int(y2 + (h2 - y2) / 2))
                    p0 = (int(x + (w - x) / 2), int(y + (h - y) / 2))

                    for j in range(len(roi)):
                        if self._intersect(p0, p1, roi[j][0], roi[j][1]):
                            self.count[j] += 1
                            del self.previous[self.indexIDs[i]]
                i += 1

    def getPeSortCountResult(self):
        return self.count


class ReverseVehicle:
    def __init__(self, count: int, last_seen_time: float):
        self.count = count
        self.last_seen_time = last_seen_time


class ReverseDrivingDetector:
    def isAngleInReverseRange(self, carDir: float) -> bool:
        # logger_reverse_driving.info(
        #     "isAngleInReverseRange :"+"road_dir="+str(self.ROAD_DIR)+" carDir="+str(carDir))
        # 计算角度ROAD_DIR的反向角度ra
        ra = self.ROAD_DIR + 180 if self.ROAD_DIR < 180 else self.ROAD_DIR - 180

        # 计算角度b与ra的差值db，考虑边界情况
        db = carDir - ra
        # logger_reverse_driving.info(
        #     "isAngleInReverseRange :"+" ra= "+str(ra)+" db="+str(db))
        if db > 180:
            db -= 360
        elif db < -180:
            db += 360

        # 判断角度b是否在角度a的反向范围内上下30度
        return db >= -ReverseDriving.SCOPE and db <= ReverseDriving.SCOPE

    def checkForReverseDriving(self, id: int, carDir: float, speed: float) -> int:
        if self.isAngleInReverseRange(carDir) and speed >= ReverseDriving.SPEED_THRESHOLD:
            # logger.info(
            #     "checkForReverseDriving: "+str(id)+" ReverseDriving")
            current_time = time.monotonic()
            # 去掉map中触发逆行较久的数据
            for i in list(self.count_map.keys()):
                elapsed_time = current_time - self.count_map[i].last_seen_time
                if elapsed_time > ReverseDriving.REMOVE_TIME:
                    del self.count_map[i]

            if id not in self.count_map:
                self.count_map[id] = ReverseVehicle(1, current_time)
            else:
                self.count_map[id].count += 1
                self.count_map[id].last_seen_time = current_time

            logger_reverse_driving.info(
                "checkForReverseDriving:id"+id+" count"+self.count_map[id].count)

            # 当检测到当前id车辆的逆行次数超过ReverseDriving.COUNT时，触发逆行事件报警，并将该车的逆行计数清零
            if self.count_map[id].count >= ReverseDriving.COUNT:
                # logger_reverse_driving.info(
                #     "checkForReverseDriving: "+str(id)+" ReverseDriving")
                self.count_map[id].count = 0
                return id
            else:
                return -1
        else:
            # logger_reverse_driving.info(
            #     "checkForReverseDriving: "+str(id)+" no ReverseDriving")
            return -1

    def calcMedian(self, angles) -> float:
        n = len(angles)
        if n % 2 == 0:
            return (angles[n // 2 - 1] + angles[n // 2]) / 2
        else:
            return angles[n // 2]

    def calcMean(self, angles) -> float:
        n = len(angles)

        if n < ReverseDriving.VECTOR_SIZE:
            return 0
        angles.sort()
        median = self.calcMedian(angles)
        # print(sum(angles)/n)
        # print(median)
        diffs = [abs(angle - median) for angle in angles]
        q1 = self.calcMedian(angles[:n // 2])
        q3 = self.calcMedian(angles[n // 2 + n % 2:])
        iqr = q3 - q1
        threshold = 1.5 * iqr

        sumAngle = 0
        count = 0
        for i, angle in enumerate(angles):
            if diffs[i] <= threshold:
                sumAngle += angle
                count += 1

        self.ROAD_DIR = sumAngle / count
        return sumAngle / count

    def __init__(self, roadDir):
        self.count_map = {}
        self.angles = []
        self.ROAD_DIR = roadDir


class CongestionLevel(Enum):
    CLEAR = 0  # 畅通
    LIGHT = 1  # 轻度拥堵
    MODERATE = 2  # 中度拥堵
    SERIOUS = 3  # 严重拥堵
    INVALID = 4  # 无效值


# 定义目标物的数据结构
class Target:
    def __init__(self):
        self.enter_time = datetime.datetime.now()
        self.last_seen_time = datetime.datetime.now()
        self.duration_time = 0.0
        self.flag = False  # 离开标志位,false代表目标车辆还在当前路段,true表示已经离开路段


class CongestionDetector:
    def __init__(self):
        self.m_targets: Dict[int, Target] = {}  # 用于存储目标物的数据结构
        self.m_mutex = threading.Lock()  # 用于保护数据结构的读写
        self.total_time = 0.0  # 时间间隔内，车辆通过路段的总耗时
        self.m_vehicle_counts = 0  # 时间间隔内，通过路段的车辆数
        self.m_average_speed = 0

    def addObject(self, id: int):
        if self.isInmap(id):
            self.updateTarget(id)
        else:
            self.addTarget(id)

    def addTarget(self, id: int):
        with self.m_mutex:
            current_time = datetime.datetime.now()
            self.m_targets[id] = Target()
            self.m_targets[id].enter_time = current_time
            self.m_targets[id].last_seen_time = current_time

    def updateTarget(self, id: int):
        with self.m_mutex:
            current_time = datetime.datetime.now()
            # print(current_time)
            if id in self.m_targets:
                self.m_targets[id].last_seen_time = current_time
                # print(self.m_targets[id].last_seen_time)
                # 在统计周期内，但默认已经离开路段的目标物，再次出现在路段中时，把统计到的内容从总量中去掉
                if self.m_targets[id].flag == True:
                    self.total_time -= self.m_targets[id].duration_time
                    self.m_targets[id].flag = False
                    self.m_vehicle_counts -= 1

    def calTotalTime(self):
        logger_congestion.info("start cal total time")
        while True:
            time.sleep(Congestion.CAL_INTERVAL / 1000)
            current_time = datetime.datetime.now()
            self.m_mutex.acquire()

            # 遍历map中的所有目标物，计算它最后出现的时间距离现在多久，如果超过Congestion.TIME_INTERVAL，认为已经超过统计的要求时间，该目标物不再有价值
            for target_id, target in list(self.m_targets.items()):
                elapsed_time = (
                    current_time - target.last_seen_time).total_seconds()
                if elapsed_time > Congestion.TIME_INTERVAL:
                    self.total_time -= self.m_targets[target_id].duration_time
                    if self.total_time < 0.00001:
                        self.total_time = 0
                    del self.m_targets[target_id]
                    self.m_vehicle_counts -= 1
                    logger_congestion.info("vehicle remove area id"+target_id)
                # 如果在统计周期内，但是大于Congestion.DEPARTURE_TIME，表示该车已经这么多秒没有出现在视野中，视为已经离开，那就把这辆车的数据统计到总时长和车辆计数中。
                elif elapsed_time > Congestion.DEPARTURE_TIME:
                    if target.flag != True:

                        self.m_targets[target_id].duration_time = (current_time -
                                                                   target.enter_time).total_seconds() - Congestion.DEPARTURE_TIME
                        # 过滤掉在当前路段停留时间小于两秒的车辆信息，避免误检和漏检车辆数据干扰最终结果
                        if self.m_targets[target_id].duration_time > 2:
                            self.total_time += (current_time -
                                                target.enter_time).total_seconds() - Congestion.DEPARTURE_TIME
                            logger_congestion.info("cal vechicle driving time info id"+str(
                                target_id)+" total time"+str(self.m_targets[target_id].duration_time))
                            self.m_targets[target_id].flag = True
                            self.m_vehicle_counts += 1
                        # logger.info("cal vechicle info id"+str(target_id))
                else:
                    pass
            self.m_mutex.release()

    def getTotalTime(self):
        # logger.info("getTotalTime "+str(self.total_time))
        return self.total_time

    def isInmap(self, id: int):
        return id in self.m_targets

    def getVehicleCounts(self):
        # logger.info("getVehicleCounts "+str(self.m_vehicle_counts))
        return self.m_vehicle_counts

    def get_average_speed(self):
        if self.total_time <= 0:
            self.m_average_speed = 0
        else:
            self.m_average_speed = Congestion.ROAD_LENGTH * \
                self.m_vehicle_counts / self.total_time * 3.6
        # logger.info("cal get_average_speed "+str(self.m_average_speed))
        return self.m_average_speed

    def check_congestion_level(self):
        ratio = self.m_average_speed / Congestion.FREE_VELOCITY
        # logger.info("cal check_congestion_level "+str(ratio))

        if ratio == 0:
            return CongestionLevel.INVALID
        elif ratio > 0.7:
            return CongestionLevel.CLEAR
        elif ratio > 0.5:
            return CongestionLevel.LIGHT
        elif ratio > 0.3:
            return CongestionLevel.MODERATE
        else:
            return CongestionLevel.SERIOUS
