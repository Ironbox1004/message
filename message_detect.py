from collections import Counter, deque
from configs import ReverseDriving, Congestion, Vehicle_line_head
from enum import Enum
from typing import Dict
import math
import time
import threading
import datetime
import numpy as np


class RegionalJudgmentSort:

    def __init__(self):
        self.in_bbox = {}
        self.result_now = {}
        self.bbox = []
        self.track_id = []

    def _is_poi_in_poly(self, pt, poly):
        """
        判断点是否在多边形内部的 pnpoly 算法
        :param pt: 点坐标 [x,y]
        :param poly: 点多边形坐标 [[x1,y1],[x2,y2],...]
        :return: 点是否在多边形之内
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
            x = (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]
            if ((verty[i] > testy) != (verty[j] > testy)) and (testx < x):
                res = not res
            j = i
        return res

    def _in_poly_area_dangerous(self, xyxy, area_poly):
        """
        检测人体是否在多边形危险区域内
        :param xyxy: 人体框的坐标
        :param img_name: 检测的图片标号，用这个来对应图片的危险区域信息
        :return: True -> 在危险区域内，False -> 不在危险区域内
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
        return self._is_poi_in_poly([object_cx, object_cy], area_poly)

    def _in_poly_area_line(self, xyxy, area_poly):
        """
        检测人体是否在多边形危险区域内
        :param xyxy: 人体框的坐标
        :param img_name: 检测的图片标号，用这个来对应图片的危险区域信息
        :return: True -> 在危险区域内，False -> 不在危险区域内
        """
        # print(area_poly)
        if not area_poly:  # 为空
            return False
        # 求物体框的底边中点
        object_x1 = int(xyxy[0])
        object_y1 = int(xyxy[1])
        object_x2 = int(xyxy[2])
        object_y2 = int(xyxy[3])
        object_w = object_x2 - object_x1
        object_h = object_y2 - object_y1
        object_cx = object_x1 + (object_w / 2)
        object_cy = object_y1 + object_h*0.9
        return self._is_poi_in_poly([object_cx, object_cy], area_poly)
    
    def get_length(self, xyxy, area_mark):
        head = Vehicle_line_head.list[area_mark]
        object_x1 = int(xyxy[0])
        object_y1 = int(xyxy[1])
        object_x2 = int(xyxy[2])
        object_y2 = int(xyxy[3])
        object_w = object_x2 - object_x1
        object_h = object_y2 - object_y1
        object_cx = object_x1 + (object_w / 2)
        object_cy = object_y1 + (object_h / 2)
        length_single =  int(2/73*np.sqrt((object_cx-head[0])**2+(object_cy-head[1])**2))
        return length_single

    def regional_judgment_length(self, sort_results: list, roi: list):
        """
        :param sort_results: sort检测结果
        :param roi: 车道区域列表
        :return: 返回在区域内的最大长度
        """
        length_now = [0,0,0,0,0,0]
        count_now = [0,0,0,0,0,0]
        if len(sort_results) > 0:
            for track in sort_results:
                bbox = track[:4]
                label = track[-1]
                track_id = track[4]
                if label != 3:
                    for i in range(len(roi)):
                        if self._in_poly_area_line(bbox, roi[i]) == True:
                            length_now[i]=max(self.get_length(bbox,i),length_now[i])
                            count_now[i]+=1
        self.result_now = {
                "length": length_now,
                "count": count_now
            }

    def regional_judgment_sort(self, sort_results: list, roi: list):
        """
        :param sort_results: sort检测结果
        :param roi: 危险区域列表
        :return: 返回在危险区域内的人体框坐标和track_id
        """
        if len(sort_results) > 0:
            for track in sort_results:
                bbox = track[:4]
                label = track[-1]
                track_id = track[4]
                if label == 3:
                    for i in range(len(roi)):
                        if self._in_poly_area_dangerous(bbox, roi[i]) == True:
                            self.bbox.append(bbox)
                            self.track_id.append(track_id)
            self.in_bbox = {
                "bbox": self.bbox,
                "track_id": self.track_id
            }

    def getPersonResult(self):
        return self.in_bbox

    def getVehicleResult(self):
        return self.result_now

class SortCount:
    def __init__(self,
                 up_count=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 down_count=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 already_counted=deque(maxlen=500), class_counter=Counter(), paths={},
                 total_counter=0, track_cls=0, total_track=0, last_track_id=-1, angle=-1
                 ):
        self.already_counted = already_counted
        self.class_counter = class_counter
        self.paths = paths
        self.total_counter, self.track_cls, self.up_count, self.down_count, self.total_track = \
            total_counter, track_cls, up_count, down_count, total_track
        self.last_track_id = last_track_id
        self.angle = angle

    def _tlbr_midpoint(self, box: list):
        """
        :param box: [x1, y1, x2, y2]
        :return: midpoint of box
        """
        minX, minY, maxX, maxY = box
        midpoint = (int((minX + maxX) / 2), int((minY + maxY) / 2))  # minus y coordinates to get proper xy format
        return midpoint

    def _ccw(self, A, B, C):
        # 判断三个点A、B、C是否组成了一个逆时针方向的三角形
        """
        :param A: point A
        :param B: point B
        :param C: point C
        :return: True if A, B, C are in counter-clockwise order
        """
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def _intersect(self, A, B, C, D):
        # 判断两条线段AB和CD是否相交
        """
        :param A: point A
        :param B: point B
        :param C: point C
        :param D: point D
        :return: True if line segments AB and CD intersect
        """
        return self._ccw(A, C, D) != self._ccw(B, C, D) and self._ccw(A, B, C) != self._ccw(A, B, D)

    def _vector_angle(self, midpoint, previous_midpoint):
        # 计算两个点构成的向量与x轴正方向之间的夹角
        """
        :param midpoint: current midpoint
        :param previous_midpoint: previous midpoint
        :return: angle between midpoint and previous_midpoint
        """
        x = midpoint[0] - previous_midpoint[0]
        y = midpoint[1] - previous_midpoint[1]
        return math.degrees(math.atan2(y, x))

    def sortcount(self, sort_results, line: list):
        if len(sort_results) > 0:
            for track in sort_results:
                bbox = track[:4]
                track_id = track[4]
                midpoint = self._tlbr_midpoint(bbox)
                origin_midpoint = (midpoint[0], 1920 - midpoint[1])  # 1080==im.shape[0]

                if track_id not in self.paths:
                    self.paths[track_id] = deque(maxlen=2)
                    self.total_track = track_id

                self.paths[track_id].append(midpoint)
                previous_midpoint = self.paths[track_id][0]
                origin_previous_midpoint = (previous_midpoint[0], 1920 - previous_midpoint[1])  # 1080==im.shape[0]

                for i in range(len(line)):
                    if self._intersect(midpoint, previous_midpoint, line[i][0], line[i][1]) \
                            and track_id not in self.already_counted:

                        self.class_counter[self.track_cls] += 1
                        self.total_counter += 1

                        self.already_counted.append(track_id)  # Set already counted for ID to true.

                        self.angle = self._vector_angle(origin_midpoint, origin_previous_midpoint)

                        if self.angle > 0:
                            self.up_count[i] += 1

                        if self.angle < 0:
                            self.down_count[i] += 1

    def getSortCountResult(self):
        return self.up_count, self.down_count

class Pe_Sort_Count:
    def __init__(self,
                 boxes=[],
                 indexIDs=[],
                 memory={},):
        super().__init__()
        self.boxes = boxes
        self.indexIDs = indexIDs
        self.memory = memory
        self.count = [0,0,0,0]

    def _ccw(self,A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def _intersect(self, A, B, C, D):
        return self._ccw(A, C, D) != self._ccw(B, C, D) and self._ccw(A, B, C) != self._ccw(A, B, D)

    def __call__(self, sort_results, roi):
        self.previous = self.memory.copy()
        for track in sort_results:
            self.boxes.append([track[0], track[1], track[2], track[3]])
            self.indexIDs.append(int(track[4]))
            self.memory[self.indexIDs[-1]] = self.boxes[-1]
            label = track[-1]
        if label == 3:
            i = int(0)
            for box in self.boxes:
                (x, y) = (int(box[0]), int(box[1]))
                (w, h) = (int(box[2]), int(box[3]))
                if self.indexIDs[i] in self.previous:
                    previous_box = self.previous[self.indexIDs[i]]
                    (x2, y2) = (int(previous_box[0]), int(previous_box[1]))
                    (w2, h2) = (int(previous_box[2]), int(previous_box[3]))
                    p1 = (int(x2 + (w2 - x2) / 2), int(y2 + (h2 - y2) / 2))
                    p0 = (int(x + (w - x) / 2), int(y + (h - y) / 2))
                    for j in range(len(roi)):
                        if self._intersect(p0, p1, roi[j][0], roi[j][1]):
                            self.count[j] += 1
                            del self.previous[self.indexIDs[i]]
                            # del self.memory[self.indexIDs[i]]
                i += 1

    def get_count(self):
        return self.count


class ReverseVehicle:
    def __init__(self, count: int, last_seen_time: float):
        self.count = count
        self.last_seen_time = last_seen_time


class ReverseDrivingDetector:
    def isAngleInReverseRange(self, carDir: float) -> bool:
        # 计算角度ROAD_DIR的反向角度ra
        ra = ReverseDriving.ROAD_DIR + 180 if ReverseDriving.ROAD_DIR < 180 else ReverseDriving.ROAD_DIR - 180

        # 计算角度b与ra的差值db，考虑边界情况
        db = carDir - ra
        if db > 180:
            db -= 360
        elif db < -180:
            db += 360

        # 判断角度b是否在角度a的反向范围内上下30度
        return db >= -ReverseDriving.SCOPE and db <= ReverseDriving.SCOPE

    def checkForReverseDriving(self, id: int, carDir: float, speed: float) -> int:
        if self.isAngleInReverseRange(carDir) and speed >= ReverseDriving.SPEED_THRESHOLD:
            print("xiangfan")
            current_time = time.monotonic()
            # 去掉map中触发逆行较久的数据
            for i in list(self.count_map.keys()):
                elapsed_time = current_time - self.count_map[i].last_seen_time
                if elapsed_time > ReverseDriving.REMOVE_TIME:
                    del self.count_map[i]

            if id not in self.count_map:
                self.count_map[id] = ReverseVehicle(0, current_time)

            self.count_map[id].count += 1
            self.count_map[id].last_seen_time = current_time

            if self.count_map[id].count >= ReverseDriving.COUNT:
                print(id, "nixingdetector")
                return id
            else:
                return -1
        else:
            print(id, "no nixing")
            return -1

    def calcMedian(self, angles):
        n = len(angles)
        if n % 2 == 0:
            return (angles[n // 2 - 1] + angles[n // 2]) / 2
        else:
            return angles[n // 2]

    def calcMean(self, angles):
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

        ReverseDriving.ROAD_DIR = sumAngle / count
        return sumAngle / count

    def __init__(self):
        self.count_map = {}
        self.angles = []


class CongestionLevel(Enum):
    CLEAR = 0  # 畅通
    BASIC_CLEAR = 1  # 基本畅通
    LIGHT = 2  # 轻度拥堵
    MODERATE = 3  # 中度拥堵
    SERIOUS = 4  # 严重拥堵
    INVALID = 5  # 无效值


# 定义目标物的数据结构
class Target:
    def __init__(self):
        self.enter_time = datetime.now()
        self.last_seen_time = datetime.now()
        self.duration_time = 0.0
        self.flag = False


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
            current_time = datetime.now()
            self.m_targets[id] = Target()
            self.m_targets[id].enter_time = current_time
            self.m_targets[id].last_seen_time = current_time

    def updateTarget(self, id: int):
        with self.m_mutex:
            current_time = datetime.now()
            if id in self.m_targets:
                self.m_targets[id].last_seen_time = current_time
                # print(self.m_targets[id].last_seen_time)
                if self.m_targets[id].flag == True:
                    self.total_time -= self.m_targets[id].duration_time
                    self.m_targets[id].flag = False
                    self.m_vehicle_counts -= 1

    def calTotalTime(self):
        while True:
            time.sleep(Congestion.CAL_INTERVAL / 1000)
            # print(1)
            current_time = datetime.now()
            self.m_mutex.acquire()
            for target_id, target in list(self.m_targets.items()):
                elapsed_time = (current_time - target.last_seen_time).total_seconds()
                if elapsed_time > Congestion.TIME_INTERVAL:
                    self.total_time -= self.m_targets[target_id].duration_time
                    if self.total_time < 0.00001:
                        self.total_time = 0
                    del self.m_targets[target_id]
                    self.m_vehicle_counts -= 1
                elif elapsed_time > Congestion.DEPARTURE_TIME:
                    if target.flag != True:
                        self.total_time += (current_time -
                                            target.enter_time).total_seconds() - Congestion.DEPARTURE_TIME
                        self.m_targets[target_id].duration_time = (current_time -
                                                                   target.enter_time).total_seconds() - Congestion.DEPARTURE_TIME
                        self.m_targets[target_id].flag = True
                        self.m_vehicle_counts += 1
                else:
                    pass
            self.m_mutex.release()

    def getTotalTime(self):
        return self.total_time

    def isInmap(self, id: int):
        return id in self.m_targets

    def getVehicleCounts(self):
        return self.m_vehicle_counts

    def get_average_speed(self):
        if self.total_time <= 0:
            self.m_average_speed = 0
        else:
            self.m_average_speed = Congestion.ROAD_LENGTH * self.m_vehicle_counts / self.total_time * 3.6
        return self.m_average_speed

    def check_congestion_level(self):
        ratio = self.m_average_speed / Congestion.FREE_VELOCITY
        if ratio == 0:
            return CongestionLevel.INVALID
        elif ratio > 0.7:
            return CongestionLevel.CLEAR
        elif ratio > 0.5:
            return CongestionLevel.BASIC_CLEAR
        elif ratio > 0.4:
            return CongestionLevel.LIGHT
        elif ratio > 0.3:
            return CongestionLevel.MODERATE
        else:
            return CongestionLevel.SERIOUS
