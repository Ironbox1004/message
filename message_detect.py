from custom_util import in_poly_area_dangerous
from collections import Counter, deque
import math


class Regional_Judgment_Sort:

    def __init__(self):
        self.in_bbox = {}
        self.bbox = []
        self.track_id = []

    def regional_judgment_sort(self, sort_results:list, roi:list):
        if len(sort_results) > 0:
            for track in sort_results:
                bbox = track[:4]
                label = track[-1]
                track_id = track[4]
                if label == 3:
                    for i in range(len(roi)):
                        if in_poly_area_dangerous(bbox,roi[i]) == True:
                            self.bbox.append(bbox)
                            self.track_id.append(track_id)
            self.in_bbox = {
                "bbox": self.bbox,
                "track_id": self.track_id
            }

        return self.in_bbox

class Sort_Count:

    def __init__(self,up_count=[0,0,0,0,0,0,0,0,0,0],down_count=[0,0,0,0,0,0,0,0,0,0],
                 already_counted=deque(maxlen=500),class_counter=Counter(),paths={},
                 total_counter=0,track_cls=0,total_track=0,
                 last_track_id=-1,angle=-1
                 ):
        self.already_counted = already_counted
        self.class_counter = class_counter
        self.paths = paths
        self.total_counter, self.track_cls, self.up_count, self.down_count, self.total_track  = \
            total_counter, track_cls, up_count, down_count, total_track
        self.last_track_id = last_track_id
        self.angle = angle


    def _tlbr_midpoint(self, box:list):
        minX, minY, maxX, maxY = box
        midpoint = (int((minX + maxX) / 2), int((minY + maxY) / 2))  # minus y coordinates to get proper xy format
        return midpoint

    def _ccw(self,A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def _intersect(self, A, B, C, D):
        return self._ccw(A, C, D) != self._ccw(B, C, D) and self._ccw(A, B, C) != self._ccw(A, B, D)

    def _vector_angle(self, midpoint, previous_midpoint):
        x = midpoint[0] - previous_midpoint[0]
        y = midpoint[1] - previous_midpoint[1]
        return math.degrees(math.atan2(y, x))


    def sort_count(self, sort_results, line:list):
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

        return self.down_count, self.up_count


















