from message_detect import *
from configs import *
from custom_util import Detect, draw_bbox, plot_one_box
import cv2
import torch
import numpy as np
from tracker.byte_tracker import BYTETracker
# from nb_log import LogManager
import schedule
from message_handler import *
# logger_vehicle_sort = LogManager('vehicle_sort').get_logger_and_add_handlers(10,
#                                                                         log_path='./log',
#                                                                         log_filename='vehicle_sort.log')
#
# logger_roi_detect = LogManager('roi_detect').get_logger_and_add_handlers(10,
#                                                                         log_path='./log',
#                                                                         log_filename='roi_detect.log')
#
# logger_reverse_driving = LogManager('ReverseDrivingDetector').get_logger_and_add_handlers(10,
#                                                                         log_path='./log',
#                                                                         log_filename='reverse_driving.log')
#
# logger_congestion = LogManager('CongestionDetector').get_logger_and_add_handlers(10,
#                                                                         log_path='./log',
#                                                                         log_filename='Congestion.log')



detect = Detect(METAINFO.engine2d_ckpt)
Tracker = BYTETracker()
def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

# 检测AB和CD两条直线是否相交
def intersect(A, B, C, D):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

if __name__ == '__main__':
    counter = [0, 0, 0, 0]
    person_count = 0
    memory = {}
    dist_coeffs = np.array([-0.355054, 0.088915, -0.000127, 0.000481, 0.], dtype=np.float32)
    intrinsic = np.array([[1257.36515, 0., 950.03113],
                              [0., 1260.895, 499.81394],
                              [0., 0., 1.]],
                             dtype=np.float32)  # 1,3,3

    capture = cv2.VideoCapture(METAINFO.video_reader)
    last_timestamp = time.time()
    while True:
        _, frame = capture.read()
        if frame is None:
            break
        frame = cv2.undistort(frame, intrinsic, dist_coeffs)
        bboxes, scores, labels = detect.detect(frame)
        save_label_mask = torch.isin(labels.cpu(), torch.tensor(idx_list))
        bboxes, scores, labels = bboxes[save_label_mask], scores[save_label_mask], labels[save_label_mask]
        n_xyxycs = torch.cat((bboxes, labels[:, None], scores[:, None]), dim=-1)
        n_xyxycs = Tracker.update(n_xyxycs.cpu())
        im = draw_bbox(frame, n_xyxycs)

        event_handler = MessageHandler()
        # event_handler.ps_handler(n_xyxycs)
        # sort_count = SortCount()
        # area_sort = RegionalJudgmentSort()
        for i in range(len(person_sort_list.list)):
            cv2.line(frame, person_sort_list.list[i][0], person_sort_list.list[i][1], (0, 255, 255), 4)


        #调用类函数的结果
        person_sort = Pe_Sort_Count()
        count = person_sort(n_xyxycs, person_sort_list.list)
        count = person_sort.get_count()
        print("Pe_Sort_Count", count)


        #直接执行的函数 sort_count = SortCount()
        boxes = []
        indexIDs = []
        previous = memory.copy()
        for track in n_xyxycs:
            boxes.append([track[0], track[1], track[2], track[3]])
            indexIDs.append(int(track[4]))
            memory[indexIDs[-1]] = boxes[-1]
            label = track[-1]
        if label == 3:
            i = int(0)
            for box in boxes:
                (x, y) = (int(box[0]), int(box[1]))
                (w, h) = (int(box[2]), int(box[3]))

                if indexIDs[i] in previous:
                    previous_box = previous[indexIDs[i]]
                    (x2, y2) = (int(previous_box[0]), int(previous_box[1]))
                    (w2, h2) = (int(previous_box[2]), int(previous_box[3]))
                    p1 = (int(x2 + (w2 - x2) / 2), int(y2 + (h2 - y2) / 2))
                    p0 = (int(x + (w - x) / 2), int(y + (h - y) / 2))
                    for j in range(len(person_sort_list.list)):
                        if intersect(p0, p1, person_sort_list.list[j][0], person_sort_list.list[j][1]):
                            counter[j] += 1
                i += 1
        print("main", counter)
    
        

        cv2.imshow("demo", im)
        cv2.waitKey(1)




