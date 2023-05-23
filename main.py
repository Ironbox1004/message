from message_detect import *
from configs import *
from custom_util import *
import cv2
import torch
import time

import numpy as np
from tracker.byte_tracker import BYTETracker
from nb_log import LogManager
#
logger_roi_detect = LogManager('roi_detect').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='roi_detect.log')
logger_vehicle_sort = LogManager('vehicle_sort').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='vehicle_sort.log')
logger_person_sort = LogManager('person_sort').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='person_sort.log')
logger_vehicle_nums = LogManager('vehicle_nums').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='vehicle_nums.log')
detect = Detect(METAINFO.engine2d_ckpt)
Tracker = BYTETracker()

def getTracker(bboxes, scores, labels):

    save_label_mask = torch.isin(labels.cpu(), torch.tensor(idx_list))
    bboxes, scores, labels = bboxes[save_label_mask], scores[save_label_mask], labels[save_label_mask]
    n_xyxycs = torch.cat((bboxes, labels[:, None], scores[:, None]), dim=-1)
    n_xyxycs = Tracker.update(n_xyxycs.cpu())
    return n_xyxycs

if __name__ == '__main__':
    capture = cv2.VideoCapture(METAINFO.video_reader)
    # video_writer = cv2.VideoWriter(output_file, codec, fps, frame_size)
    last_timestamp = time.time()
    while True:
        _, frame = capture.read()
        if frame is None:
            break

        bboxes, scores, labels = detect.detect(frame)
        # save_label_mask = torch.isin(labels.cpu(), torch.tensor(idx_list))
        # bboxes, scores, labels = bboxes[save_label_mask], scores[save_label_mask], labels[save_label_mask]
        n_xyxycs = torch.cat((bboxes, labels[:, None], scores[:, None]), dim=-1)
        n_xyxycs = Tracker.update(n_xyxycs.cpu())
        frame = draw_bbox(frame, n_xyxycs)

        for i in range(len(person_sort_list.list)):
            cv2.line(frame, person_sort_list.list[i][0], person_sort_list.list[i][1], (0, 255, 255), 4)

        # for i in range(len(danger_area.roi)):
        #     cv2.polylines(frame, [np.array(danger_area.roi[i], np.int32)], True, [0, 255, 0], 5, 16)
        #
        # for i in range(len(Vehicle_sort_list.list)):
        #     cv2.line(frame, Vehicle_sort_list.list[i][0], Vehicle_sort_list.list[i][1], (255, 0, 0), 4)

        # regional = RegionalJudgmentSort()
        # regional.regional_judgment_length(
        #     n_xyxycs, Vehicle_area_list.list
        # )
        # vehicle_regional_result = regional.getVehicleResult()
        #
        # regional.regional_judgment_sort(
        #     n_xyxycs, danger_area.roi
        # )
        # person_regional_result = regional.getPersonResult()

        person_sort = PeSortCount()
        person_sort.peSortCount(n_xyxycs, person_sort_list.list)
        pe_count = person_sort.getPeSortCountResult()

        # vehicle = VeSortCount()
        # vehicle.veSortCount(
        #     n_xyxycs, Vehicle_sort_list.list
        # )
        #
        # vehicle_down_counts, vehicle_up_counts = vehicle.getVeSortCountResult()


        # logger_vehicle_sort.info("vehicle_down_counts:{},vehicle_up_counts:{},".
        #                          format(vehicle_down_counts, vehicle_up_counts))

        logger_person_sort.info("person_counts:{}".
                                format(pe_count))

        # logger_vehicle_nums.info("vehicle_length:{},vehicle_counts:{},".
        #                          format(vehicle_regional_result['length'], vehicle_regional_result['count']))
        #
        # logger_roi_detect.info("bbox:{},track_id:{},".
        #                        format(person_regional_result["bbox"], person_regional_result["track_id"]))


        # current_timestamp = time.time()
        # if current_timestamp - last_timestamp >= 10:
        #     vehicle_down_counts, vehicle_up_counts = vehicle_sort_count.get_results()
        #     down_results = [x - y for x, y in zip(vehicle_down_counts, vehicle_down_origin)]
        #     up_results = [x - y for x, y in zip(vehicle_up_counts, vehicle_up_origin)]
        #     print(down_results, up_results)
        #     vehicle_down_origin, vehicle_up_origin = vehicle_down_counts.copy(), vehicle_up_counts.copy()
        #     last_timestamp = current_timestamp


        cv2.imshow("demo", frame)
        cv2.waitKey(1)





