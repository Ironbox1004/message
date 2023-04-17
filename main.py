from message_detect import Regional_Judgment_Sort, Sort_Count
from configs import Vehicle_sort_list, METAINFO, idx_list, danger_area
from custom_util import Detect, draw_bbox, plot_one_box
import cv2
import torch
import numpy as np
from tracker.byte_tracker import BYTETracker


detect = Detect(METAINFO.engine2d_ckpt)
Tracker = BYTETracker()

#根据list的个数，创建出相同个数的up_count和down_count
def create_up_down_count(list):
    up_count = []
    down_count = []
    for i in range(len(list)):
        up_count.append(0)
        down_count.append(0)
    return up_count, down_count

if __name__ == '__main__':

    dist_coeffs = np.array([-0.355054, 0.088915, -0.000127, 0.000481, 0.], dtype=np.float32)
    intrinsic = np.array([[1257.36515, 0., 950.03113],
                              [0., 1260.895, 499.81394],
                              [0., 0., 1.]],
                             dtype=np.float32)  # 1,3,3

    capture = cv2.VideoCapture(METAINFO.video_reader)
    down_counts, up_counts = create_up_down_count(Vehicle_sort_list.list)

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
        sort_count = Sort_Count()
        area_sort = Regional_Judgment_Sort()
        for i in range(len(danger_area.roi)):
            cv2.polylines(frame, [np.array(danger_area.roi[i], np.int32)], True, [0, 0, 255], 5, 16)

        for i in range(len(Vehicle_sort_list.list)):
            cv2.line(frame, Vehicle_sort_list.list[i][0], Vehicle_sort_list.list[i][1], (0, 255, 255), 4)

        results = area_sort.regional_judgment_sort(n_xyxycs, danger_area.roi)
        down_counts, up_counts = sort_count.sort_count(n_xyxycs, Vehicle_sort_list.list)

        for bbox in results["bbox"]:
            im = plot_one_box(bbox, im, (0, 0, 255))


        print(down_counts, up_counts)
        print(results)

        cv2.imshow("demo", frame)
        cv2.waitKey(1)




