import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
sys.path.append(str(ROOT))



from message_detect import Regional_Judgment_Sort, Sort_Count
from configs import Vehicle_sort_list, METAINFO, idx_list, danger_area,MESSAGE
from custom_util import Detect, draw_bbox, plot_one_box
import cv2
import torch
import numpy as np
import datetime
from tracker.byte_tracker import BYTETracker
from nb_log import LogManager

logger_vehicle_sort = LogManager('vehicle_sort').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='vehicle_sort.log')

logger_roi_detect = LogManager('roi_detect').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='roi_detect.log')

logger_reverse_driving = LogManager('ReverseDrivingDetector').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='reverse_driving.log')

logger_congestion = LogManager('CongestionDetector').get_logger_and_add_handlers(10,
                                                                        log_path='./log',
                                                                        log_filename='Congestion.log')

detect = Detect(METAINFO.engine2d_ckpt)
Tracker = BYTETracker()

def state_message_publisher(publishDate,
                            installLocation="/home/../ros/catkin_ws/src/..",
                            logLocation="/home/../ros/catkin_ws/src/../log/",
                            description="****test message****"):
	# ROS节点初始化
    rospy.init_node('state_message_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/message_info的topic，消息类型为event_detection_topic::Message，队列长度10
    state_message_info_pub = rospy.Publisher('/message_info', Message, queue_size=1)

    application_state = MESSAGE.application_state
    application_state['publishDate'] = publishDate
    application_state['installLocation'] = installLocation
    application_state['logLocation'] = logLocation
    application_state['description'] = description

    state_json_result = json.dumps(application_state)
    state_json_result = bytes(state_json_result, encoding="utf8")

    # 设置循环的频率
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        state_message_msg = Message()
        state_message_msg.sof = 0x5A
        state_message_msg.version = 0x01
        state_message_msg.appId = 0x01
        state_message_msg.type = 0x01
        state_message_msg.codec = 0x01
        state_message_msg.sequence = 100
        state_message_msg.timestamp = rospy.Time.now().to_nsec() // 1000000
        state_message_msg.length = 100
        state_message_msg.payload = state_json_result
        state_message_msg.crc = 0xFFFF

        state_message_info_pub.publish(state_message_msg)
        rospy.loginfo("Publsh state_message_msg[%d, %d, %d, %d, %d, %d, %s, %d, %s, %d ]",
                      state_message_msg.sof, state_message_msg.version, state_message_msg.appId, state_message_msg.type,
                      state_message_msg.codec, state_message_msg.sequence, state_message_msg.timestamp,
                      state_message_msg.length,
                      state_message_msg.payload, state_message_msg.crc)

        rate.sleep()




if __name__ == '__main__':

    now_python = datetime.datetime.now()
    time_str = now_python.strftime("%Y-%m-%d %H:%M:%S")

    try:
        state_message_publisher(time_str)

    except rospy.ROSInterruptException:
        pass

    dist_coeffs = np.array([-0.355054, 0.088915, -0.000127, 0.000481, 0.], dtype=np.float32)
    intrinsic = np.array([[1257.36515, 0., 950.03113],
                              [0., 1260.895, 499.81394],
                              [0., 0., 1.]],
                             dtype=np.float32)  # 1,3,3

    capture = cv2.VideoCapture(METAINFO.video_reader)

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
        vehicle_down_counts, vehicle_up_counts = sort_count.sort_count(n_xyxycs, Vehicle_sort_list.list)

        for bbox in results["bbox"]:
            im = plot_one_box(bbox, im, (0, 0, 255))

        logger_vehicle_sort.info("vehicle_down_counts:{},vehicle_up_counts:{},".
                                 format(vehicle_down_counts, vehicle_up_counts))

        logger_roi_detect.info("bbox:{},track_id:{},".
                               format(results["bbox"], results["track_id"]))


        cv2.imshow("demo", frame)
        cv2.waitKey(1)




