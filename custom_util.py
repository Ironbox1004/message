# -*- coding: utf-8 -*-

import numpy as np
import cv2
import tensorrt as trt
import torch
from typing import Union, Optional, List, Tuple
from pathlib import Path
from collections import namedtuple
from configs import CLASSES2D

def draw_bbox(img,n_xyxycs):

    for (*bbox, tid, score, label) in n_xyxycs:
        bbox = [int(round(b)) for b in bbox]
        cls_id = int(label)
        cls = CLASSES2D[cls_id]
        color = [255, 0, 0]
        cv2.rectangle(img, bbox[:2], bbox[2:], color, 2)
        s = f'{cls}: {int(tid)}'
        cv2.putText(img,
                    s, (bbox[0], bbox[1] - 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, [0, 0, 255],
                    thickness=2)
    return img


class TRTModule(torch.nn.Module):
    dtypeMapping = {
        trt.bool: torch.bool,
        trt.int8: torch.int8,
        trt.int32: torch.int32,
        trt.float16: torch.float16,
        trt.float32: torch.float32
    }

    def __init__(self, weight: Union[str, Path],
                 device: Optional[torch.device]) -> None:
        super(TRTModule, self).__init__()
        self.weight = Path(weight) if isinstance(weight, str) else weight
        self.device = device if device is not None else torch.device('cuda:0')
        self.stream = torch.cuda.Stream(device=device)
        self.__init_engine()
        self.__init_bindings()

    def __init_engine(self) -> None:
        logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(logger, namespace='')
        with trt.Runtime(logger) as runtime:
            model = runtime.deserialize_cuda_engine(self.weight.read_bytes())

        context = model.create_execution_context()
        num_bindings = model.num_bindings
        names = [model.get_binding_name(i) for i in range(num_bindings)]

        self.bindings: List[int] = [0] * num_bindings
        num_inputs, num_outputs = 0, 0

        for i in range(num_bindings):
            if model.binding_is_input(i):
                num_inputs += 1
            else:
                num_outputs += 1

        self.num_bindings = num_bindings
        self.num_inputs = num_inputs
        self.num_outputs = num_outputs
        self.model = model
        self.context = context
        self.input_names = names[:num_inputs]
        self.output_names = names[num_inputs:]

    def __init_bindings(self) -> None:
        idynamic = odynamic = False
        Tensor = namedtuple('Tensor', ('name', 'dtype', 'shape'))
        inp_info = []
        out_info = []
        for i, name in enumerate(self.input_names):
            assert self.model.get_binding_name(i) == name
            dtype = self.dtypeMapping[self.model.get_binding_dtype(i)]
            shape = tuple(self.model.get_binding_shape(i))
            if -1 in shape:
                idynamic |= True
            inp_info.append(Tensor(name, dtype, shape))
        for i, name in enumerate(self.output_names):
            i += self.num_inputs
            assert self.model.get_binding_name(i) == name
            dtype = self.dtypeMapping[self.model.get_binding_dtype(i)]
            shape = tuple(self.model.get_binding_shape(i))
            if -1 in shape:
                odynamic |= True
            out_info.append(Tensor(name, dtype, shape))

        if not odynamic:
            self.output_tensor = [
                torch.empty(info.shape, dtype=info.dtype, device=self.device)
                for info in out_info
            ]
        self.idynamic = idynamic
        self.odynamic = odynamic
        self.inp_info = inp_info
        self.out_info = out_info

    def set_profiler(self, profiler: Optional[trt.IProfiler]):
        self.context.profiler = profiler \
            if profiler is not None else trt.Profiler()

    def forward(self, *inputs) -> Union[Tuple, torch.Tensor]:

        assert len(inputs) == self.num_inputs
        contiguous_inputs: List[torch.Tensor] = [
            i.contiguous() for i in inputs
        ]

        for i in range(self.num_inputs):
            self.bindings[i] = contiguous_inputs[i].data_ptr()
            if self.idynamic:
                self.context.set_binding_shape(
                    i, tuple(contiguous_inputs[i].shape))

        outputs: List[torch.Tensor] = []

        for i in range(self.num_outputs):
            j = i + self.num_inputs
            if self.odynamic:
                shape = tuple(self.context.get_binding_shape(j))
                output = torch.empty(size=shape,
                                     dtype=self.out_info[i].dtype,
                                     device=self.device)
            else:
                output = self.output_tensor[i]
            self.bindings[j] = output.data_ptr()
            outputs.append(output)

        self.context.execute_async_v2(self.bindings, self.stream.cuda_stream)
        self.stream.synchronize()

        return tuple(outputs) if len(outputs) > 1 else outputs[0]

class Detect:

    def __init__(self, engine, device=torch.device('cuda:0')):
        self.engine = engine
        self.device = device
        self.engine2d = TRTModule(self.engine, self.device)

    def _post2d(self, outputs, ratio) :
        num_dets, bboxes, scores, labels = outputs
        num_dets = num_dets.item()
        bboxes = bboxes[0, :num_dets]
        scores = scores[0, :num_dets]
        labels = labels[0, :num_dets]
        bboxes /= ratio
        return bboxes, scores, labels

    def _resize2d(self, image, size=(800, 800)):
        # size is (width, height)
        board = np.ones(((*size, 3)), dtype=np.uint8) * 114
        h, w = image.shape[:2]
        r = min(size[0] / w, size[1] / h)
        # size is (width, height)
        new_shape = (int(w * r), int(h * r))
        resized = cv2.resize(image, new_shape, interpolation=cv2.INTER_LINEAR)
        # resized = (resized - mean) / std
        board[:new_shape[1], :new_shape[0], :] = resized[:, :, ::-1]
        blob = board.transpose(2, 0, 1)[np.newaxis]
        blob = blob.astype(np.float32) / 255.  # 是否归一化
        blob = np.ascontiguousarray(blob)
        return blob, r

    def detect(self, image):
        size2d = self.engine2d.inp_info[0].shape[2:][::-1]
        blob, r = self._resize2d(image, size2d)
        tensor0 = torch.from_numpy(blob).to(self.device)
        data = self.engine2d(tensor0)
        bboxes, scores, labels = self._post2d(data, r)

        return bboxes, scores, labels

def draw_poly_area_dangerous(img,area_poly):
    """
    画多边形危险区域的框
    :param img_name: 检测的图片标号，用这个来对应图片的危险区域信息
    :param img: 图片数据
    :return: None
    """
    area_poly = np.array(area_poly, np.int32)
    cv2.polylines(img, [area_poly], isClosed=True, color=(0, 0, 255), thickness=3, lineType=cv2.LINE_AA)

def is_poi_in_poly(pt, poly):
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

def in_poly_area_dangerous(xyxy,area_poly):
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
    return is_poi_in_poly([object_cx, object_cy], area_poly)

def plot_one_box(x, img, color):
    # Plots one bounding box on image img
    tl = round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    return img

if __name__ == '__main__':
    pass
