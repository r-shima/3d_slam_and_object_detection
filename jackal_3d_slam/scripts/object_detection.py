#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import cv2
import torch
import numpy as np
import pyrealsense2 as rs

from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, set_logging
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized


class ObjectDetection(Node):
    def __init__(self):
        super().__init__("ObjectDetection")

        self.declare_parameter("weights", "yolov7-tiny.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("conf_thres", 0.25, ParameterDescriptor(description="Confidence threshold"))
        self.declare_parameter("iou_thres", 0.45, ParameterDescriptor(description="IOU threshold"))
        self.declare_parameter("device", "cpu", ParameterDescriptor(description="Name of the device"))
        self.declare_parameter("img_size", 640, ParameterDescriptor(description="Size of the image"))

        self.weights = self.get_parameter("weights").get_parameter_value().string_value
        self.conf_thres = self.get_parameter("conf_thres").get_parameter_value().double_value
        self.iou_thres = self.get_parameter("iou_thres").get_parameter_value().double_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.img_size = self.get_parameter("img_size").get_parameter_value().integer_value

        self.frequency = 1000 # Hz
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        # Initialize
        set_logging()
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device) # load FP32 model
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.img_size, s=stride)  # check img_size

        if self.half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        self.old_img_w = self.old_img_h = imgsz
        self.old_img_b = 1

    def detect(self):
        # Define image size, format, and frame rate
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start the pipeline streaming according to the configuration
        pipeline = rs.pipeline()
        profile = pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # Perform per pixel geometric transformation on the data provided
        # Align depth frame to color
        align_to = rs.stream.color
        align = rs.align(align_to)

        while True:
            # Wait for available frames
            # Frame set includes time synchronized frames of each enabled stream in the pipeline
            frames = pipeline.wait_for_frames()

            # Get aligned frames from RGB-D camera
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            # Convert frames to numpy arrays
            img = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Get color depth image
            depth_color_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)
            im0 = img.copy()
            img = img[np.newaxis, :, :, :]
            img = np.stack(img, 0)
            img = img[..., ::-1].transpose((0, 3, 1, 2))
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Warmup
            if self.device.type != 'cpu' and (self.old_img_b != img.shape[0] or self.old_img_h != img.shape[2] or self.old_img_w != img.shape[3]):
                self.old_img_b = img.shape[0]
                self.old_img_h = img.shape[2]
                self.old_img_w = img.shape[3]
                for i in range(3):
                    self.model(img)[0]

            # Inference
            t1 = time_synchronized()
            with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
                pred = self.model(img)[0]
            t2 = time_synchronized()

            # Apply NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)
            t3 = time_synchronized()

            # Process detections
            for i, det in enumerate(pred):  # detections per image
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class

                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        # Draw a boundary box around each object
                        plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=2)
                        plot_one_box(xyxy, depth_color_map, label=label, color=self.colors[int(cls)], line_thickness=2)
                        # Get box top left & bottom right coordinates
                        c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                    x = int((c2[0]+c1[0])/2)
                    y = int((c2[1]+c1[1])/2)
                    # print(f"c1 = {c1}, c2 = {c2}")
                    print(f"x = {x}, y = {y}")
                    if x < 480 and y < 480: #and depth_image[x][y] < 1000:
                        # Get depth using x and y coordinate values in the depth matrix
                        profile_stre = profile.get_stream(rs.stream.color)
                        intr = profile_stre.as_video_stream_profile().get_intrinsics()
                        depth_coords = rs.rs2_deproject_pixel_to_point(intr, [x,y], depth_image[x][y])
                        if depth_coords != [0.0,0.0,0.0]:
                            print(f"depth_coord = {depth_coords[0]*depth_scale}  {depth_coords[1]*depth_scale}  {depth_coords[2]*depth_scale}")
                            # self.get_logger().info(f"depth_coord = {depth_coords[0]*depth_scale}  {depth_coords[1]*depth_scale}  {depth_coords[2]*depth_scale}")
                cv2.imshow("Detection result", im0)
                cv2.imshow("Detection result depth", depth_color_map)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    def timer_callback(self):
        self.detect()

def main(args=None):
    """Run the main function."""
    rclpy.init(args=args)
    with torch.no_grad():
        node = ObjectDetection()
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == '__main__':
    main()