### 41Robotics ###
# Author: Damian Monea
# E-mail: damian.monea98@gmail.com
# Date: 04-09-2021
##################
import cv2
import math
import time
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import matplotlib.pyplot as plt

FRAME_HEIGHT = 480
FRAME_WIDTH = 640

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, 30)
config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, 30)

pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()

cv2.namedWindow("LaserScan", cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow("LaserScan", FRAME_WIDTH, FRAME_HEIGHT)

verts = np.array((FRAME_HEIGHT, FRAME_WIDTH, 3))

laser_scan_height = int(FRAME_HEIGHT / 2)
origin_x = int(FRAME_WIDTH / 2)
origin_y = int(FRAME_HEIGHT / 4) * 3

while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue


    image = np.full((FRAME_HEIGHT,FRAME_WIDTH,3), 255, dtype=np.uint8)

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    points = pc.calculate(depth_frame)

    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape((-1, 3))  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    # Color is in BGR space, not RGB
    for i in range(FRAME_WIDTH):
        projected_x = i
        projected_y = origin_y - int(depth_image[laser_scan_height][i] / 20)
        image = cv2.circle(image, (projected_x, projected_y), radius=0, color=(0, 0, 0), thickness=-1)

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    # If depth and color resolutions are different, resize color image to match depth image for display
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
    else:
        images = np.hstack((color_image, depth_colormap))

    images = cv2.line(images, (0, laser_scan_height), (FRAME_WIDTH, laser_scan_height), color=(0,0,255), thickness=2)

    cv2.imshow("LaserScan", image)

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)

# Stop streaming
pipeline.stop()