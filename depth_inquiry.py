import numpy as np
import cv2
import os
import pyrealsense2 as rs

coordinate=(605,269)


depth_image_path="./pick_place_task_1/raw_depth_image.npy"

depth_image=np.load(depth_image_path)

depth_value=depth_image[coordinate[1],coordinate[0]]

print(f"The depth value at coordinate {coordinate} is: {depth_value} mm")






# Function to convert a pixel to 3D coordinates
def pixel_to_3d(u, v, depth_value, intrinsics):
    fx = intrinsics.fx
    fy = intrinsics.fy
    cx = intrinsics.ppx
    cy = intrinsics.ppy
    
    # Apply the pinhole camera model
    X = (u - cx) * depth_value / fx
    Y = (v - cy) * depth_value / fy
    Z = depth_value
    
    return X, Y, Z

# Get the camera intrinsics
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Capture frames from the camera
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()

# Get the camera intrinsics
depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

# Example pixel and its depth value
u, v = 320, 240  # Pixel coordinates in the image (center)
depth_value = depth_frame.get_distance(u, v)  # Depth at pixel (u, v) in meters

# Convert pixel to 3D world coordinates
X, Y, Z = pixel_to_3d(u, v, depth_value, depth_intrinsics)

print(f"3D coordinates: X={X}, Y={Y}, Z={Z}")

# Stop the pipeline
pipeline.stop()
