import pyrealsense2 as rs
import numpy as np
import cv2
import os

import argparse

# Function to capture an image from the depth camera
def capture_image_from_depth_camera(save_folder,save_all=True):
    """

    Capture an image from the depth camera and save it to the specified folder.
    If save_all is True, the color, depth, and combined images will be saved.
    If save_all is False, only the combined image will be saved.
    
    """
    
    print("reset start")
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()

    print("reset end")
    
    
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()



    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)
    
    # Start streaming
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   # USB 2.0
        # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)    # USB 3.0


    pipeline.start(config)
    
    try:
        # Wait for a coherent pair of frames: depth and color
        for i in range(60):  # Skip first few frames for better accuracy, actually it's waiting for the camera to adjust the exposure.
            frames = pipeline.wait_for_frames()

            depth_frame = frames.get_depth_frame()
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

        
        if not depth_frame or not color_frame:
            print("Could not capture frames.")
            return None
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        raw_depth_image=depth_image.copy()
        
        # Stack both images horizontally for saving
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

            combined_images = np.hstack((resized_color_image, depth_colormap))
            color_image=resized_color_image
        else:
            combined_images = np.hstack((color_image, depth_colormap))

        
        color_image_path=os.path.join(save_folder, 'color_image.png')
        depth_image_path=os.path.join(save_folder, 'depth_image.png')
        combined_image_path=os.path.join(save_folder, 'combined_image.png')
        raw_depth_image_path=os.path.join(save_folder, 'raw_depth_image.npy')



        if save_all:
            # Save the image to the specified path
            cv2.imwrite(color_image_path, color_image)
            cv2.imwrite(depth_image_path, depth_colormap)
            cv2.imwrite(combined_image_path, combined_images)
            np.save(raw_depth_image_path,raw_depth_image)

        else:
            # Save the image to the specified path
            cv2.imwrite(combined_image_path, combined_images)

        print(f"Image saved to {save_folder}")
    
    finally:
        # Stop streaming
        pipeline.stop()

    if save_all:
        return color_image_path, depth_image_path, combined_image_path, raw_depth_image_path, depth_intrinsics
    else:
        return combined_image_path
    
def view_real_time_from_depth_camera():


    print("reset start")
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()

    print("reset end")


    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   # USB 2.0
        # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)    # USB 3.0

    # Start streaming
    pipeline.start(config)





    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

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

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)




            
    finally:

        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Capture an image from the depth camera.')
    parser.add_argument('--save', action='store_true', help='Save the image.')
    parser.add_argument('--save_folder', type=str, default='./', help='Folder to save the image.')
    args = parser.parse_args()


    save=args.save
    save_folder = args.save_folder

    if not os.path.exists(save_folder) and save:
        os.makedirs(save_folder)
    
    if save:
        capture_image_from_depth_camera(save_folder)
    else:
        view_real_time_from_depth_camera()