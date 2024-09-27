import pyrealsense2 as rs
import numpy as np
import cv2

def realsense_coordinates():
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    # pipeline.start()
    config = rs.config()
    # config.enable_all_streams()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Get the sensor once at the beginning. (Sensor index: 1)
    sensor = profile.get_device().query_sensors()[1]

    # Set the exposure anytime during the operation
    sensor.set_option(rs.option.exposure, 300.000)

    show_depth=True

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if show_depth:
                print(depth_frame)
                show_depth=False
            if not color_frame:
                print("Error")

            # Convert images to numpy arrays
            # depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # View the image in real time
            cv2.imshow('Real Sense', color_image)
            # cv2.imshow('Depth',depth_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Save RGB image
            # cv2.imwrite('color_image.jpg', color_image)

            # Optionally, save depth image (convert to uint16 first)
            # depth_image_uint16 = depth_image.astype(np.uint16)
            # cv2.imwrite('depth_image.png', depth_image_uint16)


    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return

realsense_coordinates()
