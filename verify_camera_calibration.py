## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import matplotlib.pyplot as plt 
import urx_local
import math3d as m3d
import time
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion

def capture_rgb_and_depth_from_realsense():
    """
    Taken from https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py
    """
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    # this line fixes this issue: https://github.com/IntelRealSense/librealsense/issues/6628
    device.hardware_reset()
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
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   # USB 2.0
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)    # USB 3.0

    # Start streaming
    pipeline_cfg = pipeline.start(config)

    # get camera intrinsics: https://github.com/IntelRealSense/librealsense/issues/869
    profile = pipeline_cfg.get_stream(rs.stream.depth)              # Fetch stream profile for depth stream
    intrinsics = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

    # get depth_scale: https://github.com/IntelRealSense/librealsense/issues/3473
    depth_sensor = pipeline_cfg.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

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

        # Stop streaming
        pipeline.stop()

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        return color_image, depth_image, intrinsics, depth_scale


def visualize_point_cloud(rgb, depth, intrinsics, depth_scale):
    rgb = o3d.geometry.Image(rgb)
    depth = o3d.geometry.Image(depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, depth_scale=1000, convert_rgb_to_intensity=False)
    # plt.imshow(rgbd_image.color); plt.show() # debugging: make sure rgbd image looks reasonable
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy))
    
    # this line is for debugging; it does not have editing ability to add markers
    # NOTE: you can get the camera's information by pressing ctrl + c to copy the current camera information in o3d viewer
    # o3d.visualization.draw_geometries([pcd],
    #                                 zoom=0.47999999999999976,
    #                                 front=[ 0.1172293277274971, 0.094658051301072965, -0.98858339964033515 ],
    #                                 lookat=[ -4.2709908435416708e-05, -1.838764854445177e-05, 0.000339248996169772 ],
    #                                 up=[ 0.007463149155373737, -0.99550299721168745, -0.094435607411763683 ])

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(width=1280, height=720, left=5, top=5)
    vis.add_geometry(pcd)
    ctr = vis.get_view_control()
    ctr.set_lookat([ -3.1044226837025207e-05, -2.914392155758258e-05, 0.00034436146553498567])
    ctr.set_front([ 0.011940780017795843, 0.076320815640134113, -0.99701181079894508 ])
    ctr.set_up([ 0.017836457145755921, -0.99694051288372565, -0.076101738269383337 ])
    ctr.set_zoom(0.02)
    vis.run()  # user picks points
    vis.destroy_window()
    picked_points = np.asarray(pcd.points)[vis.get_picked_points()]

    # [left to right, height with respect to the camera (below camera is positive, above camera is negative), back to front] in meters
    last_picked_point = picked_points[-1]
    print('Last picked point position: ', last_picked_point)
    return last_picked_point

def move_robot_to_last_picked_point(camera_point):
    robot = urx_local.Robot("192.10.0.11")
    acceleration = 0.2
    velocity = 0.05
    
    # Experiment 1:
    # # x y z    
    # translation = [0.38625, -0.42114, 0.51823]
    # # (x, y, z, w)
    # rotation_quat = [-0.41108, 0.6542, 0.51776, 0.36736]

    # Experiment 2:
    # after getting a bigger marker and set the appropriate marker size
    # 0.57211; -0.83268; 0.81784
    # -0.79679; 0.088439; 0.045675; 0.596
    # x y z    
    translation = [0.57211, -0.83268, 0.81784]
    # (x, y, z, w)
    rotation_quat = [-0.79679, 0.088439, 0.045675, 0.596]
    # base_frame_point:  [0.29150145 0.9604321  0.78064085 1.        ]
    # Tool tip position: <PosVec: (0.36903, -0.11269, 0.79948)>, orientation: <Orientation: RV(1.406, 1.100, -0.320)>
    # thoughts: z is very accurate. x is alright. But y is wrong.

    # Experiment 3:
    # change object frame to tool0_control
    # this is definitely wrong. camera optical frame are handeye target are both not updated.

    # Experiment 4:
    # change end-effector frame to ee_link
    # 0.60096; -0.9817; 1.0593
    # -0.8365; -0.055683; -0.082776; 0.5388
    # x y z    
    translation = [0.60096, -0.9817, 1.0593]
    # (x, y, z, w)
    rotation_quat = [-0.8365, -0.055683, -0.082776, 0.5388]
    # base_frame_point:  [ 0.64720714 -0.29296049  0.70883403  1.        ]
    # Tool tip position: <PosVec: (0.59371, -0.15363, 0.61411)>, orientation: <Orientation: RV(1.153, 0.532, -0.276)>
    # thoughts: yes, this looks much better, even y-axis looks correct now.

    # Experiment 5:
    # moved camera to a new position
    # 1.3562; 0.50264; 0.80685
    # 0.48791; 0.75911; -0.40608; -0.14419
    # x y z    
    translation = [1.3562, 0.50264, 0.80685]
    # (x, y, z, w)
    rotation_quat = [0.48791, 0.75911, -0.40608, -0.14419]
    # base_frame_point:  [ 0.70719903 -0.04652518  0.37184902  1.        ]
    # Tool tip position: <PosVec: (0.64552, -0.08997, 0.35418)>, orientation: <Orientation: RV(-0.221, 0.543, 0.758)>
    # very good! all values are fairly closed.
    # Only off by a few centimeters when I used the coordinates to move the robot arm

    # (w, x, y, z)
    rotation_quat = Quaternion(rotation_quat[-1], rotation_quat[0], rotation_quat[1], rotation_quat[2])
    camera_to_robot_base_trans_matrix = rotation_quat.transformation_matrix

    # this is how we use the result obtained from handeye calibration: https://support.zivid.com/en/latest/academy/applications/hand-eye/how-to-use-the-result-of-hand-eye-calibration.html
    # assign translation values
    camera_to_robot_base_trans_matrix[0][-1] = translation[0]
    camera_to_robot_base_trans_matrix[1][-1] = translation[1]
    camera_to_robot_base_trans_matrix[2][-1] = translation[2]

    # append 1 to point: [x, y, z, 1]
    camera_point = [camera_point[0], camera_point[1], camera_point[2], 1]

    cam_to_base_to_tcp_point = np.matmul(camera_to_robot_base_trans_matrix, camera_point)
    print('cam_to_base_to_tcp_point: ', cam_to_base_to_tcp_point)

    # use current robot's z coordinate to avoid collision with the table.
    robot_tcp = np.array(robot.getl()[:3])
    cam_to_base_to_tcp_point[2] = robot_tcp[2]
    print('Final camera-to-base-to-tcp point: ', cam_to_base_to_tcp_point)
    delta_movement_based_on_tcp = cam_to_base_to_tcp_point[:3] - robot_tcp
    print('delta_movement_based_on_tcp: ', delta_movement_based_on_tcp)

    print('Check everything before executing on the robot!!!')
    breakpoint() # Pause before executing
    robot.translate((delta_movement_based_on_tcp[0], delta_movement_based_on_tcp[1], delta_movement_based_on_tcp[2]), acceleration, velocity)
    print('Done!!')

if __name__ == '__main__':
    rgb, depth, intrinsics, depth_scale = capture_rgb_and_depth_from_realsense()
    last_picked_point = visualize_point_cloud(rgb, depth, intrinsics, depth_scale)
    move_robot_to_last_picked_point(last_picked_point)
