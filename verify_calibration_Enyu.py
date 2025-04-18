
import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt 
import urx_local
import urx
import math3d as m3d
import time
from scipy.spatial.transform import Rotation as R
from image_capture import capture_image_from_depth_camera
import re



def pixel_to_3D(x,y,depth_value,intrinsics):
    fx=intrinsics.fx
    fy=intrinsics.fy
    cx=intrinsics.ppx
    cy=intrinsics.ppy
    
    X=(x-cx)*depth_value/fx
    Y=(y-cy)*depth_value/fy
    Z=depth_value+0.015

    print(f"Original X:{X},Y:{Y},Z:{Z}")

    # if adjustment_needed:
    #     mark_x=0.1
    #     distance_X=X-mark_x
    #     adjust_X=0.9*distance_X
    #     X=mark_x+adjust_X
    #     mark_y=0.1
    #     distance_Y=Y-mark_y
    #     adjust_Y=0.9*distance_Y
    #     Y=mark_y+adjust_Y
    #     Z=Z+0.015
    # Z=depth_value
    print(f"X:{X},Y:{Y},Z:{Z}")
    return [X,Y,Z]




def get_depth_value_at_coordinate(coordinate,depth_image):
    # depth_image=np.load(depth_array_path)

    depth_value=depth_image[coordinate[1],coordinate[0]]

    return depth_value/1000




def find_corners(image_path):

    img = cv2.imread(image_path)
    img_copy=img.copy()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    blur = cv2.medianBlur(gray,5)
    th3 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    
    
    
    
    corners = cv2.goodFeaturesToTrack(gray,15,0.1,20)
    corners = np.int0(corners)
    new_corners=np.squeeze(corners,axis=1)
    in_bound_corners=[]

    # Find contours from the binary image
    contours, _ = cv2.findContours(th3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    possible_contours=[]
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        # Draw the bounding box on the original image
        if w<400 and w>50 and h<400 and h>50:
            possible_contours.append(contour)
            
            
            
    if len(possible_contours)!=1:
        best_contour=max(possible_contours, key=cv2.contourArea)
    else:
        best_contour=possible_contours[0]
        
    x, y, w, h = cv2.boundingRect(best_contour)
    y_max=y+h
    x_max=x+w
    

    
    top=y-10
    bottom=y_max+10
    left=x-10
    right=x_max+10
    
    
    for corner in new_corners:
        px,py = corner            
        
        if px>left and px<right and py<bottom and py>top:

            cv2.circle(img_copy,(px,py),4,255,-1)
            in_bound_corners.append(corner)

        
    print(f"Found {len(in_bound_corners)} corners in the image")

    print(f"The corners are {in_bound_corners}")

    for i, corner in enumerate(in_bound_corners):
        px, py = corner
        cv2.putText(img_copy, f"{px},{py}", (px, py), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imwrite(f"./corner_detection.png",img_copy)

    return in_bound_corners

    



def move_robot_to_last_picked_point(camera_point,url,matrix=None):
    acceleration = 0.2
    velocity = 0.05

    robot_ip=url
    robot = urx_local.Robot(robot_ip) 

    if matrix is not None: 
        camera_to_robot_base_trans_matrix = matrix
    else:
        print("Please provide a transformation matrix")
        return ArithmeticError


    


    # append 1 to point: [x, y, z, 1]
    camera_point = [camera_point[0], camera_point[1], camera_point[2], 1]

    robot_tcp_pos = robot.getl()[:3]
    print('Robot tcp postion: ', robot_tcp_pos)

    cam_to_base_to_tcp_point = np.matmul(camera_to_robot_base_trans_matrix, camera_point)
    print('cam_to_base_to_tcp_point: ', cam_to_base_to_tcp_point)

    cam_to_base_to_tcp_point[2]=-0.01

    robot_tcp = np.array(robot_tcp_pos)
    # cam_to_base_to_tcp_point[2] = robot_tcp[2]
    print('Final camera-to-base-to-tcp point: ', cam_to_base_to_tcp_point)
    delta_movement_based_on_tcp = cam_to_base_to_tcp_point[:3] - robot_tcp
    print('delta_movement_based_on_tcp: ', delta_movement_based_on_tcp)

    print('Check everything before executing on the robot!!!')
    # breakpoint() # Pause before executing
    robot.translate((delta_movement_based_on_tcp[0], delta_movement_based_on_tcp[1], delta_movement_based_on_tcp[2]), acceleration, velocity)
    print('Done!!')
    exit(0)

if __name__ == '__main__':

    def robot_move_to_default():
        starting_robot_joints_left = [-3.141686935884836, -2.356288772693141, 1.571568091705342, -0.7851389208139626, -1.5708287439107185, 0.000]

        starting_robot_joints_right = [3.1417192709197925, -0.7849887185641355, -1.5709848785996405, -2.356405136870646, 1.5708449348819644, 0.000]


        returning_robot_l=urx.Robot("192.10.0.12")
        returning_robot_r=urx.Robot("192.10.0.11")
        for i in range(100):
            returning_robot_l.servoj(
                starting_robot_joints_left, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
            returning_robot_r.servoj(
                starting_robot_joints_right, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
        # time.sleep(1)
        print('Finished moving the right arm to starting states.')

    robot_move_to_default()

    color_image_path,_,_,raw_depth_array_path,intrinsics=capture_image_from_depth_camera("./calibration_tests/")


    # rgb, depth, intrinsics, depth_scale = capture_image_and_depth_from_realsense()

    # cv2.imwrite('./rgb.png', rgb)

    in_bound_corners = find_corners(color_image_path)

    # input("Please list the corner's coordinate in format of x,y")

    def parse_corner_coordinates(input_string):
        pattern = r'\d+,\d+'
        matches = re.findall(pattern, input_string)
        coordinates = [tuple(map(int, match.split(','))) for match in matches]
        return coordinates

    input_string = input("Please list the corner's coordinate in format of x,y: ")
    corner_coordinates = parse_corner_coordinates(input_string)[0]
    print(f"Parsed corner coordinates: {corner_coordinates}")

    depth=np.load(raw_depth_array_path)
    # print(f"Depth shape: {depth.shape}")
    depth_value=get_depth_value_at_coordinate(corner_coordinates,depth)
    # depth_value=1.067
    print(f"Depth value at the corner: {depth_value}")
    # Assuming the last picked point is the last coordinate in the list
    camera_point=pixel_to_3D(corner_coordinates[0],corner_coordinates[1],depth_value,intrinsics)
    # last_picked_point = visualize_point_cloud(rgb, depth, intrinsics, depth_scale)


    arm=input("Which arm to use? Enter 'right' or 'left' to choose the robot arm: ")


    left_matrix=np.array([
        [0.00010114, 0.999864, 0.0165171, 0.617353],#0.617353
        [0.999882, 0.000152926, -0.01538, 0.0671707],#0.0871707
        [-0.0153805, 0.0165167, -0.999745, 1.06972],#1.06972
        [0, 0, 0, 1]
    ])

    right_matrix= np.array([
        [-0.0393895, 0.998471, -0.0387869, -0.517798],#-0.557798
        [0.999214, 0.0395301, 0.00286434, 0.0453676],#0.0353676
        [0.00439321, -0.0386436, -0.999243, 1.07479],#1.07479
        [0, 0, 0, 1]
    ],)
    

    if arm=="left":
        matrix=left_matrix
        robot_ip="192.10.0.12"
    else:
        matrix=right_matrix
        robot_ip="192.10.0.11"

    
    input("Press enter to move the robot to the last picked point")

    move_robot_to_last_picked_point(camera_point,robot_ip,matrix)




    #right arm need to be more right for x, 



