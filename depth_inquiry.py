import numpy as np
import cv2
import os
import pyrealsense2 as rs
import urx_local
import time
from ssh_transfer import transfer_file,create_remote_directory,create_ssh_client_with_cfg_file
from image_capture import capture_image_from_depth_camera
import json

TABLE_HEIGHT=0.862

def pixel_to_3D(x,y,depth_value,intrinsics):
    fx=intrinsics.fx
    fy=intrinsics.fy
    cx=intrinsics.ppx
    cy=intrinsics.ppy

    X=(x-cx)*depth_value/fx
    Y=(y-cy)*depth_value/fy
    Z=depth_value

    return X,Y,Z



def get_depth_value_at_coordinate(coordinate,depth_array_path):
    depth_image=np.load(depth_array_path)

    depth_value=depth_image[coordinate[1],coordinate[0]]

    return depth_value/1000



class MOKAPipelineExe():
    def __init__(self,local_task_folder,remote_task_folder,model_name,version=0,height_required=False,moving_height=0.2,place_height=0.05):
        self.local_task_folder = local_task_folder
        self.local_own_folder = os.path.join(local_task_folder, f"{model_name}_V{version}")

        self.remote_task_folder = remote_task_folder
        self.remote_own_folder = os.path.join(remote_task_folder, f"{model_name}_V{version}")


        self.table_height=TABLE_HEIGHT
        self.height_required=height_required
        self.moving_height=self.table_height-moving_height
        self.place_height=self.table_height-place_height


        robot = urx_local.Robot("192.10.0.12")
        self.robot=robot

    def pre_execution(self):
        color_image_path,_,_,raw_depth_array_path,self.intrisics=capture_image_from_depth_camera(self.local_own_folder)

        ssh_client=create_ssh_client_with_cfg_file("ssh_config.json")

        create_remote_directory(ssh_client,self.remote_own_folder)
        
        transfer_file(ssh_client,color_image_path,os.path.join(self.remote_own_folder,"raw_image.png"))


        print("finish pre_execution, transfered image to server, waiting for response.")

        self.depth_path=raw_depth_array_path
        


    def get_3D_locations(self):
        while True:
            result_received=os.path.exists(os.path.join(self.remote_own_folder,"log.json"))


            if result_received:
                break
            else:
                print("waiting for result")
                time.sleep(1)


        print("result received, start execution")

        with open(os.path.join(self.remote_own_folder,"log.json")) as f:
            log=json.load(f)

        if not log["plan_success"]:
            print("Plan failed, exiting")
            return None,None
        
        # get the target coordinate
        picking_point_index=log['picking_point']
        picking_point_coordinate=log['points'][picking_point_index]
        picking_point_depth=get_depth_value_at_coordinate(picking_point_coordinate,self.depth_path)


        placing_point_coordinate=log["end_point"]
        if self.height_required:
            placing_point_depth=log["height"]

        else:
            placing_point_depth=self.place_height


        pick_point_3D=pixel_to_3D(picking_point_coordinate[0],picking_point_coordinate[1],picking_point_depth,self.intrisics)

        place_point_3D=pixel_to_3D(placing_point_coordinate[0],placing_point_coordinate[1],placing_point_depth,self.intrisics)


        return pick_point_3D,place_point_3D



    def execute(self,pick_point_3D,place_point_3D):
        
        pre_grasping_offset=0.05

        pre_pick_point_3D=pick_point_3D.copy()
        pre_pick_point_3D[2]-=pre_grasping_offset


        after_grasping_point_3D=pick_point_3D.copy()
        after_grasping_point_3D[2]=self.moving_height

        if self.moving_height>pick_point_3D[2]:
            print("Moving height is lower than picking height, exiting,please check the height.")
            return
        


        self.pre_releasing_point_3D=place_point_3D.copy()
        self.pre_releasing_point_3D[2]=self.moving_height

        default_joint_position=[-0.12412961030884695, -2.144660585057312, 1.5816871877202257, -0.9976501117431009, 4.716753443560367, 1.57439925225723]



        print("moving to pre grasping point")
        self.move_robot_to_picked_point(pre_pick_point_3D)

        input("Press enter to continue to grasping point")

        print("moving to picking point")

        input("Press enter to grasp")

        self.close_gripper(self.robot)


        print("grasping done, moving to after grasping point")

        input("Press enter to continue to after grasping point")

        self.move_robot_to_picked_point(after_grasping_point_3D)


        print("moving to pre releasing point")


        input("Press enter to continue to pre releasing point")

        self.move_robot_to_picked_point(self.pre_releasing_point_3D)

        input("Press enter to continue to releasing point")
        self.move_robot_to_picked_point(place_point_3D)

        input("Press enter to release")

        self.open_gripper(self.robot)

        print("releasing done, moving to after releasing point")

        input("Press enter to continue to after releasing point")

        
        for i in range(100):
            self.robot.servoj(
                default_joint_position, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
        time.sleep(1)
        print('Finished moving the right arm to starting states.')












        # move to pick point



    def close_gripper(self,robot):

        #Close gripper
        robot.set_digital_out(8, True)
        robot.set_digital_out(9, False)

        time.sleep(0.05)


    def open_gripper(self,robot):
 
        #Open gripper
        robot.set_digital_out(8, False)
        robot.set_digital_out(9, False)
        time.sleep(0.05)


    def move_robot_to_picked_point(camera_point):
        acceleration = 0.2
        velocity = 0.05
        
        # uncomment these to control the left robot arm
        # robot = urx_local.Robot("192.10.0.11") # left robot arm

        robot = urx_local.Robot("192.10.0.12") # right robot arm


        camera_to_robot_base_trans_matrix = np.array([
            [0.0508758,  -0.998335,  0.0271692,  -0.692404],
            [-0.998165 ,-0.0499348,  0.0342567, -0.0113037],
            [ -0.032843, -0.0288622,  -0.999044,  0.859464],
            [        0,          0,          0,          1]
        ])





        # append 1 to point: [x, y, z, 1]
        camera_point = [camera_point[0], camera_point[1], camera_point[2], 1]

        robot_tcp_pos = robot.getl()[:3]
        print('Robot tcp postion: ', robot_tcp_pos)

        cam_to_base_to_tcp_point = np.matmul(camera_to_robot_base_trans_matrix, camera_point)
        print('cam_to_base_to_tcp_point: ', cam_to_base_to_tcp_point)

        # use current robot's z coordinate to avoid collision with the table.
        robot_tcp = np.array(robot_tcp_pos)
        cam_to_base_to_tcp_point[2] = robot_tcp[2]
        print('Final camera-to-base-to-tcp point: ', cam_to_base_to_tcp_point)
        delta_movement_based_on_tcp = cam_to_base_to_tcp_point[:3] - robot_tcp
        print('delta_movement_based_on_tcp: ', delta_movement_based_on_tcp)
        print('Check everything before executing on the robot!!!')
        input("Press enter to continue")

        robot.translate((delta_movement_based_on_tcp[0], delta_movement_based_on_tcp[1], delta_movement_based_on_tcp[2]), acceleration, velocity)
        robot.set_tool_voltage(24)
        print('Done!!')
        


