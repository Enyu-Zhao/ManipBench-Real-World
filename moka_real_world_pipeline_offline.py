import numpy as np
import cv2
import os
import pyrealsense2 as rs
import urx_local
import time
from ssh_transfer import transfer_file,create_remote_directory,create_ssh_client_with_cfg_file
from image_capture import capture_image_from_depth_camera
import json
import urx
import argparse
from PIL import Image

TABLE_HEIGHT=0.872 # meter

def pixel_to_3D(x,y,depth_value,intrinsics):
    fx=intrinsics.fx
    fy=intrinsics.fy
    cx=intrinsics.ppx
    cy=intrinsics.ppy

    X=(x-cx)*depth_value/fx
    Y=(y-cy)*depth_value/fy
    # Z=depth_value+0.005
    Z=depth_value

    return [X,Y,Z]




def get_depth_value_at_coordinate(coordinate,depth_array_path):
    depth_image=np.load(depth_array_path)

    depth_value=depth_image[coordinate[1],coordinate[0]]

    return depth_value/1000



class MOKAPipelineExe():
    def __init__(self,local_task_folder,remote_task_folder,model_name,version=0,height_required=False,moving_height=0.2,place_height=0.05,test_gripper=True):

        # Initialize the pipeline
        self.local_task_folder = local_task_folder
        self.local_own_folder = os.path.join(local_task_folder, f"{model_name}_V{version}")

        self.task_description_path=os.path.join(self.local_task_folder,"task_description.txt") #currently not used, I just update the task_description in the server side.
        if not os.path.exists(self.local_task_folder):
            os.makedirs(self.local_task_folder)

        if not os.path.exists(self.local_own_folder):
            os.makedirs(self.local_own_folder)

        self.remote_task_folder = remote_task_folder
        self.remote_own_folder = os.path.join(remote_task_folder, f"{model_name}_V{version}")


        # Build the info file for the server to run the model
        self.info_path=os.path.join(self.local_own_folder,"info.json")
        self.info_to_pass={}

        self.info_to_pass["model_name"]=model_name
        self.info_to_pass["version"]=version
        self.info_to_pass["remote_task_folder"]=os.path.abspath(self.local_task_folder)
        self.info_to_pass["remote_own_folder"]=os.path.abspath(self.local_own_folder)
        self.info_to_pass["task_folder"]=self.remote_task_folder
        self.info_to_pass["own_folder"]=self.remote_own_folder



        self.table_height=TABLE_HEIGHT
        self.height_required=height_required
        self.moving_height=self.table_height-moving_height
        self.place_height=self.table_height-place_height

        self.robot_url="192.10.0.11" #use right arm as default

        # initialize the robot
        self.robot=urx_local.Robot(self.robot_url)
        self.default_joints_pos=[-0.12412961030884695, -2.144660585057312, 1.5816871877202257, -0.9976501117431009, 4.716753443560367, 1.57439925225723]

        self.robot_move_to_default()

        if test_gripper:
            os.system("python test_robotiq_gripper.py")# Test the gripper. If it can open and close, then input exit to continue (you also need to ctrl+c to exit the testing gripper script to continue the pipeline)

    def robot_move_to_default(self):
        returning_robot=urx.Robot(self.robot_url)
        for i in range(100):
            returning_robot.servoj(
                self.default_joints_pos, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
        time.sleep(1)
        print('Finished moving the right arm to starting states.')

    
    def _crop_image(self,image_path,depth_path,new_shape):
        color_image=np.array(Image.open(image_path))
        depth_image=np.load(depth_path)

        color_image=color_image[:new_shape[0],:new_shape[1]]
        depth_image=depth_image[:new_shape[0],:new_shape[1]]
        

        return color_image,depth_image

    def pre_execution(self,remote_pwd_path,crop_needed=False):
        """
        This function captures the image and depth from the camera, transfer the image to the server, and wait for the response.
        """
        color_image_path,_,_,raw_depth_array_path,self.intrisics=capture_image_from_depth_camera(self.local_own_folder)


        if crop_needed:
            color_image,depth_image=self._crop_image(color_image_path,raw_depth_array_path,(370,640))
            cv2.imwrite(color_image_path,color_image)
            np.save(raw_depth_array_path,depth_image)
            

        


 
        # ssh_client=create_ssh_client_with_cfg_file("ssh_config.json")
        # create_remote_directory(ssh_client,self.remote_task_folder)
        # create_remote_directory(ssh_client,self.remote_own_folder)
        

        # transfer_file(ssh_client,color_image_path,os.path.join(self.remote_own_folder,"raw_image.png"))

        # self.info_to_pass["image_path"]=os.path.join(self.remote_own_folder,"raw_image.png")

        # self.info_to_pass["task_description_path"]=os.path.join(self.remote_task_folder,"task_description.txt")

        # with open(self.info_path,"w") as f:
        #     json.dump(self.info_to_pass,f,indent=4)

        # transfer_file(ssh_client,self.info_path,os.path.join(remote_pwd_path,"info.json"))


        print("finish pre_execution, transfered image to server, waiting for response.")

        self.depth_path=raw_depth_array_path
        


    def get_3D_locations(self):

        """
        This function waits for the result from the server, and return the 3D locations of the pick and place points. 
        """
        while True:
            result_received=os.path.exists(os.path.join(self.local_own_folder,"log.json"))


            if result_received:
                break
            else:
                print("waiting for result")
                time.sleep(1)


        print("result received, start execution")

        with open(os.path.join(self.local_own_folder,"log.json")) as f:
            log=json.load(f)

        if not log["plan_success"]:
            print("Plan failed, exiting")
            return None,None
        
        # get the target coordinate
        picking_point_index=log['picking_point']
        picking_point_coordinate=log['points'][picking_point_index]
        picking_point_depth=get_depth_value_at_coordinate(picking_point_coordinate,self.depth_path)

        print(f"picking_point_depth:{picking_point_depth}")


        placing_point_coordinate=log["end_point"]
        if self.height_required:
            placing_point_depth=log["height"]  # mm

        else:
            placing_point_depth=self.place_height


        pick_point_3D=pixel_to_3D(picking_point_coordinate[0],picking_point_coordinate[1],picking_point_depth,self.intrisics)

        place_point_3D=pixel_to_3D(placing_point_coordinate[0],placing_point_coordinate[1],placing_point_depth,self.intrisics)

        print(f"pick_point_3D:{pick_point_3D},place_point_3D:{place_point_3D}")
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
        


        pre_releasing_point_3D=place_point_3D.copy()
        pre_releasing_point_3D[2]=self.moving_height

        default_joint_position=[-0.12412961030884695, -2.144660585057312, 1.5816871877202257, -0.9976501117431009, 4.716753443560367, 1.57439925225723]

        print(f"""
        pick_point_3D:{pick_point_3D},
        place_point_3D:{place_point_3D},
        pre_pick_point_3D:{pre_pick_point_3D},
        after_grasping_point_3D:{after_grasping_point_3D},
        pre_releasing_point_3D:{pre_releasing_point_3D},
        """)

        print("moving to pre grasping point")
        self.move_robot_to_picked_point(pre_pick_point_3D)

        input("Press enter to continue to grasping point")

        print("moving to picking point")
        self.move_robot_to_picked_point(pick_point_3D)
        input("Press enter to grasp")

        self.close_gripper(self.robot)


        print("grasping done, moving to after grasping point")

        input("Press enter to continue to after grasping point")

        self.move_robot_to_picked_point(after_grasping_point_3D)


        print("moving to pre releasing point")


        input("Press enter to continue to pre releasing point")

        self.move_robot_to_picked_point(pre_releasing_point_3D)

        input("Press enter to continue to releasing point")
        self.move_robot_to_picked_point(place_point_3D)

        input("Press enter to release")

        self.open_gripper(self.robot)

        print("releasing done, moving to after releasing point")

        input("Press enter to continue to after releasing point")

        returning_robot=urx.Robot("192.10.0.11")
        for i in range(100):
            returning_robot.servoj(
                default_joint_position, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
        time.sleep(1)
        print('Finished moving the right arm to starting states.')




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


    def move_robot_to_picked_point(self,camera_point):
        acceleration = 0.2
        velocity = 0.05
        
        # uncomment these to control the left robot arm
        # robot = urx_local.Robot("192.10.0.11") # left robot arm

        robot = urx_local.Robot("192.10.0.11") # right robot arm


        camera_to_robot_base_trans_matrix = np.array([
        # [0.990668, -0.0666395,   0.118892,   0.582191],
        # [-0.13553,  -0.389404,   0.911041,  -0.576265],
        # [-0.0144143,  -0.918653,  -0.394802,     0.3589],
        # [        0,          0,          0,          1]
        [0.26834, -0.961192, 0.0640536, -0.612526],
        [-0.963265, -0.268468, 0.00676911, 0.145355],
        [0.0106899, -0.063517, -0.997924, 0.766989],
        [0,         0,         0,         1]
        ])





        # append 1 to point: [x, y, z, 1]
        camera_point = [camera_point[0], camera_point[1], camera_point[2], 1]

        robot_tcp_pos = robot.getl()[:3]
        print('Robot tcp postion: ', robot_tcp_pos)

        cam_to_base_to_tcp_point = np.matmul(camera_to_robot_base_trans_matrix, camera_point)
        print('cam_to_base_to_tcp_point: ', cam_to_base_to_tcp_point)

        # use current robot's z coordinate to avoid collision with the table.
        robot_tcp = np.array(robot_tcp_pos)
        print('Final camera-to-base-to-tcp point: ', cam_to_base_to_tcp_point)
        delta_movement_based_on_tcp = cam_to_base_to_tcp_point[:3] - robot_tcp
        print('delta_movement_based_on_tcp: ', delta_movement_based_on_tcp)
        print('Check everything before executing on the robot!!!')
        input("Press enter to continue")

        robot.translate((delta_movement_based_on_tcp[0], delta_movement_based_on_tcp[1], delta_movement_based_on_tcp[2]), acceleration, velocity)
        robot.set_tool_voltage(24)
        print('Done!!')
        


if __name__=="__main__":
    local_data_root_folder="./tasks"
    remote_data_root_folder="/home/enyuzhao/code/Ben-VLM/real_world_moka_data"
    remote_pwd_path="/home/enyuzhao/code/Ben-VLM"


    parser=argparse.ArgumentParser()
    parser.add_argument("--task_name",type=str,required=True)
    parser.add_argument("--version",type=int,default=0)
    parser.add_argument("--height_required",action="store_true")
    parser.add_argument("--test_gripper",action="store_true")
    parser.add_argument("--moving_height",type=float,default=0.2)
    parser.add_argument("--place_height",type=float,default=0.05)
    parser.add_argument("--model_name",type=str,default="gpt-4o")

    args=parser.parse_args()

    task_name=args.task_name
    version=args.version
    height_required=args.height_required
    moving_height=args.moving_height
    place_height=args.place_height
    model_name=args.model_name
    test_gripper=args.test_gripper
    local_task_folder=os.path.join(local_data_root_folder,task_name)
    remote_task_folder=os.path.join(remote_data_root_folder,task_name)


    pipeline=MOKAPipelineExe(local_task_folder,remote_task_folder,model_name,version,height_required,moving_height,place_height,test_gripper)

    pipeline.pre_execution(remote_pwd_path=remote_pwd_path,crop_needed=True)

    # input("Press enter if the json file is transferred back here")

    pick_point_3D,place_point_3D=pipeline.get_3D_locations()
    

    if pick_point_3D is not None and place_point_3D is not None:
        input("Press enter to start execution")

        pipeline.execute(pick_point_3D,place_point_3D)

    else:
        print("No pick and place points returned, exiting")