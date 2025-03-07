import numpy as np
import cv2
import os
import pyrealsense2 as rs
import urx_local
import time
from ssh_transfer import transfer_file,create_remote_directory,create_ssh_client_with_cfg_file
from scp import SCPClient
from image_capture import capture_image_from_depth_camera
import json
import urx
import argparse
from PIL import Image

# Test roboticstoolbox
import roboticstoolbox as rtb
from spatialmath import *
import threading





MODELS=["gpt-4o","o1","gpt-4o-mini","gemini-1.5-pro","gemini-1.5-flash","glm-4v","qwenvl","InternVL2","InternVL2_5","Qwen2VL"]

MODEL_WITH_DIFFERENT_SIZE={
    "InternVL2":["1B","2B","4B","8B","26B"],
    "InternVL2_5":["1B","2B","4B","8B","26B"],
    "Qwen2VL":["2B","7B"],
    }


TASK_STEP_DEFINITION={
    "CornersEdgesInward":4,
    "AllCornersInward":4,
    "DoubleStraight":3,
    "DoubleTriangle":2,
    "DoubleStraightBimanual":2,
    "CornersEdgesInwardBimanual":3
}


TABLE_HEIGHT=1.076 # Adjust this later

def pixel_to_3D(x,y,depth_value,intrinsics):
    fx=intrinsics.fx
    fy=intrinsics.fy
    cx=intrinsics.ppx
    cy=intrinsics.ppy
    print(f"fx:{fx},fy:{fy},cx:{cx},cy:{cy}")
    X=(x-cx)*depth_value/fx
    Y=(y-cy)*depth_value/fy
    Z=depth_value
    # Z=depth_value
    print(f"X:{X},Y:{Y},Z:{Z}")
    return [X,Y,Z]




def get_depth_value_at_coordinate(coordinate,depth_array_path):
    depth_image=np.load(depth_array_path)

    depth_value=depth_image[coordinate[1],coordinate[0]]

    return depth_value/1000



class GPT_Fab_PipelineExe():
    def __init__(self,task,local_task_folder,remote_pwd_path,remote_task_folder,model_name,version=0,moving_height=0.2,place_height=0.05,matrix_file_path="./cam_to_base_matrix.json",user_points="user",test_gripper=False):

        self.TASK_STEP_DEFINITION={
            "CornersEdgesInward":4,
            "AllCornersInward":4,
            "DoubleStraight":3,
            "DoubleTriangle":2,
            "DoubleStraightBimanual":2,
            "CornersEdgesInwardBimanual":3
        }
        self.rot_needed=True
        self.steps=self.TASK_STEP_DEFINITION[task]

        # Prepare for the Benchmarking pipeline if multiple VLMs are needed.

        if model_name not in MODELS:
            print(f"Model name {model_name} not supported, please choose from {MODELS}")
            return
        base_name=model_name
        
        if model_name in MODEL_WITH_DIFFERENT_SIZE:
            model_size=input(f"Model {model_name} has different sizes, please choose from {MODEL_WITH_DIFFERENT_SIZE[model_name]}")
            model_name=f"{model_name}-{model_size}"

        # Initialize the pipeline
        self.local_task_folder = local_task_folder
        self.local_own_folder = os.path.join(local_task_folder, f"{model_name}_V{version}")

        if not os.path.exists(self.local_task_folder):
            os.makedirs(self.local_task_folder)


        if not os.path.exists(self.local_own_folder):
            os.makedirs(self.local_own_folder)
        
        self.remote_task_folder = remote_task_folder
        self.remote_own_folder = os.path.join(remote_task_folder, f"{model_name}_V{version}")
        self.remote_pwd_path = remote_pwd_path
        # Build the info file for the server to run the model
        self.info_path=os.path.join(self.local_own_folder,"base_info.json")
        self.step_info_path=os.path.join(self.remote_pwd_path,"info.json")
        self.info_to_pass={}
        model_info={"model_name":model_name,"VLM_size":model_size} if base_name in MODEL_WITH_DIFFERENT_SIZE else {"model_name":model_name}

        self.info_to_pass["model_info"]=model_info
        self.info_to_pass["version"]=version
        self.info_to_pass["task"]=task
        self.info_to_pass["user_points"]=user_points
        self.info_to_pass["remote_task_folder"]=os.path.abspath(self.local_task_folder)
        self.info_to_pass["remote_own_folder"]=os.path.abspath(self.local_own_folder)
        self.info_to_pass["task_folder"]=self.remote_task_folder
        self.info_to_pass["own_folder"]=self.remote_own_folder
        self.info_to_pass["step_info_path"]=self.step_info_path
        # self.info_to_pass["bimanual"]=bimanual


        with open(self.info_path,"w") as f:
            json.dump(self.info_to_pass,f,indent=4)
        self.ssh_client=create_ssh_client_with_cfg_file("ssh_config.json")
        create_remote_directory(self.ssh_client,self.remote_task_folder)
        create_remote_directory(self.ssh_client,self.remote_own_folder)


        # Send the file to the server of the base information to start the pipeline on the other end.
        transfer_file(self.ssh_client,self.info_path,os.path.join(remote_pwd_path,"base_info.json"))


        self.table_height=TABLE_HEIGHT
        self.moving_height=self.table_height-moving_height
        self.place_height=self.table_height-place_height

        self.robot_url_r="192.10.0.11" #Right arm
        self.robot_url_l="192.10.0.12" #Left arm

        # initialize the robot
        self.robot_r=urx_local.Robot(self.robot_url_r)
        self.robot_l=urx_local.Robot(self.robot_url_l)


        self.simulated_robot_l=rtb.models.UR5()
        self.simulated_robot_r=rtb.models.UR5()
        
        
        self.starting_robot_joints_left = [-3.141686935884836, -2.356288772693141, 1.571568091705342, -0.7851389208139626, -1.5708287439107185, 0.000]
        self.simulated_robot_l.q=self.starting_robot_joints_left


        self.starting_robot_joints_right = [3.1417192709197925, -0.7849887185641355, -1.5709848785996405, -2.356405136870646, 1.5708449348819644, 0.000]
        self.simulated_robot_r.q=self.starting_robot_joints_right
        

        self.robot_move_to_default()
        

        if test_gripper:
            os.system("python test_robotiq_gripper.py")# Test the gripper. If it can open and close, then input exit to continue (you also need to ctrl+c to exit the testing gripper script to continue the pipeline)

        with open(matrix_file_path,'r') as f:
            cam_to_robot_matrices=json.load(f)


        self.camera_to_robot_right_base_matrix=np.array(cam_to_robot_matrices["right-arm-matrix"])
        self.camera_to_robot_left_base_matrix=np.array(cam_to_robot_matrices["left-arm-matrix"])

        self.img_size=(480,640)

    def set_step(self,step):
        self.step=step

    def robot_move_to_default(self):
        returning_robot_l=urx.Robot(self.robot_url_l)
        returning_robot_r=urx.Robot(self.robot_url_r)
        for i in range(100):
            returning_robot_l.servoj(
                self.starting_robot_joints_left, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
            returning_robot_r.servoj(
                self.starting_robot_joints_right, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
        # time.sleep(1)
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

     
    def _crop_image(self,image_path,depth_path,new_shape):
        image = cv2.imread(image_path)
        print(image.shape)
        x_start, y_start, x_end, y_end = new_shape[0], new_shape[1], new_shape[2], new_shape[3]  # Example coordinates

        cropped_image = image[x_start:x_end, y_start:y_end]

        # color_image=np.array(image)
        depth_image=np.load(depth_path)
        print(depth_image.shape)
        # color_image=color_image[:new_shape[0],:new_shape[1]]
        depth_image=depth_image[x_start:x_end,y_start:y_end]
        

        return cropped_image,depth_image

    def pre_execution(self,crop_needed=False,crop_shape=(0,100,480,640)):
        """
        This function captures the image and depth from the camera, transfer the image to the server, and wait for the response.
        """
        color_image_path,_,_,raw_depth_array_path,self.intrisics=capture_image_from_depth_camera(self.local_own_folder,specifier=f"_{self.step}")


        if crop_needed:
            self.crop_needed=True
            color_image,depth_image=self._crop_image(color_image_path,raw_depth_array_path,crop_shape)
            cv2.imwrite(color_image_path,color_image)
            np.save(raw_depth_array_path,depth_image)
            self.img_size=(crop_shape[2]-crop_shape[0],crop_shape[3]-crop_shape[1])

            self.x_adjust=crop_shape[1]
            self.y_adjust=crop_shape[0]
        else:
            self.crop_needed=False

        if self.rot_needed:
            color_image=np.array(Image.open(color_image_path))
            color_image=np.rot90(color_image,1)
            rotated_color_image_path=os.path.join(self.local_own_folder,f"rotated_color_image_{self.step}.png")
            color_image=Image.fromarray(color_image)
            color_image.save(rotated_color_image_path)

            depth_image=np.load(raw_depth_array_path)
            depth_image=np.rot90(depth_image,1)
            rotated_depth_image_path=os.path.join(self.local_own_folder,f"rotated_depth_image_{self.step}.npy")
            np.save(rotated_depth_image_path,depth_image)
        

        
        

        if self.rot_needed:
            transfer_file(self.ssh_client,rotated_color_image_path,os.path.join(self.remote_own_folder,f"raw_image_{self.step}.png"))
        else:
            transfer_file(self.ssh_client,color_image_path,os.path.join(self.remote_own_folder,f"raw_image_{self.step}.png"))

        self.info_to_pass["image_path"]=os.path.join(self.remote_own_folder,f"raw_image_{self.step}.png")
        self.info_to_pass["depth_path"]=os.path.join(self.remote_own_folder,f"raw_depth_image_{self.step}.npy")

 

        with open(self.info_path,"w") as f:
            json.dump(self.info_to_pass,f,indent=4)

        transfer_file(self.ssh_client,self.info_path,self.step_info_path)


        print("finish pre_execution, transfered image to server, waiting for response.")

        self.depth_path=raw_depth_array_path
        

    def wait_for_remote_file(self, ssh_client, remote_file_path, sleep_time=1):
        """
        Wait until the specified file exists on the remote machine.
        """
        print(f"Waiting for {remote_file_path} to exist on the remote machine...")
        while True:
            # Check for file existence using 'test -f' on remote
            stdin, stdout, stderr = ssh_client.exec_command(f"test -f {remote_file_path} && echo 'exists' || echo 'missing'")
            result = stdout.read().decode().strip()

            if result == "exists":
                print(f"Found {remote_file_path} on the remote machine.")
                break
            else:
                print(f"File {remote_file_path} not found. Retrying in {sleep_time} second(s)...")
                time.sleep(sleep_time)
    

    def load_info(self,info_name):
        ssh_client=create_ssh_client_with_cfg_file("ssh_config.json")
        remote_path = "/home/enyuzhao/code/GPT-Fabric-plus-plus/GPT-Fabric-PP-RL/temp_storage_for_scp"
        local_path = self.local_own_folder

        # Define the path to check for 'log.json'
        remote_log_file = os.path.join(remote_path, info_name)

        # Wait for the log file to exist
        self.wait_for_remote_file(ssh_client, remote_log_file)

        with SCPClient(ssh_client.get_transport()) as scp:
            # List the contents of the remote folder
            stdin, stdout, stderr = ssh_client.exec_command(f"ls -A {remote_path}")
            items = stdout.read().decode().splitlines()

            # Copy each item from the remote folder
            for item in items:
                remote_item_path = f"{remote_path}/{item}"
                print(f"Copying {remote_item_path} to {local_path}")
                scp.get(remote_item_path, local_path, recursive=True)
        print(f"Folder copied from {remote_path} to {local_path}")

        while True:
            result_received=os.path.exists(os.path.join(self.local_own_folder,info_name))


            if result_received:
                break
            else:
                print("waiting for result")
                time.sleep(1)


        print("result received, start execution")

        with open(os.path.join(self.local_own_folder,info_name)) as f:
            log=json.load(f)


            

        # bimanual=log["bimanual"]
        return log

    def get_3D_locations_single_arm(self,log):

        """
        This function waits for the result from the server, and return the 3D locations of the pick and place points. 
        """

        if not log["plan_success"]:
            print("Plan failed, exiting")
            return None,None
        
        # get the target coordinate
        picking_point_coordinate=log['pick_point']
        
        print(f"picking_point_coordinate:{picking_point_coordinate}")

        if self.rot_needed:
            picking_point_coordinate=(self.img_size[1]-picking_point_coordinate[1],picking_point_coordinate
            [0])

        if self.crop_needed:
            picking_point_coordinate=(self.img_size[1]-picking_point_coordinate[1]+self.y_adjust,picking_point_coordinate
            [0]+self.x_adjust)

        picking_point_depth=get_depth_value_at_coordinate(picking_point_coordinate,self.depth_path)

        print(f"picking_point_depth:{picking_point_depth}")


        placing_point_coordinate=log["place_point"]
        if self.rot_needed:
            placing_point_coordinate=(self.img_size[1]-placing_point_coordinate[1],placing_point_coordinate[0])
        if self.crop_needed:
            placing_point_coordinate=(self.img_size[1]-placing_point_coordinate[1]+self.y_adjust,placing_point_coordinate[0]+self.x_adjust)
        placing_point_depth=self.place_height

        print(f"placing_point_coordinate:{placing_point_coordinate}")
        pick_point_3D=pixel_to_3D(picking_point_coordinate[0],picking_point_coordinate[1],picking_point_depth,self.intrisics)

        place_point_3D=pixel_to_3D(placing_point_coordinate[0],placing_point_coordinate[1],placing_point_depth,self.intrisics)

        print(f"pick_point_3D:{pick_point_3D},place_point_3D:{place_point_3D}")
        return pick_point_3D,place_point_3D

    def get_3D_locations_dual_arm(self,log):

        # log=self.load_info()

        if not log["plan_success"]:
            print("Plan failed, exiting")
            return None,None,None,None
        
        # get the target coordinate
        picking_point_coordinate_l=log['pick_point_l']

        if self.rot_needed:
            picking_point_coordinate_l=(self.img_size[1]-picking_point_coordinate_l[1],picking_point_coordinate_l[0])
        if self.crop_needed:
            picking_point_coordinate_l=(picking_point_coordinate_l[0]+self.x_adjust,picking_point_coordinate_l[1]+self.y_adjust)
        picking_point_depth_l=get_depth_value_at_coordinate(picking_point_coordinate_l,self.depth_path)
        print(f"picking_point_coordinate of left arm:{picking_point_coordinate_l},depth:{picking_point_depth_l}")


        picking_point_coordinate_r=log['pick_point_r']
        if self.rot_needed:
            picking_point_coordinate_r=(self.img_size[1]-picking_point_coordinate_r[1],picking_point_coordinate_r[0])
        if self.crop_needed:
            picking_point_coordinate_r=(picking_point_coordinate_r[0]+self.x_adjust,picking_point_coordinate_r[1]+self.y_adjust)
        picking_point_depth_r=get_depth_value_at_coordinate(picking_point_coordinate_r,self.depth_path)
        print(f"placing_point_coordinate of right arm:{picking_point_coordinate_r}, depth:{picking_point_depth_r}")





        placing_point_coordinate_l=log["place_point_l"]
        
        placing_point_coordinate_r=log["place_point_r"]

        if self.rot_needed:
            placing_point_coordinate_l=(self.img_size[1]-placing_point_coordinate_l[1],placing_point_coordinate_l[0])
            placing_point_coordinate_r=(self.img_size[1]-placing_point_coordinate_r[1],placing_point_coordinate_r[0])   
        if self.crop_needed:
            placing_point_coordinate_l=(placing_point_coordinate_l[0]+self.x_adjust,placing_point_coordinate_l[1]+self.y_adjust)
            placing_point_coordinate_r=(placing_point_coordinate_r[0]+self.x_adjust,placing_point_coordinate_r[1]+self.y_adjust)
        placing_point_depth=self.place_height

        print(f"placing_point_coordinate of left arm:{placing_point_coordinate_l}")
        print(f"placing_point_coordinate of right arm:{placing_point_coordinate_r}")

        pick_point_3D_l=pixel_to_3D(picking_point_coordinate_l[0],picking_point_coordinate_l[1],picking_point_depth_l,self.intrisics)

        place_point_3D_l=pixel_to_3D(placing_point_coordinate_l[0],placing_point_coordinate_l[1],placing_point_depth,self.intrisics)

        print(f"For left arm: pick_point_3D:{pick_point_3D_l},place_point_3D:{place_point_3D_l}")



        pick_point_3D_r=pixel_to_3D(picking_point_coordinate_r[0],picking_point_coordinate_r[1],picking_point_depth_r,self.intrisics)

        place_point_3D_r=pixel_to_3D(placing_point_coordinate_r[0],placing_point_coordinate_r[1],placing_point_depth,self.intrisics)

        print(f"For right arm: pick_point_3D:{pick_point_3D_r},place_point_3D:{place_point_3D_r}")
        return pick_point_3D_l,place_point_3D_l,pick_point_3D_r,place_point_3D_r



    def _get_addtional_traj_keypoints(self,pick_point_3D,place_point_3D):
        pre_grasping_offset=0.05

        pre_pick_point_3D=pick_point_3D.copy()
        pre_pick_point_3D[2]-=pre_grasping_offset

        after_grasping_point_3D=pick_point_3D.copy()
        after_grasping_point_3D[2]=self.moving_height
        

        pre_releasing_point_3D=place_point_3D.copy()
        pre_releasing_point_3D[2]=self.moving_height


        print(f"""
        pick_point_3D:{pick_point_3D},
        place_point_3D:{place_point_3D},
        pre_pick_point_3D:{pre_pick_point_3D},
        after_grasping_point_3D:{after_grasping_point_3D},
        pre_releasing_point_3D:{pre_releasing_point_3D},
        """)

        return pre_pick_point_3D,after_grasping_point_3D,pre_releasing_point_3D

    def execute_single_arm(self,pick_point_3D,place_point_3D,arm="right"):
        pre_pick_point_3D,after_grasping_point_3D,pre_releasing_point_3D=self._get_addtional_traj_keypoints(pick_point_3D,place_point_3D)
        
        
        if self.moving_height>pick_point_3D[2]:
            print("Moving height is lower than picking height, exiting,please check the height.")
            return


        if arm=="right":
            default_joint_position=self.starting_robot_joints_right
            robot_to_manipulate=self.robot_r
        else:
            default_joint_position=self.starting_robot_joints_left
            robot_to_manipulate=self.robot_l


        print(f"""
        pick_point_3D:{pick_point_3D},
        place_point_3D:{place_point_3D},
        pre_pick_point_3D:{pre_pick_point_3D},
        after_grasping_point_3D:{after_grasping_point_3D},
        pre_releasing_point_3D:{pre_releasing_point_3D},
        """)

        print("moving to pre grasping point")
        self.move_robot_to_picked_point(pre_pick_point_3D,arm=arm)

        input("Press enter to continue to grasping point")

        print("moving to picking point")
        self.move_robot_to_picked_point(pick_point_3D,arm=arm)
        input("Press enter to grasp")

        self.close_gripper(robot_to_manipulate)


        print("grasping done, moving to after grasping point")

        input("Press enter to continue to after grasping point")

        self.move_robot_to_picked_point(after_grasping_point_3D,arm=arm)


        print("moving to pre releasing point")


        input("Press enter to continue to pre releasing point")

        self.move_robot_to_picked_point(pre_releasing_point_3D,arm=arm)

        input("Press enter to continue to releasing point")
        self.move_robot_to_picked_point(place_point_3D,arm=arm)

        input("Press enter to release")

        self.open_gripper(robot_to_manipulate)

        print("releasing done, moving to after releasing point")

        input("Press enter to continue to after releasing point")

        returning_robot=urx.Robot(self.robot_url_r) if arm=="right" else urx.Robot(self.robot_url_l)
        for i in range(20):
            returning_robot.servoj(
                default_joint_position, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
            )
        time.sleep(1)
        print('Finished moving the right arm to starting states.')


    def execute_dual_arm(self,pick_point_3D_l,place_point_3D_l,pick_point_3D_r,place_point_3D_r):
        
        pre_pick_point_3D_l,after_grasping_point_3D_l,pre_releasing_point_3D_l=self._get_addtional_traj_keypoints(pick_point_3D_l,place_point_3D_l)
        pre_pick_point_3D_r,after_grasping_point_3D_r,pre_releasing_point_3D_r=self._get_addtional_traj_keypoints(pick_point_3D_r,place_point_3D_r)
        
        
        if self.moving_height>pick_point_3D_l[2]:
            print("Moving height is lower than picking height, exiting,please check the height.")
            return
        
        if self.moving_height>pick_point_3D_r[2]:
            print("Moving height is lower than picking height, exiting,please check the height.")
            return
        
        default_joint_position_l=self.starting_robot_joints_left
        default_joint_position_r=self.starting_robot_joints_right


        print("moving to pre grasping point")
        self.move_robot_to_picked_point_dual_arm(pre_pick_point_3D_l,pre_pick_point_3D_r)

        input("Press enter to continue to grasping point")
        print("moving to picking point")
        self.move_robot_to_picked_point_dual_arm(pick_point_3D_l,pick_point_3D_r)

        input("Press enter to grasp")
        self.close_gripper(self.robot_l)
        self.close_gripper(self.robot_r)


        print("grasping done, moving to after grasping point")

        input("Press enter to continue to after grasping point")
        self.move_robot_to_picked_point_dual_arm(after_grasping_point_3D_l,after_grasping_point_3D_r)

        print("moving to pre releasing point")


        input("Press enter to continue to pre releasing point")

        self.move_robot_to_picked_point_dual_arm(pre_releasing_point_3D_l,pre_releasing_point_3D_r)

        input("Press enter to continue to releasing point")
        self.move_robot_to_picked_point_dual_arm(place_point_3D_l,place_point_3D_r)


        input("Press enter to release")
        self.open_gripper(self.robot_l)
        self.open_gripper(self.robot_r)

        print("releasing done, moving to after releasing point")

        input("Press enter to continue to after releasing point")

        self.robot_move_to_default()
        print('Finished moving the right arm to starting states.')


    def move_robot_to_picked_point(self,camera_point,arm="right"):
        # Using the right arm for now
        acceleration = 0.2
        velocity = 0.1
        if arm=="right":
            robot = urx_local.Robot(self.robot_url_r) # right robot arm

        # Update the following matrix based on the calibration result
        # save this to help with updating in the future in the json file.
        # camera_to_robot_base_trans_matrix = np.array([
        # [0.0202684,  -0.99941, 0.0277175, -0.640035],
        # [-0.999324, -0.0194006, 0.0312273,  0.145574],
        # [-0.0306712, -0.0283317, -0.999128,  0.862779],
        # [0,         0,         0,         1]
        # ]


            camera_to_robot_base_trans_matrix=self.camera_to_robot_right_base_matrix

        else:
            robot = urx_local.Robot(self.robot_url_l)
            camera_to_robot_base_trans_matrix=self.camera_to_robot_left_base_matrix




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

        delta_x=delta_movement_based_on_tcp[0]
        delta_y=delta_movement_based_on_tcp[1]
        delta_z=delta_movement_based_on_tcp[2]

        steps=1
        for i in range(steps):
            robot.translate((delta_x/steps, delta_y/steps, delta_z/steps), acceleration, velocity)

        # robot.translate((delta_movement_based_on_tcp[0], delta_movement_based_on_tcp[1], delta_movement_based_on_tcp[2]), acceleration, velocity)
        robot.set_tool_voltage(24)
        print('Done!!')


    def move_robot_to_picked_point_dual_arm_test(self,camera_point_left, camera_point_right):
        # Using the right arm for now
        acceleration = 0.2
        velocity = 0.05
        
        robot_l = urx_local.Robot(self.robot_url_l) # left robot arm
        robot_r = urx_local.Robot(self.robot_url_r) # right robot arm






        # for right arm
        # append 1 to point: [x, y, z, 1]
        camera_point_right = [camera_point_right[0], camera_point_right[1], camera_point_right[2], 1]

        # robot_tcp_pos_r = robot_r.getl()[:3]
        # print('Robot tcp postion: ', robot_tcp_pos_r)

        robot_tcp_pos_whole_r=robot_r.getl()
        print('Robot tcp postion,test: ', robot_r.getl())

        target_tcp_pos_whole_r=robot_tcp_pos_whole_r.copy()
        cam_to_base_to_tcp_point_r = np.matmul(self.camera_to_robot_right_base_matrix, camera_point_right)
        print('cam_to_base_to_tcp_point: ', cam_to_base_to_tcp_point_r)

        target_tcp_pos_whole_r[0]=cam_to_base_to_tcp_point_r[0]
        target_tcp_pos_whole_r[1]=cam_to_base_to_tcp_point_r[1]
        target_tcp_pos_whole_r[2]=cam_to_base_to_tcp_point_r[2]

        # robot_tcp_r = np.array(robot_tcp_pos_r)
        # delta_movement_based_on_tcp_r = cam_to_base_to_tcp_point_r[:3] - robot_tcp_r
        # print('delta_movement_based_on_tcp: ', delta_movement_based_on_tcp_r)


        # delta_x_r=delta_movement_based_on_tcp_r[0]
        # delta_y_r=delta_movement_based_on_tcp_r[1]
        # delta_z_r=delta_movement_based_on_tcp_r[2]


        # for left arm
        # append 1 to point: [x, y, z, 1]
        camera_point_left = [camera_point_left[0], camera_point_left[1], camera_point_left[2], 1]

        # robot_tcp_pos_l = robot_l.getl()[:3]
        # print('Robot tcp postion,test: ', robot_l.getl())

        robot_tcp_pos_whole_l=robot_l.getl()
        print('Robot tcp postion,test: ', robot_l.getl())

        target_tcp_pos_whole_l=robot_tcp_pos_whole_l.copy()
        cam_to_base_to_tcp_point_l = np.matmul(self.camera_to_robot_left_base_matrix, camera_point_left)
        print('cam_to_base_to_tcp_point: ', cam_to_base_to_tcp_point_l)


        target_tcp_pos_whole_l[0]=cam_to_base_to_tcp_point_l[0]
        target_tcp_pos_whole_l[1]=cam_to_base_to_tcp_point_l[1]
        target_tcp_pos_whole_l[2]=cam_to_base_to_tcp_point_l[2]

        # use current robot's z coordinate to avoid collision with the table.
        # robot_tcp_l = np.array(robot_tcp_pos_l)
        # delta_movement_based_on_tcp_l = cam_to_base_to_tcp_point_l[:3] - robot_tcp_l
        # print('delta_movement_based_on_tcp: ', delta_movement_based_on_tcp_l)


        # delta_x_l=delta_movement_based_on_tcp_l[0]
        # delta_y_l=delta_movement_based_on_tcp_l[1]
        # delta_z_l=delta_movement_based_on_tcp_l[2]





        base_robot_r=urx.Robot(self.robot_url_r)
        base_robot_l=urx.Robot(self.robot_url_l)


        print("base_robot_getl",base_robot_r.getl())

        pose_vec=base_robot_r.getl()
        print(dir(pose_vec))

        print("pose_vec_pose position",pose_vec.array)

        pose_vec_new=pose_vec.set_array(target_tcp_pos_whole_r)
        print("pose_vec_new",pose_vec.array)

        print('Check everything before executing on the robot!!!')
        input("Press enter to move both arms")


        for i in range(100):

            base_robot_r.servoc(target_tcp_pos_whole_r, acc=acceleration, vel=velocity,wait=False)
            base_robot_l.servoc(target_tcp_pos_whole_l, acc=acceleration, vel=velocity,wait=False)

        # steps=20
        # for i in range(steps):
        #     # robot_r.translate((delta_x_r/steps, delta_y_r/steps, delta_z_r/steps), acceleration, velocity)
        #     # robot_l.translate((delta_x_l/steps, delta_y_l/steps, delta_z_l/steps), acceleration, velocity)
        #     robot_r.servoc(target_tcp_pos_whole_r, acc=acceleration, vel=velocity)
        #     robot_l.servoc(target_tcp_pos_whole_l, acc=acceleration, vel=velocity)
    
        robot_r.set_tool_voltage(24)
        robot_l.set_tool_voltage(24)
        print('Done!!')



    def move_robot_to_picked_point_dual_arm(self, camera_point_left, camera_point_right):
        acceleration = 0.2
        velocity = 0.05

        def process_arm(camera_point, robot, camera_to_robot_matrix, robot_url):
            camera_point = [camera_point[0], camera_point[1], camera_point[2], 1]
            print(f"#########################Arm information with IP address {robot_url}###########################")
            robot_tcp_pos_whole = robot.getl()
            print('Robot TCP pose: ', robot_tcp_pos_whole)
            target_tcp_pos_whole = robot_tcp_pos_whole.copy()
            cam_to_base_to_tcp_point = np.matmul(camera_to_robot_matrix, camera_point)
            print('Camera point in arm base frame: ', cam_to_base_to_tcp_point)
            target_tcp_pos_whole[:3] = cam_to_base_to_tcp_point[:3]
            return target_tcp_pos_whole

        target_tcp_pos_whole_r = process_arm(camera_point_right, self.robot_r, self.camera_to_robot_right_base_matrix, self.robot_url_r)
        target_tcp_pos_whole_l = process_arm(camera_point_left, self.robot_l, self.camera_to_robot_left_base_matrix, self.robot_url_l)

        print('Check everything before executing on the robot!!!')
        input("Press enter to move both arms")

        def move_robot_arm_to_pose(robot, target_pos):
            robot.servoc(target_pos, acc=acceleration, vel=velocity, wait=True)

        thread_r = threading.Thread(target=move_robot_arm_to_pose, args=(self.robot_r, target_tcp_pos_whole_r))
        thread_l = threading.Thread(target=move_robot_arm_to_pose, args=(self.robot_l, target_tcp_pos_whole_l))

        thread_r.start()
        thread_l.start()

        thread_r.join()
        thread_l.join()

        self.robot_r.set_tool_voltage(24)
        self.robot_l.set_tool_voltage(24)
        print('Done!!')

    
        

        
if __name__=="__main__":
    local_data_root_folder="./tasks"
    remote_pwd_path="/home/enyuzhao/code/GPT-Fabric-plus-plus/GPT-Fabric-PP-RL"

    remote_data_root_folder=os.path.join(remote_pwd_path,"real_world_exp_data")


    img_width=480

    parser=argparse.ArgumentParser()
    parser.add_argument("--task_name",type=str,required=True)
    parser.add_argument("--version",type=int,default=0)
    parser.add_argument("--test_gripper",action="store_true")
    parser.add_argument("--moving_height",type=float,default=0.2)
    parser.add_argument("--place_height",type=float,default=0.05)
    parser.add_argument("--model_name",type=str,default="gpt-4o")
    parser.add_argument("--manual",action="store_true")
    # parser.add_argument("--bimanual",action="store_true")

    args=parser.parse_args()

    task_name=args.task_name
    version=args.version
    moving_height=args.moving_height
    place_height=args.place_height
    model_name=args.model_name
    test_gripper=args.test_gripper

    manual_input=args.manual

    if manual_input:
        user_points="user"
    else:
        user_points="llm"
    # bimanual=args.bimanual

    local_task_folder=os.path.join(local_data_root_folder,task_name)
    remote_task_folder=os.path.join(remote_data_root_folder,task_name)


    pipeline=GPT_Fab_PipelineExe(task_name,local_task_folder,remote_pwd_path,remote_task_folder,model_name,version,moving_height,place_height,user_points=user_points,test_gripper=test_gripper)


    steps_count=pipeline.steps

    for step in range(steps_count):
        pipeline.set_step(step)
        print(f"start pre_execution for step {step}")
        pipeline.pre_execution(crop_needed=True,crop_shape=(0,100,480,640))
        print(f"pre_execution done")
        log=pipeline.load_info(info_name=f"log_{step}.json")  

        bimanual=log["bimanual"]

      

        # input("Press enter if the json file is transferred back here")

        if not bimanual:
            raw_pick_point=log["pick_point"]

            arm_to_use="left" if raw_pick_point[0]<=0.5*img_width else "right"

            pick_point_3D,place_point_3D=pipeline.get_3D_locations_single_arm(log)
            

            if pick_point_3D is not None and place_point_3D is not None:
                input("Press enter to start execution")

                pipeline.execute_single_arm(pick_point_3D,place_point_3D,arm_to_use)

            else:
                print("No pick and place points returned, exiting")

        else:
            pick_point_3D_l,place_point_3D_l,pick_point_3D_r,place_point_3D_r=pipeline.get_3D_locations_dual_arm(log)
            if pick_point_3D_l is not None and place_point_3D_l is not None and pick_point_3D_r is not None and place_point_3D_r is not None:
                input("Press enter to start execution")
                # pipeline.execute_dual_arm_joint(pick_point_3D_l,place_point_3D_l,pick_point_3D_r,place_point_3D_r)

                pipeline.execute_dual_arm(pick_point_3D_l,place_point_3D_l,pick_point_3D_r,place_point_3D_r)
            else:
                print("No pick and place points returned, exiting")