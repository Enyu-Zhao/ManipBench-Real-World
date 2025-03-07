## Calibration Instruction

1. Run a docker
    1.1 Load the docker using ``
    1.2 Run the docker using: 
        ```sudo docker run -it --device /dev/tty1 --device /dev/input --privileged -v /etc/localtime:/etc/localtime:ro -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --shm-size 8g --device /dev/tty1 --device /dev/input --device-cgroup-rule="c 81:* rmw" -e GDK_SCALE -e GDK_DPI_SCALE --network host  --ipc=host  -v /home/:/home --name {container_name} {image_id}```
2. After opening the docker, 
    2.1 roslaunch ur5_moveit_config robot_realsense.launch robot_ip:=192.10.0.12
        The above command launches the configuration for the right arm. Use the IP address of 192.10.0.11 for the left arm
    2.2 roslaunch ur5_moveit_config moveit_rviz.launch config:=true
    The above command will launch the RViz UI for the calibration.



## Steps
We name the desktop that controlls the UR-5 as local machine, and the desktop for image preprocessing and inferencing as server machine.


1. On the local machine, run `python gpt_fabric_pp_fold.py --task_name {Foldtype} --version {run_id}`
    1.1 Fold types: CornersEdgesInward,AllCornersInward,DoubleStraight,DoubleTriangle, DoubleStraightBimanual,CornersEdgesInwardBimanual

2. When you see `File /home/enyuzhao/code/GPT-Fabric-plus-plus/GPT_Fab_PP_RL/temp_storage_for_scp/log_0.json not found. Retrying in 1 second(s)...`, that means the image is being taken and sent to the server machine for inference, move to step 3

3. On the server machine, run `python ./GPT-Fabric-PP-RL/folding_inference.py`, and follow the instruction;
    3.1 You need to press Enter for continuing on manipulaiton steps;
    3.2 After you see "Though process...." and there's an "Press Enter to continue", meanwhile, on the local machine, you can see something like: 
        ```X:-0.07684560425742669,Y:-0.0066863332146200214,Z:1.031
        pick_point_3D:[0.16753030713914363, -0.2734177751791499, 1.083],place_point_3D:[-0.07684560425742669, -0.0066863332146200214, 1.031]
        Press enter to start execution```

    That means the inferencing step on the server side is complete, you are good to go to execute the planned action on the local machine.