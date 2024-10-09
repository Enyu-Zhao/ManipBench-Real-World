# How to execute real-world experiments


## MOKA-Real-World-Pipeline
```
python moka_real_world.py --task_name [task_name] --moving_height [The fixed height the robot should go to during pick-and-place] --model_name [The name of model you want to test]
```

## Use camera


If you just want to have a real-time viewer:

```
python image_capture.py
```
If you want to save the current image:

```
python image_capture.py --save --save_folder [The folder you want the image to be saved to]
```
## MOve the arm to the default position 

```
python get_right_arm_info.py
```
