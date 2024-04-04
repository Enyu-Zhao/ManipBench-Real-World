import urx_local
import math3d as m3d
import time
import math
import copy

def print_tool_tip_pos_orientation(robot):
    tip_pos = robot.get_pos()
    tip_orientation = robot.get_orientation()
    print(f'Tool tip position: {tip_pos}, orientation: {tip_orientation}')

def move_to_new_pose(robot, pose, acceleration, velocity):
    print_tool_tip_pos_orientation(robot)
    input(f"Press enter to move to a new pose {pose}")
    robot.movel(pose, acceleration, velocity)
    time.sleep(1)
    print('Done!')


"""
I used pendent's initialize robot UI and get_robot_pose.py to determine the robot's pose.
Remember to launch realsense-viewer to check the AruCo marker if it is visible for each pose.
"""
if __name__ == '__main__':
    robot = urx_local.Robot("192.10.0.11")
    acceleration = 0.2
    velocity = 0.05

    # home position
    home_pos = m3d.Transform((1.280, 0.974, -0.951), (0.628, -0.406, 0.561))
    move_to_new_pose(robot, home_pos, acceleration, velocity)

    pose = m3d.Transform((1.334, 0.857, -0.802), (0.545, -0.398, 0.660))
    move_to_new_pose(robot, pose, acceleration, velocity)

    pose = m3d.Transform((1.080, 0.716, -1.279), (0.629, -0.503, 0.535))
    move_to_new_pose(robot, pose, acceleration, velocity)

    pose = m3d.Transform((0.614, 1.150, -1.677), (0.687, -0.539, 0.485))
    move_to_new_pose(robot, pose, acceleration, velocity)

    pose = m3d.Transform((1.137, -0.698, 0.173), (0.502, -0.530, 0.633))
    move_to_new_pose(robot, pose, acceleration, velocity)

    pose = m3d.Transform((1.388, 0.163, -0.202), (0.556, -0.271, 0.757))
    move_to_new_pose(robot, pose, acceleration, velocity)

    pose = m3d.Transform((1.374, 0.200, -0.483), (0.551, -0.348, 0.514))
    move_to_new_pose(robot, pose, acceleration, velocity)

    pose = m3d.Transform((0.847, 0.713, -1.078), (0.713, -0.374, 0.494))
    move_to_new_pose(robot, pose, acceleration, velocity)
    
    pose = m3d.Transform((1.006, -0.309, 0.069), (0.606, -0.494, 0.621))
    move_to_new_pose(robot, pose, acceleration, velocity)
