import urx_local
import time

def print_tool_tip_pos_orientation(robot):
    tip_pos = robot.get_pos()
    tip_orientation = robot.get_orientation()
    print(f'Tool tip position: {tip_pos}, orientation: {tip_orientation}')

if __name__ == '__main__':
    robot = urx_local.Robot("192.10.0.11")
    print_tool_tip_pos_orientation(robot)
    time.sleep(1)
    print('Done')