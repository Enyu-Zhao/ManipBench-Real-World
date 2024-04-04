import urx_local
import time

if __name__ == '__main__':
    robot = urx_local.Robot("192.10.0.11")
    trans = robot.get_pose()
    print('Robot pose: ', trans)
    time.sleep(1)
    print('Done')