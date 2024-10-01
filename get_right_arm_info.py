import urx
import urx_local
import time


def get_right_arm_info(info_type="pose"):
    robot = urx_local.Robot("192.10.0.12")
    if info_type == "pose":
        trans = robot.get_pose()
        print('Robot pose right: ', trans)

    elif info_type == "joints":
        joints = robot.getj()
        print('Right robot joints: ', joints)


if __name__ == '__main__':
    get_right_arm_info("pose")
    get_right_arm_info("joints")
    print('Done')

    robot = urx.Robot("192.10.0.12")
    default_joint_position=[-0.12412961030884695, -2.144660585057312, 1.5816871877202257, -0.9976501117431009, 4.716753443560367, 1.57439925225723]
    for i in range(100):
        robot.servoj(
            default_joint_position, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
        )
    time.sleep(1)
    print('Finished moving the right arm to starting states.')