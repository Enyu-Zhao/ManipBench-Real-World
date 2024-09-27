import urx
import urx_local


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