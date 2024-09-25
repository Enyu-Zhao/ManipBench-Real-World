import urx_local

if __name__ == '__main__':
    robot = urx_local.Robot("192.10.0.12")
    trans = robot.get_pose()
    print('Robot pose: ', trans)
    trans.pos.z -= 0.05
    robot.set_pose(trans, acc=0.02, vel=0.05)  # apply the new pose
    exit(0)