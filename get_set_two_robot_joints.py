import urx
import time

if __name__ == '__main__':
    robot_left = urx.Robot("192.10.0.11")
    joints_left = robot_left.getj()
    print('Left robot joints: ', joints_left)
    time.sleep(1)

    robot_right = urx.Robot("192.10.0.12")
    joints_right = robot_right.getj()
    print('Right robot joints: ', joints_right)

    # # starting states for our robots in GELLO and real-world experiments
    starting_robot_joints_left = [3.1417192709197925, -0.7849887185641355, -1.5709848785996405, -2.356405136870646, 1.5708449348819644, 0.000]
    starting_robot_joints_right = [-3.141686935884836, -2.356288772693141, 1.571568091705342, -0.7851389208139626, -1.5708287439107185, 0.000]

    

    for i in range(100):
        robot_left.servoj(
            starting_robot_joints_left, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
        )
        robot_right.servoj(
            starting_robot_joints_right, vel=0.1, acc=0.15, t=3.0, lookahead_time=0.2, gain=100, wait=False
        )
    time.sleep(1)
    print('Finished moving the right arm to starting states.')

    joints_left = robot_left.getj()
    print('Left robot joints: ', joints_left)
    time.sleep(1)

    robot_right = urx.Robot("192.10.0.12")
    joints_right = robot_right.getj()
    print('Right robot joints: ', joints_right)

    print('Done')
