from ur_robot import UR

def main():
    robot = UR("urExample:", name="robot")
    robot.wait_for_connection()

    home = robot.joints.readback.get()

    joints = home.copy()
    joints[1] += 10
    robot.joints.set(joints).wait()
    robot.gripper.set_close().wait()

    pose = robot.pose.readback.get()
    pose[2] -= 10
    robot.pose.set(pose).wait()
    robot.gripper.set_open().wait()

    robot.joints.set(home).wait()

if __name__ == "__main__":
    main()
