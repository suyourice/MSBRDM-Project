import rospy
from wsg_50_common.srv import Move
from std_msgs.msg import Bool

def move_gripper(width, speed):
    try:
        move = rospy.ServiceProxy('/wsg_50_driver/move', Move)
        import ipdb; ipdb.set_trace()
        resp = move(width, speed)
        return resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def grasp_object(width, speed):
    try:
        grasp = rospy.ServiceProxy('/wsg_50_driver/grasp', Move)
        resp = grasp(width, speed)
        return resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def release_object(width, speed):
    try:
        release = rospy.ServiceProxy('/wsg_50_driver/release', Move)
        resp = release(width, speed)
        return resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def main():
    rospy.init_node('gripper_control')

    # Move the gripper to a position of 50 at a speed of 30
    error = move_gripper(50, 30)
    if error != 0:
        print("Error moving gripper: %d" % error)

    # Grasp an object
    error = grasp_object(30, 10)
    if error != 0:
        print("Error grasping object: %d" % error)

    # Release the object
    error = release_object(50, 30)
    if error != 0:
        print("Error releasing object: %d" % error)

if __name__ == '__main__':
    main()
