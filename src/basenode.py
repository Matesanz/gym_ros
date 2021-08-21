import rospy


class BaseNode:
    def __init__(self) -> None:

        rospy.init_node("BaseNode", anonymous=False)
        rospy.spin()


if __name__ == "__main__":

    try:
        BaseNode()
    except rospy.ROSInterruptException:
        pass
