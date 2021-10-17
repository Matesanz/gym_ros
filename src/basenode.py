import gym
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16, Empty, Bool
from cv_bridge import CvBridge
from gym import Env
import numpy as np
import logging


class cartpoleEnv(Env):
    def __init__(self) -> None:
        super().__init__()

        rospy.init_node("GymEnvironment")
        self._bridge = CvBridge()

        # Publishers
        self.render_pub = rospy.Publisher("renderPub", Image, queue_size=1)
        self.obs_pub = rospy.Publisher("observationPub", Empty, queue_size=1)
        self.reward_pub = rospy.Publisher("rewardPub", Empty, queue_size=1)
        self.is_done_pub = rospy.Publisher("is_donePub", Bool, queue_size=1)
        self.info_pub = rospy.Publisher("infoPub", String, queue_size=1)

        # Subscribers
        self.start_sub = rospy.Subscriber("startSub", String, self.start_cb)
        self.make_sub = rospy.Subscriber("makeSub", String, self.make_cb)
        self.reset_sub = rospy.Subscriber("resetSub", Empty, self.reset_cb)
        self.render_sub = rospy.Subscriber("renderSub", Empty, self.render_cb)
        self.step_sub = rospy.Subscriber("stepSub", Int16, self.step_cb)

        rospy.spin()

    def start_cb(self, data: String) -> None:
        self.make_cb(data=data)
        self.reset_cb()
        self.render_cb()

    def make_cb(self, data: String) -> None:
        env_name = data.data
        self.env = gym.make(env_name)
        rospy.loginfo(f"Env {env_name} created!")

    def reset_cb(self, data: Empty = None) -> None:
        self.env.reset()

    def render_cb(self, data: Empty = None) -> None:
        frame = self.env.render(mode="rgb_array")
        self._publish_frame(frame)

    def step_cb(self, data: Int16) -> None:
        action = data.data
        state = obs, reward, is_done, info = self.env.step(action)
        self._publish_state(*state)
        self.render_cb()

    def _publish_frame(self, frame: np.ndarray) -> None:
        frame_msg = self._bridge.cv2_to_imgmsg(frame, "bgr8")
        self.render_pub.publish(frame_msg)

    def _publish_state(self, obs, reward: float, is_done: bool, info: str) -> None:
        self.obs_pub.publish(obs)
        self.reward_pub.publish(reward)
        self.is_done_pub.publish(is_done)
        self.info_pub.publish(info)

    def finish_cb(self, data: Empty) -> None:
        self.env.close()


if __name__ == "__main__":

    # Virtual display
    from pyvirtualdisplay import Display

    # pyglet.options['headless'] = True
    logging.basicConfig(level=logging.DEBUG)
    logging.debug("This will get logged")
    virtual_display = Display(visible=0, size=(1400, 900))
    virtual_display.start()
    try:
        node = cartpoleEnv()
    except rospy.ROSInterruptException:
        pass
    # virtual_display.stop()
