#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from gazebo_msgs.msg import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from environment import Env
from bug2.bug2 import BUG2

if __name__ == "__main__":
    rospy.init_node("path_controller_node", anonymous=False)
    env = Env()
    agent = BUG2()

    action_low = [-0.5, -0.1]
    action_high = [0.5, 0.5]

    state = env.reset()

    agent.reset()

    r = rospy.Rate(5)  # 10hz

    while not rospy.is_shutdown():
        # FACA SEU CODIGO AQUI
        action = agent.get_action(state)
        action[0] = np.clip(action[0], action_low[0], action_high[0])
        action[1] = np.clip(action[1], action_low[1], action_high[1])

        next_state = env.step(action)
        state = next_state
	
        r.sleep()
