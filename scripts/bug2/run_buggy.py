#! /usr/bin/env python3

import rospy
from bug2 import BUG2
import gym_stage
import numpy as np
import time
import gym
import os

action_low = [-.5, -0.1]
action_high = [.5, 0.5]
goal = [[0.0, 5.5], [-2,2], [-4, 4], [-1.5, 6], [3.5,2.5], [3,-1], [-3, -1]]
agent = BUG2()

rospy.init_node('aa')
env = gym.make('Stage-v0', env_stage=1, continuous=True, goal_list=goal)

position_list = []
best_reward = -float("inf")
rewards = []

for g in goal:
   
    episode_reward = 0
    num_steps = 0
    ep_start_time = time.time()
    state = env.reset(new_random_goals=False, goal=[g])
    agent.reset()
    done = False
    print(f"Goal position:{env.get_target_position()}")
    while not done:
        
        action = agent.get_action(state)
        action[0] = np.clip(action[0], action_low[0], action_high[0])
        action[1] = np.clip(action[1], action_low[1], action_high[1])

        next_state, reward, done, info = env.step(action)
        episode_reward += reward
        state = next_state

        if done:
            break

        num_steps += 1


    # Log metrics
    episode_timing = time.time() - ep_start_time
    print(f"Reward: "
          f"[{episode_reward}/200] Step: {num_steps} Episode Timing: {round(episode_timing, 2)}s")


