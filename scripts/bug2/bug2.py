#! /usr/bin/env python3
import numpy as np
import math


class BUG2:
    def __init__(self):
        self.min_lasers = [0, 0, 0, 0, 0]
        self.move_or_angle = 1   #moving forward or turning
        self.action = [0.0, 0.1]
        self.avoidance_on = 0 # obstacle avoidance or moving to target
        self.collision_distance = 0.18 * 3
        
    def reset(self):
        self.min_lasers = [0, 0, 0, 0, 0]
        self.move_or_angle = 1 #moving forward or turning
        self.action = [0.0, 0.1]
        self.avoidance_on = 0 # obstacle avoidance or moving to target
        self.collision_distance = 0.18 * 3

    def angle_towards_goal(self, angle):
    '''
    Funcion that orientates the robot towards the goal.
    '''
        difference_angle = angle
        if np.absolute(difference_angle) > 0.05:
            self.action[0] = 0.1 if difference_angle > 0 else -0.1
            self.action[1] = 0.0
        if np.absolute(difference_angle) <= 0.05:
            self.move_or_angle = 1

    def obstacle_avoidance(self):
    '''
    Function that avoid the obstacles.
    '''
        values = self.min_lasers
        if values[2] < self.collision_distance and values[3] < self.collision_distance and values[1] < self.collision_distance:
            self.action[1] = 0.005 # obstacles on all sides, stop and turn
            self.action[0] = -0.3 /4
        elif values[2] < self.collision_distance and values[3] < self.collision_distance and values[1] > self.collision_distance:
            self.action[1] = 0.005  #obstacless on the front and right, turn left
            self.action[0] = 0.4/ 4
        elif values[2] < self.collision_distance and values[3] > self.collision_distance and values[1] < self.collision_distance:
            self.action[1] = 0.005 # obstacles in the front and left, turn right
            self.action[0] = -0.4 /4  # -0.4
        elif values[2] > self.collision_distance and values[3] > self.collision_distance and values[1] > self.collision_distance:
            self.action[1] = 0.1
            self.action[0]= 0.
        elif values[2] > self.collision_distance and values[3] < self.collision_distance and values[1] > self.collision_distance:
            self.action[1] = 0.3 / 4 # obstacles in right, turn left and go forward
            self.action[0] = -0.2  / 4 
        elif values[2] < self.collision_distance and values[3] > self.collision_distance and values[1] > self.collision_distance:
            self.action[1] = 0.005    # obstacles in front, stop and turn
            self.action[0] = -0.3 / 4
        elif values[2] > self.collision_distance and values[3] > self.collision_distance and values[1] < self.collision_distance:
            self.action[1] = 0.4 / 4 # obstacles left, turn right and go forward
            self.action[0] = 0.1/ 4 

    def move(self, angle, distance):
    '''
    Function to move the robot on the direction of the goal.
    '''
        difference_angle = angle
        difference_pos = distance

        #move towards target
        if difference_pos > .1:
            self.action[1] = 0.05
            self.action[0]= 0. 
        else:
            self.move_or_angle = 2 # reached target
        # state change conditions
        if np.absolute(difference_angle) > 0.03:
            self.move_or_angle = 0

    def laser_scan(self, laser_msg):
        laser_msg = np.array(laser_msg)

        self.min_lasers = [
                  min(laser_msg[865:1081]),        #Right 
                  min(laser_msg[649:864]),        #Front Right
                  min(laser_msg[433:648]),       #Front
                  min(laser_msg[216:432]),      #Front Left
                  min(laser_msg[0:215]),       #Left
                 ]

    def get_action(self, state):
    '''
    Function to decide the action to be taken. Recieves the laser ranges, the distance and the angle from the robot to the goal.
    '''
        self.laser_scan(state[0:-2])
        values = self.min_lasers
        self.dist = state[-1]
        self.angle = state[-2]
        #if colliding, avoid obstacles
        if (values[2] < self.collision_distance or values[3] < self.collision_distance or values[1] < self.collision_distance):
                self.avoidance_on = 1
                self.obstacle_avoidance()
        
        #if colliding and close to target
        elif self.dist < 4 and values[2] < self.collision_distance:
            if self.dist < 0.5:
                self.action[0] = 0.0
                self.action[1] = 0.0
                self.angle_towards_goal(angle=state[-2])
                self.move(angle=state[-2], distance=state[-1])
                self.avoidance_on = 0

            else:
                self.avoidance_on = 1
                self.obstacle_avoidance()
        #if not colliding and far go to target
        elif self.dist > 4 or (values[2] > self.collision_distance and values[3] > self.collision_distance and values[1] > self.collision_distance):
            if self.move_or_angle == 0:
                self.angle_towards_goal(angle=state[-2])

            elif self.move_or_angle == 1:
                self.move(angle=state[-2], distance=state[-1])
        #if close and colision avoidance is on moves or orientate to the goal        
        elif self.dist < 4 and self.avoidance_on == 1:
            if self.move_or_angle == 0:
                self.angle_towards_goal(angle=state[-2])

            elif self.move_or_angle == 1:
                self.move(angle=state[-2], distance=state[-1])

            self.avoidance_on = 0
        #if really close, send it
        elif self.dist < 0.5:
            self.action[0] = 0.0
            self.action[1] = 0.0
            self.angle_towards_goal(angle=state[-2])
            self.move(angle=state[-2], distance=state[-1])
            self.avoidance_on = 0
        
        return self.action
