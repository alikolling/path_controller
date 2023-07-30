import numpy as np
import math


class BUG2:
    def __init__(self):
        self.regions = [0, 0, 0, 0, 0]
        self.flag = 0
        self.pose_flag = 0
        self.action = [0.0, 0.0]
        self.flag_1 = 0
        self.dist = 0.0
        self.first = True
        self.collision_distance = 0.18 * 3

    def reset(self):
        self.regions = [0, 0, 0, 0, 0]
        self.flag = 0
        self.pose_flag = 0
        self.action = [0.0, 0.0]
        self.flag_1 = 0
        self.dist = 0.0
        self.first = True
        self.collision_distance = 0.18 * 3
    
    def laser_scan(self, laser_msg):
        laser_msg = np.array(laser_msg)
        laser_msg = laser_msg[::15]

        self.regions = [
            min(laser_msg[[19, 18, 17]]),# Right
            min(laser_msg[[22, 21, 20, 19]]),  # Front Right
            min(laser_msg[[0, 23, 22, 1]]),  # Front
            min(laser_msg[[1, 2, 3, 4]]),  # Front Left
            min(laser_msg[[4, 5, 6]]),  # Left
        ]

    
    def flag_shift(self, f):
        self.flag = f
    
    def obstacle_avoidance(self):

        if self.regions[2] > self.collision_distance and self.regions[3] < self.collision_distance and self.regions[1] < self.collision_distance:
            self.action[1] = 0.4 / 4
            #frente livre
        elif self.regions[2] < self.collision_distance and self.regions[3] < self.collision_distance and self.regions[1] < self.collision_distance:
            self.action[1] = -0.1
            #ré
        elif self.regions[2] < self.collision_distance and self.regions[3] > self.collision_distance and self.regions[1] < self.collision_distance:
            self.action[1] = 0.2 / 4
            self.action[0] = 0.3 
            #gira esq
        elif self.regions[2] < self.collision_distance and self.regions[3] < self.collision_distance and self.regions[1] > self.collision_distance:
            self.action[1] = 0.2 / 4
            self.action[0] = -0.3
            #gira direita
        elif self.regions[2] > self.collision_distance and self.regions[3] < self.collision_distance and self.regions[1] > self.collision_distance:
            self.action[1] = 0.3 / 4 
            self.action[0] = -0.2
            #gira direita mais
        elif self.regions[2] > self.collision_distance and self.regions[3] > self.collision_distance and self.regions[1] < self.collision_distance:
            self.action[1] = 0.4 / 4
            self.action[0] = 0.2
            #gira esq mais

        
    def angle_towards_goal(self, angle):
        difference_angle = angle
        
        if math.fabs(difference_angle) > 0.05:
            self.action[0] = 1. * math.fabs(difference_angle) if difference_angle > 0. else -1. * math.fabs(difference_angle)
            print(difference_angle, self.action[0])
            self.action[1] = 0.
        if math.fabs(difference_angle) <= 0.05:
            self.flag_shift(1)

    def move(self, angle, distance):
        difference_angle = angle
        difference_pos = distance

        if difference_pos > 0.2:
            self.action[1] = 0.5 * difference_pos if self.action[1] > 0.1 else 0.1
            self.action[0] = 0.
        else:
            self.flag_shift(2)

        # state change conditions
        if math.fabs(difference_angle) > 0.03:
            self.flag_shift(0)

        
    def get_action(self, state):
        
        self.laser_scan(state[0:-2])

        self.dist = state[-1]
        self.angle = state[-2]
        
        if self.dist < 4 and (self.regions[2] < self.collision_distance or self.regions[3] < self.collision_distance or self.regions[1] < self.collision_distance):
            #Evitando obstáculos < 4
            self.obstacle_avoidance()
            self.flag_1 = 1
        
        elif self.dist < 4 and (self.regions[2] > self.collision_distance and self.regions[3] > self.collision_distance and self.regions[1] > self.collision_distance):
            if self.flag == 0:
                #angle livre
                self.angle_towards_goal(self.angle)
            elif self.flag == 1:
                #move livre
                self.move(self.angle, self.dist)

        elif self.dist < 4 and self.flag_1 == 1:
            if self.flag == 0:
                #angle
                self.angle_towards_goal(self.angle)
            elif self.flag == 1:
                #move
                self.move(self.angle, self.dist)
            self.flag_1 = 0

       
        elif self.dist > 4:
            #Evitando obstáculos > 4
            self.obstacle_avoidance()
        
        return self.action
