# importing libraries
from math import *
import random

# This robot lives in 2D, x-y space, and its motion is pointed in a random direction, initially.
# It moves in a straight line until it comes close to a wall at which point it stops.
# For measurements, it  senses the x- and y-distanceto landmarks.

class robot:
    
    
    # creates a robot with the specified parameters and initializes the location (self.x, self.y) to the center of the world
    def __init__(self, world_size = 100.0, measurement_range = 30.0,
                 motion_noise = 1.0, measurement_noise = 1.0):
        self.measurement_noise = 0.0
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size / 2.0
        self.y = world_size / 2.0
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.landmarks = []
        self.num_landmarks = 0
    
    
    # returns a positive, random float
    def rand(self):
        return random.random() * 2.0 - 1.0
    
    
    # attempts to move robot by dx, dy. If outside world boundary, then the move does nothing and instead returns failure
    def move(self, dx, dy):
        
        x = self.x + dx + self.rand() * self.motion_noise
        y = self.y + dy + self.rand() * self.motion_noise
        
        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False
        else:
            self.x = x
            self.y = y
            return True


    # sense: returns x- and y- distances to landmarks within visibility range because not all landmarks may be in this range, the list of measurementsis of variable length
    def sense(self):
           
        measurements = []
        for ind, landmark in enumerate(self.landmarks):
            dx = -self.x + landmark[0] + self.rand()*self.measurement_noise
            dy = -self.y + landmark[1] + self.rand()*self.measurement_noise
            if (self.measurement_range == -1) or ((abs(dx) <= self.measurement_range) and (abs(dy) <= self.measurement_range)):
                measurements.append([ind, dx, dy])
        return measurements

    # makes random landmarks located in the world
    def make_landmarks(self, num_landmarks):
        self.landmarks = []
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])
        self.num_landmarks = num_landmarks


    # called when print(robot) is called; prints the robot's location
    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]'  % (self.x, self.y)