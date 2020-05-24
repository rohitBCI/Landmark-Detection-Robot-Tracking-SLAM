# importing libraries
import numpy as np
from helpers import make_data
import matplotlib.pyplot as plt
from pandas import DataFrame
import seaborn as sns
from numpy.linalg import inv
from numpy.linalg import det

# world parameters
num_landmarks      = 5        # number of landmarks
N                  = 20       # time steps
world_size         = 100.0    # size of world (square)

# robot parameters
measurement_range  = 50.0     # range at which we can sense landmarks
motion_noise       = 2.0      # noise in robot motion
measurement_noise  = 2.0      # noise in the measurements
distance           = 20.0     # distance by which robot (intends to) move each iteratation 


# make_data instantiates a robot, AND generates random landmarks for a given world size and number of landmarks
data = make_data(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)


def initialize_constraints(N, num_landmarks, world_size):
	# args: number of time steps N, number of landmarks, size of world
   
   
    # Defining the constraint matrix, Omega, with two initial "strength" valuesfor the initial x, y location of our robot
    dim = 2*(N + num_landmarks)  # 2: because we are in a 2D world (x,y)
    omega = np.zeros((dim ,dim ))
    omega[0, 0] = 1 
    omega[1, 1] = 1
    
    # Defining the constraint *vector*, xi
    xi = np.zeros((dim, 1))
    xi[0] = world_size / 2.0 # initial_pos x
    xi[1] = world_size / 2.0 # initial_pos y
    
    return omega, xi
    
# slam takes in 6 arguments and returns mu
# mu is the entire path traversed by a robot (all x,y poses) *and* all landmarks locations
def slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise):
    
    omega,xi=initialize_constraints(N, num_landmarks, world_size)
    
    for i in range(len(data)):
        measurements = data[i][0]
        motion = data[i][1]
        for j in measurements:
            id_landmark = j[0]
            x = j[1]
            y = j[2]

            # x measurement update
            omega[2*i, 2*i] += 1/measurement_noise
            omega[2*i, 2*(N + id_landmark)] -= 1/measurement_noise
            omega[2*(N + id_landmark), 2*i] -= 1/measurement_noise
            omega[2*(N + id_landmark), 2*(N +id_landmark)] += 1/measurement_noise

            # y measurement update
            omega[2*i +1, 2*i +1] += 1/measurement_noise
            omega[2*i +1, 2*(N+id_landmark) +1] -= 1/measurement_noise
            omega[2*(N + id_landmark) +1, 2*i +1] -= 1/measurement_noise
            omega[2*(N + id_landmark) +1, 2*(N +id_landmark) +1] += 1/measurement_noise

            xi[2*i,0] -= x/measurement_noise
            xi[2*(N+id_landmark), 0] += x/measurement_noise

            xi[2*i +1,0] -= y/measurement_noise
            xi[2*(N+id_landmark) + 1, 0] += y/measurement_noise
            
            
        dx=motion[0]
        dy=motion[1]        
       
        # x motion update 
        omega[2*i,2*i] += 1/motion_noise
        omega[2*i,2*i +2] -= 1/motion_noise
        omega[2*i +2,2*i] -= 1/motion_noise
        omega[2*i +2,2*i +2] += 1/motion_noise

        # y motion update
        omega[2*i +1,2*i +1] += 1/motion_noise
        omega[2*i +1,2*i +3] -= 1/motion_noise
        omega[2*i +3,2*i +1] -= 1/motion_noise
        omega[2*i +3,2*i +3] += 1/motion_noise

        xi[2*i,0] -= dx/motion_noise
        xi[2*i +2,0] += dx/motion_noise

        xi[2*i +1,0] -= dy/motion_noise
        xi[2*i +3,0] += dy/motion_noise 
        
    omega_inverse=np.linalg.inv(omega)
    mu =np.dot(omega_inverse,xi)
    
    return mu



# creates a list of poses and of landmarks for ease of printing
def get_poses_landmarks(mu, N):
    # create a list of poses
    poses = []
    for i in range(N):
        poses.append((mu[2*i].item(), mu[2*i+1].item()))

    # create a list of landmarks
    landmarks = []
    for i in range(num_landmarks):
        landmarks.append((mu[2*(N+i)].item(), mu[2*(N+i)+1].item()))

    # return completed lists
    return poses, landmarks


def print_all(poses, landmarks):
    print('\n')
    print('Estimated Poses:')
    for i in range(len(poses)):
        print('['+', '.join('%.3f'%p for p in poses[i])+']')
    print('\n')
    print('Estimated Landmarks:')
    for i in range(len(landmarks)):
        print('['+', '.join('%.3f'%l for l in landmarks[i])+']')



# call the implementation of slam, passing in the necessary parameters
mu = slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise)

# print out the resulting landmarks and poses
if(mu is not None):
    # get the lists of poses and landmarks and print them out
    poses, landmarks = get_poses_landmarks(mu, N)
    print_all(poses, landmarks)