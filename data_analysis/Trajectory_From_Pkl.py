'''
Created on May 10, 2017

@author: Sascha Hornauer
'''
import os
import pickle
import numpy as np
from operator import add, div
from trajectory_generator import collision_scanner
from trajectory_generator import evasion_generator_simple as evasion_generator
from trajectory_generator.trajectory_tools import *
import matplotlib.pyplot as plt
import sys
import copy
from timeit import default_timer as timer


class Trajectory_From_Pkl:
    
    calculation_horizon = 20  # timesteps
    goal_lookahead = 60
    timestep_offset = 0
    # timestep_offset = 0
    
    def __init__(self, pkl_filename=None, framerate=1. / 30.):
        pass

    def get_trajectory(self, actual_trajectories, plot_video, start_timestep, end_timestep):
        self.timestep_offset = start_timestep
        evasion_trajectory_data = {}
        
        # For all cars in actual trajectories
        for car in actual_trajectories:
            print "Calculating trajectories for " + str(car)
            # Add all other trajectories into a list
            other_trajectories = actual_trajectories.copy()
            del other_trajectories[car]
            
            # Create a list containing all other trajectories
            other_positions = []
            # As long as there is only one other car we fake second one
            fake_positions = []
            
            for other_cars in other_trajectories:
                other_positions.append(zip(other_trajectories[other_cars]['position'][0][0], other_trajectories[other_cars]['position'][1][0]))
                fake_positions.append(zip(other_trajectories[other_cars]['position'][0][0], other_trajectories[other_cars]['position'][1][0]))
            
            # Now calculate the evasive trajectory for the car
            own_trajectories = actual_trajectories[car]
            
            evasion_trajectories = {}
            
             
            # Get the short term trajectory
            own_trajectory = own_trajectories['position']

            # Reverse all the positions to get fake positions
            fake_positions = fake_positions[0][::-1]
    
            # TODO: Create a sensible way to include the other cars
            other_xy = []
            other_xy.append(other_positions[0])
            other_xy.append(fake_positions)
    
            own_x = (own_trajectory[0][0])
            own_y = (own_trajectory[1][0])
            own_xy = zip(own_x, own_y)
            
            evasion_trajectories = convert_delta_to_steer(evasion_generator.get_evasive_trajectory(own_xy, other_xy, self.timestep_offset, self.goal_lookahead, plot_video, end_timestep))
            timestamps = actual_trajectories[car]['timestamps']
            evasion_trajectory_data[car] = {'timestamps':timestamps, 'trajectories':evasion_trajectories}
            
        return evasion_trajectory_data
        
    
    
    def get_average_position(self, left_x, left_y, right_x, right_y):
        '''
        Returns 
        '''
        return np.array(map(add, left_x, right_x)) / 2., np.array(map(add, left_y, right_y)) / 2.
    
    
if __name__ == '__main__':
    
    trajectory_getter = Trajectory_From_Pkl()
    
    home = os.path.expanduser("~")
    pickle_file = pickle.load(open(home + '/kzpy3/teg9/trajectories.pkl', "rb"))

    t1 = 1493425694.71 + 5
    t2 = 1493425899.676476 - 100
    timestamps = np.arange(t1, t2, 1 / 30.)
    
    evasion_trajectories = {}
    actual_trajectories = {}
    
    plot_video = False
    
    # Enter here the carnames which should be considered
    for car in ['Mr_Black', 'Mr_Blue']:
            
        car_left_x = [pickle_file[car]['left'][0](timestamps)]
        car_right_x = [pickle_file[car]['right'][0](timestamps)]
        car_left_y = [pickle_file[car]['left'][1](timestamps)]
        car_right_y = [pickle_file[car]['right'][1](timestamps)]
        car_x, car_y = trajectory_getter.get_average_position(car_left_x, car_left_y, car_right_x, car_right_y)
    
        actual_trajectories[car] = {'timestamps':timestamps, 'position': (car_x, car_y)}
        
    start = timer()
    start_timestep = 0
    end_timestep = len(actual_trajectories.itervalues().next()['timestamps'])
    
    evasion_trajectories = trajectory_getter.get_trajectory(actual_trajectories, plot_video, int(start_timestep), int(end_timestep))
    
    
    end = timer()
    print evasion_trajectories
    print ("##################################")
    print (end - start)
    
    
    
