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

class Trajectory_From_Pkl:
    
    calculation_horizon = 20 # timesteps
    goal_lookahead = 60
    timestep_offset = 0
    #timestep_offset = 0
    
    def __init__(self,pkl_filename=None,framerate = 1./30.):
        pass

    def get_trajectory(self,actual_trajectories, plot_video):
        
        evasion_trajectory_data = {}
        
        # For all cars in actual trajectories
        for car in actual_trajectories:
            print "Calculating trajectories for " + str(car)
            # Add all other trajectories into a list
            other_trajectories = actual_trajectories.copy()
            del other_trajectories[car]
            
            # Create a list containing all other trajectories
            other_positions = []
            for other_cars in other_trajectories:
                other_positions.append(zip(other_trajectories[other_cars]['position'][0][0],other_trajectories[other_cars]['position'][1][0]))
            

            # Now calculate the evasive trajectory for the car
            own_trajectories = actual_trajectories[car]
            
            evasion_trajectories = {}
            
            number_of_timesteps = len(own_trajectories['timestamps'])
            
            for i in range(self.timestep_offset,number_of_timesteps-(self.goal_lookahead+4)):
                print "Processing timestamp " + str(own_trajectories['timestamps'][i]) + " for " + str(car) + " and " + str((number_of_timesteps-(self.goal_lookahead+4))-i) + " to go." 
                # Get the short term trajectory
                own_trajectory = own_trajectories['position']
                # 4 is for the final heading calculation
                own_x = (own_trajectory[0][0][i:i+self.goal_lookahead+4])
                own_y = (own_trajectory[1][0][i:i+self.goal_lookahead+4])
                own_xy = zip(own_x,own_y)

                # The own trajectory, sent to the algorithm, must be longer to select
                # a long term goal. The trajectory of other vehicles can be short
                # since only within the planning horizon, which is as long as the trajectory
                # length, their movement is considered
                for other_position in other_positions:
                    other_xy = other_position[i:i+self.calculation_horizon]

                evasion_trajectories[own_trajectories['timestamps'][i]] = convert_delta_to_steer(evasion_generator.get_evasive_trajectory(own_xy, other_xy, 0, self.calculation_horizon,self.goal_lookahead,i,plot_video))
            
            evasion_trajectory_data[car] = evasion_trajectories
            
        return evasion_trajectory_data
        
    
    
    def get_average_position(self,left_x,left_y,right_x,right_y):
        '''
        Returns 
        '''
        return np.array(map(add,left_x,right_x))/2.,np.array(map(add,left_y,right_y))/2.
    
    
if __name__ == '__main__':
    
    trajectory_getter = Trajectory_From_Pkl()
    
    home = os.path.expanduser("~")
    pickle_file = pickle.load(open(home + '/kzpy3/teg9/trajectories.pkl', "rb"))

    t1 = 1493425694.71+5
    t2 = 1493425899.676476 - 100
    timestamps = np.arange(t1,t2,1/30.)
    
    evasion_trajectories = {}
    actual_trajectories = {}
    
    plot_video = False
    
    # Enter here the carnames which should be considered
    for car in ['Mr_Black','Mr_Blue']:
            
        car_left_x = [pickle_file[car]['left'][0](timestamps)]
        car_right_x = [pickle_file[car]['right'][0](timestamps)]
        car_left_y = [pickle_file[car]['left'][1](timestamps)]
        car_right_y = [pickle_file[car]['right'][1](timestamps)]
        car_x, car_y = trajectory_getter.get_average_position(car_left_x, car_left_y, car_right_x, car_right_y)
    
        actual_trajectories[car] = {'timestamps':timestamps,'position': (car_x,car_y)}
        
    evasion_trajectories = trajectory_getter.get_trajectory(actual_trajectories, plot_video)
    
    
    
    