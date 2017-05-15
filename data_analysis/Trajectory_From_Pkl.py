'''
Created on May 10, 2017

@author: Sascha Hornauer
'''
import os
import cPickle as pickle
import numpy as np
from operator import add, div
from trajectory_generator import collision_scanner
from trajectory_generator import evasion_generator_bicycle as evasion_generator


import sys

import copy
from timeit import default_timer as timer
from Trajectory_From_Pkl import actual_trajectories

class Trajectory_From_Pkl:
    
    calculation_horizon = 20  # timesteps
    goal_lookahead = 60
    timestep_offset = 0
    # adding furtive and play later TODO FIXIT
    
    
    def __init__(self, pkl_filename, start_timestamp, end_timestep, modes_selected):
        
        print "Calculating trajectories ..."
        actual_trajectories = self.process_pkl_file(pkl_filename, start_timestamp, end_timestep)
       
        start = timer()
        start_timestep = 0
        end_timestep = len(actual_trajectories.itervalues().next()['timestamps'])
        
        # TODO FIXIT go over all the modes and get the trajectories for each mode
        # As an example take the trajectory from Mr Blue to follow
        goal_trajectory = actual_trajectories['Mr_Blue']
        
        plot_video = False
        resulting_trajectories = self.get_trajectory(actual_trajectories, plot_video, int(start_timestep), int(end_timestep), goal_trajectory)
        
        end = timer()
        print ("Finished in " + str(end - start) + " seconds.")
        return resulting_trajectories
                
    
    def process_pkl_file(self,pkl_file, start_timestamp, end_timestamp):
        
        pickle_file = pickle.load(open(pkl_file, "rb"))
        
        timestamps = np.arange(start_timestamp, end_timestamp, 1 / 30.)
                
        actual_trajectories = {}
        
        # Enter here the carnames which should be considered
        for car in ['Mr_Black', 'Mr_Blue']:
                
            car_left_x = [pickle_file[car]['left'][0](timestamps)]
            car_right_x = [pickle_file[car]['right'][0](timestamps)]
            car_left_y = [pickle_file[car]['left'][1](timestamps)]
            car_right_y = [pickle_file[car]['right'][1](timestamps)]
            car_x, car_y = self.trajectory_getter.get_average_position(car_left_x, car_left_y, car_right_x, car_right_y)
        
            actual_trajectories[car] = {'timestamps':timestamps, 'position': (car_x, car_y)}
        
        return actual_trajectories

    def get_trajectory(self, actual_trajectories, plot_video, start_timestep, end_timestep, goal_trajectory):
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
            
            resulting_trajectories = {}
            
            # Add goal trajectory
            goal_trajectory_data = zip(goal_trajectory['position'][0][0], goal_trajectory['position'][1][0])
            
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
    

            # The goal_xys positions will be followed while the underlying trajectory planner
            # takes care of boundary and obstacle evasion. If it should follow the car in sight
            # or try to go in a circle is decided here            
            goal_xys_circle = evasion_generator.get_center_circle_points(own_xy)
            # follow_xys = ...
            
            # follow: 
            # follow the car in view, 
            # the nearest car
            # adjust speed
            # evade boundary
            #
            # circle EVADE OTHER 
            # EVADE BOUNDARY
            # adjust speed 
            # 
            # stop if in a car is too close 
            #             
            # 0-99,0-99,49
            #
            # furtive: 
            # close edges
            # 
            # play: 
            # close, speed more

            # write an easy method
            # pkl -> dict

            trajectories_in_delta_angles = evasion_generator.get_evasive_trajectory(own_xy, other_xy, self.timestep_offset, self.goal_lookahead, plot_video, end_timestep, goal_trajectory_data)
            resulting_trajectories = self.convert_delta_to_steer(trajectories_in_delta_angles)
            timestamps = actual_trajectories[car]['timestamps']
            evasion_trajectory_data[car] = {'timestamps':timestamps, 'trajectories':resulting_trajectories}
            
        return evasion_trajectory_data
        
    
    
    def get_average_position(self, left_x, left_y, right_x, right_y):
        '''
        Returns 
        '''
        return np.array(map(add, left_x, right_x)) / 2., np.array(map(add, left_y, right_y)) / 2.
    
    
    def get_trajectories(self, pkl_filename, start_timestamp, end_timestep, modes_selected):
        resulting_trajectories = Trajectory_From_Pkl(pkl_filename, start_timestamp, end_timestep, modes_selected)
        return resulting_trajectories
    
if __name__ == '__main__':
    pass
    
    
    

    
    
