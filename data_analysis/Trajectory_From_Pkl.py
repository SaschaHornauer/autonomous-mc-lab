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
from aruco_tools.mode import *
from trajectory_generator.trajectory_tools import *
import sys

import copy
from timeit import default_timer as timer



class Trajectory_From_Pkl:
    
    calculation_horizon = 20  # timesteps
    goal_lookahead = 60
    timestep_offset = 0
    # adding furtive and play later TODO FIXIT
    trajectories = {}
    
    def __init__(self, pkl_filename, start_timestamp, end_timestep, modes_selected):
        
        print "Calculating trajectories ..."
        actual_trajectories = self.process_pkl_file(pkl_filename, start_timestamp, end_timestep)
       
        start = timer()
        start_timestep = 0
        end_timestep = len(actual_trajectories.itervalues().next()['timestamps'])
        
        plot_video = True
        resulting_trajectories = self.get_trajectory(actual_trajectories, plot_video, int(start_timestep), int(end_timestep), modes_selected)
        
        end = timer()
        print ("Finished in " + str(end - start) + " seconds.")
        self.trajectories = resulting_trajectories
                
    
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
            car_x, car_y = self.get_average_position(car_left_x, car_left_y, car_right_x, car_right_y)
        
            actual_trajectories[car] = {'timestamps':timestamps, 'position': (car_x, car_y)}
        
        return actual_trajectories


    def get_continous_segments(self, encounter_situations):
        
        # We go through the list of encounter situations and allow
        # for a certain gap where the other car is not visible
        segments = []
        
        allowed_gap = 3
        segment_counter = 0
        inner_segment_list = []
        
        for i in range(0,len(encounter_situations)):
            
            if i == len(encounter_situations)-1:
                segments.append(inner_segment_list)
                break
            
            if encounter_situations[i+1]-encounter_situations[i] > allowed_gap:
                inner_segment_list.append(encounter_situations[i])
                segments.append(inner_segment_list)
                segment_counter += 1
                inner_segment_list = []
                continue
            else:                
                inner_segment_list.append(encounter_situations[i])
                
            
        
        return segments
    
    def get_trajectory(self, actual_trajectories, plot_video, start_timestep, end_timestep, modes_selected):
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
            # As long as there is only one other car we fake second one by taking its trajectory 
            # and turning it aroudn
            
            first_trajectory_in_dict = other_trajectories.itervalues().next()
            fake_timestamps = first_trajectory_in_dict['timestamps']
            fake_positions = first_trajectory_in_dict['position'][::-1]
            
            fake_trajectory = {'timestamps':fake_timestamps,'position':fake_positions}
            other_trajectories['Mr_Fake']=fake_trajectory
            
            
                        
            for other_cars in other_trajectories:
                other_positions.append(zip(other_trajectories[other_cars]['position'][0][0], other_trajectories[other_cars]['position'][1][0]))
                
                                    
            # Now calculate the evasive trajectory for the car
            own_trajectories = actual_trajectories[car]
            
            resulting_trajectories = {}
            
             # Get the short term trajectory
            own_trajectory = own_trajectories['position']

            # Reverse all the positions to get fake positions
            #fake_positions = fake_positions[0][::-1]
            # TODO: Create a sensible way to include the other cars
            #other_xy = []
            #other_xy.append(other_positions[0])
            #other_xy.append(fake_positions)
            
    
    
                        
            own_x = (own_trajectory[0][0])
            own_y = (own_trajectory[1][0])
            own_xy = zip(own_x, own_y)
                    
            for act_mode in modes_selected:
                
                if act_mode == behavior.circle:
                    goal_xys = evasion_generator.get_center_circle_points(own_xy)
                    
                    trajectories_in_delta_angles = evasion_generator.get_evasive_trajectory(own_xy, other_positions, self.timestep_offset, self.goal_lookahead, plot_video, end_timestep, goal_xys)
                    resulting_trajectories = convert_delta_to_steer(trajectories_in_delta_angles)
                    timestamps = actual_trajectories[car]['timestamps']
                    evasion_trajectory_data[car] = {'timestamps':timestamps, 'trajectories':resulting_trajectories}
                elif act_mode == behavior.follow:
                    
                    # First find all the points in the dataset where another car is actually close
                    
                    encounter_timestep, closest_xys, _ = collision_scanner.get_close_encounters_in_list(own_xy, other_positions, start_timestep, end_timestep)
                    
                    time_segments = self.get_continous_segments(encounter_timestep)
                    # Add those points to the goal trajectory. 
                    
                    goal_xys = [None] * time_segments[-1][-1]
                    
                    evasion_segment_data = []
                    for segment in time_segments:
                        
                        start_time = segment[0]
                        end_time = segment[len(segment)-1]
                        
                        # Add goal trajectory
                        goal_xys[start_time:end_time] = points_to_list(closest_xys)
                        
                        trajectories_in_delta_angles = evasion_generator.get_evasive_trajectory(own_xy, other_positions, start_time, self.goal_lookahead, plot_video, len(segment), goal_xys)
                        resulting_trajectories = convert_delta_to_steer(trajectories_in_delta_angles)
                        timestamps = actual_trajectories[car]['timestamps']
                        evasion_segment_data.append({'timestamps':timestamps, 'trajectories':resulting_trajectories})
                    
                    
                    evasion_trajectory_data[car]=evasion_segment_data
                
            
        return evasion_trajectory_data
        
    
    
    def get_average_position(self, left_x, left_y, right_x, right_y):
        '''
        Returns 
        '''
        return np.array(map(add, left_x, right_x)) / 2., np.array(map(add, left_y, right_y)) / 2.
    
    
def get_trajectories(pkl_filename, start_timestamp, end_timestep, modes_selected): 
    trajectory_parser = Trajectory_From_Pkl(pkl_filename, start_timestamp, end_timestep, modes_selected)
    resulting_trajectories = trajectory_parser.trajectories
    return resulting_trajectories
    
if __name__ == '__main__':
    pass
    
    
    

    
    
