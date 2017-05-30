'''
Created on May 8, 2017

@author: picard
'''
from trajectory_generator.trajectory_tools import *
import cPickle as pickle
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os
from collections import defaultdict
import sys
from timeit import default_timer as timer



distance = 8  # m
fov_angle = 110.0  # deg

class MyIter():
    
    _iterator = None
    _current_value = None
    
    def __init__(self, iterable_object):
        self._iterator = iter(iterable_object)
        self._current_value = self._iterator.next() 
        
    def next(self):
        tmp = self._current_value
        try:
            self._current_value = self._iterator.next()
        except StopIteration:
            self._current_value = None
            return None
        return tmp
    
    def peek(self):
        return self._current_value
        

class Trajectory_List():
    
    unaligned_trajectories = {}
    aligned_trajectories = {}
    
    def add_trajectory(self, car_name, trajectories_dict):
        
        # There is right now only one run name for each trajectory
        run_name = trajectories_dict[car_name].keys()[0]
        self.unaligned_trajectories[car_name] = self.get_timestamped_trajectories(car_name, run_name, trajectories_dict)
    
    def get_timestamped_trajectories(self, car_name, run_name, traj_dictionary):
        
        left_x = traj_dictionary[car_name][run_name]['self_trajectory']['left']['x']
        left_y = traj_dictionary[car_name][run_name]['self_trajectory']['left']['y']
        
        right_x = traj_dictionary[car_name][run_name]['self_trajectory']['right']['x']
        right_y = traj_dictionary[car_name][run_name]['self_trajectory']['right']['y']
        
        mid_xy = (((right_x + left_x) / 2.), ((left_y + right_y) / 2.))
        timestamps = traj_dictionary[car_name][run_name]['self_trajectory']['ts']
        
        return zip(timestamps, zip(mid_xy[0], mid_xy[1]))
    
    def align_trajectories(self):
        
        # After this step the aligned_trajectories will contain the same lists as unaligned trajectories
        # though the index and length will be the same for each (timestamp, (x,y)) entry. They will be
        # aligned so they can be handled easier in subsequent steps
        
        # Initialise the aligned list with empty lists
        for trajectory in self.unaligned_trajectories:
            self.aligned_trajectories[trajectory] = []
        
        
        timestamp_accuracy = 1 / 30.
        fringe = {}
        for trajectory in self.unaligned_trajectories:
            fringe[trajectory] = MyIter(self.unaligned_trajectories[trajectory])
        
        while True:
        
            # Find the minimum timestamp of any list
            min_timestamp = None
            finished_trajectories = 0
            
            for trajectory in fringe:
                current_trajectory = fringe[trajectory].peek()
                
                if current_trajectory == None:
                    finished_trajectories += 1
                    if finished_trajectories == len(fringe):
                        return self.aligned_trajectories
                    continue
                
                timestamp = current_trajectory[0]
                
                if min_timestamp == None:
                    min_timestamp = timestamp
                else:
                    min_timestamp = np.minimum(min_timestamp, timestamp)
        
            # Add a None to any list where the value is larger then the minimum + an allowed distance
            for trajectory in self.unaligned_trajectories:
                current_trajectory = fringe[trajectory].peek()
                if current_trajectory == None:
                    continue
                
                timestamp = current_trajectory[0]
                
                # If the timestamp of the currently viewed trajectory is bigger than the
                # minimum add an empty 'None' entry
                if (timestamp - timestamp_accuracy) > min_timestamp:
                    self.aligned_trajectories[trajectory].append(None)
                else:
                    # If the timestamp is smaller or equal append that value and iterate in that trajectory to the
                    # next entry
                    self.aligned_trajectories[trajectory].append(fringe[trajectory].next())
                    
            

class Collision_Scanner():
        
    def __init__(self):
        pass
    
    def get_fov_triangle(self, xy, heading, fov_angle, distance):
        '''
        returns the field of view as triangle, based on a sequence of
        coordinates. It will always regard the last coordinates in that list
        '''    
        limit_left = heading - np.deg2rad(fov_angle / 2.0)
        limit_right = heading + np.deg2rad(fov_angle / 2.0)
        
        a = xy  # Point a is the latest known position
        b = cv2.polarToCart(distance, limit_left)  # Point b is distance in front and half of fov to the left
        c = cv2.polarToCart(distance, limit_right)  # Point c is distance in front and half of fov to the right
        
        # Needed extraction of the values from the polarToCart method
        # and adding of a as the origin 
        b = b[0][0][0] + a[0], b[1][0][0] + a[1]
        c = c[0][0][0] + a[0], c[1][0][0] + a[1]
        
        return Triangle(to_point(a), to_point(b), to_point(c))
    
    
    
    def get_encounters(self, trajectories_dict):
        
        global distance
        global fov_angle
        
        
        encounters_cars_timesteps_xy = defaultdict(lambda: dict())
        for own_carname in trajectories_dict:
            for other_carname in trajectories_dict:
                if own_carname != other_carname:
                    encounters_cars_timesteps_xy[own_carname][other_carname] = []
            
        traj_list = Trajectory_List()
    
        for car_name in trajectories_dict:
            traj_list.add_trajectory(car_name, trajectories_dict)
    
        aligned_list = traj_list.align_trajectories()         
        
        
        for own_carname in aligned_list:
            
            list_index = 0
                    
            # Search for the first position in time where the own trajectory begins
            # and iterate the other index appropriately
            while aligned_list[own_carname][list_index] == None:
                list_index += 1
                
            own_trajectory = aligned_list[own_carname]
            smooth_factor = 10
            
            
            while list_index < len(own_trajectory):
                
                # If the own trajectory at that position is None, that is the end or a gap
                # in the own trajectory. We skip and increase the counter
                if own_trajectory[list_index] == None:
                    list_index += 1
                    continue
                
                if len(own_trajectory) < (list_index + smooth_factor):
                    smooth_factor = len(own_trajectory) - list_index
                
                # Calculate the heading based on values in the future or past, depending
                # on whether we are close to the beginning or end of the trajectory
                local_own_xys = [own_xy[1] for own_xy in own_trajectory[list_index:list_index + smooth_factor] if own_xy != None]
                
                
                heading = get_heading(local_own_xys)
                own_fov = self.get_fov_triangle(local_own_xys[0], heading, fov_angle, distance)
                
                for other_carname in aligned_list:
                    if own_carname != other_carname:
                        
                        # If the other trajectory has already ended, skip
                        if len(aligned_list[other_carname]) <= list_index:
                            continue
                        
                        other_xy = aligned_list[other_carname][list_index]
                        
                        # If other_xy is None then there is no other trajectory recorded at that point in time
                        if other_xy == None:
                            continue
                        
                        if own_fov.isInside(Point(other_xy[1][0], other_xy[1][1])):
                            encounters_cars_timesteps_xy[own_carname][other_carname].append({'timestamp':own_trajectory[list_index][0],'own_xy':own_trajectory[list_index][1],'other_ts_xy':other_xy})
    
                list_index += 1
                
        return encounters_cars_timesteps_xy
    
if __name__ == '__main__':
    
    trajectories_path = sys.argv[1]
    trajectories_dict = pickle.load(open(trajectories_path, "rb"))

    # print trajectories_dict['Mr_Black']['/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black']['self_trajectory']['left']['x']

    show_only_encounters = False
    plt.figure('top', figsize=(6, 6))
    
    # Timestamps from the last car are taken. In the future it would be good to check
    # if the timestamps differ for different cars
    
    # encounters_cars_timesteps_xy = defaultdict(lambda: defaultdict(dict))
    col_scanner = Collision_Scanner()
    encounter_situations = col_scanner.get_encounters(trajectories_dict)
    
    own_xy = []
    other_xy = []
    
    for own_carname in encounter_situations:
        for other_carname in encounter_situations[own_carname]:
            for entry in encounter_situations[own_carname][other_carname]:
                own_xy.append(entry['own_xy'])
                other_xy.append(entry['other_ts_xy'][1])

    from_index = 0
    to_index = 100 

    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.ion()
    for i in range(len(other_xy)):
        
        old_pos_own = plt.plot(own_xy[i][0], own_xy[i][1],'ro')
        old_pos_other = plt.plot(other_xy[i][0], other_xy[i][1],'bo')
        plt.show()
        plt.pause(1/30.)
        old_pos_own.pop(0).remove()
        old_pos_other.pop(0).remove()
