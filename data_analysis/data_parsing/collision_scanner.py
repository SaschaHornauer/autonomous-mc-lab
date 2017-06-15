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
from cv_bridge import CvBridge, CvBridgeError
import rosbag
from tensorflow.contrib.learn.python.learn.graph_actions import run_n
from kzpy3.data_analysis.data_parsing.Image_Bagfile_Handler import Image_Bagfile_Handler


distance = 8  # m
fov_angle = 66.0  # deg
smooth_heading_over_timesteps = 30 

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
            smooth_factor = smooth_heading_over_timesteps
            
            
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
                            encounters_cars_timesteps_xy[own_carname][other_carname].append({'run_name':trajectories_dict[own_carname].keys()[0],'fov':own_fov,'timestamp':own_trajectory[list_index][0],'own_xy':own_trajectory[list_index][1],'other_ts_xy':other_xy})
    
                list_index += 1
                
        return encounters_cars_timesteps_xy
    

def show_video_image(timestamp,run_name):
    
    # Use Image Bagfile Handler
    # Change so a certain timestep can be given and the bagfile is automatically opened, searched and displayed
    
    pass

def onClick(event):
    global pause
    pause ^= True
    
    
def draw_content(plot,i):
    fov_a = own_fov[i].a
    fov_b = own_fov[i].b
    fov_c = own_fov[i].c            
    show_video_image(timestamps[i],run_names[i])
    fov_plot_a = plot.plot([fov_a.x,fov_b.x], [fov_a.y,fov_b.y],'g-')
    fov_plot_b = plot.plot([fov_b.x,fov_c.x], [fov_b.y,fov_c.y],'g-')
    fov_plot_c = plot.plot([fov_c.x,fov_a.x], [fov_c.y,fov_a.y],'g-')
    old_pos_own = plot.plot(own_xy[i][0], own_xy[i][1],'ro')
    old_pos_other = plot.plot(other_xy[i][0], other_xy[i][1],'bo')
    #current_figure = plot.gcf()
    text = plot.text(-2, -3, timestamps[i], fontsize=10)
    plot.show()
    plot.pause(1/30.)
    old_pos_own.pop(0).remove()
    old_pos_other.pop(0).remove()
    fov_plot_a.pop(0).remove()
    fov_plot_b.pop(0).remove()
    fov_plot_c.pop(0).remove()
    text.remove()

    
    
def visualise(encounter_situations):
    
    plt.figure('top', figsize=(6, 6))
    
    own_xy = []
    other_xy = []
    own_fov = []
    timestamps = []
    run_names = []
    
    for own_carname in encounter_situations:
        for other_carname in encounter_situations[own_carname]:
            for entry in encounter_situations[own_carname][other_carname]:
                timestamps.append(entry['timestamp'])
                own_xy.append(entry['own_xy'])
                other_xy.append(entry['other_ts_xy'][1])
                own_fov.append(entry['fov'])
                run_names.append(entry['run_name'])

    
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.ion()
    fig = plt.gcf()
    fig.canvas.mpl_connect('button_press_event', onClick)
    for i in range(len(other_xy)):
        if not pause:
            draw_content(plt,i)
        else:
            while pause:
                draw_content(plt,i)


pause = False

if __name__ == '__main__':
    
    trajectories_path = sys.argv[1]
    trajectories_dict = pickle.load(open(trajectories_path, "rb"))
    
    
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-23-52_1.bag'
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-24-14_2.bag'
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-28-10_10.bag'
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-28-38_11.bag'
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-29-10_12.bag'
    
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black/bair_car_2017-04-28-17-28-19_10.bag'
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black/bair_car_2017-04-28-17-28-49_11.bag'
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black/bair_car_2017-04-28-17-29-18_12.bag'
    bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/new/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black/'
    
    #bagfile_path = '/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/'
    bagfiles =  [os.path.join(bagfile_path,file) for file in os.listdir(bagfile_path) if os.path.isfile(os.path.join(bagfile_path,file))] 
    
    # Timestamps from the last car are taken. In the future it would be good to check
    # if the timestamps differ for different cars
    
    # encounters_cars_timesteps_xy = defaultdict(lambda: defaultdict(dict))
    col_scanner = Collision_Scanner()
    encounter_situations = col_scanner.get_encounters(trajectories_dict)
    
    for bagfile in bagfiles:
        bag_handler = Image_Bagfile_Handler(bagfile)
        print "Loading bagfiles"
        own_xy = []
        other_xy = []
        own_fov = []
        timestamps = []
        run_names = []
        
    #     for own_carname in encounter_situations:
        own_carname = 'Mr_Black'
        for other_carname in encounter_situations[own_carname]:
            for entry in encounter_situations[own_carname][other_carname]:
                timestamps.append(entry['timestamp'])
                own_xy.append(entry['own_xy'])
                other_xy.append(entry['other_ts_xy'][1])
                own_fov.append(entry['fov'])
                run_names.append(entry['run_name'])
        try:
            for timestamp in timestamps:
                
                cv_image, timestamp, synced = bag_handler.get_image(timestamp)
                if not synced:
                    continue
                if(cv_image == None):
                    continue
                
                
                
                cv2.imshow('frame', cv_image)
                key = cv2.waitKey(1000 / 30) & 0xFF
                if key == ord('q'):
                    break
        except StopIteration:
            continue