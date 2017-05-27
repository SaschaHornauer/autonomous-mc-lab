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




def get_close_encounters(all_xys_with_timestamps,list_of_timestamps):
    
    encounter_timesteps_per_car = {}
    triangle_list = []
    encounter_own_other_xy= {}
    
    for own_carname in all_xys_with_timestamps:
        
        own_xys_and_timestamps = all_xys_with_timestamps[own_carname]
        
        own_xys = own_xys_and_timestamps['trajectory']
                
        for other_carname in all_xys_with_timestamps:
            if other_carname != own_carname:
                
                other_xys_and_timestamps = all_xys_with_timestamps[other_carname]
        
                other_xys = other_xys_and_timestamps['trajectory']
                
                encounter_own_other_xy[(own_carname,other_carname)], encounter_timesteps_per_car[(own_carname,other_carname)], triangle_fov = get_close_encounters_with_own(own_xys, other_xys, list_of_timestamps) 
                
                triangle_list.append(triangle_fov)
                print "Checking " + str((own_carname,other_carname))
        
     
    return encounter_own_other_xy, encounter_timesteps_per_car, triangle_list

def get_close_encounters_with_own(own_xys, other_xys, list_of_timestamps):
    '''
    own and other xy coordinates are expected in the following format:
    
    own_data = 'pos_xy':[2]
    other_data = 'other_xy':[2] 
    list_of_timestamps = list of list_of_timestamps 
    '''
    # Get triangle in front of the vehicle ( THIS IS A SIMPLIFICATION ) 
        
    encounter_list = []
    triangle_points = {}
    encounter_xys = {}

    smooth_over_list_of_timestamps = 3
    own_xys = np.transpose(own_xys)
    other_xys = np.transpose(other_xys)
    
    for t in range(smooth_over_list_of_timestamps,len(list_of_timestamps)):
        
        current_own_xys = own_xys[t-smooth_over_list_of_timestamps:t]
        
        triangle_fov = get_fov_one_camera(current_own_xys,get_heading(current_own_xys),66.,1.5)
                
        point_xy = Point(other_xys[t][0], other_xys[t][1])
            
        if(triangle_fov.isInside(point_xy)):
            encounter_list.append(list_of_timestamps[t])
            encounter_xys[triangle_fov] = point_xy
            triangle_points[str(list_of_timestamps[t])]=triangle_fov
        
    return encounter_xys, encounter_list, triangle_points

def get_close_encounters_in_list(own_xys,other_xys,start_timestep, end_timestep):
    # Get triangle in front of the vehicle ( THIS IS A SIMPLIFICATION ) 
        
    encounter_timestep = []
    other_positions = {}
    triangle_points = {}
    smooth_over_list_of_timestamps = 3
    
    
    #other_xys = other_xys[start_timestep:end_timestep]
   
    for t in range(start_timestep+smooth_over_list_of_timestamps,end_timestep):
        
        current_own_xys = own_xys[t-smooth_over_list_of_timestamps:t]
        
        # distance chosen is 1.5 m, field of view of one of the stereo cameras is 66
        triangle_fov = get_fov_one_camera(current_own_xys,get_heading(current_own_xys),66.,10.)
        
        closest_xy = None

        for other_car_id in range(0,len(other_xys)):
            # For all other car positions at that point in time t
            # This has to be a Point of a certain datastructure to make easy computation
            # of the triangle approach possible
            other_xy = Point(other_xys[other_car_id][t][0], other_xys[other_car_id][t][1])
            
            # Get the closest of that positions
            if closest_xy == None:
                closest_xy = other_xy
            else:
                new_dist = np.hypot(other_xy.x-np.mean([i[0] for i in current_own_xys]),other_xy.y-np.mean([i[1] for i in current_own_xys]))
                old_dist = np.hypot(closest_xy.x-np.mean([i[0] for i in current_own_xys]),closest_xy.y-np.mean([i[1] for i in current_own_xys]))
                if new_dist < old_dist:
                    closest_xy = other_xy
        # If that closest other xy point lies in the FOV add it to the list
        if(triangle_fov.isInside(closest_xy)):
            encounter_timestep.append(t)
            other_positions[t] = closest_xy
            triangle_points[str(t)]=triangle_fov
            
    return encounter_timestep, other_positions, triangle_points

def get_fov_one_camera(xy, heading, fov_angle, distance):
    '''
    returns the field of view as triangle, based on a sequence of
    coordinates. It will always regard the last coordinates in that list
    '''    
    limit_left = heading - np.deg2rad(fov_angle / 2.0)
    limit_right = heading + np.deg2rad(fov_angle / 2.0)
    
    a = xy[len(xy)-1]  # Point a is the latest known position
    b = cv2.polarToCart(distance, limit_left)  # Point b is distance in front and half of fov to the left
    c = cv2.polarToCart(distance, limit_right)  # Point c is distance in front and half of fov to the right
    
    # Needed extraction of the values from the polarToCart method
    # and adding of a as the origin 
    b = b[0][0][0]+a[0],b[1][0][0]+a[1]
    c = c[0][0][0]+a[0],c[1][0][0]+a[1]
    
    return Triangle(to_point(a), to_point(b), to_point(c))

if __name__ == '__main__':
    
    trajectories_path = sys.argv[1]
    trajectories_dict = pickle.load(open(trajectories_path, "rb"))
    
   
    #print trajectories_dict['Mr_Black']['/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black']['self_trajectory']['left']['x']


    show_only_encounters = False
    plt.figure('top',figsize=(6,6))
    xy_positions = defaultdict(lambda: defaultdict(dict))
    
    # Timestamps from the last car are taken. In the future it would be good to check
    # if the timestamps differ for different cars
    timestamps = None
    xy_by_timestamp = {}

    for carname in trajectories_dict:
        for run_name in trajectories_dict[carname]:
        
            # Right now there is only one runname and this loop is used to get its name
            
            left_x = trajectories_dict[carname][run_name]['self_trajectory']['left']['x']
            left_y = trajectories_dict[carname][run_name]['self_trajectory']['left']['y']
            
            right_x = trajectories_dict[carname][run_name]['self_trajectory']['right']['x']
            right_y = trajectories_dict[carname][run_name]['self_trajectory']['right']['y']
            
            mid_xy = (((right_x+left_x)/2.),((left_y + right_y)/2.))
            timestamps = trajectories_dict[carname][run_name]['self_trajectory']['ts']
            
        xy_positions[carname] = {'trajectory' : mid_xy,'timestamps' : timestamps}
            
    start = timer()
    encounter_xys, encounters, triangle_list = get_close_encounters(xy_positions,timestamps)
    print "Finished in " + str(timer()-start)
    
    
    
    
    
    #for own_carname in trajectories_dict:
    own_carname = 'Mr_Black'
    #for other_carname in trajectories_dict:
    other_carname = 'Mr_Silver'   
    xy_of_encounter = []
    for encounter in encounter_xys:
    
        for encounter_point in encounter_xys[encounter].iteritems():
            xy_of_encounter.append((encounter_point[1].x,encounter_point[1].y))

            
            
    plot_xys = np.transpose(xy_of_encounter)
    plt.plot(plot_xys[0],plot_xys[1],'.',color='b')
            
#         # Draw triangles 
#         for triangle_key in triangle_fov:
#           
#             triangle = triangle_fov[triangle_key]
#              
#             a = triangle.a
#             b = triangle.b
#             c = triangle.c
#               
#             plt.plot((a.x,b.x,c.x,a.x), (a.y,b.y,c.y,a.y), 'r')
#               
#             plt.axes().set_aspect('equal', 'datalim')
             
    plt.show()
    

 

