'''
Created on May 8, 2017

@author: picard
'''
from trajectory_tools import *
import cPickle as pickle
import matplotlib.pyplot as plt
import os

def get_close_encounters(own_xys, other_xys, timesteps):
    '''
    own and other xy coordinates are expected in the following format:
    
    own_data = 'pos_xy':[2]
    other_data = 'other_xy':[2] 
    timesteps = list of timesteps 
    '''
    # Get triangle in front of the vehicle ( THIS IS A SIMPLIFICATION ) 
        
    encounter_list = []
    triangle_points = {}

    smooth_over_timesteps = 3
    own_xys = np.transpose(own_xys)
    other_xys = np.transpose(other_xys)
    
    for t in range(smooth_over_timesteps,len(timesteps)):
        
        current_own_xys = own_xys[t-smooth_over_timesteps:t]
        
        triangle_fov = get_fov_one_camera(current_own_xys,get_heading(current_own_xys),66.,1.5)
                
        point_xy = Point(other_xys[t][0], other_xys[t][1])
            
        if(triangle_fov.isInside(point_xy)):
            encounter_list.append(timesteps[t])
            triangle_points[str(timesteps[t])]=triangle_fov
            print(t)
        
    return encounter_list, triangle_points


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
    home = os.path.expanduser("~")
    pickle_file = pickle.load(open(home + '/kzpy3/teg9/trajectories.pkl', "rb"))
    #28th april 
    t1 = 1493425694.71+5
    t2 = 1493425899.676476 - 100
    timestamps = np.arange(t1,t2,1/30.)
    
    show_only_encounters = False
    
    xy_black = [pickle_file['Mr_Black']['left'][0](timestamps),pickle_file['Mr_Black']['left'][1](timestamps)]
    xy_blue  = [pickle_file['Mr_Blue']['left'][0](timestamps),pickle_file['Mr_Blue']['left'][1](timestamps)]
    
    encounters, triangle_fov = get_close_encounters(xy_black, xy_blue, timestamps)
    
    plt.figure('top',figsize=(6,6)) 

    if show_only_encounters:
        timestamps = encounters
    
    print ("Encounters at " + str(timestamps))
    color = 'black'
    for car in ['Mr_Black','Mr_Blue']:
        for side in ['left','right']:
            x = pickle_file[car][side][0](timestamps)
            y = pickle_file[car][side][1](timestamps)
            plt.plot(x,y,'.',color=color)
        color = 'blue'

    # Draw triangles 
    for triangle_key in triangle_fov:
         
        triangle = triangle_fov[triangle_key]
        
        a = triangle.a
        b = triangle.b
        c = triangle.c
         
        plt.plot((a.x,b.x,c.x,a.x), (a.y,b.y,c.y,a.y), 'r')
         
        plt.axes().set_aspect('equal', 'datalim')
        
    plt.show()

