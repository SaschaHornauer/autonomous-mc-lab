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
    timestep_counter = 0
    
    smooth_over_timesteps = 3
    own_xys = np.transpose(own_xys)
    
    for i in range(smooth_over_timesteps,len(own_xys)):
        
        current_own_xys = own_xys[i-smooth_over_timesteps:i]
        triangle_fov = get_fov(current_own_xys,get_heading(current_own_xys))
        
        for other_xy in np.transpose(other_xys):
            point_xy = Point(other_xy[0], other_xy[1])
            
            if(triangle_fov.isInside(point_xy)):
                encounter_list.append(timesteps[timestep_counter])
                triangle_points[str(timesteps[timestep_counter])]=triangle_fov
            timestep_counter = timestep_counter + 1
        
    return encounter_list, triangle_points


def get_fov(xy, heading):
    '''
    returns the field of view as triangle, based on a sequence of
    coordinates. It will always regard the last coordinates in that list
    '''
    
    estimate_fov = 110.  # degree    
    estimate_distance = 1  # meter
    
    limit_left = heading - np.deg2rad(estimate_fov / 2.0)
    limit_right = heading + np.deg2rad(estimate_fov / 2.0)
    
    a = xy[len(xy)-1]  # Point a is the latest known position
    b = cv2.polarToCart(estimate_distance, limit_left)  # Point b is estimate_distance in front and half of fov to the left
    c = cv2.polarToCart(estimate_distance, limit_right)  # Point c is estimate_distance in front and half of fov to the right
    
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
    T = np.arange(t1,t2,1/30.)
    
    #for car in ['Mr_Black','Mr_Blue']:
    #for side in ['left','right']:
    # As a first try take only the left camera information
    xy_black = [pickle_file['Mr_Black']['left'][0](T),pickle_file['Mr_Black']['left'][1](T)]
    xy_blue  = [pickle_file['Mr_Blue']['left'][0](T),pickle_file['Mr_Blue']['left'][1](T)]
    
    encounters, triangle_fov = get_close_encounters(xy_black, xy_blue, T)
    
    #x_black_at_crash = pickle_file['Mr_Black']['left'][0](encounters)
    #y_black_at_crash = pickle_file['Mr_Black']['left'][1](encounters)
    #x_blue_at_crash  = pickle_file['Mr_Blue']['left'][0](encounters)
    #y_blue_at_crash = pickle_file['Mr_Blue']['left'][1](encounters)
    plt.figure('top',figsize=(6,6)) 
    #print(T)
    T = encounters
    print(T)
    color = 'black'
    for car in ['Mr_Black','Mr_Blue']:
        for side in ['left','right']:
            x = pickle_file[car][side][0](T)
            y = pickle_file[car][side][1](T)
            plt.plot(x,y,'.',color=color)
        color = 'blue'
    #plt.scatter(x_black_at_crash,y_black_at_crash,color='magenta')
    #plt.scatter(x_blue_at_crash,y_blue_at_crash,color='magenta')
    
    for triangle_key in triangle_fov:
         
        triangle = triangle_fov[triangle_key]
        
        a = triangle.a
        b = triangle.b
        c = triangle.c
         
        plt.plot((a.x,b.x,c.x,a.x), (a.y,b.y,c.y,a.y), 'r')
         
 
        plt.axes().set_aspect('equal', 'datalim')
        
    plt.show()

