'''
Created on May 8, 2017

@author: picard
'''
from trajectory_tools import *
import cPickle as pickle
import os

def get_close_encounters(own_xys, other_xys, timesteps):
    '''
    own and other xy coordinates are expected in the following format:
    
    own_data = 'pos_xy':[2]
    other_data = 'other_xy':[2] 
    timesteps = list of timesteps 
    '''
    # Get triangle in front of the vehicle ( THIS IS A SIMPLIFICATION ) 
    
    triangle_fov = get_fov(np.transpose(own_xys))
    
    encounter_list = []
    timestep_counter = 0
    
    for other_xy in np.transpose(other_xys):
        point_xy = Point(other_xy[0], other_xy[1])
        if(triangle_fov.isInside(point_xy)):
            encounter_list.append(timesteps[timestep_counter])
        timestep_counter = timestep_counter + 1
    
    
    return encounter_list


def get_fov(xys):
    '''
    returns the field of view as triangle, based on a number of coordinates
    '''
    
    estimate_fov = 110.  # degree    
    estimate_distance = 1  # meter
    
    own_heading = get_heading(xys)
    limit_left = own_heading - np.deg2rad(estimate_fov / 2.0)
    limit_right = own_heading + np.deg2rad(estimate_fov / 2.0)
    
    a = xys[len(xys)-1]  # Point a is the latest known position
    b = cv2.polarToCart(estimate_distance, limit_left)  # Point b is estimate_distance in front and half of fov to the left
    c = cv2.polarToCart(estimate_distance, limit_right)  # Point b is estimate_distance in front and half of fov to the left
    
    # Needed extraction of the values from the polarToCart method
    b = b[0][0][0],b[1][0][0]
    c = c[0][0][0],c[1][0][0]
    
    return Triangle(to_point(a), to_point(b), to_point(c))

if __name__ == '__main__':
    home = os.path.expanduser("~")
    pickle_file = pickle.load(open(home + '/kzpy3/teg9/trajectories.pkl', "rb"))
    
    t1 = 1493425694.71+5
    t2 = 1493425899.676476 - 100
    T = np.arange(t1,t2,1/30.)
    
    #for car in ['Mr_Black','Mr_Blue']:
    #for side in ['left','right']:
    # As a first try take only the left camera information
    xy_black = [pickle_file['Mr_Black']['left'][0](T),pickle_file['Mr_Black']['left'][1](T)]
    xy_blue  = [pickle_file['Mr_Blue']['left'][0](T),pickle_file['Mr_Blue']['left'][1](T)]
    
    encounters = get_close_encounters(xy_black, xy_blue, T)
    
    print(encounters)
