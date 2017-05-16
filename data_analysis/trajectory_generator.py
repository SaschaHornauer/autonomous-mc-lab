'''
Created on May 15, 2017

@author: Sascha Hornauer
'''
from Trajectory_From_Pkl import *
import os
import cPickle as pickle
from aruco_tools.mode import behavior


if __name__ == '__main__':

    home = os.path.expanduser("~")
    pickle_path = home + '/kzpy3/teg9/trajectories.pkl'

    t1 = 1493425694.71 + 5
    t2 = 1493425899.676476 - 100
    
    #modes = [behavior.follow, behavior.circle]
    modes = [behavior.follow]
    
    show_graphics = True
    
    resulting_trajectories = get_trajectories(pickle_path, t1, t2, modes, show_graphics)
    
    print resulting_trajectories
