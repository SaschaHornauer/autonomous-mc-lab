'''
Created on May 15, 2017

@author: Sascha Hornauer
'''
from Trajectory_From_Pkl import *
import os
import cPickle as pickle
from aruco_tools.mode import behavior
from vis import apply_rect_to_img
import sys
import rospy
import cv2
import matplotlib.pyplot as plt
from data_parsing.Image_Bagfile_Handler import Bagfile_Handler

def animate(resulting_trajectories):
    
    
    #bagfile_name = '/home/picard/2ndDisk/carData/rosbags/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-28-10_10.bag'
    #bagfile_name = '/home/picard/2ndDisk/carData/rosbags/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-28-38_11.bag'
    bagfile_name = '/home/picard/2ndDisk/carData/rosbags/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/bair_car_2017-04-28-17-29-10_12.bag'
    bagfile_handler = Bagfile_Handler(bagfile_name)
    paused_video = False
    
    bar_color = [0,0,255]
    if not paused_video:
        cv_image,bagfile_timestamp = bagfile_handler.get_image()
        
        
        #while bagfile_timestamp.to_sec() < (1493425694.71 + 5.):
        #    cv_image,bagfile_timestamp = bagfile_handler.get_image()
        #    continue

    
    #axis = plt.gca()
    #axis.set_xlim((-5, 5))
    #axis.set_ylim((-5, 5))       
    
    for cars,modes in resulting_trajectories:
                 
        blue_circle = resulting_trajectories[('Mr_Blue',behavior.circle)]
        print blue_circle.keys()
        traj_per_timestamp =  zip(blue_circle['timestamps'],blue_circle['trajectories'],blue_circle['motor_cmds'],blue_circle['pos'])
       
        for i in range(0,len(traj_per_timestamp)):
              
            if True:#try:        
                # we ignore synchronization for a second
                
                timestamp = traj_per_timestamp[i][0]
                trajectory = traj_per_timestamp[i][1]
                motor_cmd = traj_per_timestamp[i][2]
                pos = traj_per_timestamp[i][3]
                
                if timestamp +0.05< bagfile_timestamp.to_sec():
                    print "OUT OF SYNC " + str(timestamp -  bagfile_timestamp.to_sec())
                    continue
          
                while timestamp -0.05 > bagfile_timestamp.to_sec():
                    cv_image,bagfile_timestamp = bagfile_handler.get_image()
                    print "OUT OF SYNC " + str(timestamp -  bagfile_timestamp.to_sec())
                    continue
                
                
                steer = trajectory[1]
                motor = motor_cmd[1]
                
                
                cv_image,timestamp = bagfile_handler.get_image()
                
                
                #sys.exit(0)
                if not cv_image == None:
                     
                    apply_rect_to_img(cv_image,steer,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=True)
                
                    apply_rect_to_img(cv_image,motor,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=False)

                if cv_image == None:
                    continue
                cv2.imshow('frame',cv_image)
                key = cv2.waitKey(1000/30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused_video = not paused_video
                if key == ord('w'):
                    bagfile_handler.fast_forward()
            #except:
            #    pass
        


if __name__ == '__main__':
    
    

    
    home = os.path.expanduser("~")
    pickle_path = home + '/kzpy3/teg9/trajectories.pkl'
 
    t1 = 1493425694.71 + 5
    t2 = 1493425899.676476 - 100
     
    modes = [behavior.follow, behavior.circle]
     
    show_graphics = True
     
    resulting_trajectories = get_trajectories(pickle_path, t1, t2, modes, show_graphics)
    
    
    #pickle.dump( resulting_trajectories, open( "mydict.p", "wb" ) )
    resulting_trajectories = pickle.load( open( "mydict.p", "rb" ) )
    animate(resulting_trajectories)
    #animate(None)

