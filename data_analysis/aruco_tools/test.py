'''
Created on Apr 12, 2017

@author: picard
'''
import cPickle as pickle
import sys
import cv2
#import aruco_code


def pretty(d, indent=0):
    
    for key, value in d.iteritems():
        print '\t' * indent + str(key)
        if isinstance(value, dict):
            pretty(value, indent+1)
        #else:
            #print '\t' * (indent+1) + str(value)


file_array = ['/home/picard/2ndDisk/carData/rosbags/run_28apr/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/meta/marker_data.pkl']
print '--'
for i in range(0,len(file_array)):
    print("-----------------")
    print("File: " + file_array[i])
    file = open(file_array[i], "rb" )
    
    while True:
        try:
            mylist = pickle.load( file )
            pretty(mylist)
        except Exception as ex:
            print ex
            break




#cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
#while True:
    #ret, image = cap.read()
        #
    #cv2.imshow('frame',image)
    #key = cv2.waitKey(1000/30) & 0xFF
#    
    ###if key == ord('q'):
        #break
#    
    #aruco_steer,aruco_motor,aruco_only = aruco_code.do_aruco(image,55.0,60.0)
    
    
    