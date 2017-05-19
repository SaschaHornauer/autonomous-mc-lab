'''
Created on May 18, 2017

@author: Sascha Hornauer
'''
import sys
import os
import rosbag
from timeit import default_timer as timer
import cPickle as pickle
from cv_bridge import CvBridge, CvBridgeError

from Angle_Dict_Creator import get_angles_and_distance

change_stuff = False


def process_bagfiles(abs_bagfolder_name, angles_to_markers):
    
    bridge = CvBridge()
    
    for file in os.listdir(abs_bagfolder_name):
        if file.endswith(".bag"):
            print('Loading bagfile ' + file)
 
            bag = rosbag.Bag(os.path.join(abs_bagfolder_name, file))

            color_mode = "rgb8"
            topic_list = []
            for camera_side in ['left', 'right']:
                topic_list.append('/bair_car/zed/' + camera_side + '/image_rect_color')
            for topic, message, timestamp  in bag.read_messages(topics=topic_list):
                timestamp = round(timestamp.to_sec(), 3)
                angles_to_markers[camera_side][timestamp] = {}
                img = bridge.imgmsg_to_cv2(message, color_mode)
                angles_to_center, angles_surfaces, distances_marker, markers = get_angles_and_distance(img)
                angles_to_markers[camera_side][timestamp]['angles_to_center'] = angles_to_center
                angles_to_markers[camera_side][timestamp]['angles_surfaces'] = angles_surfaces
                angles_to_markers[camera_side][timestamp]['distances_marker'] = distances_marker
                angles_to_markers[camera_side][timestamp]['markers'] = markers



def process_run_folder(bag_folders_path_lst, meta_path_name, visualize=0):
    print bag_folders_path_lst
    for bag_folder_name in os.listdir(bag_folders_path_lst):
        abs_bagfolder_name = os.path.join(bag_folders_path_lst, bag_folder_name) 
        
        # Check if a meta directory exists and create it otherwise
        if os.path.isdir(os.path.join(bag_folders_path_lst, bag_folder_name)):
            if 'meta' in os.listdir(abs_bagfolder_name):
                print "Meta directory exists"
            else:
                os.mkdir(os.path.join(abs_bagfolder_name, meta_path_name))
                
        # Check if marker pkl file already exists and return if it does
        if os.path.isfile(os.path.join(abs_bagfolder_name, 'marker_data.pkl')):
            print(os.path.join(abs_bagfolder_name, 'marker_data.pkl') + ' exists, doing nothing.')
            continue

        # Create a pkl file with marker information
        angles_to_markers = {}
        angles_to_markers['bag_folder_path'] = abs_bagfolder_name
        for camera_side in ['left', 'right']:
            angles_to_markers[camera_side] = {}

        process_bagfiles(abs_bagfolder_name, angles_to_markers)
        
        pickle.dump(angles_to_markers, os.path.join(bag_folders_path_lst, meta_path_name, open('marker_data.pkl'), "wb"))
        
if __name__ == '__main__':
    
    
    # Get bag files of a specific run
    
    # # Works on a run folder, which is a folder with all the run folders of a specific day
    # # of experiments 
    
    run_folder_name = sys.argv[1]
    start = timer()
    process_run_folder(run_folder_name, 'meta')
    end = timer()
    print end-start
    
    # Go over all the files of the run and generate common trajectories
    
    # Supply a good data format for the trajectories
    
    
    
    
    
    pass

