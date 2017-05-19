'''
Created on May 18, 2017

@author: Sascha Hornauer
'''
import sys
import os
import time
import rosbag
from timeit import default_timer as timer
import cPickle as pickle
import threading
from cv_bridge import CvBridge, CvBridgeError

from Angle_Dict_Creator import get_angles_and_distance
from paramiko.transport import _active_threads

change_stuff = False


def process_markers_in_bagfiles(abs_bagfolder_name, angles_to_markers):
    
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



def create_marker_data(abs_bagfolder_name, meta_path_name):

    # Check if a meta directory exists and create it otherwise
    if os.path.isdir(abs_bagfolder_name):
        if 'meta' in os.listdir(abs_bagfolder_name):
            print "Meta directory exists"
        else:
            os.mkdir(os.path.join(abs_bagfolder_name, meta_path_name))
            
    # Check if marker pkl file already exists and return if it does
    if os.path.isfile(os.path.join(abs_bagfolder_name,meta_path_name, 'marker_data.pkl')):
        print(os.path.join(abs_bagfolder_name, 'marker_data.pkl') + ' exists, doing nothing.')
        return 

    # Create a pkl file with marker information
    angles_to_markers = {}
    angles_to_markers['bag_folder_path'] = abs_bagfolder_name
    for camera_side in ['left', 'right']:
        angles_to_markers[camera_side] = {}

    
    process_markers_in_bagfiles(abs_bagfolder_name, angles_to_markers)
    
    pickle.dump(angles_to_markers, open(os.path.join(abs_bagfolder_name, meta_path_name, 'marker_data.pkl'), "wb"))
        

def process_run_folder(run_folder_path, meta_path_name, max_threads = 1):
    print run_folder_path
    
    threads = []
    
    
    for bag_folder_name in os.listdir(run_folder_path):
        
        abs_bagfolder_name = os.path.join(run_folder_path, bag_folder_name)
        create_marker_data(abs_bagfolder_name,meta_path_name)
        
     
    
    # Since parsing a bag folder takes a lot of time, bag folders will be parsed simultanously
#     for bag_folder_name in os.listdir(run_folder_path):
#         
#         abs_bagfolder_name = os.path.join(run_folder_path, bag_folder_name)
#         thread = threading.Thread(target=create_marker_data,args=(abs_bagfolder_name,meta_path_name))
#         threads.append(thread)
#         
#         
    # Because all threads should have approximately the same length, we wait for a random thread to
    # finish and then start the next
        
#     active_threads = check_active_threads(threads)
#     thread_counter = 0
#     while thread_counter < len(threads):
#         
#         if active_threads >= max_threads:
#             
#             if threads[thread_counter].is_alive(): 
#                 threads[thread_counter].join()
#         
#         while active_threads < max_threads:
#             threads[thread_counter].start()
#             thread_counter += 1
#             active_threads = check_active_threads(threads)
#         
#     print "Parsing of all folders finished"
    
    
def check_active_threads(threads):
    active_threads = 0
    for thread in threads:
        if thread.is_alive():
            active_threads += 1
    return active_threads
        
if __name__ == '__main__':
    
    
    # Get bag files of a specific run
    
    # # Works on a run folder, which is a folder with all the run folders of a specific day
    # # of experiments 
    
    max_threads = 2
    
    run_folder_name = sys.argv[1]
    start = timer()
    process_run_folder(run_folder_name, 'meta', max_threads)
    end = timer()
    print end - start
    
    # Go over all the files of the run and generate common trajectories
    
    # Supply a good data format for the trajectories
    
    
    
    

