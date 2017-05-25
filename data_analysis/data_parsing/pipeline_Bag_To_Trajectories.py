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
from cprint import *
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate
CubicSpline = scipy.interpolate.CubicSpline
pause = plt.pause
 
array = np.array
plot = plt.plot

from Angle_Dict_Creator import get_angles_and_distance

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
        

def process_run_folder(run_folder_path, meta_path_name):
    print run_folder_path
    
    
    for bag_folder_name in os.listdir(run_folder_path):
        
        abs_bagfolder_name = os.path.join(run_folder_path, bag_folder_name)
        create_marker_data(abs_bagfolder_name,meta_path_name)
        
     
    
def check_active_threads(threads):
    active_threads = 0
    for thread in threads:
        if thread.is_alive():
            active_threads += 1
    return active_threads
        

def init_car_path(marker_data_pkl,side):
    A = {}
    A['raw_marker_data'] = marker_data_pkl[side]
    A['x_avg'] = []
    A['y_avg'] = []
    A['time_stamps'] = []
    A['median_distance_to_markers'] = []
    A['raw_time_stamps'] = sorted(A['raw_marker_data'].keys())
    return A

markers_xy_dic = {}
marker_angles_dic = {}

def get_camera_position(angles_to_center,angles_surfaces,distances_marker):
    marker_ids = angles_to_center.keys()
    x_avg = 0.0
    y_avg = 0.0
    d_sum = 0.0
    xs = []
    ys = []
    ds = []
    for m in marker_ids:
        if m in [190]: # This one gives false positives on ground.
            continue
        if m in markers_xy_dic:
            xy = markers_xy_dic[m]
            angle1 = angles_to_center[m]
            distance1 = distances_marker[m]
            distance2 = 4*107/100.
            angle2 = (np.pi+marker_angles_dic[m]) - (np.pi/2.0-angles_surfaces[m])
            xd = distance1 * np.sin(angle2)
            yd = distance1 * np.cos(angle2)
            #print (dp(np.degrees(marker_angles_dic[m]+np.pi/2.0-angles_surfaces[m]+angles_to_center[m]),2))#,dp(np.degrees(marker_angles_dic[m]),2),dp(np.degrees(angles_surfaces[m]),2),dp(np.degrees(angles_to_center[m],2)))
            if distance1 < 2*distance2 and distance1 > 0.05:
            #if distance1 < 2 and distance1 > 0.05:
                xs.append(xd+xy[0])
                ys.append(yd+xy[1])
                ds.append(distance1)
    d = 0
    for i in range(len(xs)):
        d += 1/ds[i]
        x_avg += d*xs[i]
        y_avg += d*ys[i]
        d_sum += d
    if len(ds) > 2:
        median_distance_to_markers = np.median(array(ds))
    elif len(ds) > 0:
        median_distance_to_markers = array(ds).min()
    else:
        median_distance_to_markers = None
    if d_sum == 0:
        return None,None,None,None
    x_avg /= d_sum
    y_avg /= d_sum
    return marker_ids,x_avg,y_avg,median_distance_to_markers




try:
    import numbers
    def is_number(n):
        return isinstance(n,numbers.Number)
except:
    print("Don't have numbers module")

def get_cubic_spline(time_points,data,n=100):
    n = 10
    D = []
    T = []
    for i in range(n/2,len(time_points),n):
        D.append(data[i])#-n/2:i+n/2].mean())
        T.append(time_points[i])#-n/2:i+n/2].mean())
    D,T = array(D),array(T)
    cs = CubicSpline(T,D)
    plot(time_points,data,'o')
    plot(T,D,'o', label='smoothed data')
    plot(time_points,cs(time_points),label="S")
    plt.legend(loc='lower left', ncol=2)
    pause(0.001)
    return cs




def mean_of_upper_range(data,min_proportion,max_proportion):
    return array(sorted(data))[int(len(data)*min_proportion):int(len(data)*max_proportion)].mean()


def mean_exclude_outliers(data,n,min_proportion,max_proportion):
    """
    e.g.,

    L=lo('/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017/meta/direct_rewrite_test_11May17_16h16m49s_Mr_Blue/left_image_bound_to_data.pkl' )
    k,d = get_key_sorted_elements_of_dic(L,'encoder')
    d2=mean_of_upper_range_apply_to_list(d,30,0.33,0.66)
    CA();plot(k,d);plot(k,d2)
    
    """
    n2 = int(n/2)
    rdata = []
    len_data = len(data)
    for i in range(len_data):
        if i < n2:
            rdata.append(mean_of_upper_range(data[i:i-n2+n],min_proportion,max_proportion))
        elif i < len_data + n2:
            rdata.append(mean_of_upper_range(data[i-n2:i-n2+n],min_proportion,max_proportion))
        else:
            rdata.append(mean_of_upper_range(data[i-n2:i],min_proportion,max_proportion))
    return rdata


        
def car_name_from_run_name(run_name):
    a = run_name.split('Mr_')
    car_name = 'Mr_'+a[-1]
    return car_name

def opj(*args):
    if len(args) == 0:
        args = ['']
    str_args = []
    for a in args:
        str_args.append(str(a))
    return os.path.join(*str_args)

def load_obj(name ):
    if name.endswith('.pkl'):
        name = name[:-len('.pkl')]
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

lo = load_obj

CA = plt.close('all')

def process_run_data(run_name,bag_folders_dst_meta_path,M):
    
    car_name = car_name_from_run_name(run_name)
    if car_name not in M:
        M[car_name] = {}
    if run_name not in M[car_name]:
        M[car_name][run_name] = run_name
        mdp_path = opj(bag_folders_dst_meta_path,run_name,'marker_data.pkl')
        print('Loading ' + mdp_path)
        marker_data_pkl = lo(mdp_path)
        print('processing marker data.')
        M[car_name][run_name] = {}
        for side in ['left','right']:
            M[car_name][run_name][side] = init_car_path(marker_data_pkl,side)
            for t in M[car_name][run_name][side]['raw_time_stamps']:
                angles_to_center = M[car_name][run_name][side]['raw_marker_data'][t]['angles_to_center']
                angles_surfaces = M[car_name][run_name][side]['raw_marker_data'][t]['angles_surfaces']
                distances_marker = M[car_name][run_name][side]['raw_marker_data'][t]['distances_marker']
                _,x_avg,y_avg,median_distance_to_markers = get_camera_position(angles_to_center,angles_surfaces,distances_marker)
                if is_number(x_avg) and is_number(y_avg) and is_number(median_distance_to_markers):
                    M[car_name][run_name][side]['time_stamps'].append(t)
                    M[car_name][run_name][side]['x_avg'].append(x_avg)
                    M[car_name][run_name][side]['y_avg'].append(y_avg)
                    M[car_name][run_name][side]['median_distance_to_markers'].append(median_distance_to_markers)
            M[car_name][run_name][side]['x_smooth'] = mean_exclude_outliers(M[car_name][run_name][side]['x_avg'],60,0.33,0.66)
            M[car_name][run_name][side]['y_smooth'] = mean_exclude_outliers(M[car_name][run_name][side]['y_avg'],60,0.33,0.66)
            CA()
            M[car_name][run_name][side]['cs_x'] = get_cubic_spline(M[car_name][run_name][side]['time_stamps'],M[car_name][run_name][side]['x_smooth'])
            M[car_name][run_name][side]['cs_y'] = get_cubic_spline(M[car_name][run_name][side]['time_stamps'],M[car_name][run_name][side]['y_smooth'])
            del M[car_name][run_name][side]['raw_marker_data']

def search_for_file(root_path,filename):
    file_list = [os.path.join(root, name)
             for root, dirs, files in os.walk(root_path)
             for name in files
             if name.endswith(filename)]
    return file_list

def list_immediate_directories(root_path):
    return [os.path.join(root_path,o) for o in os.listdir(root_path) if os.path.isdir(os.path.join(root_path,o))]
    
def marker_pkl_to_trajectories_pkl(root_folder_name):
    print "Process marker_data.pkl files to get cubic spline trajectories."
    
    bag_folders_meta_path = root_folder_name
    
    run_foldernames = list_immediate_directories(root_folder_name)
    
    M = {}
    for run_foldername in run_foldernames:
        
        marker_data_file = search_for_file(run_foldername,'marker_data.pkl')
        if len(marker_data_file) > 1:
            print "Warning: More than one marker_data.pkl file found in run folder . Possibly wrong root folder selected. Select a folder where the immediate sub-folders are the folders of each individual car, which contain the bag files."
            raise ValueError("Wrong path given")
        else:
            run_name = os.path.split(marker_data_file[0])[0]
        if os.path.isfile(os.path.join(os.path.split(marker_data_file)[0],'trajectory.pkl')):
            print(str(marker_data_file) + ' already processed.')
            continue
        
        try:
            process_run_data(run_name,root_folder_name,M)
            car_name = car_name_from_run_name(run_name)
            cprint(M[car_name][run_name].keys(),'yellow')
            so(opj(bag_folders_meta_path,aruco_run,'trajectory.pkl'),M[car_name][aruco_run])
        #unix('rm '+opj(bag_folders_meta_path,aruco_run,'cubic_splines.pkl'))
        except Exception as e:
            print("********** Exception ***********************")
            print(e.message, e.args)
            cprint(d2s(aruco_run,"not processed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"),'red','on_yellow')

        
        
if __name__ == '__main__':
    
    
    # Get bag files of a specific run
    
    # # Works on a run folder, which is a folder with all the run folders of a specific day
    # # of experiments 
    
    root_folder_name = sys.argv[1]
    #start = timer()
    #process_run_folder(run_folder_name, 'meta')
    #end = timer()
    #print end - start
    marker_pkl_to_trajectories_pkl(root_folder_name)
    
    # Go over all the files of the run and generate common trajectories
    
    # Supply a good data format for the trajectories
    
    
    

