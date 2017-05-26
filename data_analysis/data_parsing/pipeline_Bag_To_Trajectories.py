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
from kzpy3.teg9.arena.markers_clockwise import markers_clockwise
import kzpy3.teg9.arena.get_trajectory_points as get_trajectory_points
CubicSpline = scipy.interpolate.CubicSpline
pause = plt.pause
 
array = np.array
plot = plt.plot

from Angle_Dict_Creator import get_angles_and_distance

change_stuff = False

'''
With this module a folder can be used as a command line parameter and from all
the run folders with bagfiles in its sub folders, one trajectories.pkl file will
be generated with all trajectories. 

The file is a copy and combination of several other files in teg9 and strives
towards being convenient to use and to improve clarity. Especially the last goal is
missed by a long shot. This file can evolve to be more simplistic and understandable,
though right now it is more important to be able to use it to find collisions
fast.
''' 


def process_markers_in_bagfiles(abs_bagfolder_name, angles_to_markers):
    
    bridge = CvBridge()
    
    for file in os.listdir(abs_bagfolder_name):
        if file.endswith(".bag"):
            print('Loading bagfile ' + file)
 
            bag = rosbag.Bag(os.path.join(abs_bagfolder_name, file))

            color_mode = "rgb8"
            topic_list = []
            topic_map = {}
            
            for camera_side in ['left', 'right']:
                topic_list.append('/bair_car/zed/' + camera_side + '/image_rect_color')
                topic_map['/bair_car/zed/' + camera_side + '/image_rect_color'] = camera_side
                
            for topic, message, timestamp  in bag.read_messages(topics=topic_list):
                
                camera_side = topic_map[topic]
                
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
        if meta_path_name in os.listdir(abs_bagfolder_name):
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

def process_run_data(run_name,marker_pkl_path,M):
    
    car_name = car_name_from_run_name(run_name)
    if car_name not in M:
        M[car_name] = {}
    if run_name not in M[car_name]:
        M[car_name][run_name] = run_name
        print('Loading ' + str(marker_pkl_path))
        marker_data_pkl = lo(marker_pkl_path)
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
            plt.close('all')
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
    
def markers_pkl_to_trajectories_pkl(root_folder_name):
    print "Process marker_data.pkl files to get cubic spline trajectories."
    
    
    run_foldernames = list_immediate_directories(root_folder_name)
    
    
    for run_foldername in run_foldernames:
        M = {}
        
        if os.path.isfile(os.path.join(run_foldername,'trajectory.pkl')):
            print(str(root_folder_name) + ' already processed.')
            return
        
        
        marker_data_file = search_for_file(run_foldername,'marker_data.pkl')
        if len(marker_data_file) > 1:
            print "Warning: More than one marker_data.pkl file found in run folder . Possibly wrong root folder selected. Select a folder where the immediate sub-folders are the folders of each individual car, which contain the bag files."
            raise ValueError("Wrong path given")
        else:
            marker_data_file = marker_data_file[0]
        
        run_name = os.path.split(run_foldername)[1]
            
#        try:
        process_run_data(run_name,marker_data_file,M)
        car_name = car_name_from_run_name(run_name)
        cprint.warn(M[car_name][run_name].keys())
        
#        except Exception as e:
#         print("********** Exception ***********************")
#         print(e.message, e.args)
#         cprint(run_name + "not processed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",'red')

        pickle.dump(M[car_name][run_name], open(os.path.join(run_foldername,'trajectory.pkl'),'wb'),-1)
        
if __name__ == '__main__':
    
    global markers_xy_dic

    marker_angles_dic = {}
    marker_angles = 2*np.pi*np.arange(len(markers_clockwise))/(1.0*len(markers_clockwise))
    marker_xys = []
    for i in range(len(markers_clockwise)):
        a = marker_angles[i]
        marker_angles_dic[markers_clockwise[i]] = a
        x = 4*107/100.*np.sin(a)
        y = 4*107/100.*np.cos(a)
        marker_xys.append([x,y])
    markers_xy_dic = {}
    assert(len(markers_clockwise) == len(marker_xys))
    for i in range(len(markers_clockwise)):
        m = markers_clockwise[i]
        xy = marker_xys[i]
        markers_xy_dic[m] = xy

    root_folder_name = sys.argv[1]
    #start = timer()
    #process_run_folder(root_folder_name, 'meta')
    #end = timer()
    #print end - start
    markers_pkl_to_trajectories_pkl(root_folder_name)
    
    print ("""
    load trajectories
    """)
    print ("Process trajectory.pkl files." )
    
    run_names = list_immediate_directories(root_folder_name)
    trajectory_dict = {}
 
    for run_name in run_names:
        pkl_file_name = search_for_file(root_folder_name, 'trajectory.pkl')[0]
        car_name = get_trajectory_points.car_name_from_run_name(run_name)
        if car_name not in trajectory_dict:
            trajectory_dict[car_name] = {}
        trajectory_dict[car_name][run_name] = pickle.load(open(pkl_file_name,'rb'))
        print('loaded '+car_name+' '+run_name)
 
         
    print("""
    choose a run
    assert less than two hours long
    find all other runs that are overlapping in time
    sample all cubic splines with timestamps of given run.
    save all timestamp synched splines in given run's meta folder
    """)
 
    heights = {'Mr_Yellow':1, 'Mr_Silver':2, 'Mr_Blue':3, 'Mr_Orange':4, 'Mr_Black':5}
    Origin = 300
    Mult = 50
    dt = 1/30.0
 
    for ref_run_name in run_names:
        if True:#try:
            for side in ['left']:
                plt.clf()
                car_name = get_trajectory_points.car_name_from_run_name(ref_run_name)
                R = trajectory_dict[car_name][ref_run_name][side]['raw_time_stamps']
                R0,Rn = R[0],R[-1]
                trajectory_dict[car_name][ref_run_name]['self_trajectory'] = get_trajectory_points.get_xp_pts(trajectory_dict,ref_run_name,R,Mult,Origin,dt)
                trajectory_dict[car_name][ref_run_name]['other_trajectories'] = []
                ref_car_name = car_name
                plot([R[0],R[-1]],[heights[car_name],heights[car_name]],marker='.',linestyle='--',color='r')
                plt.title(ref_run_name)
            cases = []
            for car_name in trajectory_dict.keys():
                for run_name in trajectory_dict[car_name]:
                    if run_name != ref_run_name:
                        for side in ['left']:
                            T = trajectory_dict[car_name][run_name][side]['time_stamps']
                            t0,tn = T[0],T[-1]
                            case = False
                            if (R0<t0 and Rn>t0):
                                case = 1
                            elif (R0<tn and Rn>tn):
                                case = 2
                            elif (R0>t0 and Rn<tn):
                                case = 3
                            if case:
                                cases.append(case)
                                plot([T[0],T[-1]],[heights[car_name],heights[car_name]],marker='.',linestyle='--',color='b')
                                other_trajectories_modified_timestamps = []
                                for r in R:
                                    if r >= t0 and r <= Rn:
                                        other_trajectories_modified_timestamps.append(r)
                                #traj = get_trajectory_points.get_xp_pts(M,run_name,other_trajectories_modified_timestamps,Mult,Origin,dt)
                                trajectory_dict[ref_car_name][ref_run_name]['other_trajectories'].append(run_name) #traj)
                    else:
                        print('found ref run')
            print cases
            plt.ylim(0.5,5.5)
            pause(0.001)
 
    all_trajectories_dict = {}
    for car_name in trajectory_dict.keys():
        all_trajectories_dict[car_name] = {}
        for run_name in trajectory_dict[car_name]:
            all_trajectories_dict[car_name][run_name] = {}
            all_trajectories_dict[car_name][run_name]['self_trajectory'] = trajectory_dict[car_name][run_name]['self_trajectory']
            all_trajectories_dict[car_name][run_name]['other_trajectories'] = trajectory_dict[car_name][run_name]['other_trajectories']
    pickle.dump(all_trajectories_dict,open(os.path.join(root_folder_name,('trajectories.pkl')),'wb'),-1)
 
    

