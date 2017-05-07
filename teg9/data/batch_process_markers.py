
from kzpy3.vis import *
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge,CvBridgeError
import threading
import kzpy3.teg9.data.animate as animate
from kzpy3.data_analysis.Angle_Dict_Creator import get_angles_and_distance
from kzpy3.data_analysis.markers_clockwise import markers_clockwise

bridge = CvBridge()

image_topics = ['left_image','right_image']
camera_sides = ['left','right']





def multi_process_bag_folders(bag_folders_path_lst,meta_path,visualize=0):
    print bag_folders_path_lst
    for pl in bag_folders_path_lst:
        print pl
        bag_folders = sgg(opj(pl,'*'))
        try:
            for bf in bag_folders:
                print bf
                multi_preprocess_bagfiles(bf,meta_path,visualize)
        except Exception as e:
            print("********** Exception ***********************")
            print(e.message, e.args)



def multi_preprocess_bagfiles(bag_folder_path,meta_path,visualize=0):
    print bag_folder_path
    bag_files = sgg(opj(bag_folder_path,'*.bag'))
    run_name = fname(pname(bag_files[0]))
    if len(gg(opj(meta_path,run_name))) == 0:
        print("Making "+opj(meta_path,run_name))
        unix('mkdir -p '+opj(meta_path,run_name))
    if len(gg(opj(meta_path,run_name,'marker_data.pkl'))) == 1:
        print(opj(meta_path,run_name,'marker_data.pkl')+' exists, doing nothing.')
        return
    A = {}
    A['bag_folder_path'] = bag_folder_path
    for s in ['left','right']:
        A[s] = {}

    for path in bag_files:
        print('############## '+path+' #############')
        try:
           preprocess_bagfiles(A,path,visualize)
        except Exception as e:
            print("********** Exception ***********************")
            print(e.message, e.args)
    save_obj(A,opj(meta_path,run_name,'marker_data.pkl'))



def preprocess_bagfiles(A,path,visualize):

    ctr = 0
    timer = Timer(0)
    
    cprint('Loading bagfile '+path,'yellow')

    bag = rosbag.Bag(path)

    color_mode = "rgb8"
    for s in ['left','right']:
        for m in bag.read_messages(topics=['/bair_car/zed/'+s+'/image_rect_color']):
            t = round(m.timestamp.to_time(),3)
            A[s][t] = {}
            img = bridge.imgmsg_to_cv2(m[1],color_mode)
            angles_to_center, angles_surfaces, distances_marker, markers = get_angles_and_distance(img)
            A[s][t]['angles_to_center'] = angles_to_center
            A[s][t]['angles_surfaces'] = angles_surfaces
            A[s][t]['distances_marker'] = distances_marker
            A[s][t]['markers'] = markers
            
            if visualize > 0:
                if np.mod(ctr,visualize) == 0:
                    print(d2c(fname(path),s,t,A[s][t]['distances_marker']))
                    k = mci(img)
                    if k == ord('q'):
                        break
                ctr += 1
    print(d2s('Done in',timer.time(),'seconds'))



meta_path = '/home/karlzipser/Desktop/bair_car_data_new/meta'

Mr_Black = ['/media/karlzipser/ExtraDrive1/Mr_Black_25April2017/processed', 
            '/media/karlzipser/ExtraDrive1/Mr_Black_28April2017/processed'#, 
            #'/media/karlzipser/ExtraDrive1/Mr_Black_30April2017/processed'
            ]

Mr_Orange = ['/media/karlzipser/ExtraDrive2/Mr_Orange_25April2017/processed', 
            '/media/karlzipser/ExtraDrive2/Mr_Orange_28April2017/processed'#, 
            #'/media/karlzipser/ExtraDrive2/Mr_Orange_30April2017/processed'
            ]

Mr_Blue = ['/media/karlzipser/ExtraDrive3/Mr_Blue_25April2017/processed', 
            '/media/karlzipser/ExtraDrive3/Mr_Blue_28April2017/processed'#, 
            #'/media/karlzipser/ExtraDrive3/Mr_Blue_30April2017/processed'
            ]

Mr_Yellow_Silver = ['/media/karlzipser/ExtraDrive4/Mr_Yellow_25April2017/processed', 
            '/media/karlzipser/ExtraDrive4/Mr_Yellow_28April2017/processed', 
            '/media/karlzipser/ExtraDrive4/Mr_Yellow_30April2017/processed'#,
            #'/media/karlzipser/ExtraDrive4/Mr_Silver_28April2017/processed'
            ]


if False:

    multi_process_bag_folders(Mr_Yellow_Silver,meta_path,100)