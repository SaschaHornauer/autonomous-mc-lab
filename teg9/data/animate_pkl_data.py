
from kzpy3.vis import *
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge,CvBridgeError
import threading
import kzpy3.teg9.data.animate as animate

bridge = CvBridge()

image_topics = ['left_image','right_image']
single_value_topics = ['steer','state','motor','encoder','GPS2_lat']
vector3_topics = ['acc','gyro','gyro_heading']#,'gps']
camera_sides = ['left','right']






def multi_preprocess_bags(A,meta_path,bag_folder_path,bagfile_range=[]):

    A['meta'] = load_obj(meta_path)

    bag_files = sorted(gg(opj(bag_folder_path,'*.bag')))
    if len(bagfile_range) > 0:
        bag_files = bag_files[bagfile_range[0]:bagfile_range[1]]
    A['images'] = []
    threading.Thread(target=multi_preprocess_thread,args=[A,bag_files]).start()




def multi_preprocess_thread(A,bag_files):
    for b in bag_files:
        if A['STOP_LOADER_THREAD']:
            A['STOP_LOADER_THREAD'] = False
            print('Stopping multi_preprocess_thread.')
            break
        preprocess(A,b)




def preprocess(A,path):
    timer = Timer(0)
    
    for topic in image_topics + single_value_topics:
        if topic not in A:

            A[topic] = []
    for topic in vector3_topics:
        if topic+'_x' not in A:
            A[topic+'_x'] = []
            A[topic+'_y'] = []
            A[topic+'_z'] = []

    if True:#try:
        cprint('Loading bagfile '+path,'yellow')

        bag = rosbag.Bag(path)

        color_mode = "rgb8"
        steer_previous = 49
        motor_previous = 49
        for s in ['left']:#camera_sides:
            for m in bag.read_messages(topics=['/bair_car/zed/'+s+'/image_rect_color']):
                t = round(m.timestamp.to_time(),3)
                if A['t_previous'] > 0:            
                    if s == 'left':
                        A['left_deltas'].append([t,t-A['t_previous']])
                A['t_previous'] = t
                
                A['images'].append(bridge.imgmsg_to_cv2(m[1],color_mode))

                if t not in A['meta']:
                    print(d2s(t,"not in A['meta']"))
                try:
                    if A['SMOOTHING']:
                        A['steer'].append((A['meta'][t]['steer']+steer_previous)/2.0)
                        A['motor'].append((A['meta'][t]['motor']+motor_previous)/2.0)
                        A['state'].append(A['meta'][t]['state'])
                        steer_previous = A['steer'][-1]
                        motor_previous = A['motor'][-1]
                        #print steer_previous
                    else:
                        A['steer'].append(A['meta'][t]['steer'])
                        A['state'].append(A['meta'][t]['state'])
                        A['motor'].append(A['meta'][t]['motor'])
                    #print A['steer']
                except:
                    A['steer'].append(0)
                    A['state'].append(0)
                    A['motor'].append(0)



    #except Exception as e:
    #    print e.message, e.args
    print(d2s('Done in',timer.time(),'seconds'))





def multi_preprocess_pkl(A,meta_path,rgb_1to4_path):
    A['meta'] = load_obj(meta_path)

    steer_previous = 49
    motor_previous = 49
    bag_pkls = sgg(opj(rgb_1to4_path,'*.bag.pkl'))
    for b in bag_pkls:
        print b
        o = load_obj(b)
        ts = sorted(o['left'].keys())
        for t in ts:
            A['images'].append(o['left'][t])
            try:
                if A['SMOOTHING']:
                    A['steer'].append((A['meta'][t]['steer']+steer_previous)/2.0)
                    A['motor'].append((A['meta'][t]['motor']+motor_previous)/2.0)
                    A['state'].append(A['meta'][t]['state'])
                    steer_previous = A['steer'][-1]
                    motor_previous = A['motor'][-1]
                    #print steer_previous
                else:
                    A['steer'].append(A['meta'][t]['steer'])
                    A['state'].append(A['meta'][t]['state'])
                    A['motor'].append(A['meta'][t]['motor'])
            except:
                A['steer'].append(0)
                A['state'].append(0)
                A['motor'].append(0)







def get_new_A(_=None):
    A = {}
    A['STOP_LOADER_THREAD'] = False
    A['STOP_ANIMATOR_THREAD'] = False
    A['STOP_GRAPH_THREAD'] = False
    A['d_indx'] = 1.0
    A['current_img_index'] = -A['d_indx']
    A['t_previous'] = 0
    A['left_deltas'] = []
    A['scale'] = 1.0
    A['delay'] = None
    A['steer'] = []
    A['state'] = []
    A['SMOOTHING'] = True
    A['motor'] = []
    A['images'] = []
    A['meta'] = None
    A['color_mode'] = cv2.COLOR_RGB2BGR
    A['save_start_index'] = 1600
    A['save_stop_index'] = 1600+30*30
    return A






if __name__ == '__main__':

    hist_timer = Timer(10)
    A = {}
    A = get_new_A(A)

    if False:
        meta_path = '/home/karlzipser/Desktop/bair_car_data_Main_Dataset/meta/direct_1Sept2016_Mr_Orange_local_sidewalks/left_image_bound_to_data.pkl' 
        path2 = '/home/karlzipser/Desktop/bair_car_data_Main_Dataset/rgb_1to4/direct_1Sept2016_Mr_Orange_local_sidewalks'
        multi_preprocess_pkl(A,meta_path,path2)

    if True:
        meta_path = '/media/karlzipser/ExtraDrive2/Mr_Orange_28April2017/processed/direct_rewrite_test_28Apr17_18h55m31s_Mr_Orange/.preprocessed2/left_image_bound_to_data.pkl'
        path2 = '/media/karlzipser/ExtraDrive2/Mr_Orange_28April2017/processed/direct_rewrite_test_28Apr17_18h55m31s_Mr_Orange'
        multi_preprocess_bags(A,meta_path,path2,bagfile_range=[])



    threading.Thread(target=animate.animate,args=[A]).start()






