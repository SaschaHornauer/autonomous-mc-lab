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


def multi_preprocess_pkl_files(A,meta_path,rgb_1to4_path,print_b=False):
    for topic in image_topics + single_value_topics:
        if topic not in A:
            A[topic] = []
    for topic in vector3_topics:
        if topic+'_x' not in A:
            A[topic+'_x'] = []
            A[topic+'_y'] = []
            A[topic+'_z'] = []
    A['meta'] = load_obj(opj(meta_path,'left_image_bound_to_data.pkl'))
    steer_previous = 49
    motor_previous = 49
    bag_pkls = sgg(opj(rgb_1to4_path,'*.bag.pkl'))
    indx = 0
    for b in bag_pkls:
        if print_b:
            print b
        o = load_obj(b)
        ts = sorted(o['left'].keys())
        
        for t in ts:
            A['left'].append(o['left'][t])
            A['t_to_indx'][t] = indx
            indx += 1
            try:
                if A['SMOOTHING']:
                    A['steer'].append((A['meta'][t]['steer']+steer_previous)/2.0)
                    A['motor'].append((A['meta'][t]['motor']+motor_previous)/2.0)
                    A['state'].append(A['meta'][t]['state'])
                    steer_previous = A['steer'][-1]
                    motor_previous = A['motor'][-1]
                else:
                    A['steer'].append(A['meta'][t]['steer'])
                    A['state'].append(A['meta'][t]['state'])
                    A['motor'].append(A['meta'][t]['motor'])
            except:
                A['steer'].append(0)
                A['state'].append(0)
                A['motor'].append(0)
            try:
                A['acc_x'].append(A['meta'][t]['acc'][0])
                A['acc_y'].append(A['meta'][t]['acc'][1])
                A['acc_z'].append(A['meta'][t]['acc'][2])

            except:
                A['acc_x'].append(0)
                A['acc_y'].append(1)
                A['acc_z'].append(2)
            try:
                A['gyro_x'].append(A['meta'][t]['gyro'][0])
                A['gyro_y'].append(A['meta'][t]['gyro'][1])
                A['gyro_z'].append(A['meta'][t]['gyro'][2])

            except:
                A['gyro_x'].append(0)
                A['gyro_y'].append(1)
                A['gyro_z'].append(2)
            try:
                A['encoder'].append(A['meta'][t]['encoder'])

            except:
                A['encoder'].append(0)

    A['acc_xz_dst'] = sqrt(array(A['acc_x'])**2 + array(A['acc_z'])**2)
    A['collisions'] = 0*array(A['steer'])


