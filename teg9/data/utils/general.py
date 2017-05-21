from kzpy3.utils import *
import cv2

def get_new_Data_dic(_=None):
    D = {}
    D['STOP_LOADER_THREAD'] = False
    D['STOP_ANIMATOR_THREAD'] = False
    D['STOP_GRAPH_THREAD'] = False
    D['d_indx'] = 1.0
    D['current_img_index'] = -D['d_indx']
    D['t_previous'] = 0
    D['left_deltas'] = []
    D['scale'] = 1#4.0
    D['delay'] = None
    D['steer'] = []
    D['state'] = []
    D['SMOOTHING'] = True
    D['motor'] = []
    D['images'] = []
    D['left'] = []
    D['meta'] = None
    D['color_mode'] = cv2.COLOR_RGB2BGR
    D['save_start_index'] = 0
    D['save_stop_index'] = 100000
    D['collisions'] = []
    D['t_to_indx'] = {}
    return D

def car_name_from_run_name(rn):
    a = rn.split('Mr_')
    car_name = 'Mr_'+a[-1]
    car_name = car_name.replace('Mr_Yellow_A','Mr_Yellow')
    car_name = car_name.replace('Mr_Yellow_B','Mr_Yellow')
    return car_name

car_colors = {'Mr_Yellow':(255,255,0), 'Mr_Silver':(255,255,255), 'Mr_Blue':(0,0,255), 'Mr_Orange':(255,0,0), 'Mr_Black':(100,100,100)}
