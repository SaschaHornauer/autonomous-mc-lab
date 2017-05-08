from kzpy3.utils import *
import cv2

def get_new_A(_=None):
    A = {}
    A['STOP_LOADER_THREAD'] = False
    A['STOP_ANIMATOR_THREAD'] = False
    A['STOP_GRAPH_THREAD'] = False
    A['d_indx'] = 1.0
    A['current_img_index'] = -A['d_indx']
    A['t_previous'] = 0
    A['left_deltas'] = []
    A['scale'] = 1#4.0
    A['delay'] = None
    A['steer'] = []
    A['state'] = []
    A['SMOOTHING'] = True
    A['motor'] = []
    A['images'] = []
    A['left'] = []
    A['meta'] = None
    A['color_mode'] = cv2.COLOR_RGB2BGR
    A['save_start_index'] = 0
    A['save_stop_index'] = 100000
    A['collisions'] = []
    A['t_to_indx'] = {}
    return A