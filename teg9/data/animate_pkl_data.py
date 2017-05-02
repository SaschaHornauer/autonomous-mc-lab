
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

def multi_preprocess(A,meta_path,rgb_1to4_path):
    meta = load_obj(meta_path)
    A['steer'] = []
    A['state'] = []
    A['motor'] = []

    A['images'] = []
    bag_pkls = sgg(opj(rgb_1to4_path,'*.bag.pkl'))
    for b in bag_pkls:
        print b
        o = load_obj(b)
        print o.keys()
        ts = sorted(o['left'].keys())
        for t in ts:
            A['images'].append(o['left'][t])
            try:
                A['steer'].append(meta[t]['steer'])
                A['state'].append(meta[t]['state'])
                A['motor'].append(meta[t]['motor'])

            except:
                A['steer'].append(0)
                A['state'].append(0)
                A['motor'].append(0)             

def start_graph(A):
    A['STOP_GRAPH_THREAD'] = False
def stop_graph(A):
    A['STOP_GRAPH_THREAD'] = True
def start_animation(A):
    A['STOP_ANIMATOR_THREAD'] = False
def stop_animation(A):
    A['STOP_ANIMATOR_THREAD'] = True
def stop_loader(A):
    A['STOP_LOADER_THREAD'] = True


def get_new_A(_=None):
    A = {}
    A['STOP_LOADER_THREAD'] = False
    A['STOP_ANIMATOR_THREAD'] = False
    A['STOP_GRAPH_THREAD'] = False
    A['d_indx'] = 1.0
    A['current_img_index'] = -A['d_indx']
    A['t_previous'] = 0
    A['left_deltas'] = []
    return A

def menu(A):
    menu_functions = ['exit_menu','start_animation','start_graph','stop_animation','stop_graph','stop_loader','get_new_A','d_index_up','d_index_down']
    while True:
        for i in range(len(menu_functions)):
            print(d2n(i,') ',menu_functions[i]))
        try:
            choice = input('> ')
            if type(choice) == int:
                if choice == 0:
                    return
                if choice >-1 and choice < len(menu_functions):
                    exec_str = d2n(menu_functions[choice],'(A)')
                    exec(exec_str)
        except:
            pass



if __name__ == '__main__':
    meta_path = sys.argv[1]
    bag_folder_path = sys.argv[2]
    print bag_folder_path
    #bagfile_range=[int(sys.argv[2]),int(sys.argv[3])]
    hist_timer = Timer(10)
    A = {}
    A = get_new_A(A)
    multi_preprocess(A,meta_path,bag_folder_path)
    threading.Thread(target=animate.animate,args=[A]).start()
    #threading.Thread(target=graph_thread,args=[A]).start()
    #menu(A)





