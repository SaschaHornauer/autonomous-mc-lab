from kzpy3.utils import *
import rospy
import rosbag
import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
bridge = cv_bridge.CvBridge()
from kzpy3.data_analysis.data_parsing.Bagfile_Handler import Bagfile_Handler


def load_images(bag_file_path,color_mode="rgb8",include_flip=True):
    
    topic_name_map = {}
    
    bag_handler = Bagfile_Handler(bag_file_path,['/bair_car/zed/left/image_rect_color','/bair_car/zed/right/image_rect_color'])
    
    topic_name_map['/bair_car/zed/left/image_rect_color'] = 'left'
    topic_name_map['/bair_car/zed/right/image_rect_color'] = 'right'

    bag_img_dic = {}
    sides=['left','right']
    if bag_file_path.split('.')[-1] == 'bag':
        PKL = False
    elif bag_file_path.split('.')[-1] == 'pkl':
        PKL = True
    else:
        assert(False)

    if not PKL:
        for side in sides:
            bag_img_dic[side] = {}
        bag = rosbag.Bag(bag_file_path)
        
        topic, msg, timestamp = bag_handler.get_bag_content()
        while msg != None:
            topic, msg, timestamp = bag_handler.get_bag_content()
            timestamp = round(timestamp.to_sec(),3)
            img = bridge.imgmsg_to_cv2(msg.data,color_mode)
            bag_img_dic[topic_name_map[topic]][timestamp] = img
    else:
        bag_img_dic = load_obj(bag_file_path)

    if include_flip:
        for side in sides:
            bag_img_dic[side+'_flip'] = {}
            for t in bag_img_dic[side]:
                img = bag_img_dic[side][t]
                bag_img_dic[side+'_flip'][t] = scipy.fliplr(img)
    return bag_img_dic



def save_images(bag_file_src_path,bag_file_dst_path):
    try:
        bag_img_dic = load_images(bag_file_src_path,color_mode="rgb8",include_flip=False)
        for side in bag_img_dic:
            for t in bag_img_dic[side]:
                img = bag_img_dic[side][t]
                bag_img_dic[side][t] = cv2.resize(img,None,fx=0.25,fy=0.25,interpolation=cv2.INTER_AREA)
        print "Bag_File.load_images:: saving " + opj(bag_file_dst_path)
        save_obj(bag_img_dic,opj(bag_file_dst_path))
    except Exception as e:
        print e.message, e.args    


def bag_folder_save_images(bag_folder_src_path,bag_folder_dst_path):
    unix('mkdir -p '+bag_folder_dst_path)
    bag_file_paths = sorted(glob.glob(opj(bag_folder_src_path,'*.bag')))
    for bf in bag_file_paths:
        bag_file_dst_path = opj(bag_folder_dst_path)
        bag_file_dst_path = opj(bag_file_dst_path,fname(bf)+'.pkl')
        print bag_file_dst_path
        save_images(bf,bag_file_dst_path)


def bag_folders_save_images(bag_folders_src_path,bag_folders_dst_path):
    bag_folders_paths = sorted(glob.glob(opj(bag_folders_src_path,'*')))
    ef = sgg(opj(bag_folders_dst_path,'*'))
    existing_folders = []
    for e in ef:
        existing_folders.append(fname(e))
    for bfp in bag_folders_paths:
        if fname(bfp) not in existing_folders:
            bag_folder_save_images(bfp,opj(bag_folders_dst_path,fname(bfp)))
        else:
            cprint('Excluding '+bfp,'green','on_blue')


def bag_folders_transfer_meta(bag_folders_src_path,bag_folders_dst_path):
    #bag_folders_src_path,bag_folders_dst_path='/media/karlzipser/bair_car_data_6/bair_car_data','/home/karlzipser/Desktop/bair_car_data/meta/'
    bag_folders_paths = sgg(opj(bag_folders_src_path,'*'))
    for bfp in bag_folders_paths:
        unix('mkdir -p '+opj(bag_folders_dst_path,fname(bfp)))
        meta_dirs = sorted(glob.glob(opj(bfp,'.pre*')))
        for m in meta_dirs:
            data = sorted(glob.glob(opj(m,'left*')))
            data += sorted(glob.glob(opj(m,'pre*')))
            for d in data:
                cprint(opj(opj(bag_folders_dst_path,fname(bfp))),'yellow')
                unix_str = d2s('scp ',d,opj(bag_folders_dst_path,fname(bfp)))
                if len(gg(opj(bag_folders_dst_path,fname(bfp),fname(d)))) == 0: # test this first
                    cprint(unix_str,'red')
                    unix(unix_str)
                else:
                    cprint(d2s('NOT',unix_str))