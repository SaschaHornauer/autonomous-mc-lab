'''
Created on Apr 11, 2017

@author: picard
'''
import rosbag
from cv_bridge import CvBridge
import sys
import pickle
import os

class Image_Bagfile_Handler(object):
    
    print("Reading Bagfile")
    bag = None
    bag_access = None
    bridge = CvBridge()
    timestamp = 0
    
    # pickle_file = None
    data_for_pickle_file = []
    old_evasion_data = []
    
    def __init__(self, bag_filepath,side='left'):
        self.bag = rosbag.Bag(bag_filepath)
        head, tail = os.path.split(bag_filepath)
        # self.pickle_file = open("crash_" + tail +'.pkl', 'wb')
        self.bag_access = self.bag.read_messages(topics=['/bair_car/zed/'+side+'/image_rect_color']).__iter__()
        
        self._current_bag_entry = self.bag_access.next()
        
    def __del__(self):
        try:
            self.bag.close()
        except:
            pass

    _current_bag_entry = None

    def get_image(self,timestamp):
    
        topic, msg, bagfile_timestamp = self._current_bag_entry
        
        if timestamp + (1/30.) < bagfile_timestamp.to_sec():
            print "OUT OF SYNC " + str(timestamp -  bagfile_timestamp.to_sec())
            print "Future timestamp needed"
            return None, timestamp
        elif timestamp - (1/30.) > bagfile_timestamp.to_sec():
            print "OUT OF SYNC " + str(timestamp -  bagfile_timestamp.to_sec())
            while(timestamp - (1/30.) > bagfile_timestamp.to_sec()):
                topic, msg, bagfile_timestamp = self.bag_access.next()
                print "OUT OF SYNC " + str(timestamp -  bagfile_timestamp.to_sec())
                print "Forwarding bagfile"
            self._current_bag_entry = topic, msg, bagfile_timestamp
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        return cv_image, timestamp

    
    def fast_forward(self):
        try:
            for i in range(0, 60):
                self.bag_access.next()
        except:
            self.bag.close()
            
            
    
    
                        
