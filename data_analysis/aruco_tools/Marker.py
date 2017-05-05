'''
Created on Apr 13, 2017

@author: Sascha Hornauer
'''

class Marker(object):
    '''
    The Pythonic way of properties and setter/getters is ignored and the arrival of strong
    types and real private fields is anticipated in agony. 
    '''
    marker_id = None
    confidence = 1.0
    corners_xy = []
    rvec = None
    tvec = None
    pos_xy = None
    shift_angle_trans = None
    aquired_at_distance = None
    

    def __init__(self, marker_id=None, confidence=None, corners_xy=None, rvec=None, tvec=None, pos_xy=None, shift_angle_trans=None, distance=None):
        '''
        Constructor
        '''
        self.marker_id = marker_id
        self.confidence = confidence
        self.corners_xy = corners_xy
        self.rvec = rvec
        self.tvec = tvec
        self.pos_xy = pos_xy
        self.shift_angle_trans = shift_angle_trans
        self.aquired_at_distance = distance

    
    
    
    def __repr__(self):
        '''
        Quick string out method
        '''
        
        return str(vars(self))
   
