'''
Created on Jun 5, 2017

@author: Sascha Hornauer
'''

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from timeit.default_timer import default_timer as timer
from enum import Enum

class Rescue_Handler(object):

    state = Rescue_State.OBSERVING

    cycle_time = 1/30 # update time in ms  

    execute_behavior = False
    current_time = None
    
    _steer_cmd = 49
    _motor_cmd = 49
    state_publisher = None
    
    
    def __init__(self, params):
        
        rospy.Subscriber("/bair_car/gyro_heading", Vector3, self.heading_callback, queue_size=1)
        rospy.Subscriber('/bair_car/state', Int32, self.state_callback)
        rospy.Subscriber('/bair_car/steer', Int32, self.steer_callback,queue_size=100)
        rospy.Subscriber('/bair_car/motor', Int32, self.motor_callback,queue_size=100)
        state_publisher = rospy.Publisher('/bair_car/state', Int32)
        self.current_time = timer()
                                        
        # Track current status
        # Pass through stuck - detectors
        # If one of the detectors sees the beahvior stuck, create unstuck behavior
        # execute

    def state_callback(self):
        pass

    def heading_callback(self):
        pass
    
    def steer_callback(self):
        pass
    
    def motor_callback(self):
        pass
    
    def get_next_steer_cmd(self):
        return self._steer_cmd
            
    def get_next_motor_cmd(self):
        return self._motor_cmd
    
    def update(self):
        
        if self.current_time + timer() > self.cycle_time:
            
            # Check if the state of the car is crashed
            # Apply all the detectors
            # If one detector confirms crashing signal to the rosnode
            # select the correct rescue behavior
            # execute the correct rescue behavior
            # Signal return to normal net mode
            # return to observance
            pass
        
        pass
    
    
    
    
class Rescue_State(Enum):
    OBSERVING = 1
    EXECUTING = 2
    
    
    
        
