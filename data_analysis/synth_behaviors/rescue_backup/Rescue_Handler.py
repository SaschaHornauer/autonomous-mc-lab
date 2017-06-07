'''
Created on Jun 5, 2017

@author: Sascha Hornauer
'''

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from timeit import default_timer as timer
from enum import Enum
from detectors import state_info, Obstacle_Crash_Detector, Side_Tilted_Detector


debug_mode = True

class Rescue_State(Enum):
    OBSERVING = 1
    EXECUTING = 2    

class Rescue_Handler:

    rescue_state = Rescue_State.OBSERVING

    cycle_time = 1/30 # update time in ms  

    execute_behavior = False
    current_time = None
    
    _steer_cmd = 49
    _motor_cmd = 49
    _motor_signal = None
    _steer_signal = None
    _state = None
    _gyro_heading = None
    _acc_data = None
    
    state_publisher = None
    state_info = None
    detector_list = []
    
    def __init__(self):
        
        rospy.Subscriber("/bair_car/gyro_heading", Vector3, self.heading_callback, queue_size=1)
        rospy.Subscriber("/bair_car/acc", Vector3, self.acc_callback, queue_size=1)
        rospy.Subscriber('/bair_car/state', Int32, self.state_callback)
        rospy.Subscriber('/bair_car/steer', Int32, self.steer_callback,queue_size=100)
        rospy.Subscriber('/bair_car/motor', Int32, self.motor_callback,queue_size=100)
        self.state_publisher = rospy.Publisher('/bair_car/state', Int32,queue_size=100)
        self.current_time = timer()
        
        self.state_info = state_info(self._state,self._steer_signal,self._motor_signal,self._gyro_heading,self._acc_data)
        
        # Add all detectors
        self.detector_list.append(Obstacle_Crash_Detector())
        self.detector_list.append(Side_Tilted_Detector())

    def state_callback(self, state):
        self._state = state
        
    def acc_callback(self, acc):
        self._acc_data = acc

    def heading_callback(self, heading):
        self._gyro_heading = heading
            
    def steer_callback(self, steering):
        self._steer_signal = steering
    
    def motor_callback(self,motor):
        self._motor_signal = motor
            
    def get_next_steer_cmd(self):
        return self._steer_cmd
            
    def get_next_motor_cmd(self):
        return self._motor_cmd
    
    def update(self):
        
        if self.current_time + timer() > self.cycle_time:
            
            self.state_info.state = self._state
            self.state_info.steering_signal = self._steer_signal
            self.state_info.motor_signal = self._motor_signal
            self.state_info.gyro_heading = self._gyro_heading            
            self.state_info.acc_data = self._acc_data
            rescue_needed = False
            
            # Check with all detectors if rescue is needed
            for detector in self.detector_list:
                rescue_needed_new, detector_type = detector.check_state(self.state_info)
                rescue_needed = rescue_needed or rescue_needed_new
                
            # Execute a rescue behavior, depending on the detector
            if rescue_needed:
                self.rescue_state = Rescue_State.EXECUTING
                self.execute_rescue(detector_type)
            
            # select the correct rescue behavior
            # execute the correct rescue behavior
            # Signal return to normal net mode
            # return to observance
    
    def execute_rescue(self,detector_type):
        
        print "Handler to the rescue !!!" + str(detector_type)
        pass


if debug_mode and __name__ == '__main__':
    rospy.init_node('run_caffe',anonymous=True)
    
    rescue_behavior = Rescue_Handler()
    
    while not rospy.is_shutdown():
        
        rate = rospy.Rate(10)
        
        rescue_behavior.update()
        
        if rescue_behavior.execute_behavior:
            
            caf_steer = rescue_behavior.get_next_steer_cmd()
            caf_motor = rescue_behavior.get_next_motor_cmd()
            
            print caf_steer
            print caf_motor
            
            
        rate.sleep()
