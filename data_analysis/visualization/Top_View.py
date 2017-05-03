'''
Created on Apr 12, 2017

@author: Sascha Hornauer
'''
import numpy as np
import sys
import os
import cv2
import random
from aruco_tools.Marker import Marker
from Bagfile_Handler import Bagfile_Handler
import aruco_tools.aruco_angle_retriever as aruco_data

class Marker_Position(object):
    '''
    The Pythonic way of properties and setter/getters is ignored and the arrival of strong
    types and real private fields is anticipated in agony. 
    '''
    
    __pos_xy = None
    __shift_xy = None
    __marker_id = None
    __aquired_at_distance = None
    
    
    def __init__(self, marker_id, pos_xy, shift_xy, distance):
        self.__marker_id = marker_id
        self.__pos_xy = pos_xy
        self.__shift_xy = shift_xy
        self.__aquired_at_distance = distance
        
    def get_distance(self):
        return self.__aquired_at_distance
    
    def get_marker_id(self):
        return self.__marker_id
    
    def get_shift_xy(self):
        return self.__shift_xy
    
    def get_pos_xy(self):
        return self.__pos_xy
    
    def set_pos_xy(self,pos_xy):
        __pos_xy = pos_xy
    
    def __repr__(self):
        '''
        Quick string out method
        '''
        return str(self.__pos_xy) + "," + str(self.__shift_xy) + "," + str(self.__marker_id) + "," + str(self.__aquired_at_distance)
    
    
class Top_View(object):
    
    persistent_markers = {}
    marker_positions = {}
    base_x = 0.0
    base_y = 0.0 
    base_marker_id = None
    base_marker_found = False
    frame_number = 0
        
    def __init__(self):
        pass      

    
    
    def show_top_view(self, input_markers):
       
        cv_image = np.ones((600, 600, 3), np.uint8)
        
        # Some arbitrary scaling parameters for visualisation are set
        scale_factor = int(200.0 * (1.0 / 8.0))
        shift_factor = 300
        turn_factor = np.pi / 2.0 
        
        current_visible_markers = {}
        
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence - 0.1
        
        # Add markers to list, overwriting old markers and thereby recalculating confidence levels
        for marker in input_markers:
            self.persistent_markers[str(marker.marker_id)] = marker
            self.persistent_markers[str(marker.marker_id)].confidence = 1.0
            # Convert the input list to a dict with the id as key
            current_visible_markers[marker.marker_id] = marker

        # See if the initial marker is found, if not take the first of the visible ones
        if not self.base_marker_found:
            # first entry is used
            first_marker = current_visible_markers.itervalues().next()
            self.base_marker_id = first_marker.marker_id
            self.base_marker_found = True        
            orig_xy, orig_distance, orig_angle = self.get_marker_xy(first_marker)
            
            # That base marker position is added with shift 0,0 because it is the reference
            self.marker_positions[self.base_marker_id] = Marker_Position(self.base_marker_id, orig_xy, (0.0, (0.0, 0.0)), orig_distance)
        
        for current_marker in current_visible_markers.values():
            # Go over all visible marker to calculate its rotation and translation relative to the base
            # marker. 
            
            # If we are looking at the base marker, we just update its position. It has no shift
            if current_marker == self.base_marker_id:
                orig_xy, orig_distance, orig_angle = self.get_marker_xy(current_visible_markers[current_marker])
            
                # That base marker position is added with shift 0,0 because it is the reference
                self.marker_positions[current_marker.marker_id] = Marker_Position(current_marker.marker_id, orig_xy, (0.0, (0.0, 0.0)), orig_distance)
        
            # If we dont look at the base marker
            else:

                # for simplicity we calculate the shift new. This could be changed in the future
                
                # we check if the base marker is still visible
                if self.base_marker_id in current_visible_markers.keys():
                    
                    # do shift calculations according to base marker
                    base_marker = self.persistent_markers[str(self.base_marker_id)] 
                    new_marker_position = self.calculate_marker_to_base(base_marker, current_marker)
                    self.marker_positions[current_marker.marker_id] = new_marker_position
                         
                else:
                    # The base marker is no longer visible so the calculation is done
                    # via intermediate markers
                      
                    intermediate_marker = self.persistent_markers[str(self.marker_positions.keys()[0])]
                    new_marker_position = self.calculate_marker_to_inter(intermediate_marker, current_marker)
                    self.marker_positions[current_marker.marker_id] = new_marker_position
                          
         
        for marker in self.marker_positions.values():
            
            pos_x = marker.get_pos_xy()[0]
            pos_y = marker.get_pos_xy()[1]
            confidence_level = self.persistent_markers[str(marker.get_marker_id())].confidence
            #confidence_level = 1.0
            cv2.circle(cv_image, (int(scale_factor * pos_x) + shift_factor, int(scale_factor * pos_y) + shift_factor), 2, (255 * confidence_level, 0, 0), 2)
        
        # print(self.persistent_markers.keys())
        cv2.imshow('topView', cv_image)
        cv2.moveWindow('topView', 700, 0)

    def get_marker_xy(self, marker):
        distance = aruco_data.get_distance(marker)
        angle = aruco_data.get_angle_surface(marker)
        # angle = aruco_data.get_angle_to_center(marker)
        # Now we can get the dx and dy of movement
        xy = cv2.polarToCart(distance, angle)
        return tuple((xy[0][0][0], xy[1][0][0])), distance, angle
    
    def calculate_marker_to_base(self,base_marker, marker_b):

        
        # Get the data of the new perceived marker
        position_current_xy, distance_current, angle_surface_current = self.get_marker_xy(marker_b)
        
        # Get the data of the base marker, perceived from the new position
        base_xy, dist_base, ang_base = self.get_marker_xy(base_marker)
         
        # Calculate the difference in angle for coordination transform rotation
        phi = ang_base - angle_surface_current
         
        # Calculate the translation by calculating the coordinates according to the rotated origin
        trans_pos_xy = cv2.polarToCart(distance_current, phi)
        trans_pos_xy = (trans_pos_xy[0][0][0],trans_pos_xy[1][0][0])
        
        # The translation vector is now the difference between the coordinates, relative to
        # the base marker and the coordinates, relative to the new marker
        trans_rel_base = np.subtract(base_xy, trans_pos_xy)                  
         
        # Our position in the system of the base marker, given by the new marker, can now be calculated
        # First the rotation is applied
        new_x = position_current_xy[0] * np.cos(phi) + position_current_xy[1] * np.sin(phi)
        new_y = position_current_xy[0] * np.sin(phi) + position_current_xy[1] * np.cos(phi)
         
        # Then the translation is performed                                       
        resulting_x = new_x + trans_rel_base[0]
        resulting_y = new_y + trans_rel_base[1]
         
        # The position is written into a marker position, which is technically our position perceived under the
        # new marker, transformated into the reference frame of the base marker
        new_marker_position = Marker_Position(marker_b.marker_id, (resulting_x, resulting_y), (phi, (trans_rel_base)), distance_current)
        return new_marker_position      
        
        
        
    def calculate_marker_to_inter(self,interim_marker, marker_b):

        
        # Get the data of the new perceived marker
        position_current_xy, distance_current, angle_surface_current = self.get_marker_xy(marker_b)
        
        # Get the data of the base marker, perceived from the new position
        base_xy, dist_base, ang_base = self.get_marker_xy(interim_marker)
         
        # Calculate the difference in angle for coordination transform rotation
        phi = ang_base - angle_surface_current
         
        # Calculate the translation by calculating the coordinates according to the rotated origin
        trans_pos_xy = cv2.polarToCart(distance_current, phi)
        trans_pos_xy = (trans_pos_xy[0][0][0],trans_pos_xy[1][0][0])
        
        # The translation vector is now the difference between the coordinates, relative to
        # the base marker and the coordinates, relative to the new marker
        trans_rel_base = np.subtract(base_xy, trans_pos_xy)                  
         
        # Our position in the system of the base marker, given by the new marker, can now be calculated
        # First the rotation is applied
        new_x = position_current_xy[0] * np.cos(phi) + position_current_xy[1] * np.sin(phi)
        new_y = position_current_xy[0] * np.sin(phi) + position_current_xy[1] * np.cos(phi)
         
        # Then the translation is performed                                       
        resulting_x = new_x + trans_rel_base[0]
        resulting_y = new_y + trans_rel_base[1]
         
        # The position is written into a marker position, which is technically our position perceived under the
        # new marker, transformated into the reference frame of the base marker
        new_marker_position = Marker_Position(marker_b.marker_id, (resulting_x, resulting_y), (phi, (trans_rel_base)), distance_current)
        return new_marker_position      
        

if __name__ == "__main__":
    visualizer = Top_View()
    bagfile_path = sys.argv[1]
    
    bagfile_handler = Bagfile_Handler(bagfile_path)
    paused_video = False
    
    head, tail = os.path.split(bagfile_path)
    
    while(True):
        if not paused_video:
            cv_image = bagfile_handler.get_image()
        
            markers = aruco_data.get_markers_in_image(cv_image)
            visualizer.show_top_view(markers)
            # visualizer.visualize_markers_center_line(markers,tail,cv_image)
        cv2.imshow("video", cv_image)
        
        key = cv2.waitKey(1000 / 30) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            paused_video = not paused_video
        if key == ord('w'):
            bagfile_handler.fast_forward()
            
            
#             
#             # Get the data of the new perceived marker
#                     position_current_xy, distance_current, angle_surface_current = self.get_marker_xy(current_marker)
#                     
#                     # Check if that marker already has shift values
#                     if(current_marker.marker_id in self.marker_positions and self.marker_positions[current_marker.marker_id].get_shift_xy() != None):
#                         # If that is the case only the position is updated
#                         self.marker_positions[current_marker.marker_id].set_pos_xy(position_current_xy)
#                         continue
#                      
#                       
#                     # Get the data of the intermediate marker, perceived from the new position
#                     interim_marker_pos = self.marker_positions.values()[0]
#                     pos_interim_xy, dist_interim, ang_interim = self.get_marker_xy(self.persistent_markers[str(interim_marker_pos.get_marker_id())])
#                     
#                     # Get its shift phi and translation  
#                     phi_to_base = interim_marker_pos.get_shift_xy()[0]
#                     trans_to_base = interim_marker_pos.get_shift_xy()[1]
#                     
#                     # Calculate the angle shift in between the two markers and in between the marker and the base  
#                     # known from the interim marker to have the overall shift
#                     phi = ang_interim - angle_surface_current - phi_to_base
#                       
#                     # Calculate the translation to the interim vector, by first turning appropriately
#                     trans_pos_xy = cv2.polarToCart(distance_current, ang_interim - angle_surface_current)
#                     trans_pos_xy = (trans_pos_xy[0][0][0],trans_pos_xy[1][0][0])  
#                     
#                     trans_to_interim = np.subtract(pos_interim_xy, trans_pos_xy)
#                     # The translation vector is now the difference between the translation to the
#                     # base marker and the translation to the interim marker
#                     # trans to base is already base_xy - interim_xy
#                     trans_rel_base = np.subtract(trans_to_base, trans_to_interim)     
#                     
#                     # Our position in the system of the base marker, given by the new marker, can now be calculated
#                     # First the rotation is applied by the whole phi angle
#                     new_x = position_current_xy[0] * np.cos(phi) + position_current_xy[1] * np.sin(phi)
#                     new_y = position_current_xy[0] * np.sin(phi) + position_current_xy[1] * np.cos(phi)
#                      
#                     # Then the translation is performed by the whole translation vector                                
#                     resulting_x = new_x + trans_rel_base[0]
#                     resulting_y = new_y + trans_rel_base[1]
#                      
#                     # The position is written into a marker position, which is technically our position perceived under the
#                     # new marker, transformed into the reference frame of the base marker
#                     new_marker_position = Marker_Position(current_marker.marker_id, (resulting_x, resulting_y), (phi, (trans_rel_base)), distance_current)
#                       
