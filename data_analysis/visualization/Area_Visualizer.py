'''
Created on Apr 12, 2017

@author: Sascha Hornauer
'''
import numpy as np
import sys
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
    
    def __repr__(self):
        '''
        Quick string out method
        '''
        return str(self.__pos_xy) + "," + str(self.__shift_xy) + "," + str(self.__marker_id) + "," + str(self.__aquired_at_distance)
    
    
class Area_Visualizer(object):
    
    persistent_markers = {}
    marker_positions = {}
    base_x = 0.0
    base_y = 0.0 
    base_marker_id = None
    base_marker_found = False
    
        
    def __init__(self):
        pass      

    def visualize_markers_center_line(self, markers):
        img2 = np.ones((600, 600, 3), np.uint8)
        
        # This factor is rather arbitrary. The x,y results of the following
        # calculation are approximately in between 0 and 8 and accordingly this
        # factor is chosen
        scale_factor = 300.0 * (1.0 / 8.0)
        shift_factor = 300
        turn_factor = np.pi / 2.0  # division by int gives integer
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence - 0.1
        
        # Add markers to list, overwriting old markers and thereby increasing confidence levels
        for marker in markers:
            self.persistent_markers[str(marker.id)] = marker
        
        
        # Now draw lines onto new window
        for marker_id in self.persistent_markers:
            
            marker = self.persistent_markers[marker_id] 
            
            # Draw marker outline at the bottom of the screen
            xy1 = marker.corners_xy_pos[0]
            xy2 = marker.corners_xy_pos[1]

            cv2.line(img2, (xy1[0], xy1[1] + shift_factor), (xy2[0], xy2[1] + shift_factor), (0, 0, 255 * marker.confidence), 1)

            distance = marker.corners_distances_angles['distance']
            angle = marker.corners_distances_angles['angle']
            
            # Draw top view outline
            x_a, y_a = cv2.polarToCart(distance, angle - turn_factor)
            x_a[0] = x_a[0] * scale_factor + shift_factor
            y_a[0] = y_a[0] * scale_factor + shift_factor
            cv2.circle(img2, (x_a[0], y_a[0]), 2, (0, 0, 255 * marker.confidence), -1)
                
            # ircle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
                
            # Draw outer viewport lines
            
            fov = np.deg2rad(110.0)
                
            x_orig, y_orig = cv2.polarToCart(0.0, (-fov / 2.0) - turn_factor)
            x_dest, y_dest = cv2.polarToCart(8.0, (-fov / 2.0) - turn_factor)
            x_orig[0] = x_orig[0] * scale_factor + shift_factor
            y_orig[0] = y_orig[0] * scale_factor + shift_factor
            x_dest[0] = x_dest[0] * scale_factor + shift_factor
            y_dest[0] = y_dest[0] * scale_factor + shift_factor
            
            cv2.line(img2, (x_orig[0], y_orig[0]), (x_dest[0], y_dest[0]), (255, 255, 255), 1)
            
            x_orig_max, y_orig_max = cv2.polarToCart(0.0, (fov / 2.0) - turn_factor)
            x_dest_max, y_dest_max = cv2.polarToCart(8.0, (fov / 2.0) - turn_factor)
            x_orig_max[0] = x_orig_max[0] * scale_factor + shift_factor
            y_orig_max[0] = y_orig_max[0] * scale_factor + shift_factor
            x_dest_max[0] = x_dest_max[0] * scale_factor + shift_factor
            y_dest_max[0] = y_dest_max[0] * scale_factor + shift_factor
            
            cv2.line(img2, (x_orig_max[0], y_orig_max[0]), (x_dest_max[0], y_dest_max[0]), (255, 255, 255), 1)
            
            
            
                           
            # map test code
            # This is one of the worst places where the code can be
            # put though this is highly experimental and here I have all
            # the needed information handily available
            
            # for marker in markers:
            #    self.map.addMarker(marker_id,distance,angle)
            
            # map test code
                
            
            
            
            
        cv2.imshow('topView', img2)
        cv2.moveWindow('topView', 700, 0)
    
    def show_top_view(self, markers):
        
        
        # Further ideas:
        # To avoid some recalculations each time a new marker is detected, see if
        # the confidence of a shift is above some value
        
        cv_image = np.ones((600, 600, 3), np.uint8)
        
        # Some arbitrary scaling parameters for visualisation are set
        scale_factor = int(200.0 * (1.0 / 8.0))
        shift_factor = 300
        turn_factor = np.pi / 2.0 
        current_visible_marker_ids = []
        current_visible_marker = []
        current_xy = (0, 0)
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence - 0.1
        
        # Add markers to list, overwriting old markers and thereby recalculating confidence levels
        for marker in markers:
            self.persistent_markers[str(marker.marker_id)] = marker
            current_visible_marker_ids.append(marker.marker_id)
            current_visible_marker.append(marker)
        # This needs eventual rethinking because the more closer a marker is, the higher its
        # confidence or probability it is there is. Also two for loops here are maybe not necessary

        # Update all positions relative to the first marker
        for marker_id in self.persistent_markers:

            current_marker = self.persistent_markers[marker_id]

            if not self.base_marker_found:
                self.base_marker_id = marker_id
                self.base_marker_found = True
            
                orig_xy, orig_distance, _ = self.get_marker_xy(current_marker)
            
                # We assume this is the basis of all future calculations
                self.base_x = orig_xy[0]
                self.base_y = orig_xy[1]
                
                # That base marker position is added with shift 0,0 because it is the reference
                self.marker_positions[marker_id] = Marker_Position(marker_id, orig_xy, (0.0, (0.0,0.0)), orig_distance)
            
            
            elif marker_id != self.base_marker_id:
                # if the base marker is already found and we are not right now looking
                # at it, we check if the base marker is still visible
                
                if int(self.base_marker_id) in current_visible_marker_ids:
                    
                    # do shift calculations according to base marker
                    
                    # Get the data of the new perceived marker
                    new_pos_xy, new_distance, new_angle = self.get_marker_xy(current_marker)
                    
                    # Get the data of the base marker, perceived from the new position
                    pos_base, dist_base, ang_base = self.get_marker_xy(self.persistent_markers[self.base_marker_id])
                    
                    phi = ang_base - new_angle
                    
                    new_x = new_pos_xy[0] * np.cos(phi) + new_pos_xy[1] * np.sin(phi)
                    new_y = new_pos_xy[0] * np.sin(phi) + new_pos_xy[1] * np.cos(phi)
                    
                    shift_x = pos_base[0] - new_pos_xy[0]
                    shift_y = pos_base[1] - new_pos_xy[1]
                                       
                    resulting_x = new_x + shift_x
                    resulting_y = new_y + shift_y
                    
                    new_marker_position = Marker_Position(marker_id, (resulting_x,resulting_y), (phi,(shift_x,shift_y)), new_distance)
                    
                    self.marker_positions[marker_id] = new_marker_position
                         
                        
                else:
                    # The base marker is no longer visible so the calculation is done
                    # via intermediate markers
                    
                    # Get the data of the new perceived marker
                    new_pos_xy, new_distance, new_angle = self.get_marker_xy(current_marker)
                    
                    # Get the data of the intermediate marker, perceived from the new position
                    interim_marker_pos = self.marker_positions.values()[0]
                    pos_base, dist_base, ang_base = self.get_marker_xy(self.persistent_markers[interim_marker_pos.get_marker_id()])
                    
                    phi_to_base = interim_marker_pos.get_shift_xy()[0]
                    trans_to_base = interim_marker_pos.get_shift_xy()[1]
                    
                    phi = ang_base - new_angle - phi_to_base
                    
                    new_x = new_pos_xy[0] * np.cos(phi) + new_pos_xy[1] * np.sin(phi)
                    new_y = new_pos_xy[0] * np.sin(phi) + new_pos_xy[1] * np.cos(phi)
                    
                    shift_x = pos_base[0] - new_pos_xy[0] - trans_to_base[0]
                    shift_y = pos_base[1] - new_pos_xy[1] - trans_to_base[1]
                                       
                    resulting_x = new_x + shift_x
                    resulting_y = new_y + shift_y
                    
                    new_marker_position = Marker_Position(marker_id, (resulting_x,resulting_y),(phi,(shift_x,shift_y)), new_distance)
                    
                    self.marker_positions[marker_id] = new_marker_position
                         
                        
        
        for marker in self.marker_positions.values():
            
            pos_x = marker.get_pos_xy()[0]
            pos_y = marker.get_pos_xy()[1]
            
            cv2.circle(cv_image, (int(scale_factor*pos_x)+shift_factor,int(scale_factor*pos_y)+shift_factor),2,(255,0,0),2)
        
        # print(self.persistent_markers.keys())
        cv2.imshow('topView', cv_image)
        cv2.moveWindow('topView', 700, 0)

    def get_marker_xy(self, marker):
        distance = aruco_data.get_distance(marker)
        angle = aruco_data.get_angle_surface(marker)
        # Now we can get the dx and dy of movement
        xy = cv2.polarToCart(distance, angle)
        return tuple((xy[0][0][0], xy[1][0][0])), distance, angle

if __name__ == "__main__":
    visualizer = Area_Visualizer()
    bagfile_path = sys.argv[1]
    
    bagfile_handler = Bagfile_Handler(bagfile_path)
    paused_video = False
    while(True):
        if not paused_video:
            cv_image = bagfile_handler.get_image()
        
        
            markers = aruco_data.get_markers_in_image(cv_image)
            visualizer.show_top_view(markers)
        
        cv2.imshow("video", cv_image)
        
        key = cv2.waitKey(1000 / 30) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            paused_video = not paused_video
        if key == ord('w'):
            bagfile_handler.fast_forward()
