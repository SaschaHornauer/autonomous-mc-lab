'''
Created on Apr 12, 2017

@author: Sascha Hornauer
'''
import numpy as np
import sys
import cv2
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
        __marker_id = marker_id
        __pos_xy = pos_xy
        __shift_xy = shift_xy
        __aquired_at_distance = distance
        
    def get_distance(self):
        return self.__aquired_at_distance
        
    

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
        
        cv_image = np.ones((600, 600, 3), np.uint8)
        
        # Some arbitrary scaling parameters for visualisation are set
        scale_factor = int(300.0 * (1.0 / 8.0))
        shift_factor = 300
        turn_factor = np.pi / 2.0 
        current_marker_ids = []
        current_xy = (0,0)
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence - 0.1
        
        # Add markers to list, overwriting old markers and thereby recalculating confidence levels
        for marker in markers:
            self.persistent_markers[str(marker.marker_id)] = marker
            current_marker_ids.append(marker.marker_id)
        # This needs eventual rethinking because the more closer a marker is, the higher its
        # confidence or probability it is there is. Also two for loops here are maybe not necessary

        for marker_id in self.persistent_markers:

            current_marker = self.persistent_markers[marker_id]

            if not self.base_marker_found:
                self.base_marker_id = marker_id
                self.base_marker_found = True
            
                orig_xy, orig_distance, _= self.get_marker_xy(current_marker)
            
                # We assume this is the basis of all future calculations
                self.base_x = orig_xy[0]
                self.base_y = orig_xy[1]
                
                # That base marker position is added with shift 0,0 because it is the reference
                self.marker_positions[marker_id] = Marker_Position(marker_id, orig_xy, (0, 0), orig_distance)
            
            
            elif marker_id != self.base_marker_id:
                # if the base marker is already found and we are not right now looking
                # at it, we check if the base marker is still visible
                if self.base_marker_id in current_marker_ids:
                    # do shift calculations according to base marker
                    
                    new_pos_xy, new_distance, _ = self.get_marker_xy(current_marker)
                    shift_x = new_pos_xy[0] - self.base_x
                    shift_y = new_pos_xy[1] - self.base_y
                    new_marker_position = Marker_Position(marker_id, new_pos_xy, (shift_x, shift_y), new_distance)
                    
                    if marker_id in self.marker_positions.keys():
                        # If the shift was already calculated then check at which distance and
                        # update if the distance is smaller
                        existing_position = self.marker_positions[marker_id]
                        if existing_position.get_distance() > new_distance:
                            self.marker_positions[marker_id] = new_marker_position
                        
                else:
                    # do more complicated calculations based on other shift values
                    pass
            else:
                # The base marker is already found and the marker_id is the self marker so we
                # take the position directly
                current_xy, _, _ = self.get_marker_xy(current_marker) 
                
            pos_x = current_xy[0]
            pos_y = current_xy[1]
            print (current_xy)
            cv2.circle(cv_image, (int(scale_factor*pos_x+shift_factor),int(scale_factor*pos_y+shift_factor)),2,(0,0,255),2)
            
            # from now on, all future marker sightings of this marker mean a movement of the own vehicle
#         elif marker_id in self.persistent_markers and not self.base_marker_found:
#             marker = self.persistent_markers[marker_id]            
#            
#             distance = aruco_data.get_distance(marker)
#             angle = aruco_data.get_angle_surface(marker)
#             # Now we can get the dx and dy of movement
#             new_xy = cv2.polarToCart(distance, angle)
#             
#             dx = new_xy[0][0]-self.base_x
#             dy = new_xy[1][0]-self.base_y
#             
#             # plot to see this
#             cv2.circle(cv_image, (scale_factor*dx+shift_factor,scale_factor*dy+shift_factor),2,(0,0,255),2)
#         
#             marker_id = str(59)
#             marker = self.persistent_markers[marker_id]            
#                
#             distance = aruco_data.get_distance(marker)
#             angle = aruco_data.get_angle_surface(marker)
#             
#             marker_two_xy = cv2.polarToCart(distance, angle)
#             # Now we have the position of us, according to that second marker
#             # It is now possible to calculate the shift in between the two markers
#             
#             self.shift_x = new_xy[0][0] - marker_two_xy[0][0]
#             self.shift_y = new_xy[1][0] - marker_two_xy[1][0]
#             
#             # This is the value, all of the positions relative to the second marker are shifted.
#             # To plot this, and the inaccuracy, we will plot first the new calculated position
#             cv2.circle(cv_image, (scale_factor*marker_two_xy[0][0]+shift_factor,scale_factor*marker_two_xy[1][0]+shift_factor),2,(0,255,0),2)
#             # And the corrected first position according to the second marker
#             
#             
#             
#             print self.get_marker_xy(marker)
        # cv2.circle(cv_image, (scale_factor*marker_two_xy[0][0]+shift_factor,scale_factor*marker_two_xy[1][0]+shift_factor),2,(255,0,0),2)
        
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
