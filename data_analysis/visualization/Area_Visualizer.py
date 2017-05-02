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


class Area_Visualizer(object):
    
    persistent_markers = {}
    base_x = 0.0
    base_y = 0.0 
    encountered_first_time = True
    #map = Map()
        
    def __init__(self):
        pass      

    def visualize_markers_center_line(self,markers):
        img2 = np.ones((600,600,3), np.uint8)
        
        # This factor is rather arbitrary. The x,y results of the following
        # calculation are approximately in between 0 and 8 and accordingly this
        # factor is chosen
        scale_factor = 300.0 * (1.0/8.0)
        shift_factor = 300
        turn_factor = np.pi/2.0 # division by int gives integer
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence-0.1
        
        # Add markers to list, overwriting old markers and thereby increasing confidence levels
        for marker in markers:
            self.persistent_markers[str(marker.id)]=marker
        
        
        # Now draw lines onto new window
        for marker_id in self.persistent_markers:
            
            marker = self.persistent_markers[marker_id] 
            
            # Draw marker outline at the bottom of the screen
            xy1 = marker.corners_xy_pos[0]
            xy2 = marker.corners_xy_pos[1]

            cv2.line(img2,(xy1[0],xy1[1]+shift_factor),(xy2[0],xy2[1]+shift_factor),(0,0,255*marker.confidence),1)

            distance = marker.corners_distances_angles['distance']
            angle = marker.corners_distances_angles['angle']
            
            # Draw top view outline
            x_a,y_a = cv2.polarToCart(distance,angle-turn_factor)
            x_a[0] = x_a[0] * scale_factor + shift_factor
            y_a[0] = y_a[0] * scale_factor + shift_factor
            cv2.circle(img2,(x_a[0],y_a[0]),2,(0,0,255*marker.confidence),-1)
                
            #ircle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
                
            # Draw outer viewport lines
            
            fov = np.deg2rad(110.0)
                
            x_orig,y_orig = cv2.polarToCart(0.0,(-fov/2.0)-turn_factor)
            x_dest,y_dest = cv2.polarToCart(8.0,(-fov/2.0)-turn_factor)
            x_orig[0] = x_orig[0] * scale_factor + shift_factor
            y_orig[0] = y_orig[0] * scale_factor + shift_factor
            x_dest[0] = x_dest[0] * scale_factor + shift_factor
            y_dest[0] = y_dest[0] * scale_factor + shift_factor
            
            cv2.line(img2,(x_orig[0],y_orig[0]),(x_dest[0],y_dest[0]),(255,255,255),1)
            
            x_orig_max,y_orig_max = cv2.polarToCart(0.0,(fov/2.0)-turn_factor)
            x_dest_max,y_dest_max = cv2.polarToCart(8.0,(fov/2.0)-turn_factor)
            x_orig_max[0] = x_orig_max[0] * scale_factor + shift_factor
            y_orig_max[0] = y_orig_max[0] * scale_factor + shift_factor
            x_dest_max[0] = x_dest_max[0] * scale_factor + shift_factor
            y_dest_max[0] = y_dest_max[0] * scale_factor + shift_factor
            
            cv2.line(img2,(x_orig_max[0],y_orig_max[0]),(x_dest_max[0],y_dest_max[0]),(255,255,255),1)
            
            
            
                           
            # map test code
            # This is one of the worst places where the code can be
            # put though this is highly experimental and here I have all
            # the needed information handily available
            
            #for marker in markers:
            #    self.map.addMarker(marker_id,distance,angle)
            
            # map test code
                
            
            
            
            
        cv2.imshow('topView',img2)
        cv2.moveWindow('topView',700,0)
    
   
    
    def show_top_view(self, markers):
        
        cv_image = np.ones((600,600,3), np.uint8)
        
        # Some arbitrary scaling parameters for visualisation are set
        scale_factor = 300.0 * (1.0/8.0)
        shift_factor = 300
        turn_factor = np.pi/2.0 
        
        # Reduce confidence for each marker in the persistent list, if there are any yet
        for marker_id in self.persistent_markers:
            self.persistent_markers[marker_id].confidence = self.persistent_markers[marker_id].confidence-0.1
        
        # Add markers to list, overwriting old markers and thereby increasing confidence levels
        for marker in markers:
            self.persistent_markers[str(marker.marker_id)] = marker
        # This needs eventual rethinking because the more closer a marker is, the higher its
        # confidence or probability it is there is.
        
        # Now draw lines onto new window
        #for marker_id in self.persistent_markers:
        #    
        #    marker = self.persistent_markers[marker_id] 
        #    
        #    distance = aruco_data.get_distance(marker)
        #    angle = aruco_data.get_angle_surface(marker)
        #    
        #    naiv_xy = cv2.polarToCart(distance, angle)
        #    
        #    cv2.circle(cv_image, (scale_factor*naiv_xy[0][0],scale_factor*naiv_xy[1][0]),2,(0,0,255),2)
        
        
        # See a marker for the first time, lets say 67
        marker_id = str(67)
        if marker_id in self.persistent_markers and self.encountered_first_time:
            print("Saw marker for the first time")
            marker = self.persistent_markers[marker_id]
            self.encountered_first_time = False
            
            # This marker has a distance and angle
            distance = aruco_data.get_distance(marker)
            angle = aruco_data.get_angle_surface(marker)
            # and in that original reference frame also position x,y
            orig_xy = cv2.polarToCart(distance, angle)
            
            # We assume this is the basis of all future calculations
            self.base_x = orig_xy[0][0]
            self.base_y = orig_xy[1][0]
            
            # from now on, all future marker sightings of this marker mean a movement of the own vehicle
        elif marker_id in self.persistent_markers and not self.encountered_first_time:
            marker = self.persistent_markers[marker_id]            
           
            distance = aruco_data.get_distance(marker)
            angle = aruco_data.get_angle_surface(marker)
            # Now we can get the dx and dy of movement
            new_xy = cv2.polarToCart(distance, angle)
            
            dx = new_xy[0][0]-self.base_x
            dy = new_xy[1][0]-self.base_y
            
            # plot to see this
            cv2.circle(cv_image, (scale_factor*dx+shift_factor,scale_factor*dy+shift_factor),2,(0,0,255),2)
            
        #print(self.persistent_markers.keys())
        cv2.imshow('topView',cv_image)
        cv2.moveWindow('topView',700,0)
        
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
        
        cv2.imshow("video",cv_image)
        
        key = cv2.waitKey(1000/30) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            paused_video = not paused_video
        if key == ord('w'):
            bagfile_handler.fast_forward()
