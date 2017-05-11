'''
Created on May 8, 2017

@author: picard
'''

import numpy as np
import cv2

def get_heading(seq_xy):
    '''
    Give an average heading over all positions. Two different positions are 
    the minimum, more will return a smoothed angle
    '''

    diffsX = []
    diffsY = []
    
    # calculate the angle:
    for i in range(0,len(seq_xy)-1):
        diffsX.append(seq_xy[i+1][0]-seq_xy[i][0])
        diffsY.append(seq_xy[i+1][1]-seq_xy[i][1])
    
    myAngle = np.arctan2(diffsY, diffsX)
    
    return np.mean(myAngle)


def project_pos(xy,current_heading, distance=2):
    '''
    Take the position as xy and the current heading and returns future
    xy in a distance of distance
    '''
    
    x,y = cv2.polarToCart(distance,current_heading)
    
    return [x[0][0]+xy[0],y[0][0]+xy[1]]


def get_velocities(xy_positions,framerate):
    '''
    Returns the diffs in between two x,y positions divided by the
    framerate aka velocity in m/s since the distance in time between two
    positions is exactly that framerate
    '''
    
    velocities = []
    
    for i in range(1,len(xy_positions)):
        x = xy_positions[i-1][0]
        y = xy_positions[i-1][1]
        x_t1 = xy_positions[i][0]
        y_t1 = xy_positions[i][1]
    
        sx = x_t1 - x
        sy = y_t1 - y
        
        vx = sx / framerate
        vy = sy / framerate
        velocities.append([vx,vy])
        
    
    return velocities


def get_steering_motor_cmd(value_steer,value_motor,max_range_steer,min_range_steer,max_motor,min_motor):

    max_left_steering_angle = np.deg2rad(-90)
    max_right_steering_angle = np.deg2rad(90)
    
    max_left_command = 100
    max_right_command = 0
    
    left_range = 50
    right_range = 50

    max_motor = 60
    min_motor = 49  # Full stop. Backwards is not considered

    front_left_limit_deg = -90
    front_right_limit_deg = 90

    if(average_angle != None):   
        print("average angle " + str(np.rad2deg(average_angle)))
        opposite_angle = ((average_angle + np.pi) + np.pi) % (2 * np.pi) - np.pi 
        
        mid_steering_command = np.abs(max_right_command - max_left_command) / 2.0
        
        if opposite_angle < 0:
            steering_command = (opposite_angle / np.pi) * left_range                               
        else:
            steering_command = (opposite_angle / np.pi) * right_range 
        
        # Finally change the mapping from -50,50 to 0,100
        try:
            steering_command = (steering_command + mid_steering_command)[0]
        except:
            steering_command = (steering_command + mid_steering_command)
        
        
        # A special behaviour is investigated. This is a test
        # There is a list of steering commands. When this 
        # list is still empty, resp. the index is 0
        
        #print(steering_command)
        #print(incoming_steering_cmd)
        # Now interpolate the intermediate values from the current steering towards that command
        # in as many steps as the list is long
        increment = np.abs(incoming_steering_cmd-steering_command)/self.steering_command_length
        # Finally fill the list with those values. 
        self.steering_command_list = self.steering_command_list*increment
            
    if not 'motor_command' in vars():
        motor_command = incoming_motor_cmd
    if not 'steering_command' in vars():
        steering_command = incoming_steering_cmd
    

    return motor_command, steering_command

### Approach by Martin Thoma https://martin-thoma.com/author/martin-thoma/
class Point:
    """Represents a two dimensional point."""

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __get__(self, obj, cls=None):
        return obj

    def __repr__(self):
        return "P(%.2lf|%.2lf)" % (self.x, self.y)

    def __str__(self):
        return repr(self)
    
def to_point(xy):
    # Convenience method to convert an arbitrary tuple/array to a point
    return Point(xy[0],xy[1])

class Triangle:
    """Represents a triangle in R^2."""

    epsilon = 0.001

    def __init__(self, a, b, c):
        assert isinstance(a, Point)
        assert isinstance(b, Point)
        assert isinstance(c, Point)
        self.a = a
        self.b = b
        self.c = c

    def getArea(self):
        """Get area of this triangle.
           >>> Triangle(Point(0.,0.), Point(10.,0.), Point(10.,10.)).getArea()
           50.0
           >>> Triangle(Point(-10.,0.), Point(10.,0.), Point(10.,10.)).getArea()
           100.0
        """
        a, b, c = self.a, self.b, self.c
        return abs(a.x*(b.y-c.y)+b.x*(c.y-a.y)+c.x*(a.y-b.y))/2

    def isInside(self, p):
        """Check if p is inside this triangle."""
        assert isinstance(p, Point)
        currentArea = self.getArea()
        pab = Triangle(p,self.a, self.b)
        pac = Triangle(p,self.a, self.c)
        pbc = Triangle(p,self.b, self.c)
        newArea = pab.getArea()+pac.getArea()+pbc.getArea()
        return (abs(currentArea - newArea) < Triangle.epsilon)

