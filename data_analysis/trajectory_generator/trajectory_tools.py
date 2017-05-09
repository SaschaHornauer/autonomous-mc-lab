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

