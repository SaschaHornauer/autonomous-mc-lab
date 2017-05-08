'''
Created on May 8, 2017

@author: picard
'''
from omgtools import *
import cv2

    
#     # make and set-up vehicle
#     vehicle = Holonomic()
#     vehicle.set_initial_conditions([-1.0, -1.5])
#     vehicle.set_terminal_conditions([-1.25, -1.25])
#     vehicle.set_options({'safety_distance': 0.1})
#     
#     # make and set-up environment
#     environment = Environment(room={'shape': Square(5.)})
#     
#     # add stationary obstacles to environment
#     rectangle = Rectangle(width=3., height=0.2)
#     environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
#     environment.add_obstacle(Obstacle({'position': [ 1.7, -0.5]}, shape=rectangle))
#     
#     # generate trajectory for moving obstacle
#     traj = {'velocity': {'time': [3., 4.],
#                          'values': [[-0.15, 0.0], [0., 0.15]]}}
#     # add moving obstacle to environment
#     environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
#         simulation={'trajectories': traj}))
#     
#     # give problem settings and create problem
#     problem = Point2point(vehicle, environment)
#     problem.init()
#     
#     # simulate, plot some signals and save a movie
#     simulator = Simulator(problem)
#     vehicle.plot('input', labels=['v_x (m/s)', 'v_y (m/s)'])
#     problem.plot('scene')
#     trajectories, signals = simulator.run()
#     print(trajectories['state'][len(trajectories)])


def get_evasive_trajectory(own_xy,other_xy,timesteps_ahead):
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given as variable.
    
    Own_xy is the own trajectory, in x,y tuples, up to the point of calculation
    other_xy can be a list of x,y tuples or a list of list for each other vehicle
    
    '''
    # make and set-up vehicle
    vehicle = Holonomic()
    
    # plan from the last known position
    init_xy = own_xy[len(own_xy)]
    vehicle.set_initial_conditions([init_xy[0], init_xy[1]])
    
    # plan as if the current movement should be continued 
    
    vehicle.set_terminal_conditions([-1.25, -1.25])
    vehicle.set_options({'safety_distance': 0.1})
    
    
    
    pass


def get_heading(seq_xy, timesteps=3):
    '''
    Get over the last [timesteps] a mean heading in rad
    '''

    diffsX = []
    diffsY = []
    
    # calculate the angle:
    for i in range(len(seq_xy)-timesteps,len(seq_xy)-1):
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
    return [x[0]+xy[0],y[0]+xy[1]] 
   
if __name__ == '__main__':
    print(project_pos((3,0),np.pi/3.))
    