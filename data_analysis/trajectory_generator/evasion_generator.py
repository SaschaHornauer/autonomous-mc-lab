'''
Created on May 8, 2017

@author: picard
'''
from omgtools import *
from kzpy3.data_analysis.trajectory_generator.trajectory_tools import *
import cv2


def get_evasive_trajectory(own_xy,other_xy,timesteps_ahead, distance_ahead):
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given as variable.
    
    Own_xy is the own trajectory, in x,y tuples, up to the point of calculation
    other_xy can be a list of x,y tuples or a list of list for each other vehicle
    
    '''
    # Init our own position
    init_xy = own_xy[len(own_xy)-1]
    
    # Get the goal where we want to go
    heading = get_heading(own_xy[len(own_xy)-3:len(own_xy)])
    goal_xy = project_pos(init_xy, heading, distance_ahead) # The goal is distance_ahead m in front of us 
    
    # make and set-up vehicle
    vehicle = Holonomic()
    
    # plan from the last known position
    vehicle.set_initial_conditions([init_xy[0], init_xy[1]])
    
    # plan as if the current movement should be continued 
    vehicle.set_terminal_conditions(goal_xy)
    vehicle.set_options({'safety_distance': 0.5})
    
    # make and set-up environment #TODO
    environment = Environment(room={'shape': Square(10.)})
    vel_values = get_velocities(other_xy,1./30.)
    # generate trajectory for moving obstacle
    # TODO need to relate the seconds here to our framerate / movement
    traj = {'velocity': {'time': np.linspace(0.,timesteps_ahead*(1./30.),timesteps_ahead-1),
                         'values': vel_values}}
    # TODO check if the values here are in the correct form [x,y],[x+1,y+1],...
    # TODO chekf it more than 1 value is possible
    
    # add moving obstacle to environment
    # TODO Change size to a reasonable limit
    init_other_xy = other_xy[0]
    environment.add_obstacle(Obstacle({'position': init_other_xy}, shape=Circle(0.4),
        simulation={'trajectories': traj}))
     
    # give problem settings and create problem
    problem = Point2point(vehicle, environment)
    problem.init()
     
    # simulate, plot some signals and save a movie
    simulator = Simulator(problem)
    vehicle.plot('input', labels=['v_x (m/s)', 'v_y (m/s)'])
    problem.plot('scene')
    trajectories, signals = simulator.run()

def get_velocities(other_xy,framerate):
    
    velocities = []
    
    for i in range(1,len(other_xy)):
        x = other_xy[i-1][0]
        y = other_xy[i-1][1]
        x_t1 = other_xy[i][0]
        y_t1 = other_xy[i][1]
    
        sx = x_t1 - x
        sy = y_t1 - y
        
        vx = sx / framerate
        vy = sy / framerate

        velocities.append([vx,vy])
        
    
    return velocities
if __name__ == '__main__':
    pass
    