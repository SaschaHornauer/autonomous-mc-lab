'''
Created on May 8, 2017

@author: picard
'''
from omgtools import *
from kzpy3.data_analysis.trajectory_generator.trajectory_tools import *
import cv2
import sys
framerate = 1./30.
# This framerate should be at one point for the whole module

def get_state(own_xy, timestep_start):
     
    # Init our own position
    print(len(own_xy))
    print(timestep_start)
    init_xy = own_xy[timestep_start]
         
    # Get our intended heading based on three timesteps in the future
    # which add as slight smoothing
    heading = get_heading(own_xy[timestep_start:timestep_start+3])
    print (own_xy[timestep_start:timestep_start+10])
    # Get the velocity over the first two timesteps
    velocity = get_velocities([own_xy[timestep_start],own_xy[timestep_start+1]], framerate)
    return init_xy,heading,velocity
    
def get_evasive_trajectory(own_xy,other_xy,timestep_start, timesteps_ahead, distance_ahead):
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given in between timestep_start and timestep_ahead.
    Note that the trajectory will start one timestep further than timestep_start because
    one timestep is needed to retrieve information about the start conditions
     
    FOR NOW, OTHER_XY IS ONLY ONE OTHER VEHICLE    
    '''
     
    init_xy_own, heading_own, velocity_own = get_state(own_xy,timestep_start)
     
    goal_xy = project_pos(init_xy_own, heading_own, distance_ahead) # The goal is distance_ahead m in front of us 
     
    # make and set-up vehicle
    vehicle = Holonomic(shapes=Circle(0.1))
     
    # plan from the last known position
    vehicle.set_initial_conditions(init_xy_own,velocity_own)
    
    # plan as if the current movement should be continued 
    vehicle.set_terminal_conditions(goal_xy)
    vehicle.set_options({'safety_distance': 0.5})
     
    # make and set-up environment #TODO
    environment = Environment(room={'shape': Square(12.)})
    #vel_values = get_velocities(other_xy,1./30.)
    # generate trajectory for moving obstacle
    # TODO need to relate the seconds here to our framerate / movement
    #traj = {'velocity': {'time': np.linspace(0.,timesteps_ahead*(1./30.),timesteps_ahead-1),
    #                     'values': vel_values}}
    # TODO check if the values here are in the correct form [x,y],[x+1,y+1],...
    # TODO chekf it more than 1 value is possible
     
    # add moving obstacle to environment
    # TODO Change size to a reasonable limit
    #init_other_xy = other_xy[0]
    #environment.add_obstacle(Obstacle({'position': init_other_xy}, shape=Circle(0.4),
    #    simulation={'trajectories': traj}))
      
    # give problem settings and create problem
    problem = Point2point(vehicle, environment)
    problem.init()
      
    # simulate, plot some signals and save a movie
    simulator = Simulator(problem)
    vehicle.plot('input', labels=['v_x (m/s)', 'v_y (m/s)'])
    problem.plot('scene')
    trajectories, signals = simulator.run()


if __name__ == '__main__':

    pass
    