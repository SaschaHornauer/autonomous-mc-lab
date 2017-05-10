'''
Created on May 8, 2017

@author: picard
'''
from omgtools import *
from kzpy3.data_analysis.trajectory_generator.trajectory_tools import *
import cv2
import sys
import matplotlib.pyplot as plt


framerate = 1./30.
# This framerate should be at one point for the whole module

def get_state(own_xy, timestep_start):
     
    # Init our own position
    init_xy = own_xy[timestep_start]
         
    # Get our intended heading based on three timesteps in the future
    # which add as slight smoothing
    heading = get_heading(own_xy[timestep_start:timestep_start+3])
    # Get the velocity over the first two timesteps
    velocity = get_velocities([own_xy[timestep_start],own_xy[timestep_start+1]], framerate)
    return init_xy,heading,velocity
    
def get_evasive_trajectory(own_xy,other_xy,timestep_start, process_timesteps, d_timestep_goal):
    import time
    
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given in between timestep_start and timestep_ahead.
    Note that the trajectory will start one timestep further than timestep_start because
    one timestep is needed to retrieve information about the start conditions
     
    FOR NOW, OTHER_XY IS ONLY ONE OTHER VEHICLE    
    '''

    
    init_xy_own, heading_own, velocity_own = get_state(own_xy,timestep_start)
    init_xy_other, heading_other, velocity_init_other = get_state(other_xy,timestep_start)
      
    goal_xy = own_xy[timestep_start+d_timestep_goal] # The goal is our future position in the
    # dataset 
      
    # make and set-up vehicle
    vehicle = Holonomic(shapes=Circle(0.25))
      
    # plan from the last known position
    vehicle.set_initial_conditions(init_xy_own,velocity_own)
     
    # plan as if the current movement should be continued 
    vehicle.set_terminal_conditions(goal_xy)
    vehicle.set_options({'safety_distance': 0.5})
    vehicle.set_options({'ideal_prediction': False})
      
    # make and set-up environment #TODO
    environment = Environment(room={'shape': Square(10.)})
   
    # create a point-to-point problem
    problem = Point2point(vehicle, environment, freeT=False)
    problem.init()
     
    # get velocities of other vehicles
    other_positions = other_xy[timestep_start:timestep_start+process_timesteps]
    other_velocities = get_velocities(other_positions, 1./30.)

    traj = {'position': {'time': np.linspace(0.,process_timesteps*(1./30.),process_timesteps-1),
                         'values': other_velocities}}
 
    environment.add_obstacle(Obstacle({'position': init_xy_other}, shape=Circle(0.25),
        simulation={'trajectories': traj}))
       
    # give problem settings and create problem
    problem = Point2point(vehicle, environment)
    problem.init()
       
    # simulate, plot some signals and save a movie
    simulator = Simulator(problem, sample_time=0.33, update_time=0.33)
    
    vehicle.plot('input', labels=['v_x (m/s)', 'v_y (m/s)'])
    problem.plot('scene')
    trajectories, signals = simulator.run()


if __name__ == '__main__':

    pass
    