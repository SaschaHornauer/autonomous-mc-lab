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

def get_state(own_xy, timestep_start,timestep_end):
     
    # Init our own position
    init_xy = own_xy[timestep_start]
         
    # Get our intended heading based on three timesteps in the future
    # which add as slight smoothing
    heading = get_heading(own_xy[timestep_start:timestep_start+timestep_end])
    # Get the velocity over the first two timesteps
    velocity = get_velocities([own_xy[timestep_start],own_xy[timestep_start+1]], framerate)
    return init_xy,heading,velocity


def get_evasive_trajectory(own_xy,other_xy,timestep_start, process_timesteps, d_timestep_goal=None):
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given in between timestep_start and timestep_ahead.
    Note that the trajectory will start one timestep further than timestep_start because
    one timestep is needed to retrieve information about the start conditions
     
    FOR NOW, OTHER_XY IS ONLY ONE OTHER VEHICLE    
    '''
    
    if d_timestep_goal == None:
        d_timestep_goal = process_timesteps 
    
    init_xy_own, heading_own, velocity_own = get_state(own_xy,timestep_start,3) # smooth heading over 3 timesteps in the future
    
    goal_xy = own_xy[timestep_start+d_timestep_goal-1] # The goal is our future position in the dataset
    # Now take the last three timesteps near the goal to calculate the final heading
    goal_heading = get_heading(own_xy[timestep_start+d_timestep_goal-3:timestep_start+d_timestep_goal]) 
      
    # make and set-up vehicle
    vehicle = Bicycle(length=0.4, options={'plot_type': 'car', 'substitution': False})# Holonomic(shapes=Circle(0.25))
    vehicle.define_knots(knot_intervals=5)
    
    velocity_abs = np.hypot(velocity_own[0][0],velocity_own[0][1])
    
    # plan from the last known position
    vehicle.set_initial_conditions(state=[init_xy_own[0],init_xy_own[1], heading_own, 0.0],input=[velocity_abs]) # the assumption is that 
    # for the time being that the steering angle is 0. This can be changed in the future, based on existing data.

    # plan as if the current movement should be continued
    
    vehicle.set_terminal_conditions([goal_xy[0],goal_xy[1],goal_heading])
    vehicle.set_options({'safety_distance': 1.0})
    
    # make and set-up environment #TODO
    environment = Environment(room={'shape': Square(15.)})
    
    # get velocities of other vehicles
    other_positions = other_xy[timestep_start:timestep_start+process_timesteps]
    
    other_velocities = get_velocities(other_positions, 1./30.)


    #for other_trajectory in other_xy:
        # TODO ADD INITIAL VELOCITY OBSTACLE
        
    
    init_xy_other, heading_other, velocity_init_other = get_state(other_xy,timestep_start,3)
    
    traj = {'position': {'time': np.linspace(0.,process_timesteps*(1./30.),process_timesteps-1),
                     'values': other_velocities}}
    print other_velocities
    environment.add_obstacle(Obstacle({'position': np.array(init_xy_other)}, shape=Circle(0.25),
        simulation={'trajectories': traj}))
    
    #print(np.array(init_xy_other))
    #sys.exit(0)
    # create a point-to-point problem
    problem = Point2point(vehicle, environment, freeT=True)
    
    #problem.set_options({'solver_options':
    #{'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})
    
    problem.init()
     
    # simulate, plot some signals and save a movie
    simulator = Simulator(problem, sample_time=1./30., update_time=1./30.)
    
    #vehicle.plot('input', labels=['v_x (m/s)', 'v_y (m/s)'])
    #problem.plot('scene')
    problem.plot('scene')
    vehicle.plot('input', knots=True, labels=['v (m/s)', 'ddelta (rad/s)'])
    vehicle.plot('state', knots=True, labels=[
                 'x (m)', 'y (m)', 'theta (rad)', 'delta (rad)'])
    if vehicle.options['substitution']:
        vehicle.plot('err_pos', knots=True)
        vehicle.plot('err_dpos', knots=True)

    steering_deltas = []

    for i in range(0,d_timestep_goal-1):
        simulator.update()
        steering_deltas.append(vehicle.traj_storage['delta'][i][0][0])
        
    steering_deltas

if __name__ == '__main__':

    pass
    