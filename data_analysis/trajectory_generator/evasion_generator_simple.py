'''
Created on May 8, 2017

@author: picard
'''
from omgtools import *
from kzpy3.data_analysis.trajectory_generator.trajectory_tools import *
import cv2
import sys
import matplotlib.pyplot as plt


framerate = 1. / 30.

# This framerate should be at one point for the whole module

def get_state(own_xy, timestep_start, timestep_end):

    # Init our own position
    init_xy = own_xy[timestep_start]
         
    # Get our intended heading based on three timesteps in the future
    # which add as slight smoothing
    heading = get_heading(own_xy[timestep_start:timestep_start + timestep_end])
    # Get the velocity over the first two timesteps
    velocity = get_velocities([own_xy[timestep_start], own_xy[timestep_start + 1]], framerate)
    return init_xy, heading, velocity

def sample_result_to_trajectory(values, no_of_samples):
    
    if len(values) == no_of_samples:
        return values
    if len(values) < no_of_samples:
        raise NotImplementedError("The case that there are less trajectory points has not been implemented")
    else:
        n = len(values)
        m = no_of_samples
        # substract one for the 0 value which we want to have
        index = [i * n // m + n // (2 * m) for i in range(m)]
        index.pop(len(index) - 1)
        index.insert(0, 0)
        
        return values[index]
    

def get_evasive_trajectory(own_xy, other_xy, timestep_start, plan_horizon, d_timestep_goal, framenumber, plot_video):
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given in between timestep_start and timestep_ahead.
    Note that the trajectory will start one timestep further than timestep_start because
    one timestep is needed to retrieve information about the start conditions
     
    FOR NOW, OTHER_XY IS ONLY ONE OTHER VEHICLE    
    '''
    safety_distance = 0.2
    
    init_xy_own, heading_own, velocity_own = get_state(own_xy, timestep_start, 4)  # smooth heading over 3 timesteps in the future

    goal_xy, goal_heading, goal_velocity = get_state(own_xy, timestep_start + d_timestep_goal, 4) 
    
    init_xy_other, heading_other, velocity_init_other = get_state(other_xy, timestep_start, 4)
    end_xy_other, end_heading_other, velocity_end_other = get_state(other_xy, timestep_start + plan_horizon - 2, timestep_start + plan_horizon - 1)
    
    angle_of_obstacle = np.arctan2(init_xy_other[1]-init_xy_own[1],init_xy_other[0]-init_xy_own[0])
    
    distance_own_obstacle = np.hypot(init_xy_other[0]-init_xy_own[0],init_xy_other[1]-init_xy_own[1])
    
    if not distance_own_obstacle < safety_distance + 0.7:
            
        # make and set-up vehicle
        vehicle = HolonomicOrient(bounds={'wmax': np.pi/6., 'wmin': -np.pi/6.},options={'plot_type': 'car'}) 
        vehicle.define_knots(knot_intervals=5)
        
        # A fix is needed because the angle for HolonomicOrient seems to be in a different system
        #heading_own = heading_own + np.pi/2.
        # plan from the last known position
        vehicle.set_initial_conditions(state=[init_xy_own[0], init_xy_own[1], heading_own])  
        # SIMPLIFICATION: The desired goal heading is our current heading. The goal is going straight
        vehicle.set_terminal_conditions([goal_xy[0], goal_xy[1], goal_heading])
        vehicle.set_options({'safety_distance': safety_distance})
        
        environment = Environment(room={'shape': RegularPolyhedron(4.28, 24), 'draw': False})
        
        # get velocities of other vehicles
        # other_positions = other_xy[timestep_start:timestep_start+process_timesteps]
        
        obstacle_start_time = 0.
        obstacle_end_time = plan_horizon * (1. / 30.)
    
        traj = {'velocity': {'time': [obstacle_start_time, obstacle_end_time],
                         'values': [velocity_init_other, velocity_end_other]}}
        # sys.exit(0)
        environment.add_obstacle(Obstacle({'position': np.array(init_xy_other)}, shape=Circle(0.25),
            simulation={'trajectories': traj}))
        
        # create a point-to-point problem
        # FreeEndPoint2point(veh, environment.copy(), options, {veh: free_ind}))
        problem = Point2point(vehicle, environment, freeT=False)
        
        problem.set_options({'solver_options':
        {'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})
        
        problem.init()
         
        # simulate, plot some signals and save a movie
        simulator = Simulator(problem, sample_time=1. / 30., update_time=1. / 30.)
        
        if plot_video:
        
            plt.savefig("scene" + "_" + str(framenumber) + ".png")
            problem.plot('scene')
            #plt.close("all")
            #vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)', 'w (rad/s)'])
            #vehicle.plot('state', knots=True, labels=['x (m)', 'y (m)', 'theta (rad)'])
    #     vehicle.plot('input', knots=True, labels=['v (m/s)', 'ddelta (rad/s)'])
    #    vehicle.plot('state', knots=True, labels=[
    #                  'x (m)', 'y (m)', 'theta (rad)', 'delta (rad)'])
    #     if vehicle.options['substitution']:
    #         vehicle.plot('err_pos', knots=True)
    #         vehicle.plot('err_dpos', knots=True)
    
        
        # Right now a very simple behaviour will only calculate the 
        # trajectory once    
        simulation_steps = 1
            
        simulator.deployer.reset()
        
        for i in range(0, simulation_steps):
            simulator.update()
            simulator.update_timing()
        
        # return trajectories and signals
        trajectories, signals = {}, {}
        for vehicle in simulator.problem.vehicles:
            trajectories[str(vehicle)] = vehicle.traj_storage
            signals[str(vehicle)] = vehicle.signals
        
        
        
        if distance_own_obstacle < safety_distance+2.0 and angle_of_obstacle < np.pi/3. and angle_of_obstacle > -np.pi/3.:
            distance_based_correction = (distance_own_obstacle * np.linspace(4.0,0.0,num=20))/distance_own_obstacle
        else:
            distance_based_correction = np.linspace(1.0,1.0,num=20)
        
        sampled_theta = sample_result_to_trajectory(vehicle.traj_storage['pose'][0][2], plan_horizon+2)
        sampled_theta_leftshift = sampled_theta.copy()[2:len(sampled_theta)]
        rel_theta = (sampled_theta[0:len(sampled_theta)-2] -  sampled_theta_leftshift) 
        
        return distance_based_correction*-rel_theta
    else:
        # emergency situation. Steer away from obstacle       
        
        if angle_of_obstacle > 0:
            return np.ones(20)*-np.pi/2.
        else:
            return np.ones(20)*np.pi/2.
if __name__ == '__main__':

    pass
    
