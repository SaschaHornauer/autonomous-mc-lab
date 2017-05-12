'''
Created on May 8, 2017

@author: Sascha Hornauer
'''
from omgtools import *
from kzpy3.data_analysis.trajectory_generator.trajectory_tools import *
import cv2
import sys
import matplotlib.pyplot as plt
from omgtools.problems.point2point import FreeEndPoint2point
from operator import add

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

def sample_values(values, no_of_samples):
    
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
        return np.array(values)[index]
    

def get_evasive_trajectory(own_xy, other_xy, timestep_start, d_timestep_goal, plot_video):
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given in between timestep_start and timestep_ahead.
    Note that the trajectory will start one timestep further than timestep_start because
    one timestep is needed to retrieve information about the start conditions
     
    FOR NOW, OTHER_XY IS ONLY ONE OTHER VEHICLE    
    '''
    safety_distance = 0.2
    emergency_distance = 0.4
    obstacle_segment_factor = int(2999/10) # This factor should be made dependent on the length of the dataset
    no_datapoints = len(own_xy)
    framerate = (1. / 30.)
    diameter_arena = 4.28
    
    # For each obstacle in the obstacle trajectory list we create segments from
    # the trajectories to improve computability.

    environment = Environment(room={'shape': RegularPolyhedron(diameter_arena, 24), 'draw': False})
    
    for i in range(0,len(other_xy)):
        obstacle_xy = other_xy[i]
        # For each obstacle, get its segments
        samplepoints = np.linspace(timestep_start, no_datapoints-1, num=obstacle_segment_factor,dtype=np.int32)
        
        segments_trajectory = np.array(obstacle_xy)[samplepoints]

        # Calculate start and end time of the segment 
        obstacle_start_times = np.linspace(timestep_start, framerate*no_datapoints, num=obstacle_segment_factor)
        #obstacle_end_times = obstacle_start_times[1:]
        #obstacle_start_times = obstacle_start_times[:len(obstacle_start_times)-1]

        offset_diffs = get_pos_diff(segments_trajectory)

        # Create a trajectory for that obstacle
        traj = ({'position': {'time':obstacle_start_times,
                     'values': offset_diffs},
                 })

        # add it to the environment
        environment.add_obstacle(Obstacle({'position': segments_trajectory[0]}, shape=Circle(0.25),
            simulation={'trajectories': traj}))
         
    
    vehicle = Holonomic(options={'plot_type': 'car'}) 
    #vehicle.define_knots(knot_intervals=5)
    
    init_xy_own, heading_own, velocity_own = get_state(own_xy, timestep_start, 4)  # smooth heading over 3 timesteps in the future
    goal_xy, goal_heading, goal_velocity = get_state(own_xy, timestep_start + d_timestep_goal, 4) 
    
    #angle_of_obstacle = np.arctan2(init_xy_other[1]-init_xy_own[1],init_xy_other[0]-init_xy_own[0])
    #distance_own_obstacle = np.hypot(init_xy_other[0]-init_xy_own[0],init_xy_other[1]-init_xy_own[1])
    
    #if not distance_own_obstacle < safety_distance + 0.7:
            
    # make and set-up vehicle

    # plan from the last known position
    vehicle.set_initial_conditions(state=[init_xy_own[0], init_xy_own[1]])  
    # SIMPLIFICATION: The desired goal heading is our current heading. The goal is going straight
    vehicle.set_terminal_conditions([goal_xy[0], goal_xy[1]])
    vehicle.set_options({'safety_distance': safety_distance})
    
    # create a point-to-point problem
    # FreeEndPoint2point(veh, environment.copy(), options, {veh: free_ind}))
    problem = Point2point(vehicle, environment, freeT=False)
    
    problem.set_options({'solver_options':
    {'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})
    
    problem.init()
     
    # simulate, plot some signals and save a movie
    simulator = Simulator(problem, sample_time=1. / 30., update_time=1. / 30.)
        
    problem.plot('scene')
    
    for timestep in range(timestep_start,len(own_xy)):
                
        #if plot_video:
        
            #plt.savefig("scene" + "_" + str(framenumber) + ".png")
        
        problem.update_plot('scene',0)
        
        if timestep > 1:
            
            current_xy_own = own_xy[timestep] 
            goal_xy = own_xy[timestep + d_timestep_goal]
            
            # Check if goal is too close to an obstacle
            while True:
                obstacle_too_near = False 
                for obstacle in simulator.problem.environment.obstacles:
                    obstacle_pos = (obstacle.signals['position'][0][-1],obstacle.signals['position'][1][-1])
                    obstacle_too_near = obstacle_too_near or distance_2d(obstacle_pos,goal_xy) < emergency_distance
                # If we are far away from an obstacle, continue
                if not obstacle_too_near:
                    break
                
                # Otherwise, look for a new goal along the trajectory
                # TODO. Handle end of trajectory here
                d_timestep_goal += 10
                goal_xy = own_xy[timestep + d_timestep_goal]
                
            
            
            # Check if goal is too close to the boundary or rather if the distance to
            # the center is too large
            while (distance_2d(goal_xy,[0.0,0.0]) > (diameter_arena - emergency_distance)):
                # Otherwise, look for a new goal along the trajectory
                # TODO. Handle end of trajectory here
                d_timestep_goal += 10
                goal_xy = own_xy[timestep + d_timestep_goal]                
                                            
            simulator.problem.vehicles[0].overrule_state(current_xy_own)
            vehicle.set_terminal_conditions([goal_xy[0], goal_xy[1]])
            
        simulator.update()
        simulator.update_timing()
        
        # return trajectories and signals
        trajectories, signals = {}, {}
        for vehicle in simulator.problem.vehicles:
            trajectories[str(vehicle)] = vehicle.traj_storage
            signals[str(vehicle)] = vehicle.signals
        

        
        #sampled_theta = sample_result_to_trajectory(vehicle.traj_storage['pose'][0][2], plan_horizon+2)
        #sampled_theta_leftshift = sampled_theta.copy()[2:len(sampled_theta)]
        #rel_theta = (sampled_theta[0:len(sampled_theta)-2] -  sampled_theta_leftshift) 
        
        #return distance_based_correction*-rel_theta
#         else:
#             # emergency situation. Steer away from obstacle       
#             
#             if angle_of_obstacle > 0:
#                 return np.ones(20)*-np.pi/2.
#             else:
#                 return np.ones(20)*np.pi/2.
if __name__ == '__main__':

    pass
    
