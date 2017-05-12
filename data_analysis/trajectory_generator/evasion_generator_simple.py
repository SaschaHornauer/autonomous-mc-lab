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
import copy

framerate = 1. / 30.


################
################
# ## ISSUES: 
#
# Need to remove the debug input emulating mr yellow
# Need to find why there are some strange angle values in the end of the trajectory, probably with linear interpolation instead of sampling
# Detect when own position os too close to obstacle or boundary, choose emergency trajectory and skip over those moments in the simulator
#
#
# # This framerate should be at one point for the whole module

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
    

def convert_path_to_steeering_angles(resulting_trajectories):
    
    trajectories = []
    
    for trajectory in resulting_trajectories:
        
        trajectory_angles = []
        pos_diffs = get_pos_diff(np.transpose(trajectory))
        
        for pos_diff in pos_diffs:
            trajectory_angles.append(np.arctan2(pos_diff[1], pos_diff[0]))
        
        trajectories.append(trajectory_angles)
        
    return trajectories


def get_emergency_trajectories(obstacle_pos, current_xy_own, number):
    
    angle_of_obstacle = np.arctan2(obstacle_pos[1] - current_xy_own[1], obstacle_pos[0] - current_xy_own[0])
      
    if angle_of_obstacle > 0:
        return np.ones(number) * -np.pi / 2.
    else:
        return np.ones(number) * np.pi / 2.


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
    obstacle_segment_factor = int(2999 / 10)  # This factor should be made dependent on the length of the dataset
    no_datapoints = len(own_xy)
    framerate = (1. / 30.)
    diameter_arena = 4.28
    
    resulting_trajectories = []
    
    # For each obstacle in the obstacle trajectory list we create segments from
    # the trajectories to improve computability.

    environment = Environment(room={'shape': RegularPolyhedron(diameter_arena, 24), 'draw': False})
    
    for i in range(0, len(other_xy)):
        obstacle_xy = other_xy[i]
        # For each obstacle, get its segments
        samplepoints = np.linspace(timestep_start, no_datapoints - 1, num=obstacle_segment_factor, dtype=np.int32)
        
        segments_trajectory = np.array(obstacle_xy)[samplepoints]

        # Calculate start and end time of the segment 
        obstacle_start_times = np.linspace(timestep_start, framerate * no_datapoints, num=obstacle_segment_factor)
        # obstacle_end_times = obstacle_start_times[1:]
        # obstacle_start_times = obstacle_start_times[:len(obstacle_start_times)-1]

        offset_diffs = get_pos_diff(segments_trajectory)

        # Create a trajectory for that obstacle
        traj = ({'position': {'time':obstacle_start_times, 'values': offset_diffs}})

        # add it to the environment
        environment.add_obstacle(Obstacle({'position': segments_trajectory[0]}, shape=Circle(0.25),
            simulation={'trajectories': traj}))
    
    vehicle = Holonomic(options={'plot_type': 'car'}) 
    
    init_xy_own = own_xy[timestep_start] 
    goal_xy = own_xy[timestep_start + d_timestep_goal] 

    # Plan from the initial position
    vehicle.set_initial_conditions(state=[init_xy_own[0], init_xy_own[1]])  
    vehicle.set_terminal_conditions([goal_xy[0], goal_xy[1]])
    vehicle.set_options({'safety_distance': safety_distance})
    
    # Create a point-to-point problem
    problem = Point2point(vehicle, environment, freeT=False)
    
    problem.set_options({'solver_options':
    {'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})
    
    if plot_video:
        problem.init()
     
    # simulate, plot some signals and save a movie
    simulator = Simulator(problem, sample_time=1. / 30., update_time=1. / 30.)
        
    problem.plot('scene')
    
    for timestep in range(timestep_start, len(own_xy)):
    # for timestep in range(timestep_start,timestep_start+30):
               
        if plot_video:
            plt.savefig("scene" + "_" + str(timestep) + ".png")
            problem.update_plot('scene', 0)
        
        if timestep > timestep_start + 1:
            
            current_xy_own = own_xy[timestep] 
            goal_xy = own_xy[timestep + d_timestep_goal]
            continue_outer_loop = False
            # Check if obstacle is too close to the goal or to the vehicle
            while True:
                goal_near_obstacle = False 
                vehicle_near_obstacle = False
                for obstacle in simulator.problem.environment.obstacles:
                    obstacle_pos = (obstacle.signals['position'][0][-1], obstacle.signals['position'][1][-1])
                    goal_near_obstacle = goal_near_obstacle or distance_2d(obstacle_pos, goal_xy) < emergency_distance
                    vehicle_near_obstacle = vehicle_near_obstacle or distance_2d(current_xy_own, obstacle_pos) < emergency_distance
                    
                # If we are far away from an obstacle, continue
                if not goal_near_obstacle and not vehicle_near_obstacle:
                    break
                
                if goal_near_obstacle:
                    # Look for a new goal along the trajectory
                    # TODO. Handle end of trajectory here
                    d_timestep_goal += 10
                    goal_xy = own_xy[timestep + d_timestep_goal]
                    
                if vehicle_near_obstacle:
                    # Skip a number of simulation runs, create emergency
                    # trajectories and check again
                    resulting_trajectories.append(get_emergency_trajectories(obstacle_pos, current_xy_own, 10))
                    timestep += 10
                    continue_outer_loop = True  # Guido the great has spoken there shall be no continuation to the outer loop in this language. I don't like python. :(
            
            if continue_outer_loop:
                continue
                        
            # Check if goal is too close to the boundary or rather if the distance to
            # the center is too large
            while (distance_2d(goal_xy, [0.0, 0.0]) > (diameter_arena - emergency_distance)):
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
            
            # Calculate the resulting trajectory and sample it to 30 values
            trajectory_xs = trajectories[str(vehicle)]['pose'][-1][0]
            trajectory_ys = trajectories[str(vehicle)]['pose'][-1][1]
            
            trajectory_30_x = sample_values(trajectory_xs, 30)
            trajectory_30_y = sample_values(trajectory_ys, 30)
        
            resulting_trajectories.append((trajectory_30_x, trajectory_30_y))


    return convert_path_to_steeering_angles(resulting_trajectories)

    
if __name__ == '__main__':

    pass
    
