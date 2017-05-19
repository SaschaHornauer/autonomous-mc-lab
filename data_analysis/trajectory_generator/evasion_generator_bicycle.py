'''
Created on May 8, 2017

@author: Sascha Hornauer
'''
from omgtools import *
from kzpy3.data_analysis.trajectory_generator.trajectory_tools import *
import cv2
import sys
import matplotlib.pyplot as plt
from trajectory_generator.collision_scanner import get_close_encounters
from omgtools.problems.point2point import FreeEndPoint2point
from aruco_tools.dynamic_model import *
from operator import add
import copy
import time
import numpy as np
import numpy.linalg
from timeit import default_timer as timer
from trajectory_generator.data_structure.entities import Obstacle
from aruco_tools.mode import behavior
import angles


framerate = 1. / 30.
max_v = 4.47  # approximate max speed according to the internet for the axial bomber
distance_from_boundary_of_circle = 2.  #
desired_speed = 2.0  # 

def get_state(own_xy, timestep_start, timestep_end):

    # Init our own position
    init_xy = own_xy[timestep_start]
         
    # Get our intended heading based on three timesteps in the future
    # which add as slight smoothing
    heading = get_heading(own_xy[timestep_start:timestep_start + timestep_end])
    # Get the velocity over the first two timesteps
    # velocity = get_velocities([own_xy[timestep_start], own_xy[timestep_start + 1]], framerate)
    return init_xy, heading

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
        if len(trajectory) == 2:
            trajectory_angles = []
            xy_positions = np.transpose(trajectory)
            
            for i in range(0, len(xy_positions) - 2):
                x1 = xy_positions[i][0]
                y1 = xy_positions[i][1]
                x2 = xy_positions[i + 1][0]
                y2 = xy_positions[i + 1][1]
                x3 = xy_positions[i + 2][0]
                y3 = xy_positions[i + 2][1]
                
                side_a = np.hypot(x3 - x2, y3 - y2)
                side_b = np.hypot(x2 - x1, y2 - y1)
                side_c = np.hypot(x3 - x1, y3 - y1)
            
                trajectory_angle = np.arccos((np.power(side_a, 2) + np.power(side_b, 2) - np.power(side_c, 2)) / (2.*side_a * side_b))
                # The arccos outputs the angle in 0, pi it is however expected in -pi, pi
                trajectory_angle = (trajectory_angle * 2.) - np.pi

                trajectory_angles.append(trajectory_angle)
            
            trajectories.append(trajectory_angles)
        else:
            # If the trajectory contains angles, add it as it is
            trajectories.append(trajectory)
    return trajectories


def get_emergency_trajectories(obstacle_pos, current_xy_own, number):
    
    angle_of_obstacle = np.arctan2(obstacle_pos[1] - current_xy_own[1], obstacle_pos[0] - current_xy_own[0])
      
    if angle_of_obstacle < 0:
        return np.ones(number) * -np.pi / 2.
    else:
        return np.ones(number) * np.pi / 2.


def get_center_trajectory(own_xy, other_xys, act_timestep, trajectory_length):
    try:
        heading = get_heading(own_xy[act_timestep:act_timestep + 3])
    except:
        try:
            heading = get_heading(own_xy[act_timestep - 3:act_timestep])
        except:
            heading = 0.0 
            # own_xy, other_xys, timestep, heading_own,delta, goal_xy, desired_speed, ideal_distance, min_distance_to_goal):
    center_traj, dismissed_motor, steering_deltas = get_trajectory_to_goal(own_xy, other_xys, act_timestep, heading, 0.0, [0.0, 0.0], 2.0, 1.0 , 0.2, trajectory_length)

    return center_traj, steering_deltas

def get_center_circle_points(own_xys):
    
    goalxys = []
    goal_offset = -(np.pi / 8.0)
    circle_radius = 4.28 - distance_from_boundary_of_circle  # meter
    
    # calculate positions on a circle near the center
    for pos in own_xys:
        own_x = pos[0]
        own_y = pos[1]
            
        # Get a point on the circle
        distance_goalpoint = circle_radius
        
        # Now take the distance to the goalpoint in combination with the angle to the 
        # goalpoint to make out the intersection of the straight line in between the
        # car and that circle
        angle_center = np.arctan2(own_y, own_x)
        
        # Change the angle to be in front of the vehicle like the carrot on a stick
        angle_goalpoint = angle_center + goal_offset
        
        # Next calculate that new point
        goal_xy = cv2.polarToCart(distance_goalpoint, angle_goalpoint)
        goal_xy = [goal_xy[0][0][0], goal_xy[1][0][0]]
        
        # If the distance of that new goalpoint is too far from our position
        
        goalxys.append(goal_xy)
        
    return goalxys
    

########################################################## 
# Modified after Steve LaValle
# 2011
# http://msl.cs.illinois.edu/~lavalle/sub/rrt.py
#
#########                         

class Node():
    
    _next_p = None
    _previous_p = None
    point = None
    
    def __init__(self,point, previous_p, next_p):
        _next_p = next_p
        _previous_p = previous_p
        _point = point

def dist(p1, p2):
    
    p1 = p1.point
    p2 = p2.point
    
    return np.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def step_from_to(p1, p2, epsilon,other_xys):
    
        previous_node = p1
        
        p1 = p1.point
        p2 = p2.point
    
        if dist(p1, p2) < epsilon:
            return Node(p2,previous_node,None)
        else:
            theta = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            return Node(p1[0] + epsilon * np.cos(theta), p1[1] + epsilon * np.sin(theta),previous_node,None)

def get_safe_path(own_xy, other_xys, goal_xy, safety_range, distance_to_goal):
    
    numnodes = 5000
    nodes = []
    epsilon = 7.0
    own_point = Node((own_xy[0],own_xy[1]),None,None)
    
    nodes.append(own_point)

    for i in range(numnodes):
        random_point = Node((np.random.uniform() * 4.0, np.random.uniform() * 4.0),None,None)
        next_node = nodes[0]
        for p in nodes:
            if dist(p, random_point) < dist(next_node, random_point):
                next_node = p
        
        for other_xy in other_xys:        
            newnode = step_from_to(next_node, random_point, epsilon,other_xys)
            if dist(newnode,other_xy) < safety_range:
                continue
        nodes.append(newnode)
        # Implement from to approach here tomorrow
        if dist(newnode,goal_xy) < distance_to_goal:
            return_path = []
            
            previous = newnode._previous_p
            return_path.append(newnode)
            
            while(previous != None):
                return_path.append(previous)
                previous = previous._previous
                
            return return_path
            

#########
# Modified after Steve LaValle
# 2011
# http://msl.cs.illinois.edu/~lavalle/sub/rrt.py
#
##########################################################

def get_trajectory_to_goal(own_xy, other_xys, timestep, heading_own, delta, goal_xy, desired_speed, ideal_distance, min_distance_to_goal, trajectory_length):

    current_xy_own = own_xy[timestep] 
    heading = heading_own

    own_x = current_xy_own[0]
    own_y = current_xy_own[1]
    goal_x = goal_xy[0]
    goal_y = goal_xy[1]
    final_traj_x = []
    final_traj_y = []
    motor_speeds = []
    goal_diff = np.hypot(goal_x - own_x, goal_y - own_y)
    act_pos_x = own_x
    act_pos_y = own_y
    deltas = []
    
    max_allowed_distance_to_obstacle = 0.5 
    start_evasion_range = 1.0
    
    for i in range(trajectory_length):
       
        # Get the straight angle to the goal
        angle_own_goal = np.arctan2(goal_y - act_pos_y, goal_x - act_pos_x)
        
        # Compare our angle, get the difference to the heading against the goal
        angle_diff = angle_own_goal - heading
        
        # Calculate steering angle 
        delta = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        
        if(delta > np.pi / 2.):
            delta = np.pi / 2.
        if delta < -np.pi / 2.:
            delta = -np.pi / 2.
        
        # Since this technique leads to a dog leg (bend) we will correct this by the distance to the center line
        distance_to_center_line = np.linalg.norm(np.cross(np.array(current_xy_own) - np.array(goal_xy), np.array(goal_xy) - (act_pos_x, act_pos_y)) / np.linalg.norm(np.array(current_xy_own) - np.array(goal_xy)))
        
        # We make the assumption that no single point is further away from 
        # the center line than the center line is actually long, for normalization
        distance_cl_norm = distance_to_center_line / (goal_diff)
        
        # Finaly steering is reduced, according to the distance
        delta = delta * distance_cl_norm
        
        # The speed is calculated depending on the distance to the goal and the desired distance
        if goal_diff > ideal_distance:
            speed = desired_speed
        else:
            goal_diff_norm = (goal_diff - min_distance_to_goal) / (ideal_distance - min_distance_to_goal)
            speed = goal_diff_norm * desired_speed
        
        answer = getXYFor(act_pos_x, act_pos_y, i * 0.033, speed, heading, (i + 1) * 0.033, 0.0, delta)
        
        # check if that step leads into the area of another vehicle.
        # One alternative here would be to use velocity areas etc. though since
        # in our example the own vehicle has during data analysis time no control over
        # its actual behavior, and since all other positions and the future is known
        # we can exploit that knowledge. For the time being a simple heuristic is used
        # however, since the whole approach has to be tested quickly in the field.
        
        for other_xy in other_xys[timestep]:
            if distance_2d((answer[0], answer[1]), (other_xy[0], other_xy[1])) < start_evasion_range:
                print get_safe_path(own_xy, other_xys, (goal_x,goal_y), 0.5, 0.5)
                sys.exit(0)
                break
        
        
        
        final_traj_x.append(answer[0])
        final_traj_y.append(answer[1])
        motor_speeds.append(speed)
        act_pos_x = answer[0]
        act_pos_y = answer[1]        
        heading = answer[3]
        deltas.append(delta)    
    
    return [final_traj_x, final_traj_y], motor_speeds, deltas
    

def convert_to_motor(resulting_motor_cmds):
    
    max_motor = 75.0
    min_motor = 49.0
    range = max_motor - min_motor
    
    
    # normalize
    resulting_motor_cmds = map(div, resulting_motor_cmds, [desired_speed] * np.ones(len(resulting_motor_cmds)))
    resulting_motor_cmds = map(mul, resulting_motor_cmds, [range] * np.ones(len(resulting_motor_cmds)))
    resulting_motor_cmds = map(add, resulting_motor_cmds, [49.0] * np.ones(len(resulting_motor_cmds)))
    
    return resulting_motor_cmds
    
goal_offset_limit = 150
allowed_goal_distance = 0.3
goal_ideal_distance = 50
d_timestep_goal = 60

def get_goal_position(goal_xys, own_xy, other_xys, timestep, exact_following):      
    global d_timestep_goal
    
    if other_xys:
        goal_near_obstacle = True
    else:
        goal_near_obstacle = False 
        
    if exact_following:
        return goal_xys[timestep]
        
    while (goal_near_obstacle):
    # The goal is in front on the observed trajectory, further in time.
    # If it is too far than a certain value, keep it at that value
        if d_timestep_goal > goal_offset_limit:
            d_timestep_goal = goal_offset_limit            
        
        # Every time we try to get the goal distance towards the ideal goal distance
        # when it quite far in front without reasons
        if d_timestep_goal > goal_ideal_distance:
            d_timestep_goal -= 1
      
        # If the end of the trajectory is reached, the endpoint will 'wait' at the last timestamp 
        if timestep + d_timestep_goal >= len(own_xy):
            d_timestep_goal = 0
      
        goal_xy = goal_xys[timestep + d_timestep_goal]   
                
        # Check if obstacle is too close to the goal or to the vehicle
        for other_position in other_xys:
            obstacle_pos = (other_position[timestep][0], other_position[timestep][1])        
            goal_near_obstacle = distance_2d(obstacle_pos, goal_xy) < allowed_goal_distance
            
            # If the end of the timesteps is reached the last found goal will be returned
            # anyway. This information can be retreived through the length of own_xy
            # The -1 is for the -1 we substract each run
            if len(own_xy) <= timestep + d_timestep_goal + 1:
                return goal_xy

        if goal_near_obstacle:
            # Look for a new goal along the trajectory if there is no explicit trajectory
            # to follow for the goal
            d_timestep_goal += 10 
    
    goal_xy = goal_xys[timestep + d_timestep_goal]   
    
    return goal_xy
#             
#         if vehicle_too_near:
#             # Skip a number of simulation runs, create emergency
#             # trajectories and check again
#             emergency_traj = get_emergency_trajectories(obstacle_pos, current_xy_own, trajectory_length)
#             resulting_trajectories.append(emergency_traj)
#             resulting_motor_cmds.append(np.linspace(0.0, 0.0, trajectory_length))
#             timestep += 1
#             simulator.deployer.reset()
#             # self.current_time, self.update_time, self.sample_time)
#             # simulation_time, sample_tim
#             # print simulator.current_time
#             
#             # simulator.problem.environment.simulate(update_time,sample_time)
#             # print "Skipping timestamp, vehicle too near" + str(timestep)
#             # print emergency_traj
#             continue_outer_loop = True  # Guido the great has spoken there shall be no continuation to the outer loop in this language. I don't like python. :(
#         
#         if goal_near_obstacle or vehicle_too_near:
#             break
#         
#     if continue_outer_loop:
#         continue
#                         
#             # Check if goal is too close to the boundary or rather if the distance to
#             # the center is too large
#             # The check is not used if the goal has to be on a certain trajectory 
#             while (distance_2d(goal_xy, [0.0, 0.0]) > (radius_arena - allowed_goal_distance) and goal_trajectory_data == None):
#                 # Otherwise, look for a new goal along the trajectory
#                 d_timestep_goal += 10
#                 if timestep + d_timestep_goal > no_datapoints:
#                     # The goal at the end of the trajectory is outside the boundary. 
#                     # This can not be avoided by looking further in the future
#                     goal_xy = goal_xys[no_datapoints - 1]
#                     # print "Goal at end of trajectory"
#                     break;
#                 else:
#                     goal_xy = goal_xys[timestep + d_timestep_goal]                
#                 
#                 # todo change allowed_goal distance to allowed vehicle distance
#             
#             continue_outer_loop = False
    


def close_to_boundary(current_xy_own, radius_arena, allowed_own_distance):
    return distance_2d(current_xy_own, [0.0, 0.0]) > (radius_arena - allowed_own_distance)


def get_slowdown_cmd(trajectory_length):
    # A more sophisticated behavior can be planned later, based on the following lines
    # distance_boundary_norm = (distance_2d(current_xy_own, [0.0, 0.0]) / radius_arena)
    # resulting_motor_cmds.append(np.ones(trajectory_length)*(1-distance_boundary_norm))
    # resulting_motor_cmds.append(np.ones(trajectory_length) * (1 - distance_boundary_norm))
    return np.linspace(0.4, 0.0, trajectory_length)


def get_batch_trajectories(own_xy, other_xys, timestep_start, plot_graphics, end_timestep, goal_xys, act_behavior):
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given in between timestep_start and timestep_ahead.
    Note that the trajectory will start one timestep further than timestep_start because
    one timestep is needed to retrieve information about the start conditions
  
    '''
    allowed_own_distance = 1.5
    min_distance_to_goal = allowed_own_distance
    ideal_distance = 2.0  # meter in following and to the boundary
    framerate = (1. / 30.)
    radius_arena = 4.28

    trajectory_length = 30
    resulting_trajectories = []
    resulting_motor_cmds = []
    resulting_deltas = []

    plt_vehicle = None
    plt_boundary = None
        
    init_xy_own, heading_own = get_state(own_xy, timestep_start, 4)  # smooth heading over 3 timesteps in the future
     
    if plot_graphics:
        plt.ion()
        if plt_boundary == None:
            plt_boundary = plt.Circle((0, 0), radius_arena, color='b', fill=False)
            axis = plt.gca()
            axis.add_artist(plt_boundary)
        plt.axis([-5, 5, -5, 5])
        plt.show()

    
    ############### Iterate over all timesteps in the data
    for timestep in range(timestep_start, end_timestep):
        
        # If the behavior is follow, we fast-forward until we find a vehicle position in the goal_xys array
        # where the other positions are stored
        if goal_xys[timestep] == None:
            timestep += 1
            if timestep > end_timestep:
                break
            continue

        current_xy_own, heading_own = get_state(own_xy, timestep, 4)  # smooth heading over 3 timesteps in the future
        
        # A number of checks are performed to make sure the goal is clear of vehicles,
        # and not planned within the boundary. If behavior is follow, now dynamic goal is used,
        # which tries to avoid other vehicles
        goal_xy = get_goal_position(goal_xys, own_xy, other_xys, timestep, act_behavior == behavior.follow)
        
        if close_to_boundary(current_xy_own, radius_arena, allowed_own_distance):
            new_trajectory, new_deltas = get_center_trajectory(own_xy, other_xys, timestep, trajectory_length)   
            resulting_deltas.append(new_deltas)         
            resulting_trajectories.append(new_trajectory)
            resulting_motor_cmds.append(get_slowdown_cmd(trajectory_length))
            color = 'r'
        else:
            new_trajectory, new_motor_cmds, new_deltas = get_trajectory_to_goal(own_xy, other_xys, timestep, heading_own, 0.0, goal_xy, desired_speed, ideal_distance, min_distance_to_goal, trajectory_length)
            resulting_deltas.append(new_deltas)    
            resulting_trajectories.append(new_trajectory)
            resulting_motor_cmds.append(new_motor_cmds)
            color = 'b'
            
        
        if plot_graphics:
            lines = plt.plot(new_trajectory[0], new_trajectory[1], color)
            vehicle = plt.plot(current_xy_own[0], current_xy_own[1], 'bo')
            goal = plt.plot(goal_xy[0], goal_xy[1], 'go')
            obstacle_plot = []
            for other in other_xys:
                obstacle_plot.append(plt.plot(other[timestep][0], other[timestep][1], 'ro'))
            plt.pause(0.0001)
            plt.show()
            for other_plot in obstacle_plot:
                other_plot.pop(0).remove()
            lines.pop(0).remove()
            vehicle.pop(0).remove()
            goal.pop(0).remove()
        
    # steering_angles = convert_path_to_steeering_angles(resulting_trajectories)
    steering_angles = resulting_deltas
    motor_commands = convert_to_motor(resulting_motor_cmds)
    
    return steering_angles, motor_commands, resulting_trajectories


    
