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

def rad_to_percent_steer(ang_rad, ang_perc):
    
        max_left_steering_angle = np.deg2rad(-90)
        max_right_steering_angle = np.deg2rad(90)
        
        max_left_command = 100
        max_right_command = 0
        
        left_range = 50
        right_range = 50
        
        min_perceived_distance = 9999
        
        critical_distance = 3.0
        stop_distance = 0.5
        
        max_motor = 60
        min_motor = 49  # Full stop. Backwards is not considered
        
        motor_override = 49
        steer_override = 49
    else:
        motor_command = ar_params['ar_motor_command']
        max_left_steering_angle = ar_params['ar_max_left_steering_angle']
        max_right_steering_angle = ar_params['ar_max_right_steering_angle']
        
        max_left_command = ar_params['ar_max_left_command']
        max_right_command = ar_params['ar_max_right_command']
                
        left_range = ar_params['ar_left_range']
        right_range = ar_params['ar_right_range']
                
        min_perceived_distance = ar_params['ar_min_perceived_distance']
                
        critical_distance = ar_params['ar_critical_distance']
        stop_distance = ar_params['ar_stop_distance']
                
        max_motor = ar_params['ar_max_motor']
        min_motor = ar_params['ar_min_motor']
        
        motor_override = ar_params['ar_override_motor']
        steer_override = ar_params['ar_override_steer'] 
 
    # Check if steering command values are taken from an list of values, 
    # designed to give smooth trajectories instead of sudden movements
    if not (self.steering_command_index > 0):
   
        evasion_needed = False
        
   
        front_left_limit_deg = -90
        front_right_limit_deg = 90
                
        average_angle, min_perceived_distance, markers = aruco_angle_retriever.get_boundary_angle_min_distance(cv_image, crop, 2)
        
        #if(min_perceived_distance < critical_distance):
        evasion_needed = True
        
        if(average_angle != None):   
            print("average angle " + str(np.rad2deg(average_angle)))
            opposite_angle = ((average_angle + np.pi) + np.pi) % (2 * np.pi) - np.pi 
            
            mid_steering_command = np.abs(max_right_command - max_left_command) / 2.0
            
            if opposite_angle < 0:
                steering_command = (opposite_angle / np.pi) * left_range                               
            else:
                steering_command = (opposite_angle / np.pi) * right_range 
            
            # Finally change the mapping from -50,50 to 0,100
            try:
                steering_command = (steering_command + mid_steering_command)[0]
            except:
                steering_command = (steering_command + mid_steering_command)
            
            
            # A special behaviour is investigated. This is a test
            # There is a list of steering commands. When this 
            # list is still empty, resp. the index is 0
            
            #print(steering_command)
            #print(incoming_steering_cmd)
            # Now interpolate the intermediate values from the current steering towards that command
            # in as many steps as the list is long
            increment = np.abs(incoming_steering_cmd-steering_command)/self.steering_command_length
            # Finally fill the list with those values. 
            self.steering_command_list = self.steering_command_list*increment
                
        
            # The motor command is no longer calculated
            # Next, calculate a safe motor command
            # If the average obstacle is in front of us....
            #if(np.deg2rad(front_left_limit_deg) < average_angle < np.deg2rad(front_right_limit_deg)):
            #    if(min_perceived_distance < stop_distance):
            #        motor_command = min_motor
            #    elif (min_perceived_distance < critical_distance):
            #        distance_norm = ((min_perceived_distance - stop_distance) / (critical_distance - stop_distance))
            #        motor_command = min_motor + distance_norm * (max_motor - min_motor)
        
        
        if motor_override != 49:
            motor_command = motor_override
        if steer_override != 49:
            steer_command = steer_override   
        
        if not 'motor_command' in vars():
            motor_command = incoming_motor_cmd
        if not 'steering_command' in vars():
            steering_command = incoming_steering_cmd
        
    
    else:
        evasion_needed = True
        # The steering command is one out of the command list, the index is 
        # updated and if the index is 0 again, the calculation starts again
        steering_command = self.steering_command_list[self.steering_command_index]
        self.steering_command_index = (self.steering_command_index+1)%self.steering_command_length
        
    
def get_evasive_trajectory(own_xy,other_xy,timestep_start, process_timesteps, d_timestep_goal):
    
    
    '''
    Returns a short term evasion trajectory, in steering commands for 
    as many timesteps ahead as given in between timestep_start and timestep_ahead.
    Note that the trajectory will start one timestep further than timestep_start because
    one timestep is needed to retrieve information about the start conditions
     
    FOR NOW, OTHER_XY IS ONLY ONE OTHER VEHICLE    
    '''

    
    init_xy_own, heading_own, velocity_own = get_state(own_xy,timestep_start,3) # smooth heading over 3 timesteps in the future
    init_xy_other, heading_other, velocity_init_other = get_state(other_xy,timestep_start,3)
    
      
    goal_xy = own_xy[timestep_start+d_timestep_goal] # The goal is our future position in the dataset
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
    environment = Environment(room={'shape': Square(10.)})
   
    # get velocities of other vehicles
    other_positions = other_xy[timestep_start:timestep_start+process_timesteps]
    other_velocities = get_velocities(other_positions, 1./30.)

    traj = {'position': {'time': np.linspace(0.,process_timesteps*(1./30.),process_timesteps-1),
                         'values': other_velocities}}
 
    environment.add_obstacle(Obstacle({'position': init_xy_other}, shape=Circle(0.25),
        simulation={'trajectories': traj}))
       
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

    for i in range(0,d_timestep_goal):
        simulator.update()
        steering_deltas.append(vehicle.traj_storage['delta'][i][0][0])
        
    steering_deltas

if __name__ == '__main__':

    pass
    