"""wall_follower_2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import numpy as np
import sys
from controller import Robot

import warnings
warnings.filterwarnings('ignore')
    
    
def sub(a, b):
    return [a[0] - b[0], a[1] - b[1]]
    
def norm(vec):
    abs = math.sqrt((vec[0]) ** 2 + (vec[1]) ** 2)
    return [vec[0] / abs, vec[1] / abs]
    

def compute_angle(a, b):    
    dot_product = a[0]*b[0] + a[1]*b[1]
    return math.acos(dot_product)


def reached_goal(cur_pos, target_pos):
    threshold = 0.1
    if (abs(cur_pos[0] - target_pos[0]) < threshold) and (abs(cur_pos[1] - target_pos[1]) < threshold):
       return True
    return False

def run_robot(robot):
    """ Wall following robot """

    # get the time step of the current world.
    timestep = 1
    max_speed = 1.28
    
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    target_pos = [-0.06, 0.22]
    
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
   
   
    # GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    
    robot.step(timestep)
    cur_pos = gps.getValues()[:2]
    
    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)
    
    state =  "dilation"
    flag = 0
    cur_num_rot = 0
    count = 0
    
    # Enable proximity sensors
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[ind].enable(timestep)
    
    sensing_limit = 120
    
    while robot.step(timestep) != -1:
        s0 = prox_sensors[0].getValue() > sensing_limit
        s1 = prox_sensors[1].getValue() > sensing_limit
        s2 = prox_sensors[2].getValue() > sensing_limit
        s3 = prox_sensors[3].getValue() > sensing_limit
        s4 = prox_sensors[4].getValue() > sensing_limit
        s5 = prox_sensors[5].getValue() > sensing_limit
        s6 = prox_sensors[6].getValue() > sensing_limit
        s7 = prox_sensors[7].getValue() > sensing_limit

        # Reading the GPS value
        gps_values = gps.getValues()
        last_pos = cur_pos
            
        cur_pos = gps_values[:2]
        
        # print(s0, s1, s2, s3, s4, s5, s6, s7) 
        
        if reached_goal(cur_pos, target_pos):
            print("\n\n")
            print("**************************")
            print("REACHED THE GOAL")
            print("***************************")
            
            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(-max_speed)
            continue
            
 
        
        if sum([s0, s1, s2, s3, s4, s5, s6, s7]) == 0:
            if state == "dilation":
                print("Dilation...")
                if count < 120:
                    # print(count)
                    count += 1
                    continue
                else:
                    count = 0
                    state = "follow_robot"
                
            if state == "follow_robot":
                print("Rotating to object...")
                state = 'calc_rotations'
                
                
            if state == "calc_rotations":
                direction_vector = sub(cur_pos, last_pos)
                direction_vector = norm(direction_vector)
                print("Direction vector: ", direction_vector)
                
                target_vector = sub(target_pos, last_pos)
                target_vector = norm(target_vector)
                
                print("Target vector: ", target_vector)
                
                lookup_angle = compute_angle(target_vector, direction_vector)
                
                print("Angle to cover: ", lookup_angle)
                
                num_rotations = math.ceil((lookup_angle / 0.0288))
                
                print("Num rotations: ", num_rotations)
                
                theta_r = 0.001
                rotation_matrix = np.array([[math.cos(theta_r), -math.sin(theta_r)], [math.sin(theta_r), math.cos(theta_r)]])
                new_direction = rotation_matrix @ np.array(direction_vector)
                
                theta_new = math.acos((new_direction[0]*target_vector[0]) + (new_direction[1]*target_vector[1]))
                
                if lookup_angle - theta_new > 0:
                    rotate_type = 'anticlockwise'
                else:
                    rotate_type = 'clockwise'
                
                flag = 1
                
                state = 'rotate_to_target'
        
            if state == 'rotate_to_target':
                print("Rotating to target...")
                if rotate_type == 'anticlockwise':
                    print("Rotate anticlockwise")
                    left_motor.setVelocity(-max_speed)
                    right_motor.setVelocity(max_speed)
                elif rotate_type == 'clockwise':
                    print("Rotate clockwise")
                    left_motor.setVelocity(max_speed)
                    right_motor.setVelocity(-max_speed)
                if cur_num_rot >= num_rotations:
                    flag = 0
                    cur_num_rot = 0
                    state = 'move_straight'
                
                cur_num_rot += 1
        
            if state == 'move_straight':
                print("Move straight...")
                
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed)
                
        elif sum([s0, s1, s2, s3, s4, s5, s6, s7]) != 0 and not flag:
            state = "dilation"
            
            right_wall = s2
            right_corner = s1
            front_wall = s0
            
            print(front_wall, right_wall, right_corner)
            
            
            print("Wall follow")  
            
            # print(front_wall, right_wall, right_corner)
            
            # print(front_wall, left_corner, left_wall)  
            if front_wall:
                print("Turn left in place")
                left_speed = -max_speed
                right_speed = max_speed
                
            else:
                if right_wall:
                    print("Drive forward")
                    left_speed = max_speed
                    right_speed = max_speed
                    
                else:
                    print("Turn right")
                    left_speed = max_speed
                    right_speed = max_speed
                    # print("Follow object")
                    # state = 'calc_rotations'
                    # cur_num_rot = 0
                      
                    
                # if right_corner:
                    # print("Came too close, drive right")
                    # left_speed = max_speed
                    # right_speed = max_speed / 8
            
      
            # Process sensor data here.
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

                
            
        
    
# Enter here exit cleanup code.
if __name__ == "__main__":
   my_robot = Robot()
   run_robot(my_robot)
