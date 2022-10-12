"""wall_follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

import math
import numpy as np
import sys

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
    
    
    last_pos = [0, 0]
   
    # GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    state =  "follow_robot"
    cur_num_rot = 0
    
    count = 0
    
    # Enable proximity sensors
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[ind].enable(timestep)
        
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # for ind in range(8):
            # print("ind: {}, val: {}".format(ind, 
            # prox_sensors[ind].getValue()))
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        
        left_wall = prox_sensors[5].getValue() > 80
        left_corner = prox_sensors[6].getValue() > 80 
        front_wall = prox_sensors[7].getValue() > 80
        front2_wall = prox_sensors[0].getValue() > 80
        right2_wall = prox_sensors[1].getValue() > 80
        right_wall = prox_sensors[2].getValue() > 80
        back2_wall = prox_sensors[3].getValue() > 80
        back_wall = prox_sensors[4].getValue() > 80
        
        # Reading the gps
        gps_value = gps.getValues()
        cur_pos = gps_value[:2]
                
        left_speed = max_speed
        right_speed = max_speed
       
        if reached_goal(cur_pos, target_pos):
            print("\n\n")
            print("**************************")
            print("REACHED THE GOAL")
            print("***************************")
            
            sys.exit(0)
        
        if state == "follow_robot":
            print("Rotating to object...")
            state = 'calc_rotations'
        
        if state == "calc_rotations":

            
            print("GPS Values: ",   cur_pos)
            
            print("Timestep: ", timestep)
            
            
            if last_pos == [0, 0]:
                last_pos = cur_pos
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed)
                
            else:
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

                
                state = 'rotate_to_target'
                
            # last_pos = cur_pos
                
            
        elif state == 'rotate_to_target':
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
                cur_num_rot = 0
                state = 'move_straight'
                
            last_pos = [0, 0]
            
            cur_num_rot += 1
            
        elif state == 'move_straight':
            print("Moving straight...")
            
            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(max_speed)
            
            print(front_wall, left_wall, left_corner, front2_wall)
            if front_wall or left_wall or front2_wall:
                state = 'wall_follow'

        
        
        elif state == "wall_follow":
            if not front_wall and not left_wall and not front2_wall and not right_wall and not right2_wall and not back_wall and not back2_wall and not left_corner:
                if count < 100:
                    print("Incrementing count...")
                    left_motor.setVelocity(max_speed)
                    right_motor.setVelocity(max_speed)
                    count += 1
                    continue
                else:
                    count = 0
                    print("Follow robot")
                    state = 'follow_robot'
                
            if front_wall:
                print("Turn right in place")
                left_speed = max_speed
                right_speed = -max_speed
                
            else:
                if left_wall:
                    print("Drive forward")
                    left_speed = max_speed
                    right_speed = max_speed
                    
                else:
                    print("Turn left")
                    left_speed = max_speed / 8
                    right_speed = max_speed
                    # print("Follow object")
                    # state = 'calc_rotations'
                    # cur_num_rot = 0
                      
                    
                if left_corner:
                    print("Came too close, drive right")
                    left_speed = max_speed
                    right_speed = max_speed / 8
            
      
            # Process sensor data here.
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
        
          


# Enter here exit cleanup code.
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
