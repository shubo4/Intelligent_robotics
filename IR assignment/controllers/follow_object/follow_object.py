"""follow_object controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import numpy as np
    
    
def sub(a, b):
    return [a[0] - b[0], a[1] - b[1]]
    
def norm(vec):
    abs = math.sqrt((vec[0]) ** 2 + (vec[1]) ** 2)
    return [vec[0] / abs, vec[1] / abs]
    
   

def compute_angle(a, b):    
    dot_product = a[0]*b[0] + a[1]*b[1]
    print("Sine value: ", math.asin(dot_product))
    print("Cosine value: ", math.acos(dot_product))
    return math.acos(dot_product)


def run_robot(robot):
    # Get time step of the current world.
   
    
    target_pos = [-0.06, 0.22]
    
    # timestep = int(robot.getBasicTimeStep())
    timestep = 1
    max_speed = 1.28
    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    
    last_pos = [0, 0]
    # count = 0
    
    
    # GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    state = "calc_rotations"
    cur_num_rot = 0
    
    
    while robot.step(timestep) != -1:  
    
        if state == 'calc_rotations':    

            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(max_speed)
            
            
            # Reading the gps
            gps_value = gps.getValues()
            cur_pos = gps_value[:2]
            
            print("GPS Values: ", cur_pos)
            
            print("Timestep: ", timestep)
            
            # count += 1
            
            if last_pos == [0, 0]:
                last_pos = cur_pos
            # elif count < 5:
                # pass
            else:
                # count = 0
                direction_vector = sub(cur_pos, last_pos)
                direction_vector = norm(direction_vector)
                print("Direction vector: ", direction_vector)
                
                target_vector = sub(target_pos, last_pos)
                target_vector = norm(target_vector)
                
                print("Target vector: ", target_vector)
                
                dot = (target_vector[0]*direction_vector[0]) + (target_vector[1]*direction_vector[1]) 
                cos = math.acos(dot)
                
                lookup_angle = cos
                
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
                
            
        elif state == 'rotate_to_target':
            print("Rotating to target...")
            # if target_pos[1] - cur_pos[1] >= 0 and target_pos[0] - cur_pos[0] >= 0:
                # left_motor.setVelocity(-max_speed)
                # right_motor.setVelocity(max_speed)
            # else:
                # left_motor.setVelocity(max_speed)
                # right_motor.setVelocity(-max_speed)

            # Move anticlockwise
            # if sin >=0 and tan >= 0:
                # print("sin >= 0 and tan >= 0")
                # left_motor.setVelocity(-max_speed)
                # right_motor.setVelocity(max_speed)
            
            # Move clockwise
            # elif sin >= 0 and tan < 0:
                # print("sin >= 0 and tan < 0")
                # left_motor.setVelocity(max_speed)
                # right_motor.setVelocity(-max_speed)
            
            # Move clockwise
            # elif sin < 0 and tan >= 0:
                # print("sin < 0 and tan >= 0")
                # left_motor.setVelocity(max_speed)
                # right_motor.setVelocity(-max_speed)
            
            # Move anticlockwise
            # elif sin < 0 and tan < 0:
                # print("sin < 0 and tan < 0")
                # left_motor.setVelocity(-max_speed)
                # right_motor.setVelocity(max_speed)

            
            
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
            
            cur_num_rot += 1
            
                
        elif state == 'move_straight':
            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(max_speed)
            print("Moving straight")
            
            
        last_pos = cur_pos
            
            
# Enter here exit cleanup code.
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)