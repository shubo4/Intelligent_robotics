"""follow_object controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

def run_robot(robot):
    # Get time step of the current world.
    
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28    
    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    while robot.step(timestep) != -1:
        # Reading the gps
        gps_value = gps.getValues()
        print(gps_value)
        
        left_motor.setVelocity(max_speed * 0.25)
        right_motor.setVelocity(max_speed * 0.25)
        
        
        
# Enter here exit cleanup code.
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)