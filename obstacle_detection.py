"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor


from controller import Robot, Camera, DistanceSensor,LidarPoint
from vehicle import Driver



import numpy as np


# create the Robot instance.
robot = Driver()
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")

lidar = robot.getLidar("Sick LMS 291")



# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())



front_camera.enable(timestep)
rear_camera.enable(timestep)

lidar.enable(timestep)

lidar.enablePointCloud()






#check if enable:
# print(lidar.isPointCloudEnabled())


########parametrs of lidar

lms291_width = lidar.getHorizontalResolution()
half_width = lms291_width / 2

max_range = lidar.getMaxRange()
range_threshold = max_range / 20.0

OBSTACLE_THRESHOLD=0.01






# // gaussian function
def gaussian(x,mu,sigma):
    return (1.0 / (sigma * np.sqrt(2.0 * np.pi))) * np.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))


braitenberg_coefficients = []

for i in range(lms291_width):
    a=gaussian(i, half_width, lms291_width / 5)
    braitenberg_coefficients.append(a)






# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step() != -1:

    robot.setCruisingSpeed(50)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    lms291_values = np.array(lidar.getRangeImageArray())
    
    left_obstacle = 0.0 
    right_obstacle = 0.0
    for i in range(int(half_width)):
        if (lms291_values[i] < range_threshold):
            left_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[i] / max_range)
        
        j = lms291_width - i - 1
        
        if (lms291_values[j] < range_threshold):
            right_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[j] / max_range)
    
    
    b = left_obstacle+right_obstacle
    
    print(b)
    if(b>OBSTACLE_THRESHOLD):
        robot.setCruisingSpeed(0)
        print("object detected")
    else:
        robot.setCruisingSpeed(50)
        
        
    print(lidar.getNumberOfLayers())
    
    
    

# Enter here exit cleanup code.

