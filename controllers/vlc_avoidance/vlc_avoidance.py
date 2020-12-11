"""vlc_avoidance controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, DistanceSensor,LidarPoint
from vehicle import Driver
import numpy as np
import math
import cv2



# create the Robot instance.
robot = Driver()
driver = robot
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
front_camera.enable(timestep)
rear_camera.enable(timestep)
lidar.enable(timestep)
lidar.enablePointCloud()





########parametrs of lidar

lms291_width = lidar.getHorizontalResolution()
half_width = lms291_width / 2
max_range = lidar.getMaxRange()
range_threshold = max_range / 20.0
OBSTACLE_THRESHOLD=0.1




# // gaussian function
def gaussian(x,mu,sigma):
    return (1.0 / (sigma * np.sqrt(2.0 * np.pi))) * np.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))


braitenberg_coefficients = []

for i in range(lms291_width):
    a=gaussian(i, half_width, lms291_width / 5)
    braitenberg_coefficients.append(a)



##########LIDAR PARAMETER

lms291_width = lidar.getHorizontalResolution()
max_range = lidar.getMaxRange()
sick_fov= lidar.getFov()
def process_sick_data(sick_data):
    half_area=10
    sumx=0
    collision_count=0
    obstacle_dist=0
    x=int((lms291_width/2)- half_area)
    y=int((lms291_width/2)+half_area)
   
    for i in range(x,y):
        range_sick=sick_data[i]
        if(range_sick<20):
            sumx+=i
            collision_count+=1
            obstacle_dist+=range_sick
    if(collision_count==0):
        return 999999
   
    obstacle_dist = float(obstacle_dist/collision_count)
   
    obstacle_angle = (sumx/collision_count/lms291_width - 0.5) * sick_fov
   
    return obstacle_angle

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
def vlc_obstacle():

    driver.setCruisingSpeed(10)
    a=0
    while driver.step()!= -1:
    
        sa=driver.getSteeringAngle()
        # steeringangle.append(sa)
        
        imageArray_1 = np.array(lidar.getRangeImageArray()).T #returns a two-dimensional list of floats
        total_sum = np.sum(imageArray_1)/180
    
        front_distance_left_1= np.sum(imageArray_1[0][0:10])/10
        front_distance_right_1= np.sum(imageArray_1[0][170:180])/10
        
        fd_1= (front_distance_left_1+front_distance_right_1)/2
        
        front_distance_left= np.sum(imageArray_1[0][85:90])/10
        front_distance_right= np.sum(imageArray_1[0][90:95])/10
        
        fd= (front_distance_left+front_distance_right)/2
    
        obstacle_ang = process_sick_data(imageArray_1[0])
        
        if(obstacle_ang<1):
            print("obst_angle:",obstacle_ang)
         
        if(obstacle_ang>1):
            driver.setCruisingSpeed(5)
            if(a<0):
                driver.setSteeringAngle(-0.02)
            else:
                driver.setSteeringAngle(0.02)
        else:
            driver.setSteeringAngle(-obstacle_ang)
            a = obstacle_ang
    

vlc_obstacle()