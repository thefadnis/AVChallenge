"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, DistanceSensor,LidarPoint
from vehicle import Driver



# import cv2
import math
import numpy as np
# create the Robot instance.
robot = Driver()

front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")


lidar = robot.getLidar("Sick LMS 291")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)


front_camera.enable(timestep)
rear_camera.enable(timestep)

#LIDAR AND PLOTTING INITIALIZATION
SIZES = (1, 180)
ranges_str = "1.13114178 0.85820043 0.57785118 0.43461093 0.38639969 0.31585345 0.2667459 0.23062678 0.21593061 0.19141567 0.17178488 0.15571462 0.14872716 0.13643947 0.12597121 0.11696267"
RANGES = [float(i) for i in ranges_str.split(' ')]
EPSILON = 0.6
DISPLAY_SIZE = (1024, 1024)
DISPLAY_SCALING_FACTOR = 0.9*1024/5
PLOT_UPDATE_RATE = 1


lidar.enable(timestep)

lidar.enablePointCloud()


    
############LIDAR FILTER
def lidar_filter(imageArray, SIZES,RANGES,EPSILON):
    theta_data = np.zeros((SIZES[1],)).tolist()
    for layer in range(SIZES[0]):
        for theta in range(SIZES[1]):
                point_range = imageArray[layer][theta]
                if(point_range < RANGES[layer]*(EPSILON)):
                    theta_data[theta] = point_range
    return theta_data

############PID
def pid(input_value,previous):
    stable_value=0
    steer=0.015*input_value+1*(input_value-previous)
    speed=abs(30-abs(100*input_value))
    if speed>30:
        speed=30
    print('input value=',input_value)
    print('speed=',speed)
    return steer,speed
    


# Main loop:
# - perform simulation steps until Webots is stopping the controller
angle=0
previous=0
while robot.step() != -1:

    
    # angle = 0.3 * math.cos(robot.getTime())
    # robot.setSteeringAngle(angle)
    # robot.setCruisingSpeed(40)
    
    #########################LIDAR
    
    imageArray_1 = np.array(lidar.getRangeImageArray()).T #returns a two-dimensional list of floats
    
    imageArray_2 = np.array(lidar.getRangeImage()).T #returns a one-dimensional list of floats
    
    imageArray_3 = np.array(lidar.getLayerRangeImage(1))#function is a convenient way of getting directly the sub range image associated with one layer.
    
    
    # print(imageArray_3)
    
    # print(imageArray.shape)
    # theta_data = lidar_filter(imageArray_1, SIZES,RANGES,EPSILON)
        
    # print(imageArray_1[0][0],imageArray_1[0][-1])
    
    change,speed=pid(imageArray_1[0][-1]-imageArray_1[0][0],previous)
    previous=imageArray_1[0][-1]-imageArray_1[0][0]
    angle=angle+change
    if angle>0.523:
        angle=0.523
    if angle<-0.523:
        angle=-0.523
    robot.setSteeringAngle(angle)
    robot.setCruisingSpeed(speed)

# Enter here exit cleanup code.
