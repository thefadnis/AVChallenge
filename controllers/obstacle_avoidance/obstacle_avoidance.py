from controller import Robot, Camera, DistanceSensor,LidarPoint
from vehicle import Driver
import numpy as np
import math
import cv2



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





########parametrs of lidar

lms291_width = lidar.getHorizontalResolution()
half_width = lms291_width / 2
max_range = lidar.getMaxRange()
range_threshold = max_range / 20.0
OBSTACLE_THRESHOLD=0.1

def set_steer_angle(steering_angle):
    max_steering_angle = 1
    min_steering_angle = -1 * max_steering_angle    
    steering_angle = min(steering_angle,max_steering_angle)
    steering_angle = max(steering_angle,min_steering_angle)
    robot.setSteeringAngle(steering_angle)


def process_camera_image():
    image = front_camera.getImage()
    cam_height = front_camera.getHeight()
    cam_width  = front_camera.getWidth()
    cam_fov    = front_camera.getFov()
    num_pixels = cam_height*cam_width
    image = np.frombuffer(image, np.uint8).reshape((cam_height, cam_width, 4))
    image = image[:int(cam_height*1),:,:]
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    lower_white = np.array([200])
    upper_white = np.array([255])
    mask = cv2.inRange(gray, lower_white, upper_white)
    mask = mask[int(cam_height*0.6):int(cam_height*0.95),:]
    kernel = np.ones((2,2), np.uint8)
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    line_angle = 0
    if(len(contours[0]) > 1):
        contour = contours[-2][0][0]
        line_angle = contour[0][0] - 261
    line_angle = 0 if abs(line_angle) <= 2 else line_angle
    kp = 0.0029
    op = kp*line_angle
    return line_angle,op
   
lp_queue = []



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


def dynamic_obstacle():

    angle=0
    od=0

    while robot.step() != -1:
    
    
        imageArray_1 = np.array(lidar.getRangeImageArray()).T #returns a two-dimensional list of floats
        # print(imageArray_1)
        # print(np.argmax(imageArray_1))
        front_distance= np.sum(imageArray_1[0][90])
       
        front_distance_left= np.sum(imageArray_1[0][85:90])/5
        front_distance_right= np.sum(imageArray_1[0][90:95])/5
       
        fd= (front_distance_left+front_distance_right)/2
       
    
        
       ###################lidar left and right obstacle value
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
        
        # print(b)
        # print("front distance:",fd)
       #######################################
       
        
    
        lp,op = process_camera_image()
        lp_queue.append(lp)        
        if(b>0.1 and abs(np.mean(lp_queue)) < 10 and od==0):
            od=1
            # print(fd)
            # print("Object Detected")
            if(left_obstacle>right_obstacle):
                angle = -0.14
            else:
                angle = 0.14
    
        elif(od==1):
    
            # print("left obstacle:",left_obstacle)
            # print("right obstalce:",right_obstacle)
            factor = -1
            # print("Angle tunr:",factor * angle)
            set_steer_angle(factor * angle)
            robot.setBrakeIntensity(0.9)
            robot.setCruisingSpeed(0)
            if(robot.getCurrentSpeed()<0.00000001):
                # print("Passed")
                od=0      
        else:
            # print("Following")
            set_steer_angle(op)
            robot.setCruisingSpeed(37)
            
dynamic_obstacle()