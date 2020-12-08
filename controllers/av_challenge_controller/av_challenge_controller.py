"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Display, Lidar
from vehicle import Driver
import numpy as np
import cv2
from math import ceil
# create the Robot instance.
robot = Driver()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
steering_angle = 0
# enable cameras
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")
front_camera.enable(30)
rear_camera.enable(30)

# start engine and set cruising speed
SIZES = (1, 180)
ranges_str = "1.13114178 0.85820043 0.57785118 0.43461093 0.38639969 0.31585345 0.2667459 0.23062678 0.21593061 0.19141567 0.17178488 0.15571462 0.14872716 0.13643947 0.12597121 0.11696267"
RANGES = [float(i) for i in ranges_str.split(' ')]
EPSILON = 0.6
DISPLAY_SIZE = (1024, 1024)
DISPLAY_SCALING_FACTOR = 0.9*1024/5
PLOT_UPDATE_RATE = 1


lidar.enable(timestep)

lidar.enablePointCloud()

robot.setCruisingSpeed(20)
############LIDAR FILTER
def lidar_filter(imageArray, SIZES,RANGES,EPSILON):
    theta_data = np.zeros((SIZES[1],)).tolist()
    for layer in range(SIZES[0]):
        for theta in range(SIZES[1]):
                point_range = imageArray[layer][theta]
                if(point_range < RANGES[layer]*(EPSILON)):
                    theta_data[theta] = point_range
    return theta_data  

# camera processing stuff
def process_camera_image(image):
    cam_height = front_camera.getHeight()
    cam_width  = front_camera.getWidth()
    cam_fov    = front_camera.getFov()
    num_pixels = cam_height*cam_width
    image = np.frombuffer(image, np.uint8).reshape((cam_height, cam_width, 4))
    image = image[:int(cam_height*1),:,:]
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Define range of white color in HSV
    lower_white = np.array([190])
    upper_white = np.array([255])
    # Threshold the HSV image
    mask = cv2.inRange(gray, lower_white, upper_white)
    mask = mask[int(cam_height*0.7):,:] 
    # Remove noise
    kernel_erode = np.ones((4,4), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((6,6),np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    cv2.imwrite("/Users/suryakanoria/Projects/AVChallenge/gray.png",gray)
    cv2.imwrite("/Users/suryakanoria/Projects/AVChallenge/test.png",mask)
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    line_angle = 0
    if(len(contours[1]) > 0):
        contour = contours[1][0][0]
        line_angle = contour[0][0] - 64
    # print(line_angle)
    return 0 if abs(line_angle) <= 5 else line_angle
    # return line_angle

old_contour = 0
# contour_queue = []
integral_queue = []
def pid(mean_cont):
    global old_contour
    # setting gain coeffs
    kp, kd, ki = 0.0096, 0, 0.008
    

    de = mean_cont - (contour_queue[-1] if len(contour_queue) > 0 else 0)
    integral_queue.append(mean_cont)
    if(len(integral_queue) > 10):
        integral_queue.pop(0)
        
    # obs = np.mean(contour_queue)
    integral = np.mean(integral_queue)
    op = kp*mean_cont + kd*de + ki * integral
    print(mean_cont,de,integral,op)
    return op
    
def set_steer_angle(steering_angle):
    max_steering_angle = 0.65
    min_steering_angle = -1 * max_steering_angle    
    steering_angle = min(steering_angle,max_steering_angle)
    steering_angle = max(steering_angle,min_steering_angle)
    robot.setSteeringAngle(steering_angle)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
lidar_queue = []
contour_queue = []
while robot.step() != -1:

    # Process sensor data here.
    contour = process_camera_image(front_camera.getImage())
    imageArray_1 = np.array(lidar.getRangeImageArray()).T #returns a two-dimensional list of floats
    imageArray_1 = imageArray_1[:,70:110]
    lidar_queue.append(imageArray_1[0])
    if(len(lidar_queue) > 30):
        lidar_queue.pop(0)
    
    # print("lidar",np.argmax(imageArray_1[0]))
    lidar_queue_np = np.mean(np.array(lidar_queue),axis=0)
    contour_queue.append(contour)
    if(len(contour_queue) > 30):
        contour_queue.pop(0)
    pid_op = pid(np.mean(contour_queue))
    speed = 30
    # if abs(pid_op) < 0.3:
        # speed = np.mean(lidar_queue_np)
    # elif abs(pid_op) < 1:
        # speed = 25
    # else:
        # speed = 10
    if contour!= 0:
        set_steer_angle(pid_op)
    else:
        pid_op = pid(contour)
        robot.setSteeringAngle(0)
    robot.setCruisingSpeed(speed)
        
        
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
   
    
    # print("Lidar",0.5 * )
    pass