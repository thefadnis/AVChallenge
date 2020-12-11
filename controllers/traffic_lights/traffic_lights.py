# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Display, Lidar
from vehicle import Driver
import numpy as np
import cv2
from math import ceil
# create the Robot instance.
robot = Driver()
print (cv2.__version__)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
steering_angle = 0
# enable cameras
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")
front_camera.enable(30)
rear_camera.enable(30)

# camera processing stuff

# Main loop:
# - perform simulation steps until Webots is stopping the controller

    
def sleep(duration):
    # Waits for duration seconds before returning.
    global robot
    end_time = robot.getTime() + duration
    while robot.step() != -1 and robot.getTime() < end_time:
        pass

def process_image(image):
    cam_height = front_camera.getHeight()
    cam_width  = front_camera.getWidth()
    image = np.asarray(image, dtype=np.uint8)
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    image = cv2.flip(image, 1)

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # Define range of traffic color red in HSV
    # Green light hsv - > [ 41 194 248] 
    # Yellow light HSV -> [ 92 135 253]
    # Red light HSV -> [124 191 253]
    lower_red = np.array([118, 185, 248])
    upper_red = np.array([130, 197, 255])
    # Threshold the HSV image
    kernel_dilate = np.ones((6,6),np.uint8)
    mask_red = cv2.inRange(hsv, lower_red , upper_red )
    # Remove noise
    dilated_mask_red = cv2.dilate(mask_red , kernel_dilate, iterations=1)

    lower_green = np.array([35, 188, 243])
    upper_green = np.array([46, 200, 255])
    # Threshold the HSV image
    kernel_dilate_green = np.ones((6,6),np.uint8)
    mask_green = cv2.inRange(hsv, lower_green , upper_green )
    # Remove noise
    dilated_mask_green = cv2.dilate(mask_green , kernel_dilate_green, iterations=1)
    # print(np.sum(dilated_mask_red))
    # print (" ------------------------------------------------------- ")
    # print(np.sum(dilated_mask_green))
    if np.sum(dilated_mask_red) > 17500:
        return 1
    elif np.sum(dilated_mask_green) > 19000:
        return 2
    else:
        return 3

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
    if line_angle!= 0:
        set_steer_angle(op)
    else:
        robot.setSteeringAngle(0)
robot.setCruisingSpeed(30)
reds_detected = list()
green_detected = False
while robot.step() != -1:

    im = front_camera.getImageArray()
    signal = process_image(im)
    if signal == 1:
        # print("red light detected stopping now")
        reds_detected.append(signal)   
        robot.setBrakeIntensity(1)
        robot.setCruisingSpeed(0)
    elif signal == 2:
        # print("green detected, going ahead")
        green_detected = True   
        robot.setCruisingSpeed(30)
        sleep(2)     
    else:
        if green_detected :
            process_camera_image()
            robot.setCruisingSpeed(37)
    sleep(0.1)
