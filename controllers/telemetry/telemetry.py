"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Display, Lidar
from vehicle import Driver
import numpy as np
import cv2
from math import ceil
import matplotlib.pyplot as plt
# create the Robot instance.
robot = Driver()
GPS_1=robot.getGPS("gps")
GPS_1.enable(30)
gyro=robot.getGyro("gyro")
gyro.enable(30)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
steering_angle = 0
# enable cameras
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")
front_camera.enable(30)
rear_camera.enable(30)

lidar.enable(timestep)

lidar.enablePointCloud()

robot.setCruisingSpeed(50)
# camera processing stuff

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
    kp = 0.00347
    op = kp*line_angle
    if line_angle!= 0:
        set_steer_angle(op)
    else:
        robot.setSteeringAngle(0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

positionx=[]
positiony=[]
gps_velocity=[]
linear_velocity=[]
linear_accelaration=[]
steering_angle=[]
gps_angle=[]

step=0

while robot.step() != -1:
    # Process sensor data here.
    process_camera_image()    
    speed = 37
    robot.setCruisingSpeed(speed)
    # telemetry
    if step==0 or step==1 or step==2:
        # print("step 0")
        position_temp=[GPS_1.getValues()[0],GPS_1.getValues()[2]]
        initial_position=position_temp
        # print(initial_position)
    else:
        positionx.append(GPS_1.getValues()[0])
        positiony.append(GPS_1.getValues()[2])
    if abs(initial_position[0]-GPS_1.getValues()[0])<=2 and abs(initial_position[1]-GPS_1.getValues()[2])<=2 and step>=1000:
        break
    gps_velocity.append(GPS_1.getSpeed())
    linear_velocity.append(robot.getCurrentSpeed())
    steering_angle.append(robot.getSteeringAngle())
    step=step+1
    pass

plt.plot(positiony,positionx)
plt.title("Track mapping using GPS")
plt.show()

dx=list(np.gradient(positionx))
dy=list(np.gradient(positiony))
angle=[]
# for i in range(len(dx)):
    # angle.append(float(np.degrees(np.arctan2(dy[i],dx[i]))))
delete=[]
angle=list(np.degrees(np.arctan2(dy,dx)))
for i in range(len(angle)):
    if angle[i]==0:
        delete.append(i)
angle=np.delete(angle,delete)
# print(angle)

# line=np.arange(0,len(angle),1)
plt.plot(angle)
plt.title("Car Orientation (Top View)")
plt.show()

# plt.plot(gps_velocity)
plt.plot(linear_velocity)
plt.title("Linear Velocity")
# plt.legend(["GPS velocity","Input velocity"])
plt.show()

plt.plot(np.gradient(gps_velocity))
plt.title("Acceleration")
plt.show()

plt.plot(steering_angle)
plt.title("Steering angle")
plt.show()