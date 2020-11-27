"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Display
from vehicle import Driver
import numpy as np

unk = 99999.99
# create the Robot instance.
robot = Driver()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# enable cameras
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
front_camera.enable(30)
rear_camera.enable(30)

# start engine and set cruising speed
robot.setCruisingSpeed(20)


# get's the average/mean value of the image passed to it
# image is a 3d array of BGRA values
def avg_out_images(image):

    avged_image = [[0 for j in range(len(image[0]))] for _ in range(len(image))]

    for i in range(len(image)):
        for j in range(len(image[0])):
            avged_image[i][j]= np.mean(image[i][j][:-1])

    mean = np.mean(avged_image, axis=None)
    return mean    

# camera processing stuff
def process_camera_image(image):
    cam_height = front_camera.getHeight()
    cam_width  = front_camera.getWidth()
    cam_fov    = front_camera.getFov()
    
    num_pixels = cam_height*cam_width
    image = np.frombuffer(image, np.uint8).reshape((cam_height, cam_width, 4))

    # consider bottom 1/4th of the image for processsing
    image = image[int(cam_height*0.75):,:]
    
    # split into 3 parts
    left_part_of_image   = image [:, 5:int(cam_width*0.33)]
    center_part_of_image = image [:, int(cam_width*0.33):int(cam_width*0.66)]
    right_part_of_image  = image [:, int(cam_width*0.66):-5]
    left_avged   = avg_out_images(left_part_of_image)
    center_avged = avg_out_images(center_part_of_image)
    right_avged  = avg_out_images(right_part_of_image)
    print("left_avged = ",left_avged.round(1), ", centre_avged = ", center_avged.round(1), ", right_avged = ", right_avged.round(1))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    process_camera_image(front_camera.getImage())

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
# Enter here exit cleanup code.