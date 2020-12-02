"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Display, Lidar
from vehicle import Driver
import numpy as np
import cv2
import imutils
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
targetImage = cv2.imread("/Users/suryakanoria/Downloads/CSCI5302-AVChallenge/sample.jpg")
targetImage = targetImage[:20,:,:]
# targetImage = imutils.resize(targetImage, width=20,height=20)
# get's the average/mean value of the image passed to it
# image is a 3d array of BGRA values

def meanSquareError(img1, img2):
    assert img1.shape == img2.shape, "Images must be the same shape."
    error = np.sum((img1.astype("float") - img2.astype("float")) ** 2)
    error = error/float(img1.shape[0] * img1.shape[1] * img1.shape[2])
    return error

def compareImages(img1, img2):
    return 1/meanSquareError(img1, img2)
    

def sliding_window(image, stepSize, windowSize):
    for y in xrange(0, image.shape[0], stepSize):
        for x in xrange(0, image.shape[1], stepSize):
            yield (x, y, image[y:y+windowSize[0], x:x+windowSize[1]])
            
             

# camera processing stuff
def process_camera_image(image):
    cam_height = front_camera.getHeight()
    cam_width  = front_camera.getWidth()
    cam_fov    = front_camera.getFov()
    num_pixels = cam_height*cam_width
    image = np.frombuffer(image, np.uint8).reshape((cam_height, cam_width, 4))
    image = image[:,:,:3]
    left_part_of_image   = image [:, 5:int(cam_width*0.33)]
    # left_part_of_image   = image [50:100,5:50]
    maxSim = 0
    maxBox = None
    for (x, y, window) in sliding_window(left_part_of_image, stepSize = 5, windowSize = targetImage.shape):
        if window.shape[0] != targetImage.shape[0] or window.shape[1] != targetImage.shape[1]:
            continue
        tempSim = compareImages(targetImage, window)
        if(tempSim > maxSim):
            maxSim = tempSim
            maxBox = (x, y, targetImage.shape[0], targetImage.shape[1])
    if(maxBox != None and maxBox[0] < 10 and maxSim > 0.0006):
        print(maxSim,maxBox)
        print("STOP sign in %s m!!" % str(maxBox[1]))
    # print(maxSim,maxBox)
    

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