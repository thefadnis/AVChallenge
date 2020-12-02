"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Display
from vehicle import Driver
import numpy as np
import cv2
# create the Robot instance.
robot = Driver()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
steering_angle = 0
# enable cameras
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
front_camera.enable(30)
rear_camera.enable(30)

# start engine and set cruising speed
robot.setCruisingSpeed(50)
  

# camera processing stuff
def process_camera_image(image):
    cam_height = front_camera.getHeight()
    cam_width  = front_camera.getWidth()
    cam_fov    = front_camera.getFov()
    # print("h: ", cam_height, " w: ", cam_width)
    num_pixels = cam_height*cam_width
    image = np.frombuffer(image, np.uint8).reshape((cam_height, cam_width, 4))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    print(gray)
    # cv2.imshow(gray)
    # cv2.WaitKey(0)
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # Define range of white color in HSV
    lower_white = np.array([0, 0, 212])
    upper_white = np.array([131, 255, 255])
    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_white, upper_white)
    # Remove noise
    kernel_erode = np.ones((4,4), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((6,6),np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    contours, hierarchy = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour = list()
    line_angle = 1.57
    if contours:
        for i in contours:
            for j in i:
                for k in j:
                    contour.append(k)
            # print(contour)
            p1 = [ (contour[0][0] + contour[3][0])/2, (contour[0][1] + contour[3][1])/2]
            p2 = [ (contour[1][0] + contour[2][0])/2, (contour[1][1] + contour[2][1])/2]
            line_angle = np.arctan2( [p2[1] -p1[1]], [ p2[0]-p1[0] ]  )
            
    return line_angle

def pid(mean_cont):
    # setting gain coeffs
    kp, kd, ki = 1, 1.25, 0.006

    old_val_range = 1.57
    integral = 0
    
    diff = mean_cont - old_val_range
    integral += mean_cont
    
    op = kp*mean_cont + ki*integral + kd*diff
    # print(op, mean_cont)
    return kp*mean_cont + kd*diff
    
def set_steer_angle(yangil):
    global steering_angle
    supposed_angle = 1.57
    if (yangil - supposed_angle) >= 1:
       steering_angle += yangil - supposed_angle
    elif (yangil-supposed_angle) <= -1:
       steering_angle += yangil-supposed_angle
    else:
       steering_angle = 0
    
    if steering_angle > 0.5  : steering_angle = 0.5
    if steering_angle < -0.5 : steering_angle = -0.5
    robot.setSteeringAngle(steering_angle)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    contour = process_camera_image(front_camera.getImage())
    if contour:

        pid_op = pid(np.mean(contour))
        # print(pid_op)
        # set_steer_angle(pid_op)
    else:
        robot.setSteeringAngle(0)
        
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
       
       
       
'''
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

// filter angle of the yellow line (simple average)
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else {  // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}


double applyPID(double yellow_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (PID_need_reset) {
    oldValue = yellow_line_angle;
    integral = 0.0;
    PID_need_reset = false;
  }

  // anti-windup mechanism
  if (signbit(yellow_line_angle) != signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;

  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;

  oldValue = yellow_line_angle;
  return KP * yellow_line_angle + KI * integral + KD * diff;
}

'''
        
# Enter here exit cleanup code.