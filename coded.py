#Import libraries
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from gpiozero import LED,DistanceSensor,LineSensor,Motor
import serial

#Constatnts
DISTANCE_MULTIPLIER = 2
ANGLE_MULTIPLIER = 5
CRASH_DISTANCE

#Filter values to filter 'tennis ball yellow' for image processing
VALUE = [33, 60, 95, 45, 255, 255]
print("Filter:",value)

#Lower and upper yellow filter using values above
LOWER_YELLOW = np.array([VALUE[0], VALUE[1], VALUE[2]], dtype=np.uint8)
UPPER_YELLOW = np.array([VALUE[3],VALUE[4],VALUE[5]], dtype=np.uint8)

#Set up camera and give it time to warm up
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(1)

#Serial infomation to communicate with raspberry pi
PORT = "/dev/ttyACM0"
BAUD = 115200
s = serial.Serial(PORT)
s.baudrate = BAUD
s.parity   = serial.PARITY_NONE
s.databits = serial.EIGHTBITS
s.stopbits = serial.STOPBITS_ONE
s.readline()

#Setup distance sensors
sensor1 = DistanceSensor(19, 6)
sensor2 = DistanceSensor(26, 13)

#Set up IR sensors
IR1 = LineSensor(
IR2 = LineSensor(
        

#Set up GPIO pins for motors
lmotor = Motor(17,27)
rmotor = Motor(23,22)
#GPIO pins to control the relay
spinners = LED(14)
switch = LED(15)

#Turning the spinners off but the light on to indicate the program has started
switch.off()
spinners.on()

#Function that uses the distance sensors to check if the robot is going to crash
def crash():
    global crashed
    dist = (sensor1.distance + sensor2.distance)/2
    if dist < CRASH_DISTANCE:
        crashed = True
        return True
    else:
        if int(IR1.value) == 0 or int(IR2.value) == 0:
            time.sleep(0.1)
            if int(IR1.value) == 0 or int(IR2.value) == 0:
                crashed = True
                return True
    return False
            
            
            

#code due to be added for this  
def compass():
    return 5

def IR():
    return 6
    
#Functions for movement
def stop():
    lf.off()
    lb.off()
    rf.off()
    rb.off()
def forward(distance):
    print("forward",distance)
    lmotor.foward(speed = 1)
    rmotor.forward(speed = 1)
    time.sleep(distance*DISTANCE_MULTIPLIER**0.8)
    old = time.time()
    while time.time() <= (old+(0.8*(distance*DISTANCE_MULTIPLIER)):
        if crash():
            stop()
            return
    lmotor.foward(speed = 0.2)
    rmotor.forward(speed = 0.2)
    time.sleep(distance*DISTANCE_MULTIPLIER)
    old = time.time()
    while time.time() <= (old+(0.8*(distance*DISTANCE_MULTIPLIER)):
        if crash():
            stop()
            return
    stop()
def backward(distance):
    print("backward",distance)
    lmotor.backward(speed = 1)
    rmotor.backward(speed = 1)
    time.sleep(distance)
    stop()
def right(angle):
    global ANGLE_MULTIPLIER 
    print("right",angle)
    lmotor.backward(speed = 1)
    rmotor.forward(speed = 1)
    time.sleep(angle*ANGLE_MULTIPLIER)
    stop()
def left(angle):
    global ANGLE_MULTIPLIER 
    print("left",angle)
    lmotor.foward(speed = 1)
    rmotor.backward(speed = 1)
    time.sleep(angle*ANGLE_MULTIPLIER)
    stop()

HOME = compass()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    frame = frame.array

    #Convert image to right colour space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #creates a mask with only yellow and gets rid of any small blobs that may have passed through
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    #cv2.imshow("sdgfdfg",mask)
    moments = cv2.moments(mask)
    area = moments['m00']
    print(area)
    if(area > 10000):
        x = moments['m10'] / area
        y = moments['m01'] / area
        print(x,y)
        cv2.circle(frame, (int(x), int(y)), 10, (0,0,0))

        if x > 330:
           right((x-320)/(320*12))
        elif x < 310:
           left((320-x)/(320*12))
        elif ((310 <= x <= 330) and area<1500000):
           relay.off()
           forward(0.5)
           time.sleep(1)
           relay.on()
           time.sleep(1)
        elif ((310 <= x <= 330) and area>4000000):
           backward(0.05)
        if crashed == True:
            
    cv2.imshow("f",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    rawCapture.truncate(0)
    




