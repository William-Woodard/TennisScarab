from bluedot import BlueDot
from gpiozero import Robot
from signal import pause
from picamera import PiCamera
from time import sleep
from gpiozero import LED 
import time

# camera = PiCamera()

bd = BlueDot()
# robot = Robot(left=(27, 17), right=(24, 23))
lf = LED(17)
lb = LED(27)
rf = LED(23)
rb = LED(22)
relay = LED(14)
switch = LED(15)
switch.off()
relay.on()
rel = True
def move(pos):
    #print(pos)
    if pos.top:
        print("forward")
        lf.on()
        rf.on()
    elif pos.bottom:
        print("backwartds")
        lb.on()
        rb.on()
    elif pos.left:
        print("left")
        lf.on()
        rb.on()
    elif pos.right:
        print("right")
        rf.on()
        lb.on()

def stop():
    lf.off()
    lb.off()
    rf.off()
    rb.off()
def snap():
    global rel
    if rel == True:
        relay.on()
        rel = False
    else:
        relay.off()
        rel = True
        
# def snap():
#     print("snap!!")
#     t = time.time()
#     name = str('/home/pi/Desktop/photos/') + str(t) + str('.jpg')
#     camera.capture(name)

bd.when_pressed = move
bd.when_moved = move
bd.when_released = stop
bd.when_double_pressed = snap

pause()

