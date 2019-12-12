from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

PWM1 = 12
MOT11= 16
MOT12= 18

PWM2 = 35
MOT21= 33
MOT22= 37

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40, GPIO.OUT)
GPIO.output(40, GPIO.HIGH)
GPIO.setup(31, GPIO.OUT)
GPIO.output(31, GPIO.HIGH)

GPIO.setup(PWM1,GPIO.OUT)
GPIO.setup(MOT11,GPIO.OUT)
GPIO.setup(MOT12,GPIO.OUT)

GPIO.setup(PWM2,GPIO.OUT)
GPIO.setup(MOT21,GPIO.OUT)
GPIO.setup(MOT22,GPIO.OUT)

pwm1 = GPIO.PWM(PWM1,1000)  
pwm2 = GPIO.PWM(PWM2,1000)
pwm1.start(0)
pwm2.start(0)

camera = PiCamera()
camera.resolution = (360, 360)
camera.rotation = 180
camera.framerate = 15
rawCapture = PiRGBArray(camera, size=(360, 360))
time.sleep(0.1)

x_last = 320
y_last = 180

counter=0
start_time = time.time()

def find_marker(image):
    image = cv2.GaussianBlur(image,(3,3),0)    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.3, 100)
    if circles is not None:
            circles = np.round(circles[0, :]).astype("int")    
    else:
            return(None)
            
    return(circles)
def distance_to_camera(knownWidth, focalLength, perWidth):
    return (knownWidth * focalLength) / perWidth
KNOWN_WIDTH = 3
focalLength = 500

def control(v,steer):
    center=230
    print steer
    if v<0:
        v0=-v
    else :
        v0=v
    if steer == 0:
        print "Go"
        v1 = v0
        v2 = v0
    elif steer > 0:
        print "Turn_right"
        steer = center-steer
        v1 = v0
        v2 = int(v0*steer/center)
    elif steer < 0:
        print "Turn_left"
        steer = center+steer
        v2 = v0
        v1 = int(v0*steer/center)
    #print "V1=",v1," V2=",v2
    if v>0:
        pwm1.ChangeDutyCycle(v2)
        pwm2.ChangeDutyCycle(v1)
        GPIO.output(MOT11, 1)
        GPIO.output(MOT12, 0)
        GPIO.output(MOT21, 1)
        GPIO.output(MOT22, 0)
    else:
        print "Back"
        pwm1.ChangeDutyCycle(v1)
        pwm2.ChangeDutyCycle(v2)
        GPIO.output(MOT11, 0)
        GPIO.output(MOT12, 1)
        GPIO.output(MOT21, 0)
        GPIO.output(MOT22, 1)
        time.sleep(0.5)

V_turn=60
time1=0.75
time2=1
def turn_left():
    GPIO.output(MOT11, 1)
    GPIO.output(MOT12, 0)
    GPIO.output(MOT21, 1)
    GPIO.output(MOT22, 0)
    control(1,0)
    time.sleep(time1)
    control(V_turn,120)
    time.sleep(time2)
    control(1,0)
    time.sleep(time1)
    control(V_turn,-120)
    time.sleep(time2)
    control(1,0)
    time.sleep(5)
    

def turn_right():
    GPIO.output(MOT11, 1)
    GPIO.output(MOT12, 0)
    GPIO.output(MOT21, 1)
    GPIO.output(MOT22, 0)
    control(1,0)
    time.sleep(time1)
    control(V_turn,-120)
    time.sleep(time2)
    control(1,0)
    time.sleep(time1)
    control(V_turn,120)
    time.sleep(time2)
    control(1,0)
    time.sleep(5)
kp = .75
ap = 1
old_steer=0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    counter = counter+1
    image1 = frame.array
    image = image1[160:360, 0:360]
    marker = find_marker(image1)
    print marker
    if marker is not None:    
        for (x, y, r) in marker:   
            cm = distance_to_camera(KNOWN_WIDTH, focalLength, 2*r)                
            cv2.circle(image1, (x, y), r, (0, 0, 255), 3)
            cv2.rectangle(image1, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            cv2.putText(image1, "%.1fcm" % cm,(x, y), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
            if cm<=20:
                if x<=180:
                    print "Left"
                    turn_left()
                else:
                    print "Right"
                    turn_right()
    else:
        kernel = np.ones((3,3), np.uint8)
        Blackline = cv2.inRange(image, (0,0,0), (75,75,75))	 
        Blackline = cv2.erode(Blackline, kernel, iterations=9)
        #Blackline = cv2.dilate(Blackline, kernel, iterations=5)	
        contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #print contours_blk
        contours_blk_len = len(contours_blk)
        if contours_blk_len > 0 :
             if contours_blk_len == 1 :
                  blackbox = cv2.minAreaRect(contours_blk[0])
             else:
                   canditates=[]
                   off_bottom = 0	   
                   for con_num in range(contours_blk_len):		
                        blackbox = cv2.minAreaRect(contours_blk[con_num])
                        (x_min, y_min), (w_min, h_min), ang = blackbox		
                        box = cv2.cv.BoxPoints(blackbox)
                        (x_box,y_box) = box[0]
                        if y_box > 358 :		 
                         off_bottom += 1
                        canditates.append((y_box,con_num,x_min,y_min))		
                   canditates = sorted(canditates)
                   if off_bottom > 1:	    
                        canditates_off_bottom=[]
                        for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
                           (y_highest,con_highest,x_min, y_min) = canditates[con_num]		
                           total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                           canditates_off_bottom.append((total_distance,con_highest))
                        canditates_off_bottom = sorted(canditates_off_bottom)         
                        (total_distance,con_highest) = canditates_off_bottom[0]         
                        blackbox = cv2.minAreaRect(contours_blk[con_highest])	   
                   else:		
                        (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]		
                        blackbox = cv2.minAreaRect(contours_blk[con_highest])	 
             (x_min, y_min), (w_min, h_min), ang = blackbox
             x_last = x_min
             y_last = y_min
             if ang < -45 :
                 ang = 90 + ang
             if w_min < h_min and ang > 0:
                 ang = (90-ang)*-1
             if w_min > h_min and ang < 0:
                 ang = 90 + ang	  
             setpoint = 180
             error = int(x_min - setpoint) 
             ang = int(ang)
             old_steer = error*kp+ang*ap
             control(35,error*kp+ang*ap)
             
             box = cv2.cv.BoxPoints(blackbox)
             box = np.int0(box)
             cv2.drawContours(image,[box],0,(0,0,255),3)	 
             cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
             cv2.putText(image,str(error),(310, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
             cv2.line(image, (int(x_min),0 ), (int(x_min),360 ), (255,0,0),3)
        else :
            control(-35,old_steer)
    cv2.imshow("All", image1)
    #cv2.imshow("Interest", image)	
    rawCapture.truncate(0)	
    key = cv2.waitKey(1) & 0xFF	
    if key == ord("q"):
         break
stop_time = time.time()
frame_number = counter/(stop_time - start_time)
print "FPS: "
print frame_number
GPIO.output(40, GPIO.LOW)


