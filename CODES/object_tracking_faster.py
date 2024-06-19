
import pigpio
from time import sleep
# Start the pigpiod daemon
import subprocess
result = None
status = 1
for x in range(3):
    p = subprocess.Popen('sudo pigpiod', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    if status == 0:
        break
    sleep(0.2)
if status != 0:
    print(status, result)
'''
> Use the DMA PWM of the pigpio library to drive the servo
> Map the servo angle (0 ~ 180 degree) to (-90 ~ 90 degree)

'''

class Servo():
    MAX_PW = 1250  # 0.5/20*100
    MIN_PW = 250 # 2.5/20*100
    _freq = 50 # 50 Hz, 20ms
 
    def __init__(self, pin, min_angle=-60, max_angle=60):

        self.pi = pigpio.pi()
        self.pin = pin 
        self.pi.set_PWM_frequency(self.pin, self._freq)
        self.pi.set_PWM_range(self.pin, 10000)      
        self.angle = 0
        self.max_angle = max_angle
        self.min_angle = min_angle
        self.pi.set_PWM_dutycycle(self.pin, 0)

    def set_angle(self, angle):
        if angle > self.max_angle:
            angle = self.max_angle
        elif angle < self.min_angle:
            angle = self.min_angle
        self.angle = angle
        duty = self.map(angle, -60, 60, 250, 1250)
        self.pi.set_PWM_dutycycle(self.pin, duty)


    def get_angle(self):
        return self.angle

    # will be called automatically when the object is deleted
    # def __del__(self):
    #     pass

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ =='__main__':
    '''from vilib import Vilib
    Vilib.camera_start(vflip=True,hflip=True) 
    Vilib.display(local=True,web=True)'''

    pan = Servo(pin=23, max_angle=60, min_angle=-60)
    tilt = Servo(pin=18, max_angle=60, min_angle=-60)
    panAngle = 0
    tiltAngle = 0
    pan.set_angle(panAngle)
    tilt.set_angle(tiltAngle)
    sleep(1)

    while True:
        for angle in range(0, 60, 1):
            pan.set_angle(angle)
            tilt.set_angle(angle)
            sleep(.01)
        sleep(.5)
        for angle in range(60, -60, -1):
            pan.set_angle(angle)
            tilt.set_angle(angle)
            sleep(.01)
        sleep(.5)
        for angle in range(-60, 0, 1):
            pan.set_angle(angle)
            tilt.set_angle(angle)
            sleep(.01)
        sleep(.5)




        import cv2
        from picamera2 import Picamera2
        import time
        import numpy as np
        
        picam2 = Picamera2()

        pan=Servo(pin=23)
        tilt=Servo(pin=18)

        panAngle=0
        tiltAngle=0

        pan.set_angle(panAngle)
        tilt.set_angle(tiltAngle)

        dispW=640
        dispH=360
        picam2.preview_configuration.main.size = (dispW,dispH)
        picam2.preview_configuration.main.format = "RGB888"
        picam2.preview_configuration.controls.FrameRate=30
        picam2.preview_configuration.align()
        picam2.configure("preview")
        picam2.start()
        fps=0
        pos=(30,60)
        font=cv2.FONT_HERSHEY_SIMPLEX
        height=1.5
        weight=3
        myColor=(0,0,255)
        track=0

        def onTrack1(val):
            global hueLow
            hueLow=val
            print('Hue Low',hueLow)
        def onTrack2(val):
            global hueHigh
            hueHigh=val
            print('Hue High',hueHigh)
        def onTrack3(val):
            global satLow
            satLow=val
            print('Sat Low',satLow)
        def onTrack4(val):
            global satHigh
            satHigh=val
            print('Sat High',satHigh)
        def onTrack5(val):
            global valLow
            valLow=val
            print('Val Low',valLow)
        def onTrack6(val):
            global valHigh
            valHigh=val
            print('Val High',valHigh)
        def onTrack7(val):
            global track
            track=val
            print('Track Value',track)

        cv2.namedWindow('myTracker')

        cv2.createTrackbar('Hue Low','myTracker',10,179,onTrack1)
        cv2.createTrackbar('Hue High','myTracker',20,179,onTrack2)
        cv2.createTrackbar('Sat Low','myTracker',100,255,onTrack3)
        cv2.createTrackbar('Sat High','myTracker',255,255,onTrack4)
        cv2.createTrackbar('Val Low','myTracker',100,255,onTrack5)
        cv2.createTrackbar('Val High','myTracker',255,255,onTrack6)
        cv2.createTrackbar('Train-0 Track-1','myTracker',0,1,onTrack7)


        while True:
            tStart=time.time()
            rotated_frame= picam2.capture_array()
            frame = cv2.rotate(rotated_frame, cv2.ROTATE_180) 
            frame=cv2.flip(frame,-1)
            frameHSV=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)
            lowerBound=np.array([hueLow,satLow,valLow])
            upperBound=np.array([hueHigh,satHigh,valHigh])
            myMask=cv2.inRange(frameHSV,lowerBound,upperBound)
            myMaskSmall=cv2.resize(myMask,(int(dispW/2),int(dispH/2)))
            myObject=cv2.bitwise_and(frame,frame, mask=myMask)
            myObjectSmall=cv2.resize(myObject,(int(dispW/2),int(dispH/2)))
            
            contours,junk=cv2.findContours(myMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            if len(contours)>0:
                contours=sorted(contours,key=lambda x:cv2.contourArea(x),reverse=True)
                #cv2.drawContours(frame,contours,-1,(255,0,0),3)
                contour=contours[0]
                x,y,w,h=cv2.boundingRect(contour)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),3)
                if track==1:
                    pan_error=(x+w/2)-dispW/2
                    panAngle = panAngle - pan_error/85
                    if abs(pan_error)>50:
                        pan.set_angle(panAngle)
                   
                    tiltError=(y+h/2)-dispH/2
                    tiltAngle = tiltAngle + tiltError/85
                    if tiltAngle>40:
                        tiltAngle=40
                    if tiltAngle<-60:
                        tiltAngle=-60
                    if abs(tiltError)>50:
                        
                        tilt.set_angle(tiltAngle)
                    
                
            cv2.imshow('Camera',frame)
            cv2.imshow('Mask',myMaskSmall)
            cv2.imshow('My Object',myObjectSmall)
            if cv2.waitKey(1)==ord('q'):
                break
            tEnd=time.time()
            loopTime=tEnd-tStart
            fps=.9*fps + .1*(1/loopTime)
        cv2.destroyAllWindows()

