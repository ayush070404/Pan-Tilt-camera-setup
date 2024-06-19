
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

        dispW=320
        dispH=180
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
        
        faceCascade = cv2.CascadeClassifier('./haar/haarcascade_frontalface_default.xml')
        eyeCascade = cv2.CascadeClassifier('./haar/haarcascade_eye.xml')

        

        
        while True:
            timeStart = time.time()
            frame= picam2.capture_array()
            frameGray =cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = faceCascade.detectMultiScale(frameGray, 1.3, 5)
            for face in faces:
                x,y,w,h= face
                cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
                pan_error=(x+w/2)-dispW/2
                panAngle = panAngle - pan_error/120
                if abs(pan_error)>50:
                    pan.set_angle(panAngle)
               
                tiltError=(y+h/2)-dispH/2
                tiltAngle = tiltAngle + tiltError/120
                if tiltAngle>40:
                    tiltAngle=40
                if tiltAngle<-60:
                    tiltAngle=-60
                if abs(tiltError)>50:
                    
                    tilt.set_angle(tiltAngle)
                    
                roiGray = frameGray[y:y+h, x:x+w]
                roiColor = frame[y:y+h, x:x+w]
                eyes = eyeCascade.detectMultiScale(roiGray)
                for eye in eyes:
                    x,y,w,h = eye
                    cv2.rectangle(roiColor, (x,y), (x+w, y+h), (255,0,0), 2)
            cv2.putText(frame, str(int(fps)), pos ,font ,height ,myColor , weight)
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1)==ord('q'):
                break
            timeEnd = time.time()
            timeLoop = timeEnd -timeStart
            fps = 0.9*fps +0.1*( 1/timeLoop)
           # print(fps)
           
        cv2.destroyAllWindows()
                    
                    
            


