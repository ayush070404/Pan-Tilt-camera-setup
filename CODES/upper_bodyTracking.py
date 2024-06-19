
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


        import cv2 as cv
        import time
        from picamera2 import Picamera2

        picam2 = Picamera2()
        dispW = 160
        dispH = 90
        picam2.preview_configuration.main.size = (320, 180)
        picam2.preview_configuration.main.format = "RGB888"
        picam2.preview_configuration.controls.FrameRate = 20
        picam2.preview_configuration.align()
        picam2.configure("preview")
        picam2.start()

        fps = 0
        pos = (30, 60)
        font = cv.FONT_HERSHEY_SIMPLEX
        height = 1.5
        color = (0, 0, 255)
        weight = 3


        upper_body_Cascade = cv.CascadeClassifier('./haar/haarcascade_upperbody.xml')

        while True:
            timeStart = time.time()
            frame = picam2.capture_array()
            frameGray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            bodies = upper_body_Cascade.detectMultiScale(frameGray)

            
            for (x, y, w, h) in bodies:
                cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
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
            cv.putText(frame, f'FPS: {int(fps)}', pos, font, height, color, weight)
            cv.imshow("Camera", frame)

            # Check for 'q' key to quit
            if cv.waitKey(1) == ord('q'):
                break

            timeEnd = time.time()
            timeLoop = timeEnd - timeStart

            # Calculate and update FPS
            fps = 0.9 * fps + 0.1 * (1 / timeLoop)

        # Clean up
        cv.destroyAllWindows()
