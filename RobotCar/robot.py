from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import XVLidar as LaserModel

from xvlidar import XVLidar as Lidar
from pltslamshow import SlamShow

from math import sin, cos, pi

from gopro import GoPro
import RPi.GPIO as GPIO

import serial
import time
import curses

# Define constant
#SLAM
MAP_SIZE_METERS         = 5 #1 m = 1000 mm
MAP_SIZE_PIXELS         = 1000 #change scale to 1 mm
RANGE                   = MAP_SIZE_PIXELS/2
LIDAR_DEVICE            = '/dev/ttyS0'
RIGHT                   = -90
LEFT                    = 90
ROTATE                  = 180

#GoPro
#degree : turnspeed
turnspeed = {0:6.1, 30:6.5, 60:6.72, 90:6.7, 120:6.65}

class Robot():
    def __init__(self):
        #self.gopro = Gopro()
        
        self.arduino = Arduino()
        
        self.slam = Slam()

    # Gopro module
    def capture(self):
        (self.gopro).task_capture()
        
    # Arduino module
    def sendCommand(self, instruction):
        list = instruction.split()
        command = list[0]
        arg1 = list[1]
        arg2 = list[2]
        (self.arduino).task_sendCommand(command, arg1, arg2)

    # Slam module
    def update(self):
        (self.slam).task_update()

    def display(self):
        (self.slam).task_display()

class Gopro():
    def __init__(self):
        self.camera = GoPro.GoPro()
        self.camera.mode('photo')
        
    def verticalServoSetup(self):
        servoV = 18
        GPIO.setup(servoV,GPIO.OUT)
        pwmV = GPIO.PWM(servoV,50)
        
    def horizontalServoSetup(self):
        #setup horizontal servo
        servoH = 11
        GPIO.setup(servoH,GPIO.OUT)
        pwmH = GPIO.PWM(servoH,50)
        
    def getDutyCycle(self, degree):
        #(degree,dutycycle): (0,2.15),(180,9.5)
        m = (9.5-2.15)/180.0
        dutyCycle = m*(degree)+2.15
        return dutyCycle

    def task_capture(self):
        GPIO.setmode(GPIO.BOARD)
        degreeV = 0
        stopH = 0
        self.verticalServoSetup()
        self.horizontalServoSetup()
        
        pwmV.start(getDutyCycle(0))
        time.sleep(2)
        
        #0, 30, 60, 90, 120
        while(degreeV <= 120):
            #DesiredDegree = input("Enter degree: ")
            print degreeV
            degreeV = degreeV + 30
            pwmV.ChangeDutyCycle(getDutyCycle(degreeV))
            pwmH.start(stopH)
            time.sleep(1)
            for i in range(0,21):
                pwmH.ChangeDutyCycle(turnspeed[degreeV])
                time.sleep(0.03)
                pwmH.ChangeDutyCycle(stopH)
                time.sleep(1)
                camera.capture()
                time.sleep(1)

        pwmV.ChangeDutyCycle(getDutyCycle(80))
        time.sleep(0.5)

        pwmV.stop()
        pwmH.stop()
        GPIO.cleanup()
        
class Arduino():
        def __init__(self):
                self.setSerialPort()

        def setSerialPort(self):
                try:
                        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
                except:
                        self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=0)
                
        def task_sendCommand(self, command, degree, dist):
                delay = self.getDelay(float(dist))
                message = str(command) + ' ' + str(degree) + ' ' + str(delay)
                self.ser.write(message)
                time.sleep(1)
                
        def getDelay(self,dist):
                return -4.036e-09*math.pow(dist, 5) - 9.631e-07*math.pow(dist, 4) + 0.0002869*math.pow(dist, 3) + 0.02265 * math.pow(dist, 2) + 7.92 *  dist - 8.898
    
class Slam():
    def __init__(self):
        # Connect to Lidar unit
        self.lidar = Lidar(LIDAR_DEVICE)

        # Create an RMHC SLAM object with a laser model and optional robot model
        self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

        # Initialize empty map
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

        # Set up a SLAM display
        self.display = SlamShow(MAP_SIZE_PIXELS, MAP_SIZE_METERS*1000/MAP_SIZE_PIXELS, 'SLAM')

    def task_update(self):
        # Update SLAM with current Lidar scan, using first element of (scan, quality) pairs
        (self.slam).update([pair[0] for pair in (self.lidar).getScan()])
        (self.slam).getmap(self.mapbytes)

    def task_getMap(self):
        # Get current map bytes as grayscale
        (self.slam).getmap(self.mapbytes)

    def task_getPosition(self):
        # Get current robot position (x, y, theta)
        return self.slam.getpos()

    def task_display(self):
        x, y, theta = self.task_getPosition()
        
        (self.display).setPose(x, y, theta)
        (self.display).displayMap(self.mapbytes)
        
        key = (self.display).refresh()
        if key != None and (key&0x1A):
           exit(0)

    ## Map-Interpretation
    def task_exploreForward(self, distance):
        x, y, theta = self.task_getPosition()
        self.task_getMap()
        return self.getForwardDistance(self.mapbytes, x, y, theta, distance)

    def task_exploreBackward(self, distance):
        x, y, theta = self.task_getPosition()
        self.task_getMap()
        return self.getForwardDistance(self.mapbytes, x, y, theta+180, distance)

    def task_exploreLeft(self, distance):
        x, y, theta = self.task_getPosition()
        self.task_getMap()
        return self.getLeftDistance(self.mapbytes, x, y, theta, distance)

    def task_exploreRight(self, distance):
        x, y, theta = self.task_getPosition()
        self.task_getMap()
        return self.getRightDistance(self.mapbytes, x, y, theta, distance)

    def getForwardDistance(self, mapbytes, x, y, theta, minDist):
        coordX = x - RANGE
        coordY = y - RANGE
         
        # convert degree to radian and rotate heading left 90 degree
        angle_radL = self.getRadian(theta + LEFT)
        # Find minvalue dist left side car
        minDist = self.getMinDistance(mapbytes, coordX, coordY, theta, angle_radL, minDist,3)

        # convert degree to radian and rotate heading right 90 degree
        angle_radR = self.getRadian(theta + RIGHT)
        # Find minvalue dist right side car
        minDist = self.getMinDistance(mapbytes, coordX, coordY, theta, angle_radR, minDist,3)
        return minDist

    def getLeftDistance(self, mapbytes, x, y, theta, minDist):
        coordX = x - RANGE
        coordY = y - RANGE
        # rotate 180 degree find back center 
        angle_rotate = self.getRadian(theta + ROTATE)
        dX,dY = self.getCoordinate(coordX, coordY, angle_rotate,35)
        
        # convert degree to radian and rotate heading left 90 degree
        angle_radL = self.getRadian(theta+LEFT)
        # Find minvalue dist left side car
        minDist = self.getMinDistance(mapbytes, dX - RANGE, dY - RANGE, theta, angle_radL, minDist, 2)
        return minDist

    def getRightDistance(self, mapbytes, x, y, theta, minDist):
        coordX = x - RANGE
        coordY = y - RANGE
        # rotate 180 degree find back center
        angle_rotate = self.getRadian(theta + ROTATE)
        dX, dY = self.getCoordinate(coordX, coordY, angle_rotate, 35)
        
        # convert degree to radian and rotate heading right 90 degree
        angle_radR = self.getRadian(theta + RIGHT)
        # Find minvalue dist right side car
        minDist = self.getMinDistance(mapbytes, dX - RANGE, dY - RANGE, theta, angle_radR, minDist, 2)
        return minDist

    def getMinDistance(self, mapbytes, coordX, coordY, theta, shiftdegree, minDist, numshift):
        for shift_mm in range(1, numshift):
            dx, dy = self.getCoordinate(coordX, coordY, shiftdegree, shift_mm)
            dist = self.getSinglelineDistance(mapbytes, dx, dy, theta, minDist)
            minDist = min(dist, minDist)
        return minDist

    def getSinglelineDistance(self, mapbytes, x, y, theta, maxDist):
        coordX = x - RANGE
        coordY = y - RANGE
        angle_rad = theta * pi / 180.0
        for dist_mm in range(1, maxDist + 1):
            dx = coordX + cos(angle_rad) * dist_mm
            dy = coordY + sin(angle_rad) * dist_mm
            I = (int)(-(dy - RANGE))
            J = (int)( (dx + RANGE))
            index = I * RANGE + J
        
            if mapbytes[index] < 127:
                return dist_mm
        
        return maxDist

    def getCoordinate(self, coordX, coordY, angle_rad, dist_mm):
        dx = coordX + cos(angle_rad) * dist_mm + (MAP_SIZE_PIXELS/2)
        dy = coordY + sin(angle_rad) * dist_mm + (MAP_SIZE_PIXELS/2)
        #print dx ,dy
        return dx, dy

    def getRadian(self,theta):
        return ((theta % 360)*pi) / 180.0

if __name__ == '__main__':
    
    robot = Robot()
    while True:
        robot.update()
        robot.display()
