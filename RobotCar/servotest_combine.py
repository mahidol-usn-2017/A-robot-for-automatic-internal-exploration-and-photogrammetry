from gopro import GoPro
import RPi.GPIO as GPIO
import time
import datetime

GPIO.setmode(GPIO.BOARD)
#setup vertical servo
servoV = 18
GPIO.setup(servoV,GPIO.OUT)
pwmV = GPIO.PWM(servoV,50)

#setup horizontal servo
servoH = 11
GPIO.setup(servoH,GPIO.OUT)
pwmH = GPIO.PWM(servoH,50)

print datetime.datetime.now()
#setup camera
camera = GoPro.GoPro()
camera.mode('photo')

#degree,turnspeed
turnspeed = {0:6.1, 30:6.5, 60:6.72, 90:6.7, 120:6.65}

def getDutyCycle(degree):
    #(degree,dutycycle): (0,2.15),(180,9.5)
    m = (9.5-2.15)/180.0
    dutyCycle = m*(degree)+2.15
    return dutyCycle

degreeV = 0
stopH = 0

pwmV.start(getDutyCycle(0))
time.sleep(2)

#0,30,60,90,120
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
print datetime.datetime.now()

pwmV.stop()
pwmH.stop()
GPIO.cleanup()


