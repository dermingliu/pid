#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Programmed by DM.Liu at Taipei 29,10,2021
#
import PID
import time
import os.path
import RPi.GPIO as GPIO
fan = 13
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(fan,GPIO.OUT)
pwm = GPIO.PWM(fan,1000)
pwm.start(0)

targetT = 43
P = 10
I = 2
D = 0.1

pid = PID.PID(P, I, D)
pid.SetPoint = targetT
pid.setSampleTime(0.2)
feedback = 0

def readConfig ():
	global targetT
	with open ('/home/pi/pid/pid-1.conf', 'r') as f:
		config = f.readline().split(',')
		pid.SetPoint = float(config[0])
		targetT = pid.SetPoint
		pid.setKp (float(config[1]))
		pid.setKi (float(config[2]))
		pid.setKd (float(config[3]))

def createConfig ():
#	if not os.path.isfile('/home/pi/pid/pid-1.conf'):
		with open ('/home/pi/pid/pid-1.conf', 'w+') as f:
			f.write('%s,%s,%s,%s'%(targetT,P,I,D))

createConfig()

while 1:
	readConfig()
        tFile = open('/sys/class/thermal/thermal_zone0/temp')
        temp = float(tFile.read())
        temperature = temp/1000
	pid.update(temperature)
	t = pid.output
	print ("cpu temperature",t)
	if t < 0:
	        target = abs(t)
		print (target)
        	targetPwm = max(min( int(target), 100 ),0)
		print "Target: %.1f C | Current: %.1f C | PWM: %s %%"%(targetT, temperature, targetPwm)
		pwm.ChangeDutyCycle(targetPwm)
		time.sleep(0.1)
	else:
		targetPwm = 0
		print ("Stable or counting.......")
                pwm.ChangeDutyCycle(targetPwm)
                time.sleep(3)
