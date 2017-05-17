# -*- encoding: UTF-8 -*- 

import vrep
import time
import sys
from RobotMotion import RobotMotion
from BraitenbergMotion import BraitenbergMotion
from BumperMotion import BumperMotion
from FuzzyMotion import FuzzyMotion
import time

import numpy as np
import matplotlib.pyplot as plt

def main(robotIP, robotPort, motionMode):

	#INIT V-REP SIM
    vrep.simxFinish(-1) #close all open connections
    clientID=vrep.simxStart(robotIP,robotPort,True,True,5000,5)
    #Get all the handles
    if clientID!=-1:
        print('Connected to remote API server')
        #Get the robot handle
        res1,robotHandle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)
        #Get wheel handles
        res2,rightWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
        res3,leftWheel=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)

        #Get the sensor handles
        sensorHandles = []
        for i in range(16):
            path = 'Pioneer_p3dx_ultrasonicSensor' + str(i+1)
            _,sensor=vrep.simxGetObjectHandle(clientID,path,vrep.simx_opmode_oneshot_wait)
            sensorHandles.append(sensor)

        _, gyro = vrep.simxGetObjectHandle(clientID,"GyroSensor",vrep.simx_opmode_oneshot_wait)

        motionProxy = RobotMotion(clientID, robotHandle, leftWheel, rightWheel, sensorHandles, gyro)

        if(motionMode == 1):
            motionLogic = BraitenbergMotion(motionProxy)
        elif(motionMode == 2):
            motionLogic = BumperMotion(motionProxy)
        elif(motionMode == 3):
            motionLogic = FuzzyMotion(motionProxy)
        elif(motionMode == 4):
            motionLogic = WallFollow(motionProxy)
        else:
            print('Invalid motion mode!!')
            vrep.simxFinish(-1)
            exit(-1)

        #WHILE NOT BUMPING, KEEP WALKING
        #IF OBJECT FOUND, TURN AROUND
        print ("Starting the main loop")

        NUM_COLORS = 16
        cm = plt.get_cmap('gist_rainbow')
        colors = [cm(1. * i / NUM_COLORS) for i in range(NUM_COLORS)]
        ignore = 100

        plt.scatter(motionLogic.robot.odomY, motionLogic.robot.odomX, s=10, c='PURPLE')

        while (vrep.simxGetConnectionId(clientID)!=-1):
            #Get the image from the vision sensor
            motionLogic.DoMove()
            for i in range(16):
                x_pos, y_pos = motionLogic.robot.GetSensorPoint(i)
                if x_pos != float('inf'):
                    plt.scatter(y_pos, x_pos, s=1, c=colors[i])
            if(ignore > 0): ignore -=1
            else:
                x_pos, y_pos, _ = motionLogic.robot.GetRobotPosition()
                plt.scatter(y_pos, x_pos, s=1, c='BLACK')

                plt.scatter(motionLogic.robot.odomY, motionLogic.robot.odomX, s=1, c='GRAY')

        vrep.simxFinish(-1)
        plt.gca().invert_xaxis()
        plt.show()
        exit(-1)

    else:
        print('NOT Connected to remote API server. Check if the server is running!')


if __name__ == "__main__":
    robotIp = "127.0.0.1"
    robotPort = 25000
    motionMode = 3 #1 - Braitenberg, 2 - Bumper, 3 - Wall

    if len(sys.argv) <= 1:
        print ("Usage python moveAround.py robotIP robotPort (optional default: 127.0.0.1 25000)")
    else:
        robotIp = sys.argv[1]
        robotPort = sys.argv[2]
        motionMode = sys.argv[3]

    main(robotIp, robotPort, motionMode)