import vrep,time,sys
import numpy as np
from math import sqrt, pi, cos, sin

class RobotMotion:

    R = 0.195/2
    L = 0.381

    def __init__(self, clientId, robot, left, right, sensors, gyro):
        self.clientId = clientId
        self.robot = robot  # instance variable unique to each instance
        self.leftWheel = left  # instance variable unique to each instance
        self.rightWheel = right  # instance variable unique to each instance
        self.sensors = sensors
        self.gyro = gyro
        self.targetPosition = [0,0]

        for i in range(16):
            _, _, _, _, _ = vrep.simxReadProximitySensor(self.clientId, self.sensors[i], vrep.simx_opmode_streaming)
        _,_ = vrep.simxGetJointPosition(self.clientId, self.leftWheel, vrep.simx_opmode_streaming)
        _,_ = vrep.simxGetJointPosition(self.clientId, self.rightWheel, vrep.simx_opmode_streaming)
        _,_ = vrep.simxGetObjectOrientation(self.clientId, self.robot, -1, vrep.simx_opmode_streaming)
        _,_ = vrep.simxGetObjectPosition(self.clientId, self.robot, -1, vrep.simx_opmode_streaming)
        _, _, _, _, _ = vrep.simxReadProximitySensor(self.clientId, self.sensors[i], vrep.simx_opmode_streaming)
        for i in range(10):

            print("t = "+ str(gyro))
            time.sleep(0.5)

        time.sleep(0.5)

        self.sensorAngles = [i*pi/180 for i in [90, 50, 30, 10, -10, -30, -50, -90 ,-90, -130, -150, -170, 170, 150, 130, 90]]
        self.odomX, self.odomY, self.odomAngle = self.GetRealPose()
        self.leftAngle = 0
        self.rightAngle = 0

    def SetTargetPosition(self, x, y):
        realX, realY, realAngle = self.GetRealPose()
        deltaX = x - realX;
        deltaY = y - realY;
        targetAngle = np.arctan(deltaY/deltaX)
        self.targetPosition = [x,y,targetAngle]
        self.targetDistance = sqrt((deltaX)^2 + (deltaY)^2)
        self.targetOrientation = targetAngle - realAngle

    def GetRealPose(self):
        error, pos = vrep.simxGetObjectPosition(self.clientId, self.robot, -1, vrep.simx_opmode_streaming)
        X, Y, _ = pos
        if error != 0:
            print(error)
            print('Falha ao ler posição do robo!')
            print(exit())

        error, ori = vrep.simxGetObjectOrientation(self.clientId, self.robot, -1, vrep.simx_opmode_streaming)

        if error != 0:
            print(error)
            print('Falha ao ler orientação do robo!')
            print(exit())

        _, _, angle= ori

        return [X, Y, angle]

    def GetEstimatedPose(self):
        return [self.odomX, self.odomY, self.odomAngle]

    def Move(self, left, right):
        # START WALKING AT MAX SPEED
        vrep.simxSetJointTargetVelocity(self.clientId, self.leftWheel, left, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientId, self.rightWheel, right, vrep.simx_opmode_streaming)


    def MoveByAngle(self, w, v):
        wNorm = (w * pi) - pi/2
        vL = v+(RobotMotion.L*wNorm)/(RobotMotion.R)
        vR = v-(RobotMotion.L*wNorm)/(RobotMotion.R)
        print('w = ', wNorm)
        print('vL = ', vL)
        print('vR = ', vR)
        self.Move(vL, vR)

    def Stop(self):
        # TARGET VELOCITY
        print("Stoping!!")
        vrep.simxSetJointTargetVelocity(self.clientId, self.leftWheel, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientId, self.rightWheel, 0, vrep.simx_opmode_streaming)

    def GetRobotPosition(self):
        res, robotPosition = vrep.simxGetObjectPosition(self.clientId, self.robot, -1, vrep.simx_opmode_streaming)
        return robotPosition

    def GetRobotOrientation(self):
        res, robotOrientation = vrep.simxGetObjectOrientation(self.clientId, self.robot, -1, vrep.simx_opmode_streaming)
        return robotOrientation

    def GetSensorDistance(self, i):
        ret, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.clientId, self.sensors[i], vrep.simx_opmode_streaming)
        distance = sqrt(sum([x ** 2 for x in detectedPoint]))
        # distance = detectedPoint[2]
        return detectionState, distance

    def GetSensorPoint(self, i):
        orient = self.GetRobotOrientation()
        _, _, robot_angle = orient
        robot_x, robot_y, _ = self.GetRobotPosition()
        angle = self.sensorAngles[i] + robot_angle
        state, d = self.GetSensorDistance(i)
        if (state == 0): return [float('inf'), float('inf')]
        d = d + 0.1905
        x_delta = cos(angle) * d
        y_delta = sin(angle) * d
        if(i<8): y_delta += 0.037
        else:    y_delta -= 0.037
        x_pos = robot_x + x_delta
        y_pos = robot_y + y_delta
        return [x_pos, y_pos]

    def UpdateOdom(self):
        erro, new_l_angle = vrep.simxGetJointPosition(self.clientId, self.leftWheel, vrep.simx_opmode_streaming);
        if erro != 0:
            print(erro)
            print('Falha ao ler o ângulo da roda esquerda!')
            print(exit())

        leftDiff = new_l_angle - self.leftAngle

        erro, new_r_angle = vrep.simxGetJointPosition(self.clientId, self.rightWheel,vrep.simx_opmode_streaming);
        if erro != 0:
            print(erro)
            print('Falha ao ler o ângulo da roda direita!')
            print(exit())

        rightDiff = new_r_angle - self.rightAngle

        self.rightAngle = new_r_angle
        self.leftAngle  = new_l_angle

        if rightDiff >  pi: rightDiff -= 2*pi
        if rightDiff < -pi: rightDiff += 2*pi
        if leftDiff  >  pi:  leftDiff -= 2*pi
        if leftDiff  < -pi:  leftDiff += 2*pi

        radius = 0.195/2
        angleDiff  = radius*(rightDiff - leftDiff)/0.356
        linearDiff = radius* (rightDiff + leftDiff)/2

        self.odomX += linearDiff * cos(self.odomAngle + angleDiff / 2)
        self.odomY += linearDiff * sin(self.odomAngle + angleDiff / 2)
        self.odomAngle += angleDiff
        while self.odomAngle >  pi:  self.odomAngle -= 2*pi
        while self.odomAngle < -pi:  self.odomAngle += 2*pi