from RobotMotion import RobotMotion
import time

class WallFollow:

    def __init__(self, robotMotion, direction ='right', speed = 4):
        self.robot = robotMotion
        self.direction = direction
        self.speed = speed
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        ku = 11
        tu = 1.2
        dt = 50*1e-3
        self.kp = ku*0.2
        self.ki = (2*self.kp/tu)*dt
        self.kd = (self.kp*tu)/(3*dt)
        print("kp = "+str( self.kp))
        print("ki = "+str( self.ki))
        print("kd = "+str( self.kd))
        print("============")
        self.target = 0.15
        print ('Using Wall Follow - Following walls by 15 cm distance')

    def DoMove(self):
        self.robot.UpdateOdom()
        if(self.direction == 'left')   :  sensorId = [0,15]
        elif(self.direction == 'right'):  sensorId = [7, 8]
        else:
            print(error)
            print('Direção inválida!')
            print(exit())

        status, d0 = self.robot.GetSensorDistance(sensorId[0])
        print(status)
        if(status == 0): d0 = self.target*1.5 #if could not read a distance, distance is greater than target

        status, d1 = self.robot.GetSensorDistance(sensorId[1])
        print(status)
        if(status == 0): d1 = self.target*1.5#if could not read a distance, distance is greater than target


        d = (d0+d1)/2 # Average between lateral sensors
        dd = d1-d0    # Try to allign side sensors
        if (self.direction == 'left'): dd = -dd

        new_error = 1*(self.target-d) + 5*(dd) # d - weight = 1,   dd wheight = 5

        self.error_sum += new_error
        self.error_diff = new_error - self.error

        self.error = new_error

        dir =  self.kp*self.error +self.ki*self.error_sum + self.kd * self.error_diff
        dir = dir

        radius = 0.195/2
        l = 0.178
        vr = self.speed + (dir/radius)*(l)
        vl = self.speed - (dir/radius)*(l)
        if (self.direction == 'left'):
            vr,vl = vl,vr


        self.robot.Move(vl, vr)
        time.sleep(50*1e-3)#50 mili

