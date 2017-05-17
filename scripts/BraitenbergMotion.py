from RobotMotion import RobotMotion

class BraitenbergMotion:

    braitenbergL= (-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    braitenbergR= (-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
    noDetectionDist=0.7
    maxDetectionDist=0.1
    v0 = 3

    def __init__(self, robotMotion):
        self.robot = robotMotion
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.detect = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        print ('Using Braitenberg Simple Vehicle 2a - running away from obstacles')

    def DoMove(self):

        #START WALKING AT MAX SPEED
        vLeft=self.v0
        vRight=self.v0

        #Checks Sensor distances
        for i in range(16):
            res,dist = self.robot.GetSensorDistance(i)
            if res & (dist<self.noDetectionDist):
                if (dist<self.maxDetectionDist):
                    dist=self.maxDetectionDist
                self.detect[i]=1-((dist-self.maxDetectionDist)/(self.noDetectionDist-self.maxDetectionDist))
            else:
                self.detect[i]=0

        #Adjust speed according to Braintenberg Vector
        for i in range(16):
            self.leftSpeed=vLeft=vLeft+2.5*self.braitenbergL[i]*self.detect[i]
            self.rightSpeed=vRight=vRight+2.5*self.braitenbergR[i]*self.detect[i]

        #print ('Moving the robot with vLeft: {0} / vRight: {1}',vLeft,vRight)
        self.robot.Move(vLeft,vRight)