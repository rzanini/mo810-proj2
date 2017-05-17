import vrep,time,sys
import numpy as np
import skfuzzy as fuzz
from math import sqrt,pi, cos, sin


"""
==========================================
Fuzzy Control Systems: Wall Follow Problem
==========================================
-------------------
Let's create a fuzzy control system which models how you might choose the speed
of left and right wheels of a p3dx robot. We have 2 similar controllers, one for the
right wheel and a second one for the left wheel. Each one of them use the sensors located
at the correspondent side. The proximity sensors (side, diagonal, frontal) have values from 0 to 5.
The output (angular speed) has values from 0 to 5 as well. We connect the controller with cross inputs
and outputs, that is, the right sensors control the left wheel and the left sensors control the right wheel.

We would formulate this problem as:
* Antecedents (Inputs)
   - `side`,`diagonal`,`frontal`
      * Universe (ie, crisp value range): How close is the sensor to an obstacle, on a scale of 0m to 3m?
      * Fuzzy set (ie, fuzzy value range): close, near, medium, far
* Consequents (Outputs)
   - 'vTurn', 'vMove'
      * Universe: How much should the robot turn and how fast shoul it go?
      * Fuzzy set: vTurn (left, right)
      * Fuzzy set: vMove (slow, medium, fast)
* Rules
   We've implemented 3 sets of Rules:
   1) WallFollow - with this set, the robot will try to find a wall (or an obstacle) and then will try to follow a path around it.
   The rules are simple
   2) ObstacleAvoidance - with this set, the robot will try to move forward, avoiding obstacles on its way
   3) GoToGoal - with this set, the robot will try to go to a position (marked by a red sphere), avoiding obstacles on its way

Creating the Fuzzy Controller Using the skfuzzy control API
-------------------------------------------------------------
We can use the `skfuzzy` control system API to model this.
First, let'sdefine fuzzy variables
"""
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


#Implement Fuzzy Controller for 2 behaviors
#WallFollow
#ObstacleAvoidance
#GoToGoal
class FuzzyMotion:

    v0 = 3.0

    def __init__(self, robotMotion):
        self.robot = robotMotion
        #self.ruleMaker = ObstacleAvoidanceRules()
        self.ruleMaker = WallFollowRules()
        #self.ruleMaker = WallFollowRules()
        self.InitFuzzyController()
        print ('Using Fuzzy Controller')


    def InitMembershipFunctions(self):
        # New Antecedent/Consequent objects hold universe variables and membership
        # functions
        right_frontal = ctrl.Antecedent(np.arange(0, 3, 0.01), 'right_frontal')
        right_diagonal = ctrl.Antecedent(np.arange(0, 3, 0.01), 'right_diagonal')
        right_side = ctrl.Antecedent(np.arange(0, 3, 0.01), 'right_side')

        left_frontal = ctrl.Antecedent(np.arange(0, 3, 0.01), 'left_frontal')
        left_diagonal = ctrl.Antecedent(np.arange(0, 3, 0.01), 'left_diagonal')
        left_side = ctrl.Antecedent(np.arange(0, 3, 0.01), 'left_side')

        vLeft = ctrl.Consequent(np.arange(-2, 4, 0.1), 'vLeft')
        vRight = ctrl.Consequent(np.arange(-2, 4, 0.1), 'vRight')

        #Custom relationship function for inputs
        right_frontal['close'] = right_side['close'] = right_diagonal['close'] = fuzz.trapmf(right_frontal.universe, [0, 0, 0.2, 0.3])
        right_frontal['near'] = right_side['near'] = right_diagonal['near'] = fuzz.trapmf(right_frontal.universe, [0.2, 0.3, 0.4, 0.5])
        right_frontal['medium'] = right_side['medium'] = right_diagonal['medium'] = fuzz.trapmf(right_frontal.universe, [0.4, 0.5, 0.7, 0.8])
        right_frontal['far'] = right_side['far'] = right_diagonal['far'] = fuzz.trapmf(right_frontal.universe, [0.7, 0.8, 3, 3])

        left_frontal['close'] =  left_side['close'] = left_diagonal['close'] = fuzz.trapmf(right_frontal.universe, [0, 0, 0.2, 0.3])
        left_frontal['near'] =   left_side['near'] = left_diagonal['near'] = fuzz.trapmf(right_frontal.universe, [0.2, 0.3, 0.4, 0.5])
        left_frontal['medium'] = left_side['medium'] = left_diagonal['medium'] = fuzz.trapmf(right_frontal.universe, [0.4, 0.5, 0.7, 0.8])
        left_frontal['far'] =    left_side['far'] = left_diagonal['far'] = fuzz.trapmf(right_frontal.universe, [0.7, 0.8, 3, 3])

        # Custom membership functions can be built interactively with a familiar,
        vLeft['back'] = fuzz.trimf(vLeft.universe, [-2, -2, 0])
        vLeft['slow'] = fuzz.trimf(vLeft.universe, [0, 0, 2])
        vLeft['medium'] = fuzz.trimf(vLeft.universe, [1, 2, 3])
        vLeft['fast'] = fuzz.trimf(vLeft.universe, [2, 4, 4])

        vRight['back'] = fuzz.trimf(vRight.universe, [-2, -2, 0])
        vRight['slow'] = fuzz.trimf(vRight.universe, [0, 0, 2])
        vRight['medium'] = fuzz.trimf(vRight.universe, [1, 2, 3])
        vRight['fast'] = fuzz.trimf(vRight.universe, [2, 4, 4])

        self.inputs = [right_frontal, right_diagonal, right_side, left_frontal, left_diagonal, left_side]
        self.outputs = [vLeft, vRight]
        """
        To help understand what the membership looks like, use the ``view`` methods.
        """

        # You can see how these look with .view()
        #right_frontal.view()
        #right_diagonal.view()
        #right_side.view()
        #left_frontal.view()
        #left_diagonal.view()
        #left_side.view()

        #right_frontal.view()
        """
        .. image:: PLOT2RST.current_figure
        """
        #right_diagonal.view()
        """
        .. image:: PLOT2RST.current_figure
        """
        #right_side.view()
        """
        .. image:: PLOT2RST.current_figure
        """
        #vLeft.view()
        """
        .. image:: PLOT2RST.current_figure
        """
        #vTurn.view()
        """
        .. image:: PLOT2RST.current_figure
        """

        """
        Fuzzy rules - 2 different behaviors
        """

    def InitFuzzyController(self):
        self.InitMembershipFunctions()
        self.InitRules()
        self.speed_ctrl = ctrl.ControlSystem(self.rules)
        self.simulation = ctrl.ControlSystemSimulation(self.speed_ctrl)

    def InitRules(self):
        self.rules = []
        if self.ruleMaker:
            self.rules = self.ruleMaker.MakeRules(self.inputs, self.outputs)

    def DoMove(self):
        #START WALKING AT MAX SPEED
        vLeft=self.v0
        vRight=self.v0

        #Check the groundtruth#
        position = self.robot.GetRobotPosition()
        orientation = self.robot.GetRobotOrientation()

        left_side = left_diagonal = left_frontal = float("inf")
        right_frontal = right_diagonal = right_side = float("inf")

        #Check proximity sensor values
        res,dist = self.robot.GetSensorDistance(0)
        if res:
            left_side = dist
        #print("Sensor 1 = ", dist)

        res,dist = self.robot.GetSensorDistance(2)
        if res:
            left_diagonal = dist

        res,dist = self.robot.GetSensorDistance(3)
        #print("Sensor 4 = ", dist)
        if res:
            left_frontal = dist

        res,dist = self.robot.GetSensorDistance(4)
        if res:
            right_frontal = dist
        #print("Sensor 5 = ", dist)

        res,dist = self.robot.GetSensorDistance(5)
        if res:
            right_diagonal = dist

        res,dist = self.robot.GetSensorDistance(7)
        if res:
            right_side = dist
        #print("Sensor 8 = ", dist)

        #Pass values to simulation
        self.simulation.input['right_frontal'] = right_frontal
        self.simulation.input['right_side'] = right_side
        self.simulation.input['right_diagonal'] = right_diagonal

        #print("Sensor right_frontal = ", right_frontal)
        #print("Sensor right_side = ", right_side)
        #print("Sensor right_diagonal = ", right_diagonal)

        self.simulation.input['left_frontal'] = left_frontal
        self.simulation.input['left_side'] = left_side
        self.simulation.input['left_diagonal'] = left_diagonal

        #print("Sensor left_frontal = ", left_frontal)
        #print("Sensor left_diagonal = ", left_diagonal)
        #print("Sensor left_side = ", left_side)

        # Crunch the numbers
        self.simulation.compute()

        """
        Once computed, we can view the result as well as visualize it.
        """
        vLeft = self.simulation.output['vLeft']
        vRight = self.simulation.output['vRight']

        print("vRight = ", vRight)
        print("vLeft = ", vLeft)

        #Move the robot
        self.robot.Move(vLeft,vRight)


class ObstacleAvoidanceRules:

    def __init__(self):
        print ('Using Obstacle Avoidance Rules')

    def MakeRules(self, inputs, outputs):

        right_frontal = inputs[0]
        right_diagonal = inputs[1]
        right_side = inputs[2]

        left_frontal = inputs[3]
        left_diagonal = inputs[4]
        left_side = inputs[5]

        vLeft = outputs[0]
        vRight = outputs[1]

        #Rules for right wheel based on right sensors

        #Regra 0 - se for bater à direita, mova-se para a esquerda devagar
        rule0_1 = ctrl.Rule(right_frontal['close'], vLeft['back'])
        rule0_2 = ctrl.Rule(right_frontal['near'], vLeft['back'])
        rule0_3 = ctrl.Rule(right_frontal['medium'], vLeft['medium'])
        rule0_4 = ctrl.Rule(right_frontal['far'], vLeft['fast'])

        #Regra 1 - se for bater à esquerda, mova-se para a direita devagar
        rule1_1 = ctrl.Rule(left_frontal['close'], vRight['back'])
        rule1_2 = ctrl.Rule(left_frontal['near'], vRight['back'])
        rule1_3 = ctrl.Rule(left_frontal['medium'], vRight['medium'])
        rule1_4 = ctrl.Rule(left_frontal['far'], vRight['fast'])

        #Regra 2 - se for bater à esquerda, mova-se para a direita devagar
        rule2_1 = ctrl.Rule(left_diagonal['close'], vRight['back']%0.6)
        rule2_2 = ctrl.Rule(left_diagonal['near'], vRight['slow']%0.4)
        rule2_3 = ctrl.Rule(left_diagonal['medium'], vRight['medium']%0.2)
        #rule2_4 = ctrl.Rule(left_diagonal['far'], vRight['fast']%0.6)

        #Regra 3 - se for bater à direita, mova-se para a esquerda devagar
        rule3_1 = ctrl.Rule(right_diagonal['close'], vLeft['back']%0.6)
        rule3_2 = ctrl.Rule(right_diagonal['near'], vLeft['slow']%0.4)
        rule3_3 = ctrl.Rule(right_diagonal['medium'], vLeft['medium']%0.2)
        #rule3_4 = ctrl.Rule(right_diagonal['far'], vLeft['fast']%0.6)

        #Regra 4 - se for bater à esquerda, mova-se para a direita devagar
        rule4_1 = ctrl.Rule(left_side['close'], vRight['back']%0.2)
        rule4_2 = ctrl.Rule(left_side['near'], vRight['slow']%0.2)
        rule4_3 = ctrl.Rule(left_side['medium'], vRight['medium']%0.2)
        #rule4_4 = ctrl.Rule(left_side['far'], vRight['fast']%0.2)

        #Regra 5 - se for bater à direita, mova-se para a esquerda devagar
        rule5_1 = ctrl.Rule(right_side['close'], vLeft['back']%0.2)
        rule5_2 = ctrl.Rule(right_side['near'], vLeft['slow']%0.2)
        rule5_3 = ctrl.Rule(right_side['medium'], vLeft['medium']%0.2)
        #rule5_4 = ctrl.Rule(right_side['far'], vLeft['fast']%0.1)


        rules = [rule0_1, rule0_2, rule0_3, rule0_4, rule1_1, rule1_2, rule1_3, rule1_4, rule2_1, rule2_2, rule2_3, rule3_1, rule3_2, rule3_3,
                 rule4_1, rule4_2, rule4_3, #rule4_4,
                 rule5_1, rule5_2, rule5_3]#, #rule5_4]
        return rules


class WallFollowRules:

    def __init__(self):
        print ('Using Wall Follow Rules')

    def MakeRules(self, inputs, outputs):

        right_frontal = inputs[0]
        right_diagonal = inputs[1]
        right_side = inputs[2]

        left_frontal = inputs[3]
        left_diagonal = inputs[4]
        left_side = inputs[5]

        vLeft = outputs[0]
        vRight = outputs[1]

        #Rules for right wheel based on right sensors

        #If it is going to hit the wall, go to the side
        #Rules for right wheel based on right sensors
        rule1 = ctrl.Rule(right_frontal['close'] | right_diagonal['close'], vRight['fast'])
        rule2 = ctrl.Rule((right_frontal['near'] | right_frontal['medium'] | right_frontal['far'])&(right_diagonal['near'] | right_diagonal['medium'] | right_diagonal['far']), vRight['medium'])
        rule3 = ctrl.Rule((right_frontal['near'] | right_frontal['medium'] )&(right_diagonal['near'] | right_diagonal['medium']), vRight['medium'])
        rule4 = ctrl.Rule(right_frontal['far'] & right_diagonal['far'] & right_side['close'], vRight['fast'])
        rule5 = ctrl.Rule(right_frontal['far'] & right_diagonal['far'] & right_side['near'], vRight['medium'])
        rule6 = ctrl.Rule(right_frontal['far'] & right_diagonal['far'] & (right_side['medium'] | right_side['far']), vRight['slow'])

        #Rules for left wheel based on right sensors
        rule7 = ctrl.Rule(right_frontal['close'] | right_frontal['near'] | right_frontal['medium'], vLeft['slow'])
        rule8 = ctrl.Rule(right_diagonal['close'] | right_diagonal['near'] | right_diagonal['medium'], vLeft['slow'])
        rule9 = ctrl.Rule(right_frontal['far'] & right_diagonal['far'], vLeft['medium'])

        #Rules for right wheel based on left sensors
        rule10 = ctrl.Rule(left_frontal['close'] | left_diagonal['close'] | left_side['close'], (vLeft['fast'], vRight['back']))

        rules = [rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10]
        return rules


class GoToGoalRules:

    def __init__(self):
        print ('Obstacle Avoidance Controller')