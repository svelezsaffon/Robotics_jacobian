__author__ = 'Santiago & Spencer'



import numpy
import helper_functions
import geometricIK
import cross_product_mth
import inverse_jacobian

import numpy.linalg as lineal
import openravepy
import sys
from openravepy import *
import time



def left_hand_matrix_template(x,y,z):
    """
        [ 45.  10.  25.   0.  45.   0.]
        [[  1.22787804e-01  -7.07106781e-01   6.96364240e-01   3.58451629e-01]
        [  1.22787804e-01   7.07106781e-01   6.96364240e-01   5.70725085e-01]
        [ -9.84807753e-01   1.66533454e-16   1.73648178e-01   1.66243707e+00]
        [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
    """
        #Old matrix Above, New Matrix Below
    """
        [[ 0.1227878  -0.70710678  0.69636424  0.35914206]
        [ 0.1227878   0.70710678  0.69636424  0.56998716]
        [-0.98480775  0.          0.17364818  0.29142397]
        [ 0.          0.          0.          1.        ]]

    """
    #matrix=[
    #        [  0.1227878,  -0.70710678,   0.69636424,   x],
    #        [  0.1227878,   0.70710678,   0.69636424,   y],
    #        [ -0.98480775,  0.        ,   0.17364818,   z],
    #        [  0.        ,  0.        ,   0.        ,   1.]
    #        ]
    """
        [[ 0.17364818  0.          0.98480775  0.65699357]
         [ 0.          1.          0.          0.14909   ]
         [-0.98480775  0.          0.17364818  0.29142397]
         [ 0.          0.          0.          1.        ]]
    """
    matrix=[
            [  0.17364818,  0.        ,   0.98480775,   x],
            [  0.        ,  1.        ,   0.        ,   y],
            [ -0.98480775,  0.        ,   0.17364818,   z],
            [  0.        ,  0.        ,   0.        ,   1.]
            ]


    return matrix


def right_hand_matrix_template(x,y,z):
    """
        [-90.  20.  25.   0.  45.   0.]
        [[  0.00000000e+00   1.00000000e+00   1.11022302e-16   1.50100000e-01]
        [ -7.77156117e-16   0.00000000e+00  -1.00000000e+00  -6.97652953e-01]
        [ -1.00000000e+00   1.11022302e-16   6.66133815e-16   1.54391792e+00]
        [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
    """
    matrix=[[  0.00000000e+00,   1.00000000e+00,   1.11022302e-16,   x]
            [ -7.77156117e-16,   0.00000000e+00,  -1.00000000e+00,  -y]
            [ -1.00000000e+00,   1.11022302e-16,   6.66133815e-16,   z]
            [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]

    return matrix

def look_for_matrix():
    print "Looking for matrix"

    out=False
    env=Environment()
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('pumaarm.dae') # load a simple scene
    robot = env.GetRobots()[0]
    #Add axes to each coordinate frame of the robot



    helper= helper_functions.helper()

    #robot.SetDOFValues([0,1,2,3,4,5],[numpy.radians(57),numpy.radians(0),numpy.radians(90),numpy.radians(0),numpy.radians(0),numpy.radians(0)])

    while out is False:
        joint=int(raw_input("Joint:"))
        angle=float(raw_input("angle:"))
        if joint <= 6:
            print "Moving"
            robot.SetDOFValues([numpy.radians(angle)],[joint])
            T0 = robot.GetLinks()[0].GetTransform() # get the transform of link 1
            T1 = robot.GetLinks()[1].GetTransform() # get the transform of link 2
            T2 = robot.GetLinks()[2].GetTransform() # get the transform of link 3
            T3 = robot.GetLinks()[3].GetTransform() # get the transform of link 4
            T4 = robot.GetLinks()[4].GetTransform() # get the transform of link 5
            T5 = robot.GetLinks()[5].GetTransform() # get the transform of link 6
            handles=[]
            handles.append(misc.DrawAxes(env,T0,0.3,3))
            handles.append(misc.DrawAxes(env,T1,0.3,3))
            handles.append(misc.DrawAxes(env,T2,0.3,3))
            handles.append(misc.DrawAxes(env,T3,0.3,3))
            handles.append(misc.DrawAxes(env,T4,0.3,3))
            handles.append(misc.DrawAxes(env,T5,0.3,3))
        elif joint==10:
            print numpy.degrees(robot.GetDOFValues())
            print robot.GetLinks()[6].GetTransform()
        elif joint==11:
            #mat=left_hand_matrix_template(-1.50100000e-01,6.97652953e-01,1.54391792e+00)
            #robot.GetLinks()[6].SetTransform(mat)
            print helper.create_0Ai_matrix(6,numpy.degrees(robot.GetDOFValues()))
        else:
            out=True



class circular_movement(object):

    def __init__(self):
        self.speed=0.1
        self.x=0.65699357
        self.r=0.15

        self.env=Environment()
        self.env.SetViewer('qtcoin') # attach viewer (optional)
        self.env.Load('pumaarm.dae') # load a simple scene
        self.robot = self.env.GetRobots()[0]

        self.cros= cross_product_mth.cross_product_method()

    def __init__(self, environment, robot):
        self.speed=0.1
        self.x=0.65699357
        self.r=0.15

        self.env = environment
        self.robot = robot

        self.cros= cross_product_mth.cross_product_method()        

    def IK_move_in_circle(self,amount=80):

        angle=0.1

        inita= [ 0,  10,  25,   0,  45,   0]

        self.robot.SetDOFValues(numpy.radians(inita),[0,1,2,3,4,5])

        #amount=int(raw_input("Enter when ready"))
        handles = []

        for i in range(0,amount):
            y=self.r*numpy.cos(angle)+0.14909
            z=self.r*numpy.sin(angle)+0.29142397

            pos_matrix=(left_hand_matrix_template(self.x,y,z))

            inita=geometricIK.callGeometricIK(numpy.matrix(pos_matrix))
            """
            inita=[]
            inita.append(initab[0])
            inita.append(initab[1])
            inita.append(initab[2])
            inita.append(initab[3])
            inita.append(initab[4])
            inita.append(initab[5])
            """

            self.robot.SetDOFValues(numpy.radians(inita),[0,1,2,3,4,5])
            T6 = self.robot.GetLinks()[6].GetTransform() # get the transform of link 6
            T6[0][3] += 0.09
            handles.append(misc.DrawAxes(self.env,T6,0.01,3))
            #print inita
            time.sleep(0.1)
            #

            """
            jac=self.cros.solve_angles(inita)

            matjacs= self.cros.map_of_jacs_into_matrix(jac)

            inita=numpy.dot(matjacs,inita)

            self.robot.SetDOFValues(numpy.radians(inita),[0,1,2,3,4,5])

            time.sleep(0.1)
            """

            angle=angle+self.speed
        pause=raw_input("Enter when ready")

    def Jac_move_in_circle(self,amount=80):
        
        angle=0.1

        inita= [ 0,  10,  25,   0,  45,   0]

        self.robot.SetDOFValues(numpy.radians(inita),[0,1,2,3,4,5])

        #amount=int(raw_input("Enter when ready"))
        handles = []
        inverser=inverse_jacobian.inverse_method()

        for i in range(0,amount):
            self.speed
            y=self.r*numpy.cos(angle)+0.14909
            z=self.r*numpy.sin(angle)+0.29142397

            pos_matrix=(left_hand_matrix_template(self.x,y,z))

            inita=geometricIK.callGeometricIK(numpy.matrix(pos_matrix))


            jac = self.cros.solve_angles(inita)
            matjacs= self.cros.map_of_jacs_into_matrix(jac)
            moore= inverser.moore_penrose_equation(matjacs)

            x=[
                [self.x],
                [y],
                [z],
                [0],
                [0],
                [0]
            ]
            vel=numpy.dot(moore,x)

            new_angles=[0,0,0,0,0,0]

            for i in range(0,6):
                new_angles[i]=inita[i]+(vel[i][0]*self.speed)

            #print inita
            #print "----------"
            #print new_angles


            self.robot.SetDOFValues(numpy.radians(new_angles),[0,1,2,3,4,5])
            T6 = self.robot.GetLinks()[6].GetTransform() # get the transform of link 6
            T6[0][3] += 0.09
            handles.append(misc.DrawAxes(self.env,T6,0.01,3))
            #print inita
            time.sleep(0.1)
            #

            """
            jac=self.cros.solve_angles(inita)

            matjacs= self.cros.map_of_jacs_into_matrix(jac)

            inita=numpy.dot(matjacs,inita)

            self.robot.SetDOFValues(numpy.radians(inita),[0,1,2,3,4,5])

            time.sleep(0.1)
            """

            angle=angle+self.speed
        pause=raw_input("Enter when ready")

    def print_something(self):
        print "something"

    def quit(self):
        self.env.Destroy()

def main():
    lin=circular_movement()
    lin.IK_move_in_circle()
    #lin.Jac_move_in_circle()
    #look_for_matrix()



if __name__ == '__main__':
    main()
