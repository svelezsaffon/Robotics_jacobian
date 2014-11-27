__author__ = 'Santiago & Spencer'



import numpy
import helper_functions
import project
import cross_product_mth

import numpy.linalg as lineal
import openravepy
import sys
from openravepy import *
import time



def left_hand_matrix_template(x,y,z):
    """
    [ 90.  20.  25.   0.  45.   0.]

        [[ -2.22044605e-16  -1.00000000e+00   0.00000000e+00  -1.50100000e-01]
        [  1.73648178e-01   0.00000000e+00   9.84807753e-01   6.57027156e-01]
        [ -9.84807753e-01   2.22044605e-16   1.73648178e-01   1.66243707e+00]
        [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
    """
    matrix=[
        [  0.00000000e+00,  -1.00000000e+00,   1.66533454e-16,  x],
        [  1.66533454e-16,   1.11022302e-16,   1.00000000e+00,   y],
        [ -1.00000000e+00,   0.00000000e+00,   1.11022302e-16,   1.54391792e+00],
        [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]
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



class linear_movement(object):

    def __init__(self,a=1,b=1):
        self.a=a
        self.b=b
        self.speed=0.01
        self.z=1.54391792e+00
        self.initx=-1.50100000e-01

        self.env=Environment()
        self.env.SetViewer('qtcoin') # attach viewer (optional)
        self.env.Load('pumaarm.dae') # load a simple scene
        self.robot = self.env.GetRobots()[0]



        self.cros= cross_product_mth.cross_product_method()


    def move_in_line(self,amount=20):

        inita=[ 90,  20,  25,   0,  45,   0]

        xcopy=self.initx

        for i in range(0,amount):

            y=self.a*xcopy +self.b

            auxmat=numpy.matrix(left_hand_matrix_template(xcopy,y,None))

            inita=project.solve_matrix(auxmat,inita[0],inita[1],inita[2],inita[3],inita[4],inita[5])


            print ""

            print (inita)


            """
            jac=self.cros.solve_angles(inita)

            matjacs= self.cros.map_of_jacs_into_matrix(jac)

            inita=numpy.dot(matjacs,inita)

            self.robot.SetDOFValues(inita,[0,1,2,3,4,5])

            time.sleep(0.1)
            """
            xcopy += self.speed



def main():
    lin=linear_movement((0.254098361),0.735140164)
    lin.move_in_line(2)
    #look_for_matrix()




if __name__ == '__main__':
    main()
