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

    [[  0.00000000e+00  -1.00000000e+00   2.22044605e-16  -1.50100000e-01]
    [  8.88178420e-16   2.22044605e-16   1.00000000e+00   6.97652953e-01]
    [ -1.00000000e+00   0.00000000e+00   8.88178420e-16   1.54391792e+00]
    [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]

    """
    matrix=[[  0.00000000e+00,  -1.00000000e+00,   2.22044605e-16,  x],
            [  8.88178420e-16,   2.22044605e-16,   1.00000000e+00,   y],
            [ -1.00000000e+00,   0.00000000e+00,   8.88178420e-16,   z],
            [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]

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

    helper= helper_functions.helper()

    #robot.SetDOFValues([0,1,2,3,4,5],[numpy.radians(57),numpy.radians(0),numpy.radians(90),numpy.radians(0),numpy.radians(0),numpy.radians(0)])

    while out is False:
        joint=int(raw_input("Joint:"))
        angle=float(raw_input("angle:"))
        if joint <= 6:
            print "Moving"
            robot.SetDOFValues([numpy.radians(angle)],[joint])
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

    def __init__(self,a=1,b=2):
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

        mat=left_hand_matrix_template(-1.50100000e-01,6.97652953e-01,self.z)

        inita=project.solve_matrix(numpy.matrix(mat),inita[0],inita[1],inita[2],inita[3],inita[4],inita[5])

        self.robot.SetDOFValues(inita,[0,1,2,3,4,5])

        xcopy=self.initx+self.speed

        for i in range(0,amount):

            y=self.a*xcopy +self.b

            mat=left_hand_matrix_template(xcopy,y,self.z)

            inita=project.solve_matrix(numpy.matrix(mat),inita[0],inita[1],inita[2],inita[3],inita[4],inita[5])

            jac=self.cros.solve_angles(inita)

            matjacs= self.cros.map_of_jacs_into_matrix(jac)

            inita=numpy.dot(matjacs,inita)

            self.robot.SetDOFValues(inita,[0,1,2,3,4,5])

            time.sleep(0.1)

            xcopy += self.speed



def main():
    lin=linear_movement()
    lin.move_in_line(1)
    #look_for_matrix()




if __name__ == '__main__':
    main()
