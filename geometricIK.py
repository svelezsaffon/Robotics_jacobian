__author__ = 'Santiago & Spencer'



import numpy
import helper_functions
import cross_product_mth

from numpy import *
import numpy.linalg as lineal
import openravepy
import sys
from openravepy import *
import time
import json



def load(filename):
        #Read the D-H parameters from a file (given by the user).
        f = open(filename, 'r')

        # format {"Joint i" : {"theta" : 0, "alpha" : 0, "a" : 0, "d" : 0, "range min" : 0, "range max" : 0}}
        global params
        params = json.load(f)
        f.close()

def callGeometricIK(T6):
        filename = 'defaultdhparameters.txt'
        load(filename)
        #Indicators defined based on positioning of Circle or Line being drawn
        ARM = -1 # Left Arm
        ELBOW = -1 # Below arm
        WRIST = -1 # Wrist Up
        FLIP = -1 # Do Not Flip Wrist Orientation
        M = 1 #?
        #Temporarily tries T6 as all 3 matricies, need to pass in appropriate T3 and T4

        return geometricApproach(T6, T6, T6, ARM, ELBOW, WRIST, M)

def geometricApproach(T3, T4, T6, ARM, ELBOW, WRIST, M):        
        res_angles=[0,0,0,0,0,0]
        global params
        px6 = T6.item(3)
        py6 = T6.item(7)
        pz6 = T6.item(11)


        ax6 = T6.item(2)
        ay6 = T6.item(6)
        az6 = T6.item(10)

        d6 = params["Joint 6"]["d"]


        px=px6
        py=py6
        pz=pz6



        d2 = params["Joint 2"]["d"]
        d4 = params["Joint 4"]["d"]        
        a2 = params["Joint 2"]["a"]
        a3 = params["Joint 3"]["a"]
        a1 = params["Joint 1"]["a"]

        #Solve for theta1
        theta1 = arctan2(-ARM * py * sqrt(power(px,2) + power(py,2) - power(d2,2)) - px * d2, -ARM * px * sqrt(power(px,2) + power(py,2) - power(d2,2)) + py * d2)
        res_angles[0]= degrees(theta1)



        #Solve for theta2
        R = sqrt(power(px,2) + power(py,2) + power(pz,2) - power(d2,2))
        r = sqrt(power(px,2) + power(py,2) - power(d2,2))

        #"""
        sinalpha = -pz/R
        cosalpha = -(ARM * r)/R

        cosbeta = ((power(a2,2) + power(R,2) - (power(d4,2) + power(a3,2)))/(2*a2*R))

        sinbeta = sqrt( 1 - power(cosbeta, 2))
        sintheta2 = sinalpha * cosbeta + (ARM * ELBOW) * cosalpha * sinbeta
        costheta2 = cosalpha * cosbeta - (ARM * ELBOW) * sinalpha * sinbeta
        theta2 = arctan2(sintheta2, costheta2)

        """

        upper=py*cos(theta1)-px*sin(theta1)
        lower=py*sin(theta1)+px*cos(theta1)-a1

        theta2=arctan2(upper,lower)
        """
        res_angles[1]= degrees(theta2)

        #Solve for theta3

        #"""
        cosphi = (power(a2,2) + (power(d4,2) + power(a3,2)) - power(R,2))/(2*a2*sqrt(power(d4,2) + power(a3,2)))
        sinphi = ARM * ELBOW * sqrt(1 - power(cosphi, 2))
        sinbeta2 = d4/sqrt(power(d4,2) + power(a3,2))
        cosbeta2 = abs(a3)/sqrt(power(d4,2) + power(a3,2))
        sintheta3 = sinphi*cosbeta2 - cosphi*sinbeta2
        costheta3 = cosphi*cosbeta2 + sinphi*sinbeta2
        theta3 = arctan2(sintheta3, costheta3)
        """

        nx = T3.item(0)
        ny = T3.item(4)

        upper=ny*cos(theta1+theta2)-nx*sin(theta1+theta2)
        lower=ny*sin(theta1+theta2)+nx*cos(theta1+theta2)

        theta3=2*arctan2(-upper,lower)

        """
        res_angles[2]= degrees(theta3)

        #Solve for theta4
        nx = T3.item(0)
        sx = T3.item(1)
        ax = T3.item(2)
        ny = T3.item(4)
        sy = T3.item(5)
        ay = T3.item(6)
        nz = T3.item(8)
        sz = T3.item(9)
        az = T3.item(10)
        theta4 = arctan2(M * (cos(theta1) * ay - sin(theta1) * ax), M * (cos(theta1) * cos(theta2 + theta3) * ax + sin(theta1) * sin(theta2 + theta3) * ay - sin(theta2 + theta3) * az))
        res_angles[3]=degrees(theta4)
        #Solve for theta5
        theta5 = arctan2((cos(theta1) * cos(theta2 + theta3) * cos(theta4) - sin(theta1) * sin(theta4)) * ax + (sin(theta1) * cos(theta2 + theta3) * cos(theta4) + cos(theta1) * sin(theta4)) * ay - sin(theta2 + theta3) * cos(theta4) * az, cos(theta1) * sin(theta2 + theta3) * ax + sin(theta1) * sin(theta2 + theta3) * ay + cos(theta2 + theta3) * az)
        res_angles[4]= degrees(theta5)
        #Solve for theta6
        theta6 = arctan2((-sin(theta1) * cos(theta4) - cos(theta1) * cos(theta2 + theta3) * sin(theta4)) * nx + (cos(theta1) * cos(theta4) - sin(theta1) * cos(theta2 + theta3) * sin(theta4)) * ny + (sin(theta2 + theta3) * sin(theta4)) * nz, (-sin(theta1) * cos(theta4) - cos(theta1) * cos(theta2 + theta3) * sin(theta4)) * sx + (cos(theta1) * cos(theta4) - sin(theta1) * cos(theta2 + theta3) * sin(theta4)) * sy + (sin(theta2 + theta3) * sin(theta4)) * sz)
        res_angles[5]=degrees(theta6)


        return res_angles
