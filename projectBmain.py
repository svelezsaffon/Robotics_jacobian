##########################################################
# Title       : CS569 OpenRave Project Part B            #
#                                                        #
# Author      : Spencer Carver &                         #
#               Santiago Velez Saffon                    #
#                                                        #
# Description : Forward and Inverse Jacobian calculation #
#               through 2 methods. Circular and Linear   #
#               drawing applications to verify expected  #
#               behavior of each method.                 #
##########################################################

#############
#  IMPORTS  #
#############
import cross_product_mth
import dtrm_approach
import inverse_jacobian
import circle_movement
import line_movement

import openravepy
import sys
from openravepy import *
import time

#############
#GLOBAL VARS#
#############
global theta1
global theta2
global theta3
global theta4
global theta5
global theta6

#############
# FUNCTIONS #
#############

def set_thetas():
        global theta1
        theta1 = int(input("Enter a new theta1 (currently: %d): " % (theta1)))
        global theta2
        theta2 = int(input("Enter a new theta2 (currently: %d): " % (theta2)))
        global theta3
        theta3 = int(input("Enter a new theta3 (currently: %d): " % (theta3)))
        global theta4
        theta4 = int(input("Enter a new theta4 (currently: %d): " % (theta4)))
        global theta5
        theta5 = int(input("Enter a new theta5 (currently: %d): " % (theta5)))
        global theta6
        theta6 = int(input("Enter a new theta6 (currently: %d): " % (theta6)))

def main():
        #Initialize globals
        global theta1
        theta1 = 56
        global theta2
        theta2 = 47
        global theta3
        theta3 = 23
        global theta4
        theta4 = 14
        global theta5
        theta5 = 5
        global theta6
        theta6 = 8

        #Initialize menu
        menu = {}
        menu['1']="Solve Jacobian (Vector-Cross Product)" 
        menu['2']="Solve Jacobian (DTRM)"
        menu['3']="Solve Moore-Penrose Pseudo Inverse (Vector-Cross Product)"
        menu['4']="Solve Moore-Penrose Pseudo Inverse (DTRM)"
        menu['5']="Draw Circle (Inverse Kinematic)"
        menu['6']="Draw Circle (Inverse Jacobian)" 
        menu['7']="Draw Line (Inverse Kinematic)"
        menu['8']="Draw Line (Inverse Jacobian)"
        menu['9']="Set Thetas for Jacobian Solver (Currently [%d,%d,%d,%d,%d,%d])" % (theta1,theta2,theta3,theta4,theta5,theta6)
        menu['Q']="Exit"

        env = Environment()
        env.SetViewer('qtcoin') # attach viewer (optional)
        env.Load('pumaarm.dae') # load a simple scene
        robot = env.GetRobots()[0]

        circ = circle_movement.circular_movement(env, robot)
        lin = line_movement.linear_movement(env, robot, 0.1, 0.3)

        while True: 
                menu['9']="Set Thetas for Jacobian Solver (Currently [%d,%d,%d,%d,%d,%d])" % (theta1,theta2,theta3,theta4,theta5,theta6)
                options=menu.keys()
                options.sort()
                print ""
                print "~~~~~~~ ECE/CS 569 OpenRave PUMA Robotic Arm Main Menu~~~~~~~"
                for entry in options: 
                        print entry, ")", menu[entry]
                selection=raw_input("Please Select:") 
                if selection =='1': 
                        print ""
                        cross_product_mth.getJacobian(theta1,theta2,theta3,theta4,theta5,theta6)
                elif selection == '2':
                        print "" 
                        dtrm_approach.getJacobian(theta1,theta2,theta3,theta4,theta5,theta6)
                elif selection == '3':
                        print ""
                        cross_product_mth.getInverseJacobian(theta1,theta2,theta3,theta4,theta5,theta6)
                elif selection == '4':
                        print ""
                        dtrm_approach.getInverseJacobian(theta1,theta2,theta3,theta4,theta5,theta6)
                elif selection == '5':
                        print ""
                        circ.IK_move_in_circle()
                elif selection == '6':
                        print ""
                        circ.Jac_move_in_circle()
                elif selection == '7':
                        print ""
                        print "Enter the equation parameters y=ax+b"

                        a=100

                        while a>0.5:
                            a=float(raw_input("Enter a in range(0,0.5): "))

                        b=100

                        while b>0.4:
                            b=float(raw_input("Enter b in range(0,0.5): "))

                        speed=100

                        while speed>0.4:
                            speed=float(raw_input("Enter speed in range(0,0.5): "))

                        lin.speed=speed
                        lin.a=a
                        lin.b=b
                        lin.IK_move_in_line()
                elif selection == '8': 
                        print ""

                        print "Enter the equation parameters y=ax+b"

                        a=100

                        while a>0.5:
                            a=float(raw_input("Enter a in range(0,0.5): "))

                        b=100

                        while b>0.4:
                            b=float(raw_input("Enter b in range(0,0.4): "))

                        speed=100

                        while speed>0.4:
                            speed=float(raw_input("Enter speed in range(0,0.5): "))

                        lin.speed=speed
                        lin.a=a
                        lin.b=b

                        lin.Jac_move_in_line()
                elif selection == '9':
                        print ""
                        set_thetas()
                elif (selection == 'Q' or selection == 'q'):
                        env.Destroy()
                        break
                else: 
                        print "Unknown Option Selected!"

if __name__ == '__main__':
    main()
