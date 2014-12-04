#########################################################
# Title       : CS569 OpenRave Project Part A           #
#                                                       #
# Author      : Spencer Carver                          #
#                                                       #
# Description : Basic Kinematic and Inverse Kinematic   #
#               functionalities for the PUMA560 robotic #
#               arm examined in class.                  #
#########################################################

import cross_product_mth
import dtrm_approach
import circle_movement
import line_movement

import openravepy
import sys
from openravepy import *
import time

def main():
        menu = {}
        menu['1']="Solve Jacobian (Vector-Cross Product Approach)" 
        menu['2']="Solve Jacobian (DTRM Approach)"
        menu['3']="Draw Circle (Inverse Kinematic)"
        menu['4']="Draw Circle (Inverse Jacobian)" 
        menu['5']="Draw Line (Inverse Kinematic)"
        menu['6']="Draw Line (Inverse Jacobian)"
        menu['7']="Exit"

        env = Environment()
        env.SetViewer('qtcoin') # attach viewer (optional)
        env.Load('pumaarm.dae') # load a simple scene
        robot = env.GetRobots()[0]

        circ = circle_movement.circular_movement(env, robot)
        lin = line_movement.linear_movement(env, robot, 0.1, 0.3)

        while True: 
                options=menu.keys()
                options.sort()
                print ""
                print "~~~~~~~ ECE/CS 569 OpenRave PUMA Robotic Arm Main Menu~~~~~~~"
                for entry in options: 
                        print entry, ")", menu[entry]
                selection=raw_input("Please Select:") 
                if selection =='1': 
                        print ""
                        cross_product_mth.main()
                elif selection == '2':
                        print "" 
                        dtrm_approach.main()
                elif selection == '3':
                        print ""
                        circ.IK_move_in_circle()
                elif selection == '4':
                        print ""
                        circ.Jac_move_in_circle()
                elif selection == '5':
                        print ""
                        lin.IK_move_in_line()
                elif selection == '6': 
                        print ""
                        lin.Jac_move_in_line()
                elif selection == '7':
                        env.Destroy()
                        break
                else: 
                        print "Unknown Option Selected!"

if __name__ == '__main__':
    main()
