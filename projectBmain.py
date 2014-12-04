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

import cross_product_mth
import dtrm_approach
import inverse_jacobian
import circle_movement
import line_movement

import openravepy
import sys
from openravepy import *
import time

def main():
        menu = {}
        menu['1']="Solve Jacobian (Vector-Cross Product)" 
        menu['2']="Solve Jacobian (DTRM)"
        menu['3']="Moore-Penrose Pseudo Inverse (Vector-Cross Product)"
        menu['4']="Moore-Penrose Pseudo Inverse (DTRM)"
        menu['5']="Draw Circle (Inverse Kinematic)"
        menu['6']="Draw Circle (Inverse Jacobian)" 
        menu['7']="Draw Line (Inverse Kinematic)"
        menu['8']="Draw Line (Inverse Jacobian)"
        menu['9']="Exit"

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
                        inverse_jacobian.main()
                elif selection == '4':
                        print ""
                        inverse_jacobian.main()
                elif selection == '5':
                        print ""
                        circ.IK_move_in_circle()
                elif selection == '6':
                        print ""
                        circ.Jac_move_in_circle()
                elif selection == '7':
                        print ""
                        lin.IK_move_in_line()
                elif selection == '8': 
                        print ""
                        lin.Jac_move_in_line()
                elif selection == '9':
                        env.Destroy()
                        break
                else: 
                        print "Unknown Option Selected!"

if __name__ == '__main__':
    main()
