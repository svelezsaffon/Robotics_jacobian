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

def main():
        menu = {}
        menu['1']="Solve Jacobian (Vector-Cross Product Approach)" 
        menu['2']="Solve Jacobian (DTRM Approach)"
        menu['3']="Draw Circle (Inverse Kinematic)"
        menu['4']="Draw Circle (Inverse Jacobian)" 
        menu['5']="Draw Line (Inverse Kinematic)"
        menu['6']="Draw Line (Inverse Jacobian)"
        menu['7']="Exit"
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
                        lin=circle_movement.circular_movement()
                        lin.IK_move_in_circle()
                        lin.quit()
                elif selection == '4':
                        print ""
                        lin=circle_movement.circular_movement()
                        lin.Jac_move_in_circle()
                        lin.quit()
                elif selection == '5':
                        print ""
                        line_movement.main()
                elif selection == '6': 
                        print ""
                        line_movement.main()
                elif selection == '7':
                        break
                else: 
                        print "Unknown Option Selected!"

if __name__ == '__main__':
    main()
