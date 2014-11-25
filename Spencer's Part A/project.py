#########################################################
# Title       : CS569 OpenRave Project Part A           #
#                                                       #
# Author      : Spencer Carver                          #
#                                                       #
# Description : Basic Kinematic and Inverse Kinematic   #
#               functionalities for the PUMA560 robotic #
#               arm examined in class.                  #
#########################################################

###########
# Imports #
###########
import sys
from openravepy import *
import time
from numpy import *
import json

####################
# Global Variables #
####################
global env
global robot
global params
#These values are not specified for our robot, and are instead taken from the default PUMA robot
global gripper_min
gripper_min = -56
global gripper_max
gripper_max = 8
#Degree step
global degree_step
degree_step = 5
#End Effector HTM
global a_matricies
global T6
global T3
global T4
global G2
#Indicators for the Inverse Kinematic Solutions
global ARM
global ARM2
global ELBOW
global WRIST
global M

###########
# Methods #
###########
def calculateT6(thetas):
        global params
        A1 = zeros((4,4))
        A1[0,0] = cos(thetas[0])
        A1[0,2] = -sin(thetas[0])
        A1[1,0] = sin(thetas[0])
        A1[1,2] = cos(thetas[0])
        A1[2,1] = -1
        A1[3,3] = 1
        A2 = zeros((4,4))
        A2[0,0] = cos(thetas[1])
        A2[0,1] = -sin(thetas[1])
        A2[0,3] = params["Joint 2"]["a"] * cos(thetas[1])
        A2[1,0] = sin(thetas[1])
        A2[1,1] = cos(thetas[1])
        A2[1,3] = params["Joint 2"]["a"] * sin(thetas[1])
        A2[2,2] = 1
        A2[2,3] = params["Joint 2"]["d"]
        A2[3,3] = 1
        A3 = zeros((4,4))
        A3[0,0] = cos(thetas[2])
        A3[0,2] = sin(thetas[2])
        A3[0,3] = params["Joint 3"]["a"] * cos(thetas[2])
        A3[1,0] = sin(thetas[2])
        A3[1,2] = -cos(thetas[2])
        A3[1,3] = params["Joint 3"]["a"] * sin(thetas[2])
        A3[2,1] = 1
        A3[3,3] = 1
        A4 = zeros((4,4))
        A4[0,0] = cos(thetas[3])
        A4[0,2] = -sin(thetas[3])
        A4[1,0] = sin(thetas[3])
        A4[1,2] = cos(thetas[3])
        A4[2,1] = -1
        A4[2,3] = params["Joint 4"]["d"]
        A4[3,3] = 1
        A5 = zeros((4,4))
        A5[0,0] = cos(thetas[4])
        A5[0,2] = sin(thetas[4])
        A5[1,0] = sin(thetas[4])
        A5[1,2] = -cos(thetas[4])
        A5[2,1] = 1
        A5[3,3] = 1
        A6 = zeros((4,4))
        A6[0,0] = cos(thetas[5])
        A6[0,1] = -sin(thetas[5])
        A6[1,0] = sin(thetas[5])
        A6[1,1] = cos(thetas[5])
        A6[2,2] = 1
        A6[2,3] = params["Joint 6"]["d"]
        A6[3,3] = 1
        T1 = A1
        T2 = dot(A1, A2)
        T3 = dot(T2, A3)
        T4 = dot(T3, A4)
        T5 = dot(T4, A5)
        T6 = dot(T5, A6)
        return [T1, T2, T3, T4, T5, T6]

def printMatricies(printBool):
        global params
        #General form of an A matrix (symbolic)
        if printBool:
                print "general form of an A matrix: "        
                symMatrix = [["[[cos(theta)", "-sin(theta)cos(alpha)", " sin(theta)sin(alpha)", "a cos(theta)]"], 
                  [" [sin(theta)", " cos(theta)cos(alpha)", "-cos(theta)sin(alpha)", "a sin(theta)]"], 
                  [" [    0     ", "      sin(alpha)     ", "      cos(alpha)     ", "      d     ]"],  
                  [" [    0     ", "          0          ", "          0          ", "      1     ]]"]]
                print '\n'.join([' '.join(row) for row in symMatrix])
                print ""

        #Output all its link transformation matrices in a symbolic form (as a check to see your input is correct)
        global a_matricies
        a_matricies = []
        for i in range(1,7) :
                a_temp = matrix([[cos(radians(params["Joint %d" % (i)]["theta"])), -sin(radians(params["Joint %d" % (i)]["theta"]))*cos(radians(params["Joint %d" % (i)]["alpha"])), sin(radians(params["Joint %d" % (i)]["theta"]))*sin(radians(params["Joint %d" % (i)]["alpha"])), params["Joint %d" % (i)]["a"]*cos(radians(params["Joint %d" % (i)]["theta"]))], [sin(radians(params["Joint %d" % (i)]["theta"])), cos(radians(params["Joint %d" % (i)]["theta"]))*cos(radians(params["Joint %d" % (i)]["alpha"])), -cos(radians(params["Joint %d" % (i)]["theta"]))*sin(radians(params["Joint %d" % (i)]["alpha"])), params["Joint %d" % (i)]["a"]*sin(radians(params["Joint %d" % (i)]["theta"]))], [0, sin(radians(params["Joint %d" % (i)]["alpha"])), cos(radians(params["Joint %d" % (i)]["alpha"])), params["Joint %d" % (i)]["d"]], [0, 0, 0, 1]])
                a_matricies.append(a_temp)

        global T6
        global T4
        global T3
        global G2
        T3 = matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        T4 = matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        T6 = matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        T3 = T3 * a_matricies[0] * a_matricies[1] * a_matricies[2]
        T4 = T4 * a_matricies[0] * a_matricies[1] * a_matricies[2] * a_matricies[3]
        #T6 = T6 * a_matricies[0] * a_matricies[1] * a_matricies[2] * a_matricies[3] * a_matricies[4] * a_matricies[5]
        T6 = dot(dot(dot(dot(dot(dot(T6, a_matricies[0]), a_matricies[1]), a_matricies[2]), a_matricies[3]), a_matricies[4]), a_matricies[5])
        if printBool:
                print "0A1 = ", a_matricies[0]
                print "1A2 = ", a_matricies[1]
                print "2A3 = ", a_matricies[2]
                print "3A4 = ", a_matricies[3]
                print "4A5 = ", a_matricies[4]
                print "5A6 = ", a_matricies[5]

def load(filename):
        #Read the D-H parameters from a file (given by the user).
        f = open(filename, 'r')

        # format {"Joint i" : {"theta" : 0, "alpha" : 0, "a" : 0, "d" : 0, "range min" : 0, "range max" : 0}}
        global params
        params = json.load(f)
        f.close()
        #Load the robot from the local files (since ours differs from the default PUMA)
        global robot
	robot = env.ReadRobotXMLFile('robots/puma569.robot.xml')
	env.Add(robot)

        #Set the DOF Limits for the robot based on what is read in from the DH parameters
        global gripper_min
        global gripper_max
        robot.SetDOFLimits(radians([params["Joint 1"]["range min"], params["Joint 2"]["range min"], params["Joint 3"]["range min"], params["Joint 4"]["range min"], params["Joint 5"]["range min"], params["Joint 6"]["range min"], gripper_min]), radians([params["Joint 1"]["range max"], params["Joint 2"]["range max"], params["Joint 3"]["range max"], params["Joint 4"]["range max"], params["Joint 5"]["range max"], params["Joint 6"]["range max"], gripper_max]))

        #Add axes to each coordinate frame of the robot
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
        
        #Set the DOF Values for the robot based on what is read in from the DH parameters
        robot.SetDOFValues(radians([params["Joint 1"]["theta"], params["Joint 2"]["theta"], params["Joint 3"]["theta"], params["Joint 4"]["theta"], params["Joint 5"]["theta"], params["Joint 6"]["theta"], 0]), [0,1,2,3,4,5,6])

def calculateIndicators(T6, theta2, theta3, theta4):
        global params        
        global ARM
        global ARM2
        global ELBOW
        global WRIST
        global M
        global FLIP
        a2 = params["Joint 2"]["a"]
        a3 = params["Joint 3"]["a"]
        d4 = params["Joint 4"]["d"]
        # + if right, - if left
        ARM = sign(-d4 * sin(theta2 + theta3) - a3 * cos(theta2 + theta3) - a2 * cos(theta2))
        # for circle substitution
        ARM2 = sign(cos(theta2) * (d4 * cos(theta3) - a3 * sin(theta3)) - sin(theta2) * (d4 * sin(theta3) + a3 * cos(theta3) + a2))
        # + if above, - if below
        ELBOW = ARM * sign(d4 * cos(theta3) - a3 * sin(theta3))
        # Wrist
        global a_matricies
        z3 = a_matricies[3][2, :3]
        z4 = a_matricies[4][2, :3]
        n = T6[:3, 0]
        s = T6[:3, 1]
        a = T6[:3, 2]
        if dot(s.T, z4.T) == 0 :
                WRIST = (sign(dot(n.T, z4.T))).item(0)
        else :
                WRIST = (sign(dot(s.T, z4.T))).item(0)
        # M
        temp = cross(z3, a.T)
        if dot(s.T, temp.T) == 0 :
                M = (WRIST * sign(dot(n.T, (temp/linalg.norm(temp)).T))).item(0)
        else :
                M = (WRIST * sign(dot(s.T, (temp/linalg.norm(temp)).T))).item(0)
        # FLIP        
        FLIP = 1
        #print "INVERSE KINEMATIC JOINT INDICATORS:"
        #print "ARM = %d" % (ARM)
        #print "ARM2 = %d" % (ARM2)
        #print "ELBOW = %d" % (ELBOW)
        #print "WRIST = %d" % (WRIST)
        #print "M = %d" % (M)

def writeToPointsFile(f, dof):
        transformationMatricies = calculateT6(dof)
        T6 = transformationMatricies[5]
        for i in range(0, 16):
                f.write(str(`T6.item(i)`+" "))
        f.write("\n")
        for val in dof:
                f.write(`dof[val]`+" ")
        f.write("\n")

def forwardKinematics():
        global robot
        f = open('points.txt', 'w')
        #Move each joint through its entire range of motion
        for k in range(0,5):
                t = radians(params["Joint %d" % (k+1)]["theta"])
                #Move from starting position to minimum	
		while (t > radians(params["Joint %d" % (k+1)]["range min"])):
		    t -= radians(degree_step)
		    robot.SetDOFValues([t],[k]) # set joint 0 to value 0.5
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
                    writeToPointsFile(f, robot.GetActiveDOFValues())
		    time.sleep(0.01)  
                #Move from minimum to maximum
		while (t < radians(params["Joint %d" % (k+1)]["range max"])):
		    t += radians(degree_step)
		    robot.SetDOFValues([t],[k]) # set joint 0 to value 0.5
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
                    writeToPointsFile(f, robot.GetActiveDOFValues())
		    time.sleep(0.01) 
                #Move from maximum back to starting position
		while (t > radians(params["Joint %d" % (k+1)]["theta"])):
		    t -= radians(degree_step)
		    robot.SetDOFValues([t],[k]) # set joint 0 to value 0.5
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
                    writeToPointsFile(f, robot.GetActiveDOFValues())
		    time.sleep(0.01) 
        #Move the gripper through its entire range of motion
        global gripper_min
        global gripper_max
        #Move from starting position to minimum
        while (t > radians(gripper_min)):
	    t -= radians(degree_step)
	    robot.SetDOFValues([t],[6]) # set joint 0 to value 0.5
	    time.sleep(0.01)
        #Move from minimum to maximum
        while (t < radians(gripper_max)):
            t += radians(degree_step)
            robot.SetDOFValues([t],[6]) # set joint 0 to value 0.5
            time.sleep(0.01)
        #Move from maximum back to starting position
        while (t > 0):
            t -= radians(degree_step)
            robot.SetDOFValues([t],[6]) # set joint 0 to value 0.5
            time.sleep(0.01)
        f.close()

def geometricApproach():
        printMatricies(False)
        theta1 = input("theta1 = ")
        theta2 = input("theta2 = ")
        theta3 = input("theta3 = ")
        theta4 = input("theta4 = ")
        theta5 = input("theta5 = ")
        theta6 = input("theta6 = ")
        thetas = radians([theta1, theta2, theta3, theta4, theta5, theta6])
        robot.SetDOFValues(thetas,[0,1,2,3,4,5])
        transformationMatricies = calculateT6(thetas)
        T6 = transformationMatricies[5]
        calculateIndicators(T6, radians(theta2), radians(theta3), radians(theta4))
        print transformationMatricies[5]
        T3 = transformationMatricies[2]
        T4 = transformationMatricies[3]
        global params
        global ARM
        global ELBOW
        px = T4.item(3)
        py = T4.item(7)
        pz = T4.item(11)
        d2 = params["Joint 2"]["d"]
        d4 = params["Joint 4"]["d"]        
        a2 = params["Joint 2"]["a"]
        a3 = params["Joint 3"]["a"]
        #Solve for theta1
        theta1 = arctan2(-ARM * py * sqrt(power(px,2) + power(py,2) - power(d2,2)) - px * d2, -ARM * px * sqrt(power(px,2) + power(py,2) - power(d2,2)) + py * d2)
        print "theta1 = " , degrees(theta1)
        #Solve for theta2
        R = sqrt(power(px,2) + power(py,2) + power(pz,2) - power(d2,2))
        r = sqrt(power(px,2) + power(py,2) - power(d2,2))
        sinalpha = -pz/R
        cosalpha = -(ARM * r)/R
        cosbeta = (power(a2,2) + power(R,2) - (power(d4,2) + power(a3,2)))/(2*a2*R)
        sinbeta = sqrt(1 - power(cosbeta, 2))
        sintheta2 = sinalpha * cosbeta + (ARM * ELBOW) * cosalpha * sinbeta
        costheta2 = cosalpha * cosbeta - (ARM * ELBOW) * sinalpha * sinbeta
        theta2 = arctan2(sintheta2, costheta2)
        print "theta2 = " , degrees(theta2)
        #Solve for theta3
        cosphi = (power(a2,2) + (power(d4,2) + power(a3,2)) - power(R,2))/(2*a2*sqrt(power(d4,2) + power(a3,2)))
        sinphi = ARM * ELBOW * sqrt(1 - power(cosphi, 2))
        sinbeta2 = d4/sqrt(power(d4,2) + power(a3,2))
        cosbeta2 = abs(a3)/sqrt(power(d4,2) + power(a3,2))
        sintheta3 = sinphi*cosbeta2 - cosphi*sinbeta2
        costheta3 = cosphi*cosbeta2 + sinphi*sinbeta2
        theta3 = arctan2(sintheta3, costheta3)
        print "theta3 = ", degrees(theta3)
        #Solve for theta4
        global WRIST
        global M
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
        print "theta4 = " , degrees(theta4)
        #Solve for theta5
        theta5 = arctan2((cos(theta1) * cos(theta2 + theta3) * cos(theta4) - sin(theta1) * sin(theta4)) * ax + (sin(theta1) * cos(theta2 + theta3) * cos(theta4) + cos(theta1) * sin(theta4)) * ay - sin(theta2 + theta3) * cos(theta4) * az, cos(theta1) * sin(theta2 + theta3) * ax + sin(theta1) * sin(theta2 + theta3) * ay + cos(theta2 + theta3) * az)
        print "theta5 = " , degrees(theta5)
        #Solve for theta6
        theta6 = arctan2((-sin(theta1) * cos(theta4) - cos(theta1) * cos(theta2 + theta3) * sin(theta4)) * nx + (cos(theta1) * cos(theta4) - sin(theta1) * cos(theta2 + theta3) * sin(theta4)) * ny + (sin(theta2 + theta3) * sin(theta4)) * nz, (-sin(theta1) * cos(theta4) - cos(theta1) * cos(theta2 + theta3) * sin(theta4)) * sx + (cos(theta1) * cos(theta4) - sin(theta1) * cos(theta2 + theta3) * sin(theta4)) * sy + (sin(theta2 + theta3) * sin(theta4)) * sz)
        print "theta6 = " , degrees(theta6)

def circleSubstitution(T6):
        global params
        global ARM
        global ELBOW
        nx = T6.item(0)
        sx = T6.item(1)
        ax = T6.item(2)
        ny = T6.item(4)
        sy = T6.item(5)
        ay = T6.item(6)
        nz = T6.item(8)
        sz = T6.item(9)
        az = T6.item(10)
        px = T6.item(3)
        py = T6.item(7)
        pz = T6.item(11)
        d2 = params["Joint 2"]["d"]
        d4 = params["Joint 4"]["d"]        
        a2 = params["Joint 2"]["a"]
        a3 = params["Joint 3"]["a"]
        #Solve for theta1
        r = sqrt(power(px, 2) + power(py, 2))
        theta1 = arctan2(py,px) - arctan2(d2,-ARM * sqrt(power(r,2) - power(d2, 2)))
        #Solve for theta3
        d = power(cos(theta1) * px + sin(theta1) * py, 2) + power(pz, 2) - power(d4, 2) - power(a3, 2) - power(a2, 2)
        e2 = 4 * power(a2, 2) * power(a3, 2) + 4 * power(a2, 2) * power(d4, 2)   
        theta3 = arctan2(d, ARM * ELBOW * sqrt(e2 - power(d, 2))) - arctan2(a3,d4)
        #Solve for theta2
        f = cos(theta1) * px + sin(theta1) * py
        h2 = power(d4, 2) + power(a2, 2) + power(a3, 2) + 2*a2*d4*sin(theta3) + 2*a2*a3*cos(theta3)
        theta2 = arctan2(f, ARM2 * sqrt(h2 - power(f, 2))) - arctan2(d4 * sin(theta3) + a3 * cos(theta3) + a2,d4 * cos(theta3) - a3 * sin(theta3))
        #Solve for theta4
        theta4 = arctan2(WRIST*(cos(theta1)*ay - sin(theta1)*ax),WRIST*(cos(theta1)*cos(theta2 + theta3)*ax + sin(theta1)*cos(theta2+theta3)*ay - sin(theta2 + theta3)*az))
        #Solve for theta5        
        sintheta5 = cos(theta4)*(cos(theta2 + theta3)*(cos(theta1)*ax + sin(theta1)*ay) - sin(theta2 + theta3)*az) + sin(theta4)*(-sin(theta1)*ax + cos(theta1)*ay)
        costheta5 = sin(theta2 + theta3)*(cos(theta1)*ax + sin(theta1)*ay) + cos(theta2 + theta3)*az
        theta5 = arctan2(sintheta5,costheta5)
        #Solve for theta6
        sintheta6 = -sin(theta4)*(cos(theta2 + theta3)*(cos(theta1)*nx + sin(theta1)*ny) - sin(theta2 + theta3)*nz) + cos(theta4)*(-sin(theta1)*nx + cos(theta1)*ny)
        costheta6 = -sin(theta4)*(cos(theta2 + theta3)*(cos(theta1)*sx + sin(theta1)*sy) - sin(theta2 + theta3)*sz) + cos(theta4)*(-sin(theta1)*sx + cos(theta1)*sy)
        theta6 = arctan2(sintheta6, costheta6)
        return [theta1, theta2, theta3, theta4, theta5, theta6]

def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i + 1

def circleSubstitutionMain():
        T6 = zeros((4,4))
        lines=file_len('points.txt')
        f = open('points.txt', 'r')
        error = open('error.txt', 'w')
        errortot= [0, 0, 0, 0, 0, 0]
        for line in range(0, lines/2):
            mat=f.readline().split(" ")
            dofl=f.readline().split(" ")
            dof=[]
            for aux in range(0,len(dofl)-1):
                dof.append(float(dofl[aux]))
            pos=[]
            aux=0
            for i in range(0,len(mat)-1):
                T6.itemset(i, float(mat[i]))
            calculateIndicators(T6, dof[1], dof[2], dof[3])
            sol = circleSubstitution(T6)
            if not math.isnan(sol[0]) and not math.isnan(sol[1]) and not math.isnan(sol[2]) and not math.isnan(sol[3]) and not math.isnan(sol[4]) and not math.isnan(sol[5]):
                
                robot.SetDOFValues(sol,[0,1,2,3,4,5])
                errorlocal = [0,0,0,0,0,0]
                for i in range(0,len(sol)) :
                    errorlocal[i]=dof[i]-sol[i]
                    error.write(str(errorlocal))
                    error.write("\n")
                    errortot[i]+=errorlocal[i]

                time.sleep(0.1)

            else:
                print "Unable to calculate angles for data on line: %d" % (2*line)



        error.write(str(errortot))

        error.close()
        f.close()

def halfAngleFormula(T6):
        global params
        global ARM
        global ELBOW
        nx = T6.item(0)
        sx = T6.item(1)
        ax = T6.item(2)
        ny = T6.item(4)
        sy = T6.item(5)
        ay = T6.item(6)
        nz = T6.item(8)
        sz = T6.item(9)
        az = T6.item(10)
        px = T6.item(3)
        py = T6.item(7)
        pz = T6.item(11)
        d2 = params["Joint 2"]["d"]
        d4 = params["Joint 4"]["d"]        
        a2 = params["Joint 2"]["a"]
        a3 = params["Joint 3"]["a"]
        #Solve for theta1
        theta1 = 2*arctan2(-px-ARM*sqrt(power(px,2) + power(py,2) - power(d2,2)),d2 + py)
        #Solve for theta3
        d = power(cos(theta1) * px + sin(theta1) * py, 2) + power(pz, 2) - power(d4, 2) - power(a3, 2) - power(a2, 2)
        e2 = 4 * power(a2, 2) * power(a3, 2) + 4 * power(a2, 2) * power(d4, 2)   
        theta3 = 2*arctan2(2*a2*d4 - ARM * ELBOW * sqrt(e2 - power(d, 2)), d+2*a2*a3)
        #Solve for theta2
        f = cos(theta1) * px + sin(theta1) * py
        h2 = power(d4, 2) + power(a2, 2) + power(a3, 2) + 2*a2*d4*sin(theta3) + 2*a2*a3*cos(theta3)
        theta2 = 2*arctan2((d4*sin(theta3) - a3*sin(theta3))-ARM2*sqrt(h2 - power(f,2)),f + (d4*sin(theta3) + a3*cos(theta3) + a2))
        #Solve for theta4
        theta4 = arctan2(WRIST*(cos(theta1)*ay - sin(theta1)*ax),WRIST*(cos(theta1)*cos(theta2 + theta3)*ax + sin(theta1)*cos(theta2+theta3)*ay - sin(theta2 + theta3)*az))
        #Solve for theta5        
        sintheta5 = cos(theta4)*(cos(theta2 + theta3)*(cos(theta1)*ax + sin(theta1)*ay) - sin(theta2 + theta3)*az) + sin(theta4)*(-sin(theta1)*ax + cos(theta1)*ay)
        costheta5 = sin(theta2 + theta3)*(cos(theta1)*ax + sin(theta1)*ay) + cos(theta2 + theta3)*az
        theta5 = arctan2(sintheta5,costheta5)
        #Solve for theta6
        sintheta6 = -sin(theta4)*(cos(theta2 + theta3)*(cos(theta1)*nx + sin(theta1)*ny) - sin(theta2 + theta3)*nz) + cos(theta4)*(-sin(theta1)*nx + cos(theta1)*ny)
        costheta6 = -sin(theta4)*(cos(theta2 + theta3)*(cos(theta1)*sx + sin(theta1)*sy) - sin(theta2 + theta3)*sz) + cos(theta4)*(-sin(theta1)*sx + cos(theta1)*sy)
        theta6 = arctan2(sintheta6, costheta6)
        return [theta1, theta2, theta3, theta4, theta5, theta6]

def halfAngleFormulaMain():
        T6 = zeros((4,4))
        lines=file_len('points.txt')
        f = open('points.txt', 'r')
        error = open('error.txt', 'w')
        errortot= [0, 0, 0, 0, 0, 0]
        for line in range(0, lines/2):
            mat=f.readline().split(" ")
            dofl=f.readline().split(" ")
            dof=[]
            for aux in range(0,len(dofl)-1):
                dof.append(float(dofl[aux]))
            pos=[]
            aux=0
            for i in range(0,len(mat)-1):
                T6.itemset(i, float(mat[i]))
            calculateIndicators(T6, dof[1], dof[2], dof[3])
            sol = halfAngleFormula(T6)
            if not math.isnan(sol[0]) and not math.isnan(sol[1]) and not math.isnan(sol[2]) and not math.isnan(sol[3]) and not math.isnan(sol[4]) and not math.isnan(sol[5]):
                
                robot.SetDOFValues(sol,[0,1,2,3,4,5])
                errorlocal = [0,0,0,0,0,0]
                for i in range(0,len(sol)) :
                    errorlocal[i]=dof[i]-sol[i]
                    error.write(str(errorlocal))
                    error.write("\n")
                    errortot[i]+=errorlocal[i]

                time.sleep(0.1)

            else:
                print "Unable to calculate angles for data on line: %d" % (2*line)



        error.write(str(errortot))

        error.close()
        f.close()

def inverseKinematics():
        printMatricies(False)
        menu = {}
        menu['1']="Circle-Equation Substitution (from points file)"
        menu['2']="Half-Angle Formula (from points file)"
        menu['3']="Back"
        while True: 
                options=menu.keys()
                options.sort()
                print ""
                print "~~~~~~~ Inverse Kinematics Method Selection Menu~~~~~~~"
                for entry in options: 
                        print entry, ")", menu[entry]
                selection=raw_input("Please Select:") 
                if selection == '1':
                        print "" 
                        circleSubstitutionMain()
                elif selection == '2':
                        print ""
                        halfAngleFormulaMain()
                elif selection == '3': 
                        print ""
                        break
                else: 
                        print "Unknown Option Selected!"

def rotateViewer():
        #Does not work correctly at the moment
        global env
        Tz = matrixFromAxisAngle([numpy.pi/2,numpy.pi/4,0])
        with env:
                for body in env.GetBodies():
                        body.SetTransform(numpy.dot(Tz,body.GetTransform()))

def mainMenu():
        menu = {}
        menu['1']="Print A Matricies (values from provided dh parameters)" 
        menu['2']="Forward Kinematics (generates points file)"
        menu['3']="Inverse Kinematics (requires points file)"
        menu['4']="Geometric Inverse (manually input thetas)" 
        menu['5']="Change Settings"
        menu['6']="Exit"
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
                        printMatricies(True)
                elif selection == '2':
                        print "" 
                        forwardKinematics()
                elif selection == '3':
                        print ""
                        inverseKinematics()
                elif selection == '4':
                        print ""
                        geometricApproach()
                elif selection == '5':
                        print ""
                        global degree_step
                        degree_step = int(input("Select a theta step size (default 5 degrees):"))
                elif selection == '6': 
                        break
                else: 
                        print "Unknown Option Selected!"

def main():
        if len(sys.argv) == 1:
                # no dh parameters specified, alert user and use defaults
                filename = 'defaultdhparameters.txt'
                print "No DH-Parameters specified. Loading parameters from: %s" % (filename)
        elif len(sys.argv) == 2:
                # dh parameters specified
                filename = sys.argv[1]
        else:   
                #Invalid number of parameters
                print "ERROR: invalid number of parameters (expected: 1 or 2, recieved: %d)" % (len(sys.argv))                
                sys.exit(0)
        try:
                global env
                env = Environment()
                env.SetViewer('qtcoin')
                load(filename)
                #rotateViewer()
                #print planningutils.GetDHParameters(robot)
                mainMenu()

        finally:
	        env.Destroy()

main()
#body = env.ReadKinBodyXMLFile(filename='data/puma_tabletop.env.xml')
#env.AddKinBody(body)
#body.SetTransform([1,0,0,0,0,0,0])
#pose = poseFromMatrix(body.GetTransform())
  	
#robot.drawarrow(p1=[0.0,0.0,0.0],p2=[500,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0])
#handles=[]
#handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[500,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0]))
#handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,500,0.0],linewidth=0.01,color=[0.0,1.0,0.0]))
#handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.0,500],linewidth=0.01,color=[0.0,0.0,0.1]))

#raw_input("")
#with env: # lock the environment since robot will be used
#raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))
#robot.SetDOFValues([0.5],[0]) # set joint 0 to value 0.5
#T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
#raveLogInfo("The transformation of link 1 is:\n"+repr(T))
#time.sleep(10)
