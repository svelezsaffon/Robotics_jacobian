__author__ = 'santiago & spencer'


import numpy


dict={}

def load_DH_table():
    global dict

    joint={}
    joint['theta']=100
    joint['alpha']=-90
    joint['a']=0
    joint['d']=0
    joint['range']=range(-160,160)

    dict[1]=joint

    joint1={}
    joint1['theta']=25
    joint1['alpha']=0
    joint1['a']=0.4318
    joint1['d']=0.14909
    joint1['range']=range(-225,45)

    dict[2]=joint1

    joint2={}
    joint2['theta']=32
    joint2['alpha']=90
    joint2['a']=-0.02032
    joint2['d']=0
    joint2['range']=range(-45,225)

    dict[3]=joint2


    joint3={}
    joint3['theta']=0
    joint3['alpha']=-90
    joint3['a']=0
    joint3['d']=0.43307
    joint3['range']=range(-110,170)

    dict[4]=joint3

    joint4={}
    joint4['theta']=0
    joint4['alpha']=90
    joint4['a']=0
    joint4['d']=0
    joint4['range']=range(-100,100)

    dict[5]=joint4

    joint5={}
    joint5['theta']=0
    joint5['alpha']=0
    joint5['a']=0
    joint5['d']=0.05625
    joint5['range']=range(-266,266)

    dict[6]=joint5




def cosine(angle):
    return numpy.cos(angle)

def sine(angle):
    return numpy.sin(angle)


def cosine_deg(angle):
    return numpy.cos(numpy.radians(angle))

def sine_deg(angle):
    return numpy.sin(numpy.radians(angle))

class check_cpm(object):

    def __init__(self):
        self.angles=[]

    def set_angles_to_check(self,angle):
        self.angles=angle

    def check_j1(self):
        return self.j1(self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4], dict[2]['d'], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j1(self, theta1, theta2, theta3, theta4, theta5, d2, a2, a3, d4, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = -sine(theta1) * (d6 * (cosine(theta2 + theta3) * cosine(theta4) * sine(theta5) + sine(theta2 + theta3) * cosine(theta5)) + d4 * sine(theta2 + theta3) + a3 * cosine(theta2 + theta3) + a2 * cosine(theta2)) - cosine(theta1) * (d6 * sine(theta4) * sine(theta5) + d2)
        matrix[1][0] = cosine(theta1) * (d6 * (cosine(theta2 + theta3) * cosine(theta4) * sine(theta5) + sine(theta2 + theta3) * cosine(theta5)) + d4 * sine(theta2 + theta3) + a3 * cosine(theta2 + theta3) + a2 * cosine(theta2)) - sine(theta1) * (d6 * sine(theta4) * sine(theta5) + d2)
        matrix[2][0] = 0
        matrix[3][0] = 0
        matrix[4][0] = 0
        matrix[5][0] = 1
        return matrix

    def check_j2(self):
        return self.j2(self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j2(self, theta1, theta2, theta3, theta4, theta5, a2, a3, d4, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = cosine(theta1) * (d6 * (cosine(theta2 + theta3) * cosine(theta5) - sine(theta2 + theta3) * cosine(theta4) * sine(theta5)) + d4 * cosine(theta2 + theta3) - a3 * sine(theta2 + theta3) - a2 * sine(theta2))
        matrix[1][0] = sine(theta1) * (d6 * (cosine(theta2 + theta3) * cosine(theta5) - sine(theta2 + theta3) * cosine(theta4) * sine(theta5)) + d4 * cosine(theta2 + theta3) - a3 * sine(theta2 + theta3) - a2 * sine(theta2))
        matrix[2][0] = -1 * (d6 * (sine(theta2 + theta3) * cosine(theta5) + cosine(theta2 + theta3) * cosine(theta4) * sine(theta5)) + d4 * sine(theta2 + theta3) + a3 * cosine(theta2 + theta3) + a2 * cosine(theta2))
        matrix[3][0] = -sine(theta1)
        matrix[4][0] = cosine(theta1)
        matrix[5][0] = 0
        return matrix

    def check_j3(self):
        return self.j3(self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j3(self, theta1, theta2, theta3, theta4, theta5, a3, d4, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = cosine(theta1) * (d6 * (cosine(theta2 + theta3) * cosine(theta5) - sine(theta2 + theta3) * cosine(theta4) * sine(theta5)) + d4 * cosine(theta2 + theta3) - a3 * sine(theta2 + theta3))
        matrix[1][0] = sine(theta1) * (d6 * (cosine(theta2 + theta3) * cosine(theta5) - sine(theta2 + theta3) * cosine(theta4) * sine(theta5)) + d4 * cosine(theta2 + theta3) - a3 * sine(theta2 + theta3))
        matrix[2][0] = -1 * (d6 * (sine(theta2 + theta3) * cosine(theta5) + cosine(theta2 + theta3) * cosine(theta4) * sine(theta5)) + d4 * sine(theta2 + theta3) + a3 * cosine(theta2 + theta3))
        matrix[3][0] = -sine(theta1)
        matrix[4][0] = cosine(theta1)
        matrix[5][0] = 0
        return matrix

    def check_j4(self):
       return self.j4(self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4],  dict[6]['d'])

    def j4(self, theta1, theta2, theta3, theta4, theta5, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = -d6 * sine(theta5) * (sine(theta1) * cosine(theta4) + cosine(theta1) * cosine(theta2 + theta3) * sine(theta4))
        matrix[1][0] = d6 * sine(theta5) * (cosine(theta1) * cosine(theta4) - sine(theta1) * cosine(theta2 + theta3) * sine(theta4))
        matrix[2][0] = d6 * sine(theta2 + theta3) * sine(theta4) * sine(theta5)
        matrix[3][0] = cosine(theta1) * sine(theta2 + theta3)
        matrix[4][0] = sine(theta1) * sine(theta2 + theta3)
        matrix[5][0] = cosine(theta2 + theta3)
        return matrix

    def check_j5(self):
       return self.j5(self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4],  dict[6]['d'])

    def j5(self, theta1, theta2, theta3, theta4, theta5, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = -d6 * (sine(theta1) * sine(theta4) * cosine(theta5) + cosine(theta1) * sine(theta2 + theta3) * sine(theta5) - cosine(theta1) * cosine(theta2 + theta3) * cosine(theta4) * cosine(theta5))
        matrix[1][0] = d6 * (cosine(theta1) * sine(theta4) * cosine(theta5) - sine(theta1) * sine(theta2 + theta3) * sine(theta5) + sine(theta1) * cosine(theta2 + theta3) * cosine(theta4) * cosine(theta5))
        matrix[2][0] = -d6 * (cosine(theta2 + theta3) * sine(theta5) + sine(theta2 + theta3) * cosine(theta4) * cosine(theta5))
        matrix[3][0] = -cosine(theta1) * cosine(theta2 + theta3) * sine(theta4) - sine(theta1) * cosine(theta4)
        matrix[4][0] = -sine(theta1) * cosine(theta2 + theta3) * sine(theta4) + cosine(theta1) * cosine(theta4)
        matrix[5][0] = sine(theta2 + theta3) * sine(theta4)
        return matrix

    def check_j6(self):
       return self.j6(self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4])

    def j6(self, theta1, theta2, theta3, theta4, theta5):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = 0
        matrix[1][0] = 0
        matrix[2][0] = 0
        matrix[3][0] = sine(theta5) * (cosine(theta1) * cosine(theta2 + theta3) * cosine(theta4) - sine(theta1) * sine(theta4)) + cosine(theta1) * sine(theta2 + theta3) * cosine(theta5)
        matrix[4][0] = sine(theta5) * (sine(theta1) * cosine(theta2 + theta3) * cosine(theta4) + cosine(theta1) * sine(theta4)) + sine(theta1) * sine(theta2 + theta3) * cosine(theta5)
        matrix[5][0] = -sine(theta2 + theta3) * cosine(theta4) * sine(theta5) + cosine(theta2 + theta3) * cosine(theta5)
        return matrix

class check_dtrm(object):

    def __init__(self):
        self.angles=[]

    def set_angles_to_check(self,angle):
        self.angles=angle

    def check_j1(self):
        self.j1(self.angles[1], self.angles[2], self.angles[3], self.angles[4], self.angles[5], dict[2]['d'], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j1(self, theta2, theta3, theta4, theta5, theta6, d2, a2, a3, d4, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = (d6 * (cosine(theta2 + theta3) * cosine(theta4) * sine(theta5) + sine(theta2 + theta3) * cosine(theta5)) + d4 * sine(theta2 + theta3) + a3 * cosine(theta2 + theta3) + a2 * cosine(theta2)) * (sine(theta4) * cosine(theta5) * cosine(theta6) + cosine(theta4) * sine(theta6)) - (d6 * sine(theta4) * sine(theta5) + d2) * (cosine(theta2 + theta3) * (cosine(theta4) * cosine(theta5) * cosine(theta6) - sine(theta4) * sine(theta6)) - sine(theta2 + theta3) * sine(theta5) * cosine(theta6))
        matrix[1][0] = (d6 * (cosine(theta2 + theta3) * cosine(theta4) * sine(theta5) + sine(theta2 + theta3) * cosine(theta5)) + d4 * sine(theta2 + theta3) + a3 * cosine(theta2 + theta3) + a2 * cosine(theta2)) * (-sine(theta4) * cosine(theta5) * sine(theta6) + cosine(theta4) * cosine(theta6)) - (d6 * sine(theta4) * sine(theta5) + d2) * (-cosine(theta2 + theta3) * (cosine(theta4) * cosine(theta5) * sine(theta6) - sine(theta4) * cosine(theta6)) - sine(theta2 + theta3) * sine(theta5) * sine(theta6))
        matrix[2][0] = (d6 * (cosine(theta2 + theta3) * cosine(theta4) * sine(theta5) + sine(theta2 + theta3) * cosine(theta5)) + d4 * sine(theta2 + theta3) + a3 * cosine(theta2 + theta3) + a2 * cosine(theta2)) * (sine(theta4) * sine(theta5)) - (d6 * sine(theta4) * sine(theta5) + d2) * (cosine(theta2 + theta3) * cosine(theta4) * sine(theta5) + sine(theta2 + theta3) * cosine(theta5))
        matrix[3][0] = -1 * (sine(theta2 + theta3) * (cosine(theta4) * cosine(theta5) * cosine(theta6) - sine(theta4) * sine(theta6)) + cosine(theta2 + theta3) * sine(theta5) * cosine(theta6))
        matrix[4][0] = sine(theta2 + theta3) * (cosine(theta4) * cosine(theta5) * sine(theta6) - sine(theta4) * cosine(theta6)) + cosine(theta2 + theta3) * sine(theta5) * sine(theta6)
        matrix[5][0] = -sine(theta2 + theta3) * cosine(theta4) * sine(theta5) + cosine(theta2 + theta3) * cosine(theta5)
        return matrix

    def check_j2(self):
        self.j2(self.angles[1], self.angles[2], self.angles[3], self.angles[4], self.angles[5], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j2(self, theta2, theta3, theta4, theta5, theta6, a2, a3, d4, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = (cosine(theta4) * cosine(theta5) * cosine(theta6) - sine(theta4) * sine(theta6)) * (d6 * cosine(theta5) + d4 + a2 * sine(theta3)) + sine(theta5) * cosine(theta6) * (d6 * cosine(theta4) * sine(theta5) + a3 + a2 * cosine(theta3))
        matrix[1][0] = -1 * (cosine(theta4) * cosine(theta5) * sine(theta6) + sine(theta4) * cosine(theta6)) * (d6 * cosine(theta5) + d4 + a2 * sine(theta3)) - sine(theta5) * sine(theta6) * (d6 * cosine(theta4) * sine(theta5) + a3 + a2 * cosine(theta3))
        matrix[2][0] = cosine(theta4) * sine(theta5) * (d6 * cosine(theta5) + d4 + a2 * sine(theta3)) - cosine(theta5) * (d6 * cosine(theta4) * sine(theta5) + a3 + a2 * cosine(theta3))
        matrix[3][0] = sine(theta4) * cosine(theta5) * cosine(theta6) + cosine(theta4) * sine(theta6)
        matrix[4][0] = -sine(theta4) * cosine(theta5) * sine(theta6) + cosine(theta4) * cosine(theta6)
        matrix[5][0] = sine(theta4) * sine(theta5)
        return matrix

    def check_j3(self):
        self.j3(self.angles[3], self.angles[4], self.angles[5], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j3(self, theta4, theta5, theta6, a3, d4, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = (a3 + d6 * cosine(theta4) * sine(theta5)) * (sine(theta5) * cosine(theta6)) + (d4 + d6 * cosine(theta5)) * (cosine(theta4) * cosine(theta5) * cosine(theta6) - sine(theta4) * sine(theta6))
        matrix[1][0] = -1 * (a3 + d6 * cosine(theta4) * sine(theta5)) * (sine(theta5) * sine(theta6)) - (d4 + d6 * cosine(theta5)) * (cosine(theta4) * cosine(theta5) * sine(theta6) + sine(theta4) * cosine(theta6))
        matrix[2][0] = -1 * (a3 + d6 * cosine(theta4) * sine(theta5)) * cosine(theta5) + (d4 + d6 * cosine(theta5)) * cosine(theta4) * sine(theta5)
        matrix[3][0] = sine(theta4) * cosine(theta5) * cosine(theta6) + cosine(theta4) * sine(theta6)
        matrix[4][0] = -sine(theta4) * cosine(theta5) * sine(theta6) + cosine(theta4) * cosine(theta6)
        matrix[5][0] = sine(theta4) * sine(theta5)
        return matrix

    def check_j4(self):
        self.j4(self.angles[4], self.angles[5], dict[6]['d'])

    def j4(self, theta5, theta6, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = d6 * sine(theta5) * sine(theta6)
        matrix[1][0] = d6 * sine(theta5) * cosine(theta6)
        matrix[2][0] = 0
        matrix[3][0] = -sine(theta5) * cosine(theta6)
        matrix[4][0] = sine(theta5) * sine(theta6)
        matrix[5][0] = cosine(theta5)
        return matrix

    def check_j5(self):
        self.j5(self.angles[5],  dict[6]['d'])

    def j5(self, theta6, d6):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = d6 * cosine(theta6)
        matrix[1][0] = -d6 * sine(theta6)
        matrix[2][0] = 0
        matrix[3][0] = sine(theta6)
        matrix[4][0] = cosine(theta6)
        matrix[5][0] = 0
        return matrix

    def check_j6(self):
        self.j6()

    def j6(self):
        matrix = [[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]]
        matrix[0][0] = 0
        matrix[1][0] = 0
        matrix[2][0] = 0
        matrix[3][0] = 0
        matrix[4][0] = 0
        matrix[5][0] = 1
        return matrix
