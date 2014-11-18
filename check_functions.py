__author__ = 'santiago & spencer'


import numpy


global dict

def cosine(angle):
    return numpy.cos(angle)

def sine(angle):
    return numpy.sin(angle)

def cosine_deg(angle):
    return numpy.cos(numpy.radians(angle))

def sine_deg(angle):
    return numpy.sin(numpy.radians(angle))

class check_cpm(object):

    def check_j1(self):
        j1(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'], dict[2]['d'], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

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
        j2(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

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
        j3(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

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
        j4(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'],  dict[6]['d'])

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
        j5(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'],  dict[6]['d'])

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
        j6(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'])

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

    def check_j1(self):
        j1(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'], dict[2]['d'], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j1(self, theta1, theta2, theta3, theta4, theta5, d2, a2, a3, d4, d6):
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

    def check_j2(self):
        j2(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'], dict[2]['a'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j2(self, theta1, theta2, theta3, theta4, theta5, a2, a3, d4, d6):
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

    def check_j3(self):
        j3(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'], dict[3]['a'], dict[4]['d'], dict[6]['d'])

    def j3(self, theta1, theta2, theta3, theta4, theta5, a3, d4, d6):
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

    def check_j4(self):
        j4(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'],  dict[6]['d'])

    def j4(self, theta1, theta2, theta3, theta4, theta5, d6):
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

    def check_j5(self):
        j5(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'],  dict[6]['d'])

    def j5(self, theta1, theta2, theta3, theta4, theta5, d6):
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

    def check_j6(self):
        j6(dict[1]['theta'], dict[2]['theta'], dict[3]['theta'], dict[4]['theta'], dict[5]['theta'])

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
        matrix[3][0] = 0
        matrix[4][0] = 0
        matrix[5][0] = 1
        return matrix
