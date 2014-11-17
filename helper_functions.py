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


class helper(object):

    def create_t_matrix(self,index,angle):
        matrix=[
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,1]
        ]

        matrix[0][0]=cosine(angle)
        matrix[1][0]=sine(angle)

        if index==1:
            matrix[0][2]=-sine(angle)
            matrix[1][2]=cosine(angle)
            matrix[2][1]=-1

        if index==2:
            matrix[0][1]=-sine(angle)
            matrix[1][1]=cosine(angle)
            a2=dict[2]['a']
            d2=dict[2]['d']
            matrix[0][3]=a2*cosine(angle)
            matrix[1][3]=a2*sine(angle)
            matrix[2][3]=d2
            matrix[2][2]=1

        if index==3:
            matrix[0][2]=sine(angle)
            matrix[1][2]=-cosine(angle)
            matrix[2][1]=1
            a3=dict[3]['a']

            matrix[0][3]=a3*cosine(angle)
            matrix[1][3]=a3*sine(angle)

        if index==4:
            matrix[2][1]=-1
            d4=dict[4]['d']
            matrix[2][3]=d4
            matrix[0][2]=-sine(angle)
            matrix[1][2]=cosine(angle)

        if index==5:
            matrix[2][1]=1
            matrix[0][2]=sine(angle)
            matrix[1][2]=-cosine(angle)

        if index==6:
            matrix[2][2]=1
            matrix[0][1]=-sine(angle)
            matrix[1][1]=cosine(angle)


        return matrix


    def multiply_matrix(self,a,b):
        aa=numpy.matrix(a)
        bb=numpy.matrix(b)
        res=numpy.dot(aa,bb)


        matrix=[
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0]
        ]

        for row in range(0,4):
            for col in range(0,4):
                matrix[row][col]=res.item((row,col))

        return matrix



    def create_0Ai_matrix(self,index,angles):
        matrix=self.create_t_matrix(1,angles[0])

        for mat in range(2,index):
            matrix=self.multiply_matrix(matrix,self.create_t_matrix(mat,angles[mat-1]))

        return matrix
