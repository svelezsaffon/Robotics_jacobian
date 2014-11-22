__author__ = 'santiago & spencer'


import numpy

dict={}


def cosine(angle):
    return numpy.cos(angle)

def sine(angle):
    return numpy.sin(angle)

def cosine_deg(angle):
    return numpy.cos(numpy.radians(angle))

def sine_deg(angle):
    return numpy.sin(numpy.radians(angle))


def load_DH_table():
    global dict

    joint={}
    joint['theta']=90
    joint['alpha']=-90
    joint['a']=0
    joint['d']=0
    joint['range']=range(-160,160)

    dict[1]=joint

    joint1={}
    joint1['theta']=0
    joint1['alpha']=0
    joint1['a']=0.4318
    joint1['d']=0.14909
    joint1['range']=range(-225,45)

    dict[2]=joint1

    joint2={}
    joint2['theta']=90
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
            matrix[2][3]=dict[6]['d']

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


    def get_range(self,joint):
        return dict[joint]['range']


    def create_0Ai_matrix(self,index,angles):
        matrix=self.create_t_matrix(1,angles[0])

        for mat in range(2,index):
            matrix=self.multiply_matrix(matrix,self.create_t_matrix(mat,angles[mat-1]))

        return matrix
