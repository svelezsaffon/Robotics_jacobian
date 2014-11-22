__author__ = 'santiago & spencer'



import numpy.linalg as num
import numpy



class inverse_method(object):


    def moore_penrose_equation(self,jacobian,n=6,m=6):

        if m < n:
            jt=numpy.transpose(jacobian)

            right=num.inv(numpy.dot(jacobian,jt))

            return numpy.dot(jt,right)

        if m > n:
            jt=numpy.transpose(jacobian)

            left=num.inv(numpy.dot(jt,jacobian))

            return numpy.dot(left,jt)



        return num.inv(jacobian)



    def property_1(self,jacobian,moore):
        left_side=numpy.dot(jacobian,moore)
        left_side=numpy.dot(left_side,jacobian)

        return numpy.abs(left_side-jacobian)


    def property_2(self,jacobian,moore):
        left_side=numpy.dot(moore,jacobian)
        left_side=numpy.dot(left_side,moore)
        return numpy.abs( left_side-moore)

    def property_3(self,jacobian,moore):
        left_side=numpy.transpose(numpy.dot(jacobian,moore))
        right=numpy.dot(jacobian,moore)
        return numpy.abs( left_side-right)

    def property_4(self,jacobian,moore):
        inner=numpy.dot(moore,jacobian)
        left=numpy.transpose(inner)

        return numpy.abs(left-inner)

    def real_penrose_equation(self,jacobian):
        return num.pinv(jacobian)
        #return numpy.p(jacobian)

