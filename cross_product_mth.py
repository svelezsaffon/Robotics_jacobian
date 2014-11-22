__author__ = 'santiago & spencer'


import helper_functions
import check_functions
import numpy





class cross_product_method(object):


    def __init__(self):

        self.angles=None
        self.helper= helper_functions.helper()
        self.checker=check_functions.check_cpm()


    def change_all_into_radians(self):
        pos=0
        for i in self.angles:
            self.angles[pos]=numpy.radians(i)
            pos+=1




    def solve_angles(self,angles):
        self.angles=angles


        self.change_all_into_radians()

        self.checker.set_angles_to_check(self.angles)

        link_matrix=self.create_link_transformation_matrices()

        based_mat=self.create_link_transformation_base_referenced(link_matrix)

        z_vectors=self.create_axis_of_motion(based_mat)

        pos_vectors=self.find_position_vector_previous_link_referenced(based_mat)


        jacobians=self.calculate_jacobians(z_vectors,pos_vectors)

        real= self.checker.check_j5()
        pos=4


        ##this part is just to check the error.
        error=[0,0,0,0,0,0]
        error[0]= numpy.abs(real[0][0]-jacobians[pos][0])
        error[1]= numpy.abs(real[1][0]-jacobians[pos][1])
        error[2]= numpy.abs(real[2][0]-jacobians[pos][2])
        error[3]= numpy.abs(real[3][0]-jacobians[pos][3])
        error[4]= numpy.abs(real[4][0]-jacobians[pos][4])
        error[5]= numpy.abs(real[5][0]-jacobians[pos][5])



        print "error"
        print error



    def calculate_jacobians(self,zvectors,pvectors):
        jacobians={}
        for pos in range(0,len(pvectors)):
            jac=[]
            jac=self.cross_product(zvectors[pos],pvectors[pos])
            jac.append(zvectors[pos][0])
            jac.append(zvectors[pos][1])
            jac.append(zvectors[pos][2])
            jacobians[pos]=jac

        return jacobians



    def cross_product(self,a,b):
        cross=numpy.cross(a,b)
        ret=[]
        ret.append(cross[0])
        ret.append(cross[1])
        ret.append(cross[2])


        return ret


    def get_vector_from_mat(self,mat,col):
        vector=[]
        if col=='p':
            col=3
        if col=='z':
            col=2
        if col=='y':
            col=1
        if col=='x':
            col=0

        for row in range(0,3):
            vector.append(mat[row][col])

        return vector

    def find_position_vector_previous_link_referenced(self,based_mat):
        pos_vectors={}

        pos_vectors[0]=self.get_vector_from_mat(based_mat[6],'p')

        for pos in range(1,6):
            pos_vectors[pos]=self.substract_vectors(pos_vectors[0],self.get_vector_from_mat(based_mat[pos],'p'))


        return pos_vectors

    def substract_vectors(self,a,b):
        aux=numpy.subtract(a,b)
        ret=[]
        ret.append(aux[0])
        ret.append(aux[1])
        ret.append(aux[2])


        return ret


    """
    This method created the axis of motions, this is, the 0Zi vector
    This method also returns a dictionary with the follwoing format
    dict[1]=0Z1
    dict[2]=0Z2
    """

    def create_axis_of_motion(self,based_mat):
        axis_motion={}

        axis_motion[0]=[0,0,1]

        pos=1
        for mat in based_mat:
            zvect=[]

            zvect.append(based_mat[pos][0][2])
            zvect.append(based_mat[pos][1][2])
            zvect.append(based_mat[pos][2][2])


            axis_motion[pos]=zvect

            pos+=1

        return axis_motion



        """
         With this method we are creating all the link transformation matrices.
         This link transformation matrices are referenced with the previous link= (i-1)A(i)
         They will be stored in a dictionary in the following structure
         dict[1] is the matrix 0A1
         dict[2] is the matrix 1A2
         ....
        """
    def create_link_transformation_matrices(self):
        matrix={}

        pos=1
        for angle in self.angles:
            matrix[pos]=self.helper.create_t_matrix(pos,angle)
            pos+=1

        return matrix


        """
         With this method we are creating all the link transformation matrices.
         This link transformation matrices are referenced with the base coordinate system= (0)A(i)
         They will be stored in a dictionary in the following structure
         dict[1] is the matrix 0A1
         dict[2] is the matrix 0A2
         ....
        """
    def create_link_transformation_base_referenced(self,matrix):
        base_mat={}

        base_mat[1]=matrix[1]

        for pos in range(2,len(matrix)+1):
            base_mat[pos]=self.helper.multiply_matrix(base_mat[pos-1],matrix[pos])

        return base_mat

def main():

    helper_functions.load_DH_table()
    check_functions.load_DH_table()

    cros= cross_product_method()
    angles=[100,25,32,0,0,0]

    cros.solve_angles(angles)




if __name__ == '__main__':
    main()
