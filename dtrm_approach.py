__author__ = 'santiago & spencer'


import helper_functions
import check_functions
import inverse_jacobian
import numpy





class dtrm_approach(object):


    def __init__(self):

        self.angles=None
        self.helper= helper_functions.helper()
        self.checker=check_functions.check_dtrm()


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

        return self.calculate_jacobians_map(based_mat)




    def map_of_jacs_into_matrix(self,jacobians):
        jac_mat=[
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0]
        ]

        for i in range(0,6):
                jac_mat[0][i] = jacobians[i + 1][0][0]
                jac_mat[1][i] = jacobians[i + 1][1][0]
                jac_mat[2][i] = jacobians[i + 1][2][0]
                jac_mat[3][i] = jacobians[i + 1][3][0]
                jac_mat[4][i] = jacobians[i + 1][4][0]
                jac_mat[5][i] = jacobians[i + 1][5][0]

        return numpy.transpose(jac_mat)

    def error_statistics(self):

        file=open("dtrm_stats.txt",'ws')
        angles=[0,0,0,0,0,0]
        for t1 in self.helper.get_range(1):
            angles[0]=t1
            for t2 in self.helper.get_range(2):
                angles[1]=t1
                for t3 in self.helper.get_range(3):
                    angles[2]=t1
                    for t1 in self.helper.get_range(4):
                        angles[3]=t1
                        for t2 in self.helper.get_range(5):
                            angles[4]=t1
                            for t3 in self.helper.get_range(6):
                                angles[5]=t1
                                self.checker.set_angles_to_check(angles)
                                jack=self.solve_angles(angles)
                                error=self.calculate_errors(self.checker.check_j1(),jack[0])
                                file.write(str(error))
                                error=self.calculate_errors(self.checker.check_j2(),jack[1])
                                file.write(str(error))
                                error=self.calculate_errors(self.checker.check_j3(),jack[2])
                                file.write(str(error))
                                error=self.calculate_errors(self.checker.check_j4(),jack[3])
                                file.write(str(error))
                                error=self.calculate_errors(self.checker.check_j5(),jack[4])
                                file.write(str(error))
                                error=self.calculate_errors(self.checker.check_j6(),jack[5])
                                file.write(str(error))


        file.close()

    def calculate_errors(self,real,jac):
        error=[0,0,0,0,0,0]
        error[0]= numpy.abs(real[0][0]-jac[0])
        error[1]= numpy.abs(real[1][0]-jac[1])
        error[2]= numpy.abs(real[2][0]-jac[2])
        error[3]= numpy.abs(real[3][0]-jac[3])
        error[4]= numpy.abs(real[4][0]-jac[4])
        error[5]= numpy.abs(real[5][0]-jac[5])

        return error

    def calculate_jacobians_matrix(self, based_mat):
        mat=[
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0]
        ]

        jac=self.calculate_jacobians_map(based_mat)

        for i in range(0,len(based_mat)):
            for j in range(0,6):
                mat[j][i]=jac[i][j]

        return numpy.matrix(mat)

    def calculate_jacobians_map(self,based_mat):
        jacobians={}
        for pos in range(1,len(based_mat) + 1):
            jac=[[0],
                 [0],
                 [0],
                 [0],
                 [0],
                 [0]]
            px = based_mat[pos][0][3]
            py = based_mat[pos][1][3]
            nx = based_mat[pos][0][0]
            sx = based_mat[pos][0][1]
            ax = based_mat[pos][0][2]
            ny = based_mat[pos][1][0]
            sy = based_mat[pos][1][1]
            ay = based_mat[pos][1][2]
            nz = based_mat[pos][2][0]
            sz = based_mat[pos][2][1]
            az = based_mat[pos][2][2]
            jac[0][0] = px * ny - py * nx
            jac[1][0] = px * sy - py * sx
            jac[2][0] = px * ay - py * ax
            jac[3][0] = nz
            jac[4][0] = sz
            jac[5][0] = az
            jacobians[pos]=jac

        return jacobians


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
         dict[1] is the matrix 0A6
         dict[2] is the matrix 1A6
         dict[3] is the matrix 2A6
         ....
        """
    def create_link_transformation_base_referenced(self,matrix):
        base_mat={}

        for pos in range(1,len(matrix)+1):
            base_mat[pos] = matrix[pos]
            for subpos in range(pos+1, len(matrix)+1):
                base_mat[pos]=self.helper.multiply_matrix(base_mat[pos],matrix[subpos])

        return base_mat

def main():

    helper_functions.load_DH_table()
    check_functions.load_DH_table()
    inverser=inverse_jacobian.inverse_method()
    dtrm = dtrm_approach()

    angles=[56,47,23,14,5,8]


    jac= dtrm.solve_angles(angles)


    matjacs= dtrm.map_of_jacs_into_matrix(jac)


    moore= inverser.moore_penrose_equation(matjacs)

    print inverser.property_4(matjacs,moore)






if __name__ == '__main__':
    main()
