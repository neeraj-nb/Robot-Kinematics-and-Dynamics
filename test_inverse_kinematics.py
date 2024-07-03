import unittest
import numpy as np
import sympy as sym
import numpy.testing as np_test
import inverse_kinematics as inv_kin

theta1, theta2, theta3, theta4, theta5, theta6 = sym.symbols('theta1 theta2 theta3 theta4 theta5 theta6')

class TestNewtonRaphson(unittest.TestCase):
    def test_kinematic_decoupling(self):
        Tmatrix = np.array([[-0.95710678,0.25,0.14644661,0.43072012],
                            [0.25,0.45710678,0.85355339,0.66823729],
                            [0.14644661,0.85355339,-0.5,0.35855129],
                            [0,0,0,1,]])
        DHtable = sym.Matrix([
            [-90,0,0.67183,theta1],
            [0,0.4318,0.13970,theta2],
            [90,0.0203,0,theta3],
            [-90,0,0.4318,theta4],
            [90,0,0,theta5],
            [0,0,0.0565,theta6]
        ])
        init_guess = {theta1:30,theta2:30,theta3:30,theta4:30,theta5:30,theta6:30}
        result = inv_kin.netwon_raphson_rf_method(Tmatrix,DHtable,10,0.01,init_guess)
        print(result)

if __name__ == "__main__":
    unittest.main()