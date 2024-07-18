import unittest
import numpy as np
import sympy as sym
import numpy.testing as np_test
import inverse_kinematics as inv_kin

class TestNewtonRaphson(unittest.TestCase):
    def test_kinematic_decoupling(self):
        Tmatrix = np.array([[-0.95710678,0.25,0.14644661,0.43072012],
                            [0.25,0.45710678,0.85355339,0.66823729],
                            [0.14644661,0.85355339,-0.5,0.35855129],
                            [0,0,0,1,]])
        DHtable = np.array([
            [-90,0,0.67183,0],
            [0,0.4318,0.13970,0],
            [90,0.0203,0,0],
            [-90,0,0.4318,0],
            [90,0,0,0],
            [0,0,0.0565,0]
        ])
        init_guess = np.array([[45,45,45,45,45,45]])
        result = inv_kin.netwon_raphson_rf_method(Tmatrix,DHtable,10,0.01,init_guess)
        print(result)

if __name__ == "__main__":
    unittest.main()