import unittest
import numpy as np
import numpy.testing as np_test
import velocity_kinematics as vel_kin

class TestJacobian(unittest.TestCase):
    def test_jacobian(self):
        DHtable = np.array([
            [-90,0,0.67183,45],
            [0,0.4318,0.13970,45],
            [90,0.0203,0,45],
            [-90,0,0.4318,45],
            [90,0,0,45],
            [0,0,0.0565,45]
        ])
        result = vel_kin.jacobian(DHtable)

class TestSingularity(unittest.TestCase):
    def test_singularity(self):

        DHtable = np.array([
            [-90,0,0.67183,45],
            [0,0.4318,0.13970,45],
            [90,0.0203,0,45],
            [-90,0,0.4318,45],
            [90,0,0,45],
            [0,0,0.0565,45]
        ])
        jointlimits = [[0,10],[0,10],[0,10],[0,10],[0,10],[0,10]]
        result= vel_kin.singularity(DHtable,jointlimits)
        print(result)
    
if __name__ == "__main__":
    unittest.main()