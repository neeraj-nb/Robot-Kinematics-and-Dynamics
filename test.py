import unittest
import numpy as np
import numpy.testing as np_test
import forward_kinematics as forw_kin

class TestCreateTmatrix(unittest.TestCase):
    theta = 45
    r_matrix = np.array([
            [np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta)), 0],
            [np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta)), 0],
            [0, 0, 1]
        ])
    translation = np.array([[1],[2],[3]])
    perspective = np.array([0,0,0])
    scale = 1

    def test_create(self):
        result = forw_kin.create_Tmatrix(self.r_matrix,self.translation,self.perspective,self.scale)
        expected = np.array([[0.707107,-0.707107,0,1],[0.707107,0.707107,0,2],[0,0,1,3],[0,0,0,1]])
        np_test.assert_array_almost_equal(result,expected)

    def test_create_default(self):
        result = forw_kin.create_Tmatrix(self.r_matrix,self.translation)
        expected = np.array([[0.707107,-0.707107,0,1],[0.707107,0.707107,0,2],[0,0,1,3],[0,0,0,1]])
        np_test.assert_array_almost_equal(result,expected)

class TestExtractTmatrix(unittest.TestCase):
    case = np.array([[0.707107,-0.707107,0,1],[0.707107,0.707107,0,2],[0,0,1,3],[0,0,0,1]])
    def test_extract(self):
        r_matrix, translation, perspective, scale = forw_kin.extract_Tmatrix(self.case)
        theta = 45
        expected_r_matrix = np.array([
            [np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta)), 0],
            [np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta)), 0],
            [0, 0, 1]
        ])
        expected_translation = np.array([[1],[2],[3]])
        expected_perspective = np.array([0,0,0])
        expected_scale = 1
        np_test.assert_array_almost_equal(r_matrix,expected_r_matrix)
        np_test.assert_array_almost_equal(translation,expected_translation)
        np_test.assert_array_almost_equal(perspective,expected_perspective)
        np_test.assert_array_almost_equal(scale,expected_scale)

class TestGenerateTmatrixFromDHparameter(unittest.TestCase):
    def test_simple_rotation_z(self):
        alpha = 0
        a = 0
        d = 0
        theta = 45
        result = forw_kin.generate_Tmatrix_from_DHparameter(alpha,a,d,theta)
        expected = np.array([[0.707107,-0.707107,0,0],[0.707107,0.707107,0,0],[0,0,1,0],[0,0,0,1]])
        np_test.assert_array_almost_equal(result,expected)
    
    def test_simple_translation_z(self):
        alpha = 0
        a = 0
        d = 1
        theta = 0
        result = forw_kin.generate_Tmatrix_from_DHparameter(alpha,a,d,theta)
        expected = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
        np_test.assert_array_almost_equal(result,expected)

    def test_simple_rotation_x(self):
        alpha = 45
        a = 0
        d = 0
        theta = 0
        result = forw_kin.generate_Tmatrix_from_DHparameter(alpha,a,d,theta)
        expected = np.array([[1,0,0,0],[0,0.707107,-0.707107,0],[0,0.707107,0.707107,0],[0,0,0,1]])
        np_test.assert_array_almost_equal(result,expected)
    
    def test_simple_translation_x(self):
        alpha = 0
        a = 1
        d = 0
        theta = 0
        result = forw_kin.generate_Tmatrix_from_DHparameter(alpha,a,d,theta)
        expected = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        np_test.assert_array_almost_equal(result,expected)

    def test_combined(self):
        alpha = 45
        a = 1
        d = 1
        theta = 45
        result = forw_kin.generate_Tmatrix_from_DHparameter(alpha,a,d,theta)
        expected = np.array([[0.707107,-0.707107,0,1],[0.707107*0.707107,0.707107*0.707107,-0.707107,-0.707107],[0.707107*0.707107,0.707107*0.707107,0.707107,0.707107],[0,0,0,1]])
        np_test.assert_array_almost_equal(result,expected)

if __name__ == "__main__":
    unittest.main()