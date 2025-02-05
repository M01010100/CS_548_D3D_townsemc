import pytest
import numpy as np
import open3d as o3d
import Assign01 as A01

files_to_test = [
    "./data/assign01/BunnyXYZ.pcd",
    "./data/assign01/BunnyXYZN.pcd",    
    "./data/assign01/BunnyXYZRGB.pcd",    
    "./data/assign01/BunnyXYZNRGB.pcd"   
]

class Test_Assign01:
    @pytest.mark.parametrize("filename", files_to_test)
    def test_load_pcd(self, filename):
        # Load with our function
        user_pcd = A01.load_pcd(filename)
        # Load with Open3D function
        ground_pcd = o3d.io.read_point_cloud(filename)
        
        # Check points, normals, and colors
        np.testing.assert_allclose(user_pcd.points, ground_pcd.points, err_msg="Points not loaded correctly.")
        np.testing.assert_allclose(user_pcd.normals, ground_pcd.normals, err_msg="Normals not loaded correctly.")
        np.testing.assert_allclose(user_pcd.colors, ground_pcd.colors, err_msg="Colors not loaded correctly.")
        