import numpy as np
import open3d as o3d
import os

###########################################################################
# GLOBALS
###########################################################################

INPUT_DATA_DIR = os.path.join("data", "assign04", "input")
GROUND_DATA_DIR = os.path.join("data", "assign04", "ground")

RTOL = 1e-6
ATOL = 1e-6

filename_pairs = {
    "BUNNY_SAME":                     [ "bunny.pcd", "bunny.pcd", (0,0,0), (0,0,0)],
    "BUNNY_TR_SMALL":                 [ "bunny.pcd", "bunny.pcd", (0,0,0), (2,0,0)],
    "BUNNY_TR_BIG":                   [ "bunny.pcd", "bunny.pcd", (0,0,0), (5,2,0)],
    
    "BUNNY_ROT_SMALL_X":              [ "bunny.pcd", "bunny.pcd", (10,0,0), (0,0,0)],
    "BUNNY_ROT_SMALL_Y_TR_SMALL":     [ "bunny.pcd", "bunny.pcd", (0,10,0), (2,0,0)],
    "BUNNY_ROT_SMALL_Z_TR_BIG":       [ "bunny.pcd", "bunny.pcd", (0,0,10), (5,2,0)],
    "BUNNY_ROT_SMALL_ALL_TR_BIG":     [ "bunny.pcd", "bunny.pcd", (10,10,10), (5,2,0)],
    
    "BUNNY_ROT_BIG_X":                [ "bunny.pcd", "bunny.pcd", (45,0,0), (0,0,0)],
    "BUNNY_ROT_BIG_ALL_TR_BIG":       [ "bunny.pcd", "bunny.pcd", (45,45,45), (5,2,0)],
    
    "CUBES":                    [ "cube_0.pcd", "cube_1.pcd", (45,45,45), (5,2,0)],
    "PLANES":                   [ "plane.pcd", "plane.pcd", (45,45,45), (5,2,0)],
}

filename_keys = list(filename_pairs.keys())

iter_values = [50, 10, 2]
rmse_values = [1e-6, 0.5]

###########################################################################
# HELPER FUNCTIONS
###########################################################################

def create_transform_4x4(R, Tr):
    R_mat = np.eye(4)
    R_mat[0:3,0:3] = R
    
    Tr_mat = np.eye(4)
    Tr_mat[3] = np.concatenate([Tr, [1]], axis=0)
    Tr_mat = Tr_mat.T
    
    transform = Tr_mat @ R_mat
    
    return transform

def apply_transform_4x4(points, transform):
    ones = np.ones((points.shape[0], 1))                     
    tmp_points = np.concatenate([points, ones], axis=1)        
    tmp_points = (transform @ tmp_points.T).T
    tmp_points = tmp_points[:, 0:3]    
    return tmp_points

def get_matching_points_ground_filename(key):
    return "match_" + key + ".npy"

def get_centered_cloud_ground_filename(key):
    return "centered_cloud_" + key + ".npz"

def compute_RMSE_ground_filename(key):
    return "rmse_" + key + ".npy"

def compute_point_to_point_iteration_ground_filename(key, i):
    return "icp_ptp_%s_iter_%03d.npz" % (key, i)

def do_point_to_point_icp_ground_filename(key, max_iter, rmse_index):
    return "icp_ptp_%s_max_iter_%03d_rmse_%02d.npz" % (key, max_iter, rmse_index)

def prepare_pair(example):
    first_model = o3d.io.read_point_cloud(os.path.join(INPUT_DATA_DIR, example[0]))
    second_model = o3d.io.read_point_cloud(os.path.join(INPUT_DATA_DIR, example[1])) 

    first_model.paint_uniform_color((0,1,0))
    second_model.paint_uniform_color((0,0,1))

    angles = np.radians(np.array(example[2]))
    R = o3d.geometry.get_rotation_matrix_from_xyz(angles)    
    second_model.rotate(R)
    second_model.translate(example[3])  
    
    return first_model, second_model, R, np.array(example[3])  
