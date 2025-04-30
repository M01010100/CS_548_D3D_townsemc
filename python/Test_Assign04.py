import pytest
import numpy as np
import os
import Assign04 as A04
from General_Assign04 import *

###########################################################################
# get_matching_points
###########################################################################   

@pytest.mark.parametrize("key", filename_keys)
def test_get_matching_points(key):
    example = filename_pairs[key]
    first_model, second_model, R, Tr = prepare_pair(example)
    
    # Get actual points
    p_points = np.asarray(first_model.points)
    q_points = np.asarray(second_model.points)
    
    # Get matching points
    q_indices = A04.get_matching_points(p_points, q_points)
    
    # Load ground truth
    ground_data = np.load(os.path.join(GROUND_DATA_DIR,
                            get_matching_points_ground_filename(key)))
    # Check
    np.testing.assert_allclose(q_indices, ground_data, rtol=RTOL, atol=ATOL)
        
###########################################################################
# get_centered_cloud
###########################################################################   

@pytest.mark.parametrize("key", filename_keys)
def test_get_centered_cloud(key):
    example = filename_pairs[key]
    first_model, second_model, R, Tr = prepare_pair(example)
    
    # Get actual points
    p_points = np.asarray(first_model.points)
    q_points = np.asarray(second_model.points)
        
    # Load ground truth match data
    q_indices = np.load(os.path.join(GROUND_DATA_DIR,
                                get_matching_points_ground_filename(key)))
    
    # Get the matching points
    q_matches = q_points[q_indices]
    
    # Get centered cloud    
    c_q, centroid_q = A04.get_centered_cloud(q_matches)
    
    # Get ground truth for (target) centered clouds
    ground_centered_q = np.load(os.path.join(GROUND_DATA_DIR,
                                get_centered_cloud_ground_filename(key)))
    
    ground_c_q = ground_centered_q["c_q"]
    ground_centroid = ground_centered_q["centroid_q"]
    
    # Check
    np.testing.assert_allclose(c_q, ground_c_q, rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(centroid_q, ground_centroid, rtol=RTOL, atol=ATOL)
    
###########################################################################
# compute_RMSE
###########################################################################   

@pytest.mark.parametrize("key", filename_keys)
def test_compute_RMSE(key):
    example = filename_pairs[key]
    first_model, second_model, R, Tr = prepare_pair(example)
    
    # Get actual points
    p_points = np.asarray(first_model.points)
    q_points = np.asarray(second_model.points)
    
    # Load ground truth match data
    q_indices = np.load(os.path.join(GROUND_DATA_DIR,
                                get_matching_points_ground_filename(key)))
    
    # Get the matching points
    q_matches = q_points[q_indices]
        
    # Calculate RMSE
    rmse = A04.compute_RMSE(p_points, q_matches)    
    
    # Get ground truth for RMSE
    ground_rmse = np.load(os.path.join(GROUND_DATA_DIR,
                                compute_RMSE_ground_filename(key)))    
    
    # Check
    np.testing.assert_allclose(rmse, ground_rmse, rtol=RTOL, atol=ATOL)
    
###########################################################################
# compute_point_to_point_iteration
###########################################################################   

@pytest.mark.parametrize("key", filename_keys)
def test_compute_point_to_point_iteration(key):
    example = filename_pairs[key]
    first_model, second_model, R, Tr = prepare_pair(example)
    
    # Get actual points
    p_points = np.asarray(first_model.points)
    q_points = np.asarray(second_model.points)        

    # Check each iteration
    updated_p_points = np.copy(p_points)
    for i in range(3):
        updated_p_points, q_matches, R, Tr = A04.compute_point_to_point_iteration(updated_p_points, q_points)   
                
        # Grab ground truth iteration data
        ground_iter_data = np.load(os.path.join(GROUND_DATA_DIR,
                                compute_point_to_point_iteration_ground_filename(key, i)))
        
        ground_q_matches = ground_iter_data["q_matches"]
        ground_R = ground_iter_data["R"]
        ground_Tr = ground_iter_data["Tr"]        
           
        # Check
        err_msg = "Iteration %d" % i
        np.testing.assert_allclose(q_matches, ground_q_matches, rtol=RTOL, atol=ATOL, err_msg=err_msg)
        np.testing.assert_allclose(R, ground_R, rtol=RTOL, atol=ATOL, err_msg=err_msg)
        np.testing.assert_allclose(Tr, ground_Tr, rtol=RTOL, atol=ATOL, err_msg=err_msg)
    
###########################################################################
# do_point_to_point_icp
###########################################################################   

@pytest.mark.parametrize("key", filename_keys)
def test_do_point_to_point_icp(key):
    example = filename_pairs[key]
    first_model, second_model, R, Tr = prepare_pair(example)
    
    # Get actual points
    p_points = np.asarray(first_model.points)
    q_points = np.asarray(second_model.points)        
    
    # For each max iteration and min RMSE variation
    for max_iter in iter_values:
            for rmse_index in range(len(rmse_values)):        
                updated_p_points = np.copy(p_points)
                updated_p_points, total_transform, used_iter, current_rmse = A04.do_point_to_point_icp(updated_p_points, q_points, 
                                                                                                        max_iter=max_iter, 
                                                                                                        min_rmse=rmse_values[rmse_index])
                # Load ground truth
                ground_icp_data = np.load(os.path.join(GROUND_DATA_DIR,
                                do_point_to_point_icp_ground_filename(key, max_iter, rmse_index)))
        
                ground_total_transform = ground_icp_data["total_transform"]
                ground_used_iter = ground_icp_data["used_iter"]                
                ground_current_rmse = ground_icp_data["current_rmse"]
                
                # Check
                err_msg = "Max iter %d, min RMSE %f" % (max_iter, rmse_values[rmse_index])
                np.testing.assert_allclose(total_transform, ground_total_transform, rtol=RTOL, atol=ATOL, err_msg=err_msg)
                np.testing.assert_allclose(used_iter, ground_used_iter, rtol=RTOL, atol=ATOL, err_msg=err_msg)
                np.testing.assert_allclose(current_rmse, ground_current_rmse, rtol=RTOL, atol=ATOL, err_msg=err_msg)
