from General_Assign04 import *
import copy
import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree

def get_matching_points(p_points, q_points):
    kdtree = cKDTree(q_points)
    _, indices = kdtree.query(p_points)
    
    return indices

def get_centered_cloud(points):
    centered_points = np.copy(points)
    centroid = np.mean(centered_points, axis=0)
    centered_points = centered_points - centroid
    
    return centered_points, centroid

def compute_point_to_point_iteration(p_points, q_points):
    match_indices = get_matching_points(p_points, q_points)
    q_matches = q_points[match_indices]
    
    centered_p, p_centroid = get_centered_cloud(p_points)
    centered_q, q_centroid = get_centered_cloud(q_matches)
    
    cross_variance = centered_p.T @ centered_q
    
    U, _, Vh = np.linalg.svd(cross_variance)
    V = Vh.T
    R = V @ U.T
    
    if np.linalg.det(R) < 0:
        V[:, 2] = -V[:, 2] 
        R = V @ U.T
    
    Tr = q_centroid - (R @ p_centroid)
    updated_p_points = (R @ p_points.T).T + Tr
    
    return updated_p_points, q_matches, R, Tr

def compute_RMSE(p_points, q_points):
    # squared distances
    squared_distances = np.sum((p_points - q_points)**2, axis=1)
    
    # RMSE
    rmse = np.sqrt(np.mean(squared_distances))
    
    return rmse

def do_point_to_point_icp(p_points, q_points, max_iter, min_rmse):
    # Initialize variables
    current_rmse = float('inf')
    iterations = 0
    total_transform = np.eye(4)
    
    # Copy p_points to avoid modifying the original
    current_p_points = np.copy(p_points)
    
    while current_rmse > min_rmse:
        
        updated_p_points, q_matches, R, Tr = compute_point_to_point_iteration(current_p_points, q_points)
        
        current_transform = create_transform_4x4(R, Tr)        
        total_transform = current_transform @ total_transform
        
        current_rmse = compute_RMSE(updated_p_points, q_matches)
        current_p_points = updated_p_points
        
        iterations += 1
        
        # Check iterations 
        if iterations >= max_iter:
            break
    
    return current_p_points, total_transform, iterations, current_rmse


def main(): 
    # Set key (see General_Assign04 for list of options) 
    key = "BUNNY_ROT_SMALL_ALL_TR_BIG" 
     
    # Get example 
    example = filename_pairs[key]   
                 
    # Load data 
    p_cloud, q_cloud, R, Tr = prepare_pair(example) 
         
    # Get actual points 
    p_points = np.asarray(p_cloud.points) 
    q_points = np.asarray(q_cloud.points) 
         
    # Copy p_cloud 
    orig_p_cloud = copy.deepcopy(p_cloud) 
             
    # Set current error to max error to start 
    MAX_ERROR = 40000000 
    current_error = MAX_ERROR 
    current_iteration = 0 
     
    # Set minimum error 
    min_error = 1e-6 
     
 
    # Animation function     
    def animate(vis): 
        nonlocal p_points 
        nonlocal current_error 
        nonlocal current_iteration 
         
        if current_error > min_error: 
            # Do Point-to-Point 
            updated_p_points, q_matches, R, Tr = compute_point_to_point_iteration(p_points, q_points) 
             
            # Increment iteration 
            current_iteration += 1 
             
            # Get RMSE 
            current_error = compute_RMSE(updated_p_points, q_matches) 
            print("Iteration %03d, RMSE: %f" % (current_iteration, current_error)) 
             
            # Set new points 
            p_points = updated_p_points 
             
            # Update cloud visualization 
            p_cloud.points = o3d.utility.Vector3dVector(updated_p_points)         
            vis.update_geometry(p_cloud) 
         
        return False   
     
    def reset_cloud(vis): 
        nonlocal p_points 
        nonlocal current_error 
        nonlocal current_iteration 
         
        current_error = MAX_ERROR 
        current_iteration = 0 
        p_points = np.asarray(orig_p_cloud.points) 
        p_cloud.points = orig_p_cloud.points 
        vis.update_geometry(p_cloud) 
         
    def close_window(vis): 
        vis.close()            
        return False  
         
    # Create visualization
    vis = o3d.visualization.VisualizerWithKeyCallback() 
    vis.create_window("ICP", width=800, height=600) 
    vis.add_geometry(p_cloud) 
    vis.add_geometry(q_cloud) 
 
    vis.register_key_callback(256, close_window)   
    vis.register_key_callback(ord(" "), reset_cloud)   
     
    while True: 
        animate(vis)          
        if not vis.poll_events(): 
            break           
        vis.update_renderer() 
 
    vis.destroy_window() 
 
if __name__ == "__main__": 
    main()