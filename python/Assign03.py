import open3d as o3d
#import G03
import numpy as np
import copy

def compute_distances(center, points):
    return np.sqrt(np.sum((points - center)**2, axis=1))

def compute_gaussian_weights(center, points, sigma):
    distances = compute_distances(center, points)
    weights = np.exp(-(distances**2) / (2 * sigma**2))
    return weights

def compute_weighted_PCA(points, weights):
    total_weight = np.sum(weights)
    weighted_points = points * weights[:, np.newaxis]
    centroid = np.sum(weighted_points, axis=0) / total_weight
    
    centered_points = points - centroid
    
    cov_matrix = np.zeros((3, 3))
    for i in range(len(points)):
        point = centered_points[i].reshape(3, 1)
        cov_matrix += weights[i] * (point @ point.T)
    cov_matrix /= total_weight
    
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    
    W = eigenvectors[:, 0]  # Normal vector
    V = eigenvectors[:, 1]  # Second vector
    U = eigenvectors[:, 2]  # Most important vector
    '''    
    if U[0] < 0:
        U = -U
    if V[0] < 0:
        V = -V    
    if W[0] < 0:
        W = -W
    '''    
    return centroid, U, V, W

def project_points_to_plane(points, centroid, U, V, W):
    centered = points - centroid
    
    transform = np.vstack([U, V, W])
    
    projected = centered @ transform.T
    
    return projected
    
def reverse_plane_projection(projected, centroid, U, V, W):
    transform = np.vstack([U, V, W])
    
    unprojected = (projected @ transform) + centroid
    
    return unprojected
    
    
def make_design_matrix_A(projected):
    u = projected[:, 0]
    v = projected[:, 1]
    
    A = np.column_stack((
        np.ones_like(u),
        u,
        v,
        u**2,
        u*v,
        v**2
    ))
    
    return A
    
def make_vector_b(projected):
    w = projected[:, 2]
    
    return w.reshape(-1, 1)

def make_weight_matrix_G(weights):
    return np.diag(weights)

def compute_polynomial_coefficients(projected, weights):
    A = make_design_matrix_A(projected)
    b = make_vector_b(projected)
    G = make_weight_matrix_G(weights)
    
    AT_G = np.transpose(A) @ G #A.T @ G
    AT_G_A = AT_G @ A
    AT_G_b = AT_G @ b
    
    a = np.linalg.inv(AT_G_A) @ AT_G_b
    
    return a

def project_points_to_polynomial(points, centroid, U, V, W, a):
    projected = project_points_to_plane(points, centroid, U, V, W)
    
    A = make_design_matrix_A(projected)
    
    predicted_w = A @ a
    
    projected_new = projected.copy()
    projected_new[:, 2] = predicted_w.flatten()
    
    result = reverse_plane_projection(projected_new, centroid, U, V, W)
    
    return result

def fit_to_polynomial(center, points, sigma):
    weights = compute_gaussian_weights(center, points, sigma)
    
    centroid, U, V, W = compute_weighted_PCA(points, weights)
    
    projected = project_points_to_plane(points, centroid, U, V, W)
    
    a = compute_polynomial_coefficients(projected, weights)
    
    center_reshaped = center.reshape(1, 3)
    updated_center = project_points_to_polynomial(center_reshaped, centroid, U, V, W, a)
    
    return updated_center[0], W

def perform_moving_least_squares(cloud, radius, sigma):
    kdtree = o3d.geometry.KDTreeFlann(cloud)
    points = np.asarray(cloud.points)
    
    output_points = np.zeros_like(points)
    output_normals = np.zeros_like(points)
    output_colors = np.zeros_like(points)
    
    for i in range(len(points)):
        query_point = points[i]
        
        [_, idx, _] = kdtree.search_radius_vector_3d(query_point, radius)
        
        if len(idx) < 6:  
            output_points[i] = query_point
            output_normals[i] = [0, 0, 1]  
            output_colors[i] = [0, 0, 0] 
            continue
        
        neighbors = points[idx]
        
        updated_point, normal = fit_to_polynomial(query_point, neighbors, sigma)
        
        output_points[i] = updated_point
        output_normals[i] = normal
        
        distance = np.linalg.norm(query_point - updated_point)
        output_colors[i] = [distance, 0, 0] 
   
    output_cloud = o3d.geometry.PointCloud()
    output_cloud.points = o3d.utility.Vector3dVector(output_points)
    output_cloud.normals = o3d.utility.Vector3dVector(output_normals)
    output_cloud.colors = o3d.utility.Vector3dVector(output_colors)
    
    return output_cloud

def visualize_clouds(all_clouds, point_show_normal=False): 
    adjusted_clouds = [] 
    x_inc = 20.0 
    y_inc = 20.0 
     
    for i in range(len(all_clouds)): 
        one_set_clouds = all_clouds[i] 
                 
        for j in range(len(one_set_clouds)): 
            center = (x_inc*j, y_inc*i, 0) 
            adjusted_clouds.append(one_set_clouds[j].translate(center)) 
                                
    o3d.visualization.draw_geometries(adjusted_clouds,  
            point_show_normal=point_show_normal)       
      
def main():     
    cloud = o3d.io.read_point_cloud( 
            "data/assign03/input/noise_pervasive_large_bunny.pcd") 
     
    radius = 1.0 
    sigma = radius / 3.0 
     
    output_cloud = perform_moving_least_squares(cloud, radius, sigma) 
           
    output_points_only = copy.deepcopy(output_cloud) 
    output_points_only.colors = o3d.utility.Vector3dVector([]) 
    output_points_only.normals = o3d.utility.Vector3dVector([]) 
     
    output_points_normals = copy.deepcopy(output_cloud) 
    output_points_normals.colors = o3d.utility.Vector3dVector([]) 
              
    visualize_clouds([[cloud, output_points_only,  
              output_points_normals, output_cloud]])     
 
if __name__ == "__main__": 
    main()