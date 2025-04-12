import open3d as o3d
import numpy as np
import copy
def compute_distance(center, points):
    ##TODO: compute the distance between center and points    

def compute_gaussian_weight(center, point, sigma):
    #TODO: compute the gaussian weight of point with respect to center
    
def compute_weighted_PCA(points, weights):
    
def project_points_to_plane(points, centroid, U, V, W):
    
def reverse_plane_projection(projected, centroid, U, V, W):
    
def make_design_matrix_A(projected):
    
def make_vector_b(projected):
    
def make_weight_matrix_G(weights):
    
def compute_polynomial_coefficients(projected, weights):

def projected_points_to_polynomial(points, centroid, U, V, W, a):
    
def fit_to_polynomial(center, points, sigma):
    
def perform_moving_least_squares(cloud, radius, sigma):
    points = np.asarray(cloud.points)
    normals = np.asarray(cloud.normals)
    
    kdtree = o3d.geometry.KDTreeFlann(cloud)
    
    output_cloud = o3d.geometry.PointCloud()
    
    for i in range(len(points)):
        point = points[i]
        
        [k, idx, _] = kdtree.search_radius_vector_3d(point, radius)
        
        if k > 0:
            neighbors = points[idx]
            weights = compute_distance(point, neighbors)
            
            a, U, V, W = fit_to_polynomial(point, neighbors, sigma)
            
            projected_point = projected_points_to_polynomial(point, point, U, V, W, a)
            
            output_cloud.points.append(projected_point)
    
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
              
    G03.visualize_clouds([[cloud, output_points_only,  
              output_points_normals, output_cloud]])     
 
if __name__ == "__main__": 
    main()