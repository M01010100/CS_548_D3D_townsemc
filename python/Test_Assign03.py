import pytest
import numpy as np
import open3d as o3d
import os
import Assign03 as A03
from pathlib import Path

INPUT_DATA_DIR = os.path.join("data", "assign03", "input")
GROUND_DATA_DIR = os.path.join("data", "assign03", "ground")

RTOL = 1e-6
ATOL = 1e-6

all_input_filenames = [    
    "orig_sphere.pcd",
    "noise_outlier_sphere.pcd",
    "noise_pervasive_sphere.pcd",
    "orig_large_bunny.pcd",
    "noise_outlier_large_bunny.pcd",
    "noise_pervasive_large_bunny.pcd"  
]

all_test_point_center_indices = [
    0, 128, 1024
]

def load_cloud_and_extract_neighborhood(filename, center_index, radius):
    input_cloud = o3d.io.read_point_cloud(os.path.join(INPUT_DATA_DIR, filename))
    tree = o3d.geometry.KDTreeFlann(input_cloud)
    all_points = np.array(input_cloud.points) 
    query = all_points[center_index]   
    [k, idx, _] = tree.search_radius_vector_3d(query, radius)
    neighbors = all_points[idx]
    return query, neighbors

###############################################################################
# test_compute_distances
###############################################################################

cd_radius = [ 0.5, 1.0, 1.5 ]

def get_compute_distance_ground_filename(filename, center_index, radius):
    basename = Path(filename).stem
    return f"cd_{center_index:d}_{radius:.3f}_{basename}.npy"
                   

@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_compute_distances(filename, center_index, radius):
    # Get neighbors
    query, neighbors = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Compute
    dists = A03.compute_distances(query, neighbors)
    
    # Load ground truth
    ground_dist = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_distance_ground_filename(
                                filename, center_index, radius)))
    # Check
    np.testing.assert_allclose(dists, ground_dist, err_msg="Distances not equal.", rtol=RTOL, atol=ATOL)
    
    
###############################################################################
# test_compute_gaussian_weights
###############################################################################

cgw_sigma = [ 0.333, 0.5, 1.0 ]

def get_compute_gaussian_weights_ground_filename(filename, center_index, radius, sigma):
    basename = Path(filename).stem
    return f"cgw_{center_index:d}_{radius:.3f}_{sigma:.3f}_{basename}.npy"
                   
@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_compute_gaussian_weights(filename, center_index, radius, sigma):
    # Get neighbors
    query, neighbors = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Compute
    weights = A03.compute_gaussian_weights(query, neighbors, sigma)
    
    # Load ground truth
    ground_weights = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_gaussian_weights_ground_filename(
                                filename, center_index, radius, sigma)))
    # Check
    np.testing.assert_allclose(weights, ground_weights, err_msg="Weights not equal.", rtol=RTOL, atol=ATOL)
    

###############################################################################
# test_compute_weighted_PCA
###############################################################################

def get_compute_weighted_PCA_ground_filename(filename, center_index, radius, sigma):
    basename = Path(filename).stem
    return f"wpca_{center_index:d}_{radius:.3f}_{sigma:.3f}_{basename}.npz"
                   
@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_compute_weighted_PCA(filename, center_index, radius, sigma):
    # Get neighbors
    query, neighbors = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Compute weights
    weights = A03.compute_gaussian_weights(query, neighbors, sigma)
    
    # Compute PCA
    weighted_centroid, U, V, W = A03.compute_weighted_PCA(neighbors, weights)
    
    # Load ground truth
    ground_data = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_weighted_PCA_ground_filename(
                                filename, center_index, radius, sigma)))
    # Check
    np.testing.assert_allclose(weighted_centroid, ground_data["weighted_centroid"], err_msg="weighted_centroid not equal.", rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(U, ground_data["U"], err_msg="U not equal.", rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(V, ground_data["V"], err_msg="V not equal.", rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(W, ground_data["W"], err_msg="W not equal.", rtol=RTOL, atol=ATOL)
   
###############################################################################
# test_project_points_to_plane
###############################################################################

def get_project_points_to_plane_ground_filename(filename, center_index, radius, sigma):
    basename = Path(filename).stem
    return f"pptp_{center_index:d}_{radius:.3f}_{sigma:.3f}_{basename}.npy"
                   
@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_project_points_to_plane(filename, center_index, radius, sigma):
    # Get neighbors
    query, neighbors = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Compute weights
    weights = A03.compute_gaussian_weights(query, neighbors, sigma)
    
    # Compute PCA
    weighted_centroid, U, V, W = A03.compute_weighted_PCA(neighbors, weights)
    
    # Project points
    projected = A03.project_points_to_plane(neighbors, weighted_centroid, U, V, W)
    
    # Load ground truth
    ground_project = np.load(os.path.join(GROUND_DATA_DIR,
                            get_project_points_to_plane_ground_filename(
                                filename, center_index, radius, sigma)))
    # Check
    np.testing.assert_allclose(projected, ground_project, err_msg="projection to plane not equal.", rtol=RTOL, atol=ATOL)
    
###############################################################################
# test_reverse_plane_projection
###############################################################################

@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_reverse_plane_projection(filename, center_index, radius, sigma):
    # Get neighbors
    query, neighbors = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Compute weights
    weights = A03.compute_gaussian_weights(query, neighbors, sigma)
    
    # Compute PCA
    weighted_centroid, U, V, W = A03.compute_weighted_PCA(neighbors, weights)
    
    # Project points
    projected = A03.project_points_to_plane(neighbors, weighted_centroid, U, V, W)
    
    # Reverse it
    reverse_project = A03.reverse_plane_projection(projected, weighted_centroid, U, V, W)
    
    # Check
    np.testing.assert_allclose(reverse_project, neighbors, err_msg="reverse projection to plane not equal.", rtol=RTOL, atol=ATOL)
    
###############################################################################
# test_make_design_matrix_A
###############################################################################

def test_make_design_matrix_A():
    # Make some fake projected points
    points = np.array([
        [1,2,3],
        [4,5,6],
        [7,8,9],        
        [-3,-2,-1]
    ])
    
    ground = np.array([
        [1, 1, 2, 1, 2, 4],
        [1, 4, 5, 16, 20, 25],
        [1, 7, 8, 49, 56, 64],
        [1, -3, -2, 9, 6, 4]
    ])
    
    A = A03.make_design_matrix_A(points)
    
    np.testing.assert_allclose(A, ground, err_msg="A not equal.", rtol=RTOL, atol=ATOL)
    
###############################################################################
# test_make_vector_b
###############################################################################

def test_make_vector_b():
    # Make some fake projected points
    points = np.array([
        [1,2,3],
        [4,5,6],
        [7,8,9],
        [10,11,12],
        [-3,-2,-1]
    ])
    
    ground = np.array([[3],[6],[9],[12],[-1]])
    
    b = A03.make_vector_b(points)
    
    np.testing.assert_allclose(b, ground, err_msg="b not equal.", rtol=RTOL, atol=ATOL)
    
###############################################################################
# test_make_weight_matrix_G
###############################################################################

@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_make_weight_matrix_G(filename, center_index, radius, sigma):
    # Get neighbors
    query, neighbors = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Compute
    weights = A03.compute_gaussian_weights(query, neighbors, sigma)
    
    # Get weight matrix
    weight_matrix = A03.make_weight_matrix_G(weights)
    
    # Load ground truth
    ground_weights = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_gaussian_weights_ground_filename(
                                filename, center_index, radius, sigma)))
    ground_mat = np.diag(ground_weights)
    
    # Check
    np.testing.assert_allclose(weight_matrix, ground_mat, err_msg="Weight matrix not equal.", rtol=RTOL, atol=ATOL)
   
###############################################################################
# test_compute_polynomial_coefficients
###############################################################################

def get_compute_polynomial_coefficients_ground_filename(filename, center_index, radius, sigma):
    basename = Path(filename).stem
    return f"cpc_{center_index:d}_{radius:.3f}_{sigma:.3f}_{basename}.npy"
                   
@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_compute_polynomial_coefficients(filename, center_index, radius, sigma):
    # Get neighbors
    query, neighbors = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Load ground truth weights
    weights = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_gaussian_weights_ground_filename(
                                filename, center_index, radius, sigma)))
    
    # Load ground truth projected points
    projected = np.load(os.path.join(GROUND_DATA_DIR,
                            get_project_points_to_plane_ground_filename(
                                filename, center_index, radius, sigma)))
    
    # Calculate coefficients
    coeff = A03.compute_polynomial_coefficients(projected, weights)
    
    # Load ground truth for coefficients
    ground_coef = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_polynomial_coefficients_ground_filename(
                                filename, center_index, radius, sigma)))
    # Check
    np.testing.assert_allclose(coeff, ground_coef, err_msg="polynomial coefficients not equal.", rtol=RTOL, atol=ATOL)
    
###############################################################################
# test_project_points_to_polynomial
###############################################################################

def get_project_points_to_polynomial_ground_filename(filename, center_index, radius, sigma):
    basename = Path(filename).stem
    return f"ppp_{center_index:d}_{radius:.3f}_{sigma:.3f}_{basename}.npy"
                   
@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_project_points_to_polynomial(filename, center_index, radius, sigma):
    # Get neighbors
    query, points = load_cloud_and_extract_neighborhood(filename, center_index, radius)
    
    # Load ground truth
    pca_data = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_weighted_PCA_ground_filename(
                                filename, center_index, radius, sigma)))
    centroid = pca_data["weighted_centroid"]
    U = pca_data["U"]
    V = pca_data["V"]
    W = pca_data["W"]
    
    # Load ground truth for coefficients
    coeff = np.load(os.path.join(GROUND_DATA_DIR,
                            get_compute_polynomial_coefficients_ground_filename(
                                filename, center_index, radius, sigma)))
    
    # Project
    projected = A03.project_points_to_polynomial(points, centroid, U, V, W, coeff)
    
    # Load ground truth for comparison
    ground_proj = np.load(os.path.join(GROUND_DATA_DIR,
                            get_project_points_to_polynomial_ground_filename(
                                filename, center_index, radius, sigma)))

    # Check
    np.testing.assert_allclose(projected, ground_proj, err_msg="points projected to polynomial not equal.", rtol=RTOL, atol=ATOL)
    
###############################################################################
# test_fit_to_polynomial
###############################################################################

def get_fit_to_polynomial_ground_filename(filename, center_index, radius, sigma):
    basename = Path(filename).stem
    return f"ftp_{center_index:d}_{radius:.3f}_{sigma:.3f}_{basename}.npz"
                   
@pytest.mark.parametrize("sigma", cgw_sigma)
@pytest.mark.parametrize("radius", cd_radius)
@pytest.mark.parametrize("center_index", all_test_point_center_indices)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_fit_to_polynomial(filename, center_index, radius, sigma):
    # Get neighbors
    query, points = load_cloud_and_extract_neighborhood(filename, center_index, radius)
        
    # Fit
    fit_center, fit_W = A03.fit_to_polynomial(query, points, sigma)
    
    # Load ground truth for comparison
    ground_fit = np.load(os.path.join(GROUND_DATA_DIR,
                            get_fit_to_polynomial_ground_filename(
                                filename, center_index, radius, sigma)))
    ground_center = ground_fit["updated_center"]
    ground_W = ground_fit["W"]

    # Check
    np.testing.assert_allclose(fit_center, ground_center, err_msg="polynomial fit center not equal.", rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(fit_W, ground_W, err_msg="polynomial fit W not equal.", rtol=RTOL, atol=ATOL)
    
###############################################################################
# test_perform_moving_least_squares
###############################################################################
   
mls_other_params = [
    (1.0, 0.333),
    (1.0, 1.0),
    (1.5, 0.5),
]

def get_mls_ground_filename(basename, radius, sigma):
    return f"mls_{radius:.3f}_{sigma:.3f}_{basename}"

@pytest.mark.parametrize("radius, sigma", mls_other_params)
@pytest.mark.parametrize("filename", all_input_filenames)
def test_perform_moving_least_squares(filename, radius, sigma):
    # Get ground filename
    ground_filename = get_mls_ground_filename(filename, radius, sigma)
    # Load both clouds
    input_cloud = o3d.io.read_point_cloud(os.path.join(INPUT_DATA_DIR, filename))
    ground_cloud = o3d.io.read_point_cloud(os.path.join(GROUND_DATA_DIR, ground_filename))
    # Compute MLS
    out_cloud = A03.perform_moving_least_squares(input_cloud, radius, sigma)
    # Check if equal
    np.testing.assert_allclose(out_cloud.points, ground_cloud.points, err_msg="Points not equal.", rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(out_cloud.normals, ground_cloud.normals, err_msg="Normals not equal.", rtol=RTOL, atol=ATOL)
     