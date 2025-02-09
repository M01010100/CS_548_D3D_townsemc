import open3d as o3d
import numpy as np

def load_pcd(filename):
	# Load the point cloud using Open3D
	pcd = o3d.io.read_point_cloud(filename)
	
	# Return the point cloud object
	return pcd
