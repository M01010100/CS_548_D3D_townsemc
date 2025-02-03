import open3d as o3d
import numpy as np
import copy

def main():
    
    #pcd = o3d.io.read("./data/samples/bunny.pcd")
    
    #points = np.asarray(pcd.points)
    #colors = np.zeros(points.shape, dtype=points.dtype)
    #colors[:,0] = 0.0
    #colors[:,1] = 0.0
    #colors[:,2] = 1.0
    #pcd.point["colors"] = o3d.utility.Vector3dVector(colors)
    
    #o3d.visualization.draw(pcd)
    
    dataset = o3d.data.PCDPointCloud()
    rcpd = o3d.io.read_point_cloud(dataset.path)
    
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    
if __name__ == "__main__":
    main() 