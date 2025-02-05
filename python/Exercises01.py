import open3d as o3d
import numpy as np
import copy

def main():
    
    filename = "./data/samples/bunny.pcd"
    pcd = o3d.io.read_point_cloud(filename)
    
    points = np.asarray(pcd.points)
    colors = np.zeros(points.shape, dtype=points.dtype)
    colors[:,0] = 1.0
    colors[:,1] = 0.0
    colors[:,2] = 0.0
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    #o3d.visualization.draw(pcd)
    
    tpcd = o3d.t.io.read_point_cloud(filename)
    points = tpcd.point["positions"].numpy()
    colors = np.zeros(points.shape, dtype=points.dtype)
    colors[:,0] = 0.0
    colors[:,1] = 0.0
    colors[:,2] = 1.0
    tpcd.point["colors"] = o3d.core.Tensor.from_numpy(colors)
    
    #o3d.visualization.draw(tpcd)
    
    dataset = o3d.data.PCDPointCloud()
    rpcd = o3d.io.read_point_cloud(dataset.path)
    
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    
    #o3d.visualization.draw([rpcd, mesh])
    
    dataset = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(dataset.path)
    mesh.compute_vertex_normals()
    
    #o3d.visualization.draw_geometries([mesh])
    
    dataset = o3d.data.LivingRoomPointClouds()
    pcd_list = []
    offset = 0
    for path in dataset.paths:
        cloud = o3d.io.read_point_cloud(path)
        cloud.translate((offset,0,0))
        pcd_list.append(cloud)
        offset += 5.0
        
    o3d.visualization.draw(pcd_list)
      

if __name__ == "__main__":
    main()
    