import open3d as o3d

def main():
    print(o3d.__version__)

    mesh = o3d.geometry.TriangleMesh.create_sphere()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])
    
if __name__ == "__main__":
    main()
    