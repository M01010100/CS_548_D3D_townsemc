import open3d as o3d
import numpy as np
import sys

def load_pcd(filename):
    # Dictionary to store field indices
    field_indices = {}
    points_count = 0
    
    # Read header
    with open(filename, 'r') as f:
        while True:
            line = f.readline().strip()
            
            # Skip comments and blank lines
            if not line or line.startswith('#'):
                continue
                
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
                for i, field in enumerate(fields):
                    field_indices[field] = i
                    
            elif line.startswith('POINTS'):
                points_count = int(line.split()[1])
                
            elif line.startswith('DATA'):
                if line.split()[1] != 'ascii':
                    raise ValueError("Only ascii data format is supported")
                break
        
        # Read point data
        points = np.zeros((points_count, 3), dtype=np.float32)
        normals = np.zeros((points_count, 3), dtype=np.float32) if any(f.startswith('normal') for f in field_indices) else None
        colors = np.zeros((points_count, 3), dtype=np.float32) if 'rgb' in field_indices else None
        
        for i in range(points_count):
            values = [float(x) for x in f.readline().split()]
            
            # Process coordinates
            points[i] = [values[field_indices[f]] for f in ['x', 'y', 'z']]
            
            # Process normals if present
            if normals is not None:
                normals[i] = [values[field_indices[f]] for f in ['normal_x', 'normal_y', 'normal_z']]
            
            # Process colors if present
            if colors is not None:
                rgb_f = np.array([values[field_indices['rgb']]], dtype=np.float32)
                rgb_i = rgb_f.view(np.uint32)
                colors[i] = [
                    ((rgb_i >> 16) & 255) / 255.0,  # Red
                    ((rgb_i >> 8) & 255) / 255.0,   # Green
                    (rgb_i & 255) / 255.0           # Blue
                ]
    
    # Create Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if normals is not None:
        pcd.normals = o3d.utility.Vector3dVector(normals)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd

def main():
    if len(sys.argv) < 2:
        print("Error: Please provide input PCD file path")
        exit(1)
        
    input_file = sys.argv[1]
    pcd = load_pcd(input_file)
    o3d.visualization.draw_geometries([pcd], point_show_normal=pcd.has_normals())

if __name__ == "__main__":
    main()