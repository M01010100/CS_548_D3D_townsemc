import open3d as o3d
import numpy as np
import sys

def load_pcd(filename):

    field_map = {}
    points_count = 0
    
    with open(filename, 'r') as f:
        while True:
            line = f.readline().strip()
            
            if line.startswith('#') or not line:
                continue
                
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
                for i, field in enumerate(fields):
                    field_map[field] = i
                    
            elif line.startswith('POINTS'):
                points_count = int(line.split()[1])
                
            elif line.startswith('DATA'):
                if line.split()[1] != 'ascii':
                    raise ValueError("Only ASCII data format is supported")
                break
        
        points = np.zeros((points_count, 3), dtype=np.float32)
        has_normals = all(f in field_map for f in ['normal_x', 'normal_y', 'normal_z'])
        has_colors = 'rgb' in field_map
        
        if has_normals:
            normals = np.zeros((points_count, 3), dtype=np.float32)
        if has_colors:
            colors = np.zeros((points_count, 3), dtype=np.float32)
        
        for i in range(points_count):
            values = [float(x) for x in f.readline().strip().split()]
            
            # coordinates
            points[i, 0] = values[field_map['x']]
            points[i, 1] = values[field_map['y']]
            points[i, 2] = values[field_map['z']]
             
            if has_normals:
                normals[i, 0] = values[field_map['normal_x']]
                normals[i, 1] = values[field_map['normal_y']]
                normals[i, 2] = values[field_map['normal_z']]
                 
            if has_colors:
                rgb_f = np.array([values[field_map['rgb']]], dtype=np.float32)
                rgb_i = rgb_f.view(np.uint32)[0]
                
                # Extract RGB and normalize 
                colors[i, 0] = float((rgb_i >> 16) & 255) / 255.0  # R
                colors[i, 1] = float((rgb_i >> 8) & 255) / 255.0   # G
                colors[i, 2] = float(rgb_i & 255) / 255.0          # B
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    if has_normals:
        pcd.normals = o3d.utility.Vector3dVector(normals)
    if has_colors:
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
    return pcd

def main():
    if len(sys.argv) < 2:
        print("Error: Please provide a PCD file path")
        sys.exit(1)
        
    input_file = sys.argv[1]
    try:
        pcd = load_pcd(input_file)
        o3d.visualization.draw_geometries([pcd], point_show_normal=pcd.has_normals())
    except Exception as e:
        print(f"Error loading PCD file: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
    