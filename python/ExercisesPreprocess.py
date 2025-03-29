import numpy as np
import open3d as o3d
import copy

def main():
    orig_model = o3d.io.read_point_cloud(
        "data/assign01/BunnyXYZ.pcd")
    down_model = copy.deepcopy(orig_model)
    down_model = down_model.translate((0.2, 0, 0))
    
    '''
    # RANDOM
    sample_size = 1.0
    sample_inc = 0.01
    min_sample_size = 0.001
    max_sample_size = 1.0
    '''
    '''
    # UNIFORM
    sample_size = 1
    sample_inc = 10
    min_sample_size = 1
    max_sample_size = 40000
    '''
    
    # Voxel
    sample_size = 0.00001
    sample_inc = 0.001
    min_sample_size = 1e-6
    max_sample_size = 1
    
    '''
    # FPS
    sample_size = 100
    sample_inc = 100
    min_sample_size = 100
    max_sample_size = len(orig_model.points)
    '''
        
    def update_clouds(vis):
        view_status = vis.get_view_status()
        #down_model = orig_model.random_down_sample(sample_size)
        #down_model = orig_model.uniform_down_sample(sample_size)
        down_model = orig_model.voxel_down_sample(sample_size)
        #down_model = orig_model.farthest_point_down_sample(sample_size)
        
        down_model = down_model.translate((0.2, 0, 0))
        
        down_model.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(down_model.points), 3)))

        down_model = down_model.translate((0.2, 0, 0))
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(down_model, sample_size)
        down_model = down_model.translate((-0.2, 0, 0))
        
        vis.clear_geometries()
        vis.add_geometry(orig_model)
        vis.add_geometry(down_model)
        vis.add_geometry(voxel_grid)
        vis.set_view_status(view_status)
        
        print(f"Sample size:{sample_size:.3f}, Points:{len(down_model.points)}")
        return False
    
    def increase_sample(vis):
        nonlocal sample_size
        sample_size += sample_inc
        sample_size = min(sample_size, max_sample_size)
        update_clouds(vis)
    
    def decrease_sample(vis):
        nonlocal sample_size
        sample_size -= sample_inc
        sample_size = max(sample_size, min_sample_size)
        update_clouds(vis)
    
    key_callbacks = {}
    key_callbacks[ord("J")] = decrease_sample
    key_callbacks[ord("K")] = increase_sample
    
    o3d.visualization.draw_geometries_with_key_callbacks(
        [orig_model, down_model], key_callbacks
    )

if __name__ == "__main__":
    main()
    