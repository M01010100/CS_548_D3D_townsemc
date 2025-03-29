import open3d as o3d
import numpy as np
from enum import Enum
import copy

class CLEANUP_TYPE(Enum):
     NOTHING = (0,0)
     SOR = (1, 0.1)
     ROR = (1, 0.001)

def add_overall_noise(cloud, offset):
    rng = np.random.default_rng(seed=42)
    points = np.asarray(cloud.points)
    
    for i in range(len(points)):
        rp = rng.uniform(low=-offset, 
                         high=offset, 
                         size=3)
        points[i] += rp
        
def add_outlier_noise(cloud, offset, prob):
    rng = np.random.default_rng(seed=42)
    points = np.asarray(cloud.points)
    
    for i in range(len(points)):
        dice = rng.random() # [0,1)
        
        if dice < prob:       
            rp = rng.uniform(low=-offset, 
                            high=offset, 
                            size=3)
            points[i] += rp
     
def add_noise(cloud, use_outlier_noise, 
              noise_scale, noise_prob):
    if use_outlier_noise:
        add_outlier_noise(cloud, noise_scale, noise_prob)
    else:
        add_overall_noise(cloud, noise_scale)
    

def main():
    orig_cloud = o3d.io.read_point_cloud(
        "data/assign01/BunnyXYZ.pcd")
    
    use_outlier_noise = True
    noise_scale = 0.005
    noise_prob = 0.1
    clean_type = CLEANUP_TYPE.NOTHING
    
    noise_cloud = copy.deepcopy(orig_cloud)
    add_noise(noise_cloud, use_outlier_noise,
              noise_scale, noise_prob)
    noise_cloud = noise_cloud.translate((0.2, 0, 0))
    
    clean_cloud = copy.deepcopy(noise_cloud)    
    clean_cloud = clean_cloud.translate((0.2, 0, 0))
    
    def update_clouds(vis):
        view_status = vis.get_view_status()
        
        noise_cloud = copy.deepcopy(orig_cloud)
        add_noise(noise_cloud, use_outlier_noise,
                    noise_scale, noise_prob)
        noise_cloud = noise_cloud.translate((0.2, 0, 0))
    
        clean_cloud = copy.deepcopy(noise_cloud)
        if clean_type == CLEANUP_TYPE.SOR:        
            clean_cloud, _ = clean_cloud.remove_statistical_outlier(20, 1.0)
        elif clean_type == CLEANUP_TYPE.ROR:
            clean_cloud, _ = clean_cloud.remove_radius_outlier(40, 0.005)
                    
        clean_cloud = clean_cloud.translate((0.2, 0, 0))
    
        vis.clear_geometries()
        vis.add_geometry(orig_cloud)
        vis.add_geometry(noise_cloud)
        vis.add_geometry(clean_cloud)
        vis.set_view_status(view_status)
        
        return False
    
    def toggle_noise_type(vis):
        nonlocal use_outlier_noise
        use_outlier_noise = not use_outlier_noise
        return update_clouds(vis)
    
    def change_clean_type(vis):
        nonlocal clean_type
        clean_list = list(CLEANUP_TYPE)
        current_index = clean_list.index(clean_type)
        current_index = (current_index+1)%len(clean_list)
        clean_type = clean_list[current_index]
        return update_clouds(vis)
        
    key_callbacks = {}
    key_callbacks[ord("Z")] = toggle_noise_type
    key_callbacks[ord("E")] = change_clean_type
    
    o3d.visualization.draw_geometries_with_key_callbacks(
        [orig_cloud, noise_cloud, clean_cloud],
        key_callbacks
    )

if __name__ == "__main__":
    main()
    