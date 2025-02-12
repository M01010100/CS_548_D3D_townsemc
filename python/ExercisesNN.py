import open3d as o3d
import numpy as np

def get_distance(P, Q):
    return np.linalg.norm(P - Q)

def get_sorted_distances(points):
    points = np.expand_dims(points, axis=0)
    print(points.shape)
    points_T = np.transpose(points, axes=[1,0,2])
    dist = points_T - points
    dist = np.square(dist)
    dist = np.sum(dist, axis=-1)
    dist = np.sqrt(dist)
    
    indices = np.argsort(dist, axis=1)
    
    return dist, indices

class BruteSearch:
    def __init__(self, points):
        self.points = points
        #self.distances, self.sort_index = get_sorted_distances(points)
        #print("Done with precalc")
                
    def get_nearest_neighbor(self, query_index):
        Q = self.points[query_index]
        best_dist = -1
        best_index = -1
        for i in range(len(self.points)):
            if i != query_index:
                P = self.points[i]
                dist = get_distance(P,Q)
                if dist < best_dist or best_index == -1:
                    best_index = i
                    best_dist = dist
        return best_index

def main():
    pcd = o3d.io.read_point_cloud("data/assign01/BunnyXYZ.pcd")
    
    query_index = 700
    
    points = np.asarray(pcd.points)
    
    search = BruteSearch(points)
    n_index = search.get_nearest_neighbor(query_index)
        
    colors = np.zeros_like(points)
    colors[query_index] = (1,0,0)
    colors[n_index] = (0,0,1)
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.visualization.draw(pcd)

if __name__ == "__main__":
    main()
    