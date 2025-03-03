import open3d as o3d 
import numpy as np 
from collections import deque 
 
class OctoNode: 
    indices = None 
    children = None 
    minP = None 
    maxP = None 
     
def traverse_octree(node): 
    if node.indices is not None: 
        return node.indices 
    else: 
        index_list = [] 
        if node.children is not None:
            for child in node.children:
                index_list.append(list(traverse_octree(child))) 
        return index_list
    
def get_best_AABB(cloud, margin=(0,0,0)): 
    points = np.asarray(cloud.points)
    minP = np.min(points, axis=0) - np.asarray(margin)
    maxP = np.max(points, axis=0) + np.asarray(margin)
    return minP, maxP

def split_box_octree(minP, maxP):
    children = []
    for x in range(2):
        for y in range(2):
            for z in range(2):
                child = OctoNode()
                midP = (minP + maxP) / 2
                child.minP = np.array([
                    minP[0] if x == 0 else midP[0],
                    minP[1] if y == 0 else midP[1],
                    minP[2] if z == 0 else midP[2]])
                child.maxP = np.array([
                    midP[0] if x == 0 else maxP[0],
                    midP[1] if y == 0 else maxP[1],
                    midP[2] if z == 0 else maxP[2]])
                children.append(child)
    return children

def get_distance_from_box(minP, maxP, point):
    distance = 0
    for i in range(3):
        if point[i] < minP[i]:
            distance += (minP[i] - point[i]) ** 2
        elif point[i] > maxP[i]:
            distance += (point[i] - maxP[i]) ** 2
    return distance

def get_indices_inside(points, indices, minP, maxP):
    indices_inside = []
    for i in indices:
        if (minP[0] <= points[i][0] <= maxP[0] and 
            minP[1] <= points[i][1] <= maxP[1] and 
            minP[2] <= points[i][2] <= maxP[2]):
            indices_inside.append(i)
    return indices_inside

def build_octree(cloud, margin, max_depth):
    points = np.asarray(cloud.points)    
    root = OctoNode()
    root.minP, root.maxP = get_best_AABB(cloud, margin)
    queue = deque([(root, list(range(len(points))), 0)])
    while queue:
        current_node, current_indices, current_depth = queue.popleft()
        if current_depth >= max_depth:
            current_node.indices = current_indices
            continue
        current_node.children = split_box_octree(current_node.minP, current_node.maxP)
        remaining_indices = current_indices
        has_children = False
        for child in current_node.children:
            indices_inside = get_indices_inside(points, remaining_indices, child.minP, child.maxP)
            remaining_indices = [idx for idx in remaining_indices if idx not in indices_inside]
            if indices_inside:
                queue.append((child, indices_inside, current_depth + 1))
                has_children = True
        if not has_children:
            current_node.children = None
    return root

def do_radius_search_octree(cloud, tree, query_point, radius):
    points = np.asarray(cloud.points)
    search_indices = []
    radius_squared = radius * radius
    queue = deque([tree])
    
    while queue:
        current_node = queue.popleft()
        
        if current_node.indices is not None:
            for idx in current_node.indices:
                dist = np.sum((points[idx] - query_point) ** 2)
                if dist <= radius_squared:
                    search_indices.append(idx)
        
        elif current_node.children is not None:
            for child in current_node.children:
                dist_to_box = get_distance_from_box(child.minP, child.maxP, query_point)
                if dist_to_box <= radius_squared:
                    queue.append(child)
    
    return search_indices

def main(): 
    filename = "data/assign02/bunny.pcd" 
     
    cloud = o3d.io.read_point_cloud(filename)  
     
    points = np.asarray(cloud.points) 
    colors = np.asarray(cloud.colors) 
         
    tree = build_octree(cloud, margin=(0.1,0.1,0.1), max_depth=4)  
     
    query_point = np.array((6.79621601, 5.66879749, 8.07549858))     
    radius = 1.7 
    search_indices = do_radius_search_octree(cloud, tree, np.array(query_point), 
radius) 
     
    for i in range(len(colors)): 
        colors[i] = (0,0,0) 
     
    for index in search_indices: 
        colors[index] = (255,0,0) 
     
    # Visualize models 
    o3d.visualization.draw_geometries([cloud])       
     
if __name__ == "__main__": 
    main()