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
    if margin is None:
        margin = np.array([0, 0, 0], dtype=np.float64)
    else:
        margin = np.asarray(margin, dtype=np.float64)
    minP = np.min(points, axis=0) - margin
    maxP = np.max(points, axis=0) + margin
    return minP, maxP

def split_box_octree(minP, maxP):
    children = []
    midP = (minP + maxP) / 2
    
    for z in range(2):
        for y in range(2):
            for x in range(2):
                child_minP = np.array([
                    minP[0] if x == 0 else midP[0],
                    minP[1] if y == 0 else midP[1],
                    minP[2] if z == 0 else midP[2]
                ])
                child_maxP = np.array([
                    midP[0] if x == 0 else maxP[0],
                    midP[1] if y == 0 else maxP[1],
                    midP[2] if z == 0 else maxP[2]
                ])
                children.append((child_minP, child_maxP))
    return children

def get_distance_from_box(minP, maxP, point):
    distance = 0
    for i in range(3):
        if point[i] < minP[i]:
            distance += (minP[i] - point[i]) ** 2
        elif point[i] > maxP[i]:
            distance += (point[i] - maxP[i]) ** 2
    
    distance = np.sqrt(distance)
    return distance

def get_indices_inside(points, indices, minP, maxP):
    indices_inside = []
    indices_outside = []
    
    for i in indices:
        if (minP[0] <= points[i][0] <= maxP[0] and 
            minP[1] <= points[i][1] <= maxP[1] and 
            minP[2] <= points[i][2] <= maxP[2]):
            indices_inside.append(i)
        else:
            indices_outside.append(i)        
    return indices_inside, indices_outside

def build_octree(cloud, margin, max_depth):
    points = np.asarray(cloud.points)
    root = OctoNode()
    root.minP, root.maxP = get_best_AABB(cloud, margin)
    
    if max_depth == 0:
        root.indices = list(range(len(points)))
        return root
    
    current_indices = list(range(len(points)))
    
    def create_tree(node, indices, depth):
        if depth >= max_depth or len(indices) <= 1:
            node.indices = indices
            return
        
        node.children = []
        box_list = split_box_octree(node.minP, node.maxP)
        
        has_children = False
        
        for (cminP, cmaxP) in box_list:
            child_node = OctoNode()
            child_node.minP = cminP
            child_node.maxP = cmaxP
            
            inside_indices, outside_indices = get_indices_inside(points, indices, cminP, cmaxP)
            
            node.children.append(child_node)
            
            if inside_indices:
                has_children = True
                create_tree(child_node, inside_indices, depth + 1)
            else:
                child_node.indices = []            
            indices = outside_indices
        
        if not has_children:
            node.children = None
            node.indices = indices
        else:
            node.indices = None
    
    create_tree(root, current_indices, 0)
    return root

def do_radius_search_octree(cloud, tree, query_point, radius):
    points = np.asarray(cloud.points)
    radius_squared = radius * radius
    results = []
    queue = deque([tree])
    
    while queue:
        node = queue.popleft()
        if node.indices is not None:
            for i in node.indices:
                dist = np.sum((points[i] - query_point)**2)
                if dist <= radius_squared:
                    results.append((i, dist))
        else:
            if node.children is not None:
                for child in node.children:
                    dist_to_box = get_distance_from_box(child.minP, child.maxP, query_point)
                    if dist_to_box <= radius_squared:
                        queue.append(child)
    
    #results.sort(key=lambda x: x[1])
    return [idx for idx, _ in results]

def main():
    filename = "data/assign02/bunny.pcd"
    cloud = o3d.io.read_point_cloud(filename)
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)
    
    tree = build_octree(cloud, margin=(0.1,0.1,0.1), max_depth=4)
    
    query_point = np.array([6.79621601, 5.66879749, 8.07549858])
    radius = 1.7
    search_indices = do_radius_search_octree(cloud, tree, query_point, radius)
    
    for i in range(len(colors)):
        colors[i] = (0,0,0)
    for idx in search_indices:
        colors[idx] = (255,0,0)
    
    o3d.visualization.draw_geometries([cloud])

if __name__ == "__main__":
    main()
    