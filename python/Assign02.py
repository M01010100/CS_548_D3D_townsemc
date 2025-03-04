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
    
    for x in range(2):
        for y in range(2):
            for z in range(2):
                child = OctoNode()
                
                child_minP = np.array([
                    minP[0] if x == 0 else midP[0],
                    minP[1] if y == 0 else midP[1],
                    minP[2] if z == 0 else midP[2]])
                
                child_maxP = np.array([
                    midP[0] if x == 0 else maxP[0],
                    midP[1] if y == 0 else maxP[1],
                    midP[2] if z == 0 else maxP[2]])
                
                child.minP = child_minP
                child.maxP = child_maxP
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
    
    # Process the root node first
    current_indices = list(range(len(points)))
    
    # Create the octree structure
    def create_tree(node, indices, depth):
        if depth >= max_depth or len(indices) <= 1:
            node.indices = indices
            return
        
        # Create children nodes
        node.children = []
        child_boxes = split_box_octree(node.minP, node.maxP)
        
        # Store whether any children were created
        has_children = False
        
        # Assign points to children
        for child_box in child_boxes:
            node.children.append(child_box)  # Add the child node
            
            # Find points inside this child's box
            inside_indices, _ = get_indices_inside(points, indices, child_box.minP, child_box.maxP)
            
            if inside_indices:
                has_children = True
                # Recursively build tree for this child
                create_tree(child_box, inside_indices, depth + 1)
            else:
                # Empty leaf node
                child_box.indices = []
        
        if not has_children:
            # No children had any points, so make this a leaf
            node.children = None
            node.indices = indices
        else:
            # This is an internal node, so clear its indices
            node.indices = None
    
    # Start the recursive tree building
    create_tree(root, current_indices, 0)
    
    return root

def do_radius_search_octree(cloud, tree, query_point, radius):
    points = np.asarray(cloud.points)
    search_results = []  # Store indices and distances
    radius_squared = radius * radius
    queue = deque([tree])
    
    while queue:
        current_node = queue.popleft()
        
        # If this is a leaf node with indices
        if current_node.indices is not None:
            for idx in current_node.indices:
                dist = np.sum((points[idx] - query_point) ** 2)
                if dist <= radius_squared:
                    search_results.append((idx, dist))
        
        # If this is an internal node with children
        elif current_node.children is not None:
            for child in current_node.children:
                dist_to_box = get_distance_from_box(child.minP, child.maxP, query_point)
                if dist_to_box <= radius_squared:
                    queue.append(child)
    
    # Sort by distance from query point
    search_results.sort(key=lambda x: x[1])
    
    # Extract just the indices in the sorted order
    search_indices = [idx for idx, _ in search_results]
    
    return search_indices

def main():
    # Set filename        
    filename = "data/assign02/bunny.pcd"
    
    # Load cloud
    cloud = o3d.io.read_point_cloud(filename) 
    
    # Get points and colors
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)
        
    # Build tree
    tree = build_octree(cloud, margin=(0.1,0.1,0.1), max_depth=4) 
    
    # Do radius search
    query_point = np.array((6.79621601, 5.66879749, 8.07549858))    
    radius = 1.7
    search_indices = do_radius_search_octree(cloud, tree, np.array(query_point), radius)
    
    # Set colors based on indices    
    for i in range(len(colors)):
        colors[i] = (0,0,0)
    
    for index in search_indices:
        colors[index] = (255,0,0)
    
    # Visualize models
    o3d.visualization.draw_geometries([cloud])      
    
if __name__ == "__main__":
    main()