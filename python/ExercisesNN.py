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

class HeapNode:
    def __init__(self, index=-1, distance=0.0):
        self.index = index
        self.distance = distance
        
    def __str__(self):
        return "(" + str(self.distance) + ")"
    
class MaxHeap:
    def __init__(self, max_cnt):
        self.max_cnt = max_cnt
        self.heap = []
        for _ in range(max_cnt):
            self.heap.append(HeapNode())
        self.cnt = 0
    
    def get_point_indices(self):
        indices = []
        for i in range(len(self.heap)):
            if self.heap[i].index != -1:
                indices.append(self.heap[i].index)
        return indices
    
    def insert(self, newnode):
        if self.cnt < self.max_cnt:
            self.heap[self.cnt] = newnode
            self.cnt += 1
            self.do_sift_up(self.cnt-1)
        else:
            if newnode.distance < self.heap[0].distance:
                self.heap[0] = newnode
                self.do_heapify(0)
                
    def do_sift_up(self, index):
        while index > 0:
            parent = (index-1)//2
            if self.heap[parent].distance < self.heap[index].distance:
                tmp = self.heap[parent]
                self.heap[parent] = self.heap[index]
                self.heap[index] = tmp
                index = parent
            else:
                break   
            
    def do_heapify(self, index):
        largest = index
        left = index*2 + 1
        right = index*2 + 2
        
        if (left < self.max_cnt and
            self.heap[left].distance > self.heap[largest].distance):
            largest = left
        if (right < self.max_cnt and
            self.heap[right].distance > self.heap[largest].distance):
            largest = right
        
        if largest != index:
            tmp = self.heap[index]
            self.heap[index] = self.heap[largest]
            self.heap[largest] = tmp
            self.do_heapify(largest)
        
    def __str__(self):
        s = ""
        for i in range(len(self.heap)):
            s += str(self.heap[i]) + " "
        return s                    
    
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
    
    heap = MaxHeap(5)
    heap.insert(HeapNode(0, 3))
    heap.insert(HeapNode(1, 5))
    heap.insert(HeapNode(2, 13))
    heap.insert(HeapNode(3, 20))
    heap.insert(HeapNode(4, 2))
    heap.insert(HeapNode(5, 18))
    heap.insert(HeapNode(6, 27))
    heap.insert(HeapNode(5, 1))
    
    print(heap)

    
    
    
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
    