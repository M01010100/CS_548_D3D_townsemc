import numpy as np

points = np.array([[0, 7], [6, 4], [4, 8], [0, 6], [2, 0]])

mean = np.mean(points, axis=0)
centered_points = points - mean

covariance_matrix = np.cov(centered_points.T)

eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

largest_eigenvalue_index = np.argmax(eigenvalues)
largest_eigenvector = eigenvectors[:, largest_eigenvalue_index]

rounded_eigenvector = np.round(largest_eigenvector, decimals=2)

print(rounded_eigenvector)


import heapq

def build_max_heap(arr):
    max_heap = [-x for x in arr]  
    heapq.heapify(max_heap)
    return max_heap

def get_max_heap_array(max_heap):
    return [-x for x in max_heap]

arr = [5, 7, 3, 1, 8, 4]
max_heap = build_max_heap(arr)
max_heap_array = get_max_heap_array(max_heap)

print(max_heap_array)