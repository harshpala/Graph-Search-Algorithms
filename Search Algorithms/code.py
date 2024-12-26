import numpy as np
import pickle

# Algorithm: Iterative Deepening Search (IDS)

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 2, 9, 8, 5, 97, 98, 12]

def get_ids_path(adj_matrix, start_node, goal_node):
  n = len(adj_matrix)
  # maxDepth = 20
  if(get_bidirectional_search_path(adj_matrix, start_node, goal_node) == None):
     return None

  for depth_limit in range(n):
    stack = [(start_node, [start_node], 0)]
    # visited = set()

    while stack:
      (node, path, depth) = stack.pop()
      # print(depth)

      if node == goal_node:
        return path
    
      if depth < depth_limit:
        neighbours_list = []
        for neighbour, cost in enumerate(adj_matrix[node]):
          # print(neighbour,path)
          if cost > 0 and neighbour not in path:# and neighbour not in visited:
            neighbours_list.append(neighbour)
            # print(stack)
            # print('\n')
        for neighbour in reversed(neighbours_list):
          stack.append((neighbour, path + [neighbour], depth+1))

      # visited.add(node)
        
  return None



# Algorithm: Bi-Directional Search

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 2, 9, 8, 5, 97, 98, 12]

def get_bidirectional_search_path(adj_matrix, start_node, goal_node):
  # n = len(adj_matrix)

  if start_node == goal_node:
      return [start_node]

  forward_queue = [(start_node, [start_node])]
  backward_queue = [(goal_node, [goal_node])]

  forward_visited = {start_node: [start_node]}
  backward_visited = {goal_node: [goal_node]}

  while forward_queue and backward_queue:
      # searching from starting node
      current_node, path = forward_queue.pop(0)
      for neighbour, cost in (enumerate((adj_matrix[current_node]))):
          if cost > 0 and neighbour not in forward_visited:
                
            new_path = path + [neighbour]
            forward_visited[neighbour] = new_path
            forward_queue.append((neighbour, new_path))
            if neighbour in backward_visited:
                return new_path + backward_visited[neighbour][::-1][1:]

      # searching from the goal node
      current_node, path = backward_queue.pop(0)
      for neighbour, cost in reversed(list(enumerate(adj_matrix[current_node]))):
          if cost > 0 and neighbour not in backward_visited:
                
            new_path = path + [neighbour]
            backward_visited[neighbour] = new_path
            backward_queue.append((neighbour, new_path))
            if neighbour in forward_visited:
                return forward_visited[neighbour] + new_path[::-1][1:]

  return None


'''
To Compare the memory usage and time of execution for both algorithms.
'''

# import time
# import tracemalloc
# import matplotlib.pyplot as plt


# def compare(adj_matrix,node_attributes):
#   n = len(adj_matrix)
#   ids_time = 0
#   bs_time = 0
#   as_time = 0
#   bhs_time = 0

#   ids_memory = 0
#   bs_memory = 0
#   as_memory = 0
#   bhs_memory = 0

#   ids_path = []
#   bds_path = []
#   as_path = []
#   bhs_path = []

#   total_pairs = 0

#   for start_node in range(n):
#      for goal_node in range(n):
#       if start_node == goal_node:
#         continue

      
      
#       # if(get_bidirectional_search_path(adj_matrix, start_node, goal_node) != None):
#       total_pairs+=1
#       tracemalloc.start()
#       start_time = time.perf_counter()
#       path = get_ids_path(adj_matrix, start_node, goal_node)
#       if path is not None:
#           ids_path.append(path)
#       ids_time += time.time() - start_time
#       ids_memory += tracemalloc.get_traced_memory()[1]
#       tracemalloc.stop()

#       tracemalloc.start()
#       start_time = time.time()
#       path = get_bidirectional_search_path(adj_matrix, start_node, goal_node)
#       if path is not None:
#           bds_path.append(path)
#       bs_time += time.time() - start_time
#       bs_memory += tracemalloc.get_traced_memory()[1]
#       tracemalloc.stop()

#       tracemalloc.start()
#       start_time = time.time()
#       path = get_astar_search_path(adj_matrix, node_attributes, start_node, goal_node)
#       if path is not None:
#           as_path.append(path)      
#       as_time += time.time() - start_time
#       as_memory += tracemalloc.get_traced_memory()[1]
#       tracemalloc.stop()

#       tracemalloc.start()
#       start_time = time.time()
#       path = get_bidirectional_heuristic_search_path(adj_matrix, node_attributes, start_node, goal_node)
#       if path is not None:
#           bhs_path.append(path)
#       bhs_time += time.time() - start_time
#       bhs_memory += tracemalloc.get_traced_memory()[1]
#       tracemalloc.stop()


#   total_ids_cost = sum(
#         sum(adj_matrix[path[i], path[i + 1]] for i in range(len(path) - 1))
#         for path in ids_path if path  # Ensure path is not None
#     )

#   total_bds_cost = sum(
#         sum(adj_matrix[path[i], path[i + 1]] for i in range(len(path) - 1))
#         for path in bds_path if path  # Ensure path is not None
#     )
  
#   total_as_cost = sum(
#         sum(adj_matrix[path[i], path[i + 1]] for i in range(len(path) - 1))
#         for path in as_path if path  # Ensure path is not None
#     )

#   total_bhs_cost = sum(
#         sum(adj_matrix[path[i], path[i + 1]] for i in range(len(path) - 1))
#         for path in bhs_path if path  # Ensure path is not None
#     )

#   avg_ids_time = ids_time / total_pairs
#   avg_ids_memory = ids_memory / total_pairs
#   avg_bs_time = bs_time / total_pairs
#   avg_bs_memory = bs_memory / total_pairs
#   avg_as_time = as_time / total_pairs
#   avg_as_memory = as_memory / total_pairs
#   avg_bhs_time = bhs_time / total_pairs
#   avg_bhs_memory = bhs_memory / total_pairs
#   avg_ids_cost = total_ids_cost / total_pairs
#   avg_bds_cost = total_bds_cost / total_pairs
#   avg_as_cost = total_as_cost / total_pairs
#   avg_bhs_cost = total_bhs_cost / total_pairs

#   print(f"Iterative Deepening Search (IDS) - Average Time: {avg_ids_time} seconds, Average Memory: {avg_ids_memory} bytes and Average Path Cost: {avg_ids_cost}")
#   print(f"Bidirectional BFS - Average Time: {avg_bs_time} seconds, Average Memory: {avg_bs_memory} bytes and Average Path Cost: {avg_bds_cost}")
#   print(f"A* Search Algorithm - Average Time: {avg_as_time} seconds, Average Memory: {avg_as_memory} bytes and Average Path Cost: {avg_as_cost}")
#   print(f"Bi-Directional Heuristic Search - Average Time: {avg_bhs_time} seconds, Average Memory: {avg_bhs_memory} bytes and Average Path Cost: {avg_bhs_cost}")


#   algorithms = ['Iterative Deepening Search', 'Bidirectional Search', 'A*', 'Bidirectional Heuristic Search Path']
#   avg_time = [0.0718, 0.0042, 0.0095, 0.0106]  # in seconds
#   avg_memory = [2144.45, 2673.30, 11183.73, 9367.21]  # in bytes
#   avg_cost = [1567.13, 1162.65, 1066.68, 1065.09]  # path cost


#   x_pos = range(len(algorithms))

#   # 1. Average Time vs Algorithms scatter plot
#   plt.figure(figsize=(10, 6))
#   plt.scatter(x_pos, avg_time, color='blue', s=100)
#   plt.xticks(x_pos, algorithms)
#   plt.title("Average Time vs Algorithms")
#   plt.xlabel("Algorithm")
#   plt.ylabel("Average Time (seconds)")
#   plt.grid(True)
#   plt.show()

#   # 2. Average Memory vs Algorithms scatter plot
#   plt.figure(figsize=(10, 6))
#   plt.scatter(x_pos, avg_memory, color='green', s=100)
#   plt.xticks(x_pos, algorithms)
#   plt.title("Average Memory vs Algorithms")
#   plt.xlabel("Algorithm")
#   plt.ylabel("Average Memory (bytes)")
#   plt.grid(True)
#   plt.show()

#   # 3. Average Path Cost vs Algorithms scatter plot
#   plt.figure(figsize=(10, 6))
#   plt.scatter(x_pos, avg_cost, color='red', s=100)
#   plt.xticks(x_pos, algorithms)
#   plt.title("Average Path Cost vs Algorithms")
#   plt.xlabel("Algorithm")
#   plt.ylabel("Average Path Cost")
#   plt.grid(True)
#   plt.show()

'''
Iterative Deepening Search Path- 
                                
                                Average Time: 0.009689935038166661 seconds, 
                                Average Memory: 5252.4548387096775 bytes

Bidirectional Search Path - 
                                Average Time: 0.000611376408607729 seconds, 
                                Average Memory: 2673.3026451612905 bytes

A* Search Algorithm - 
                                Average Time: 0.001388100424120503 seconds, 
                                Average Memory: 6649.003419354839 bytes
                        
Bi-Directional Heuristic Search -
                                Average Time: 0.0034843215019472185 seconds, 
                                Average Memory: 3402.9229677419353 bytes
  
  
'''
# compare(get_ids_path,adj_matrix) # type: ignore

# Algorithm: A* Search Algorithm

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - node_attributes: Dictionary of node attributes containing x, y coordinates for heuristic calculations.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 28, 10, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 27, 9, 8, 5, 97, 28, 10, 12]
def get_astar_search_path(adj_matrix, node_attributes, start_node, goal_node):
  
  x1, y1=node_attributes[start_node]['x'], node_attributes[start_node]['y']
  x2, y2=node_attributes[goal_node]['x'], node_attributes[goal_node]['y']
  f = ((x2 - x1)**2 +(y2-y1)**2)**0.5

  openlist = [(start_node, [start_node], 0, f)]
  closelist = []

  while openlist:
     openlist.sort(key=lambda x: x[3])
     node, path, nodeG, nodeF = openlist.pop(0)

     if node == goal_node:
        return path

     for neighbour, cost in enumerate(adj_matrix[node]):
        if cost > 0:
            xx1, yy1 = node_attributes[neighbour]['x'], node_attributes[neighbour]['y']
            xx2, yy2 = node_attributes[goal_node]['x'], node_attributes[goal_node]['y']
            nextH = ((xx2 - xx1)**2 + (yy2 - yy1)**2)**0.5
            nextG = nodeG + cost
            nextF = nextG + nextH

            InOpenList = any(neighbour == open_node[0] and nextF >= open_node[3] for open_node in openlist)
            InClosedList = any(neighbour == closed_node[0] and nextF >= closed_node[3] for closed_node in closelist)

            if not InOpenList and not InClosedList:
                openlist.append((neighbour, path + [neighbour], nextG, nextF))

     closelist.append((node, path, nodeG, nodeF))

  return None

# Algorithm: Bi-Directional Heuristic Search

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - node_attributes: Dictionary of node attributes containing x, y coordinates for heuristic calculations.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 34, 33, 11, 32, 31, 3, 5, 97, 28, 10, 12]

def get_bidirectional_heuristic_search_path(adj_matrix, node_attributes, start_node, goal_node):

  if start_node == goal_node:
      return [start_node]

  x1, y1 = node_attributes[start_node]['x'], node_attributes[start_node]['y']
  x2, y2 = node_attributes[goal_node]['x'], node_attributes[goal_node]['y']
  startF = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
  goalF = startF

  forwardList = [(start_node, [start_node], 0, startF)]
  backwardList = [(goal_node, [goal_node], 0, goalF)]

  forwardClosed = []
  backwardClosed = []

  forwardPaths = {start_node: [start_node]}
  backwardPaths = {goal_node: [goal_node]}

  while forwardList and backwardList:
      forwardList.sort(key=lambda x: x[3])
      backwardList.sort(key=lambda x: x[3])

      current_forward, path_forward, g_forward, f_forward = forwardList.pop(0)
      forwardClosed.append(current_forward)

      if current_forward in backwardPaths:
          intersection = backwardPaths[current_forward]
          return path_forward + intersection[::-1][1:]

      for neighbour, cost in enumerate(adj_matrix[current_forward]):
          if cost > 0 and neighbour not in forwardClosed:
              nextG = g_forward + cost
              xx1, yy1 = node_attributes[neighbour]['x'], node_attributes[neighbour]['y']
              xx2, yy2 = node_attributes[goal_node]['x'], node_attributes[goal_node]['y']
              nextH = ((xx2 - xx1)**2 + (yy2 - yy1)**2)**0.5
              nextF = nextG + nextH

              InOpenforward = any(neighbour == open_node[0] and nextF >= open_node[3] for open_node in forwardList)
              if not InOpenforward:
                  forwardList.append((neighbour, path_forward + [neighbour], nextG, nextF))
                  forwardPaths[neighbour] = path_forward + [neighbour]

      current_backward, backPath, backG, backF = backwardList.pop(0)
      backwardClosed.append(current_backward)

      if current_backward in forwardPaths:
          intersection = forwardPaths[current_backward]
          return intersection + backPath[::-1][1:]

      for neighbour, cost in enumerate(adj_matrix[current_backward]):
          if cost > 0 and neighbour not in backwardClosed:
              nextG = backG + cost
              xx1, yy1 = node_attributes[neighbour]['x'], node_attributes[neighbour]['y']
              xx2, yy2 = node_attributes[start_node]['x'], node_attributes[start_node]['y']
              nextH = ((xx2 - xx1)**2 + (yy2 - yy1)**2)**0.5
              nextF = nextG + nextH

              in_open_backward = any(neighbour == open_node[0] and nextF >= open_node[3] for open_node in backwardList)
              if not in_open_backward:
                  backwardList.append((neighbour, backPath + [neighbour], nextG, nextF))
                  backwardPaths[neighbour] = backPath + [neighbour]

  return None


if __name__ == "__main__":
  adj_matrix = np.load('IIIT_Delhi.npy')
  with open('IIIT_Delhi.pkl', 'rb') as f:
    node_attributes = pickle.load(f)

  start_node = int(input("Enter the start node: "))
  end_node = int(input("Enter the end node: "))

  print(f'Iterative Deepening Search Path: {get_ids_path(adj_matrix,start_node,end_node)}')
  print(f'Bidirectional Search Path: {get_bidirectional_search_path(adj_matrix,start_node,end_node)}')
  print(f'A* Path: {get_astar_search_path(adj_matrix,node_attributes,start_node,end_node)}')
  print(f'Bidirectional Heuristic Search Path: {get_bidirectional_heuristic_search_path(adj_matrix,node_attributes,start_node,end_node)}')
  # compare(adj_matrix,node_attributes)