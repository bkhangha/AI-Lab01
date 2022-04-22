from queue import PriorityQueue
import numpy as np
from collections import deque

# ULTILITIES
# Trace back the path
def trace(start, end, visited: dict, path: list):
    goal = end # Go from goal to start point and trace back
    while (goal != start):
        path.append(goal)
        goal = visited[goal]
    path.append(start)
    path.reverse()

    return path

# Calculate Euclid distance between 2 nodes
def euclid_norm(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# Ref: https://www.geeksforgeeks.org/implementation-of-dfs-using-adjacency-matrix/
def DFS(matrix, start, end):
    # TODO: 
    path = []
    visited = {start: None}

    stack = [] 
    stack.append([start]) # first node is visited

    while (len(stack) != 0):
        path = stack.pop() # get the top node to stack
        node = path[-1]  # Inspect a node

        if (node == end):  
            break

        for i in range(len(matrix)):
            if (matrix[node][i] > 0 and i not in visited):
                visited[i] = node
                subpath = list(path) 
                subpath.append(i) 
                stack.append(subpath) 
                
        
    # path = trace(start, end, visited, path)

    return visited, path

# Ref: https://www.geeksforgeeks.org/implementation-of-bfs-using-adjacency-matrix/ 
def BFS(matrix, start, end):
    # TODO: 
    
    path = []
    visited = {start: None}
    
    queue = []  # A queue stores visitable node nearby current node
    queue.append([start]) # first node is visited

    while (len(queue)):
        path = queue.pop(0) # get first node from queue
        node = path[-1] # Inspect a node

        if (node == end):
            break
        
        for i in range(len(matrix)):
            if (matrix[node][i] != 0 and i not in visited):
                visited[i] = node
                subpath = list(path) 
                subpath.append(i) 
                queue.append(subpath)            

    # path = trace(start, end, visited_temp, path)

    return visited, path

# Ref: https://www.geeksforgeeks.org/uniform-cost-search-dijkstra-for-large-graphs/ 
def UCS(matrix, start, end):
    # TODO:  
    path = []
    visited = {}

    queue = []
    queue.append([0, start])
    visited.update({start: None})
    visit={}
    prev=start
    visited_temp={}

    while (len(queue)):
        queue = sorted(queue) # priority queue
        node = queue[-1]
        del queue[-1]
        prev = node[1]
        # node[0] *= -1

        if (node[1] == end):
            del queue[-1]
            queue = sorted(queue)
            break  

        if (node[1] not in visit):
            for i in range(len(matrix[node[1]])):
                if(matrix[node[1]][i] != 0):
                    queue.append([node[0] + matrix[node[1]][i] *- 1, i])
                    visited.update({i: node})
                    visited_temp[i] = prev
        
        visit[node[1]] = 1

    path = trace(start, end, visited_temp, path)
    return visited, path

# Ref: https://www.redblobgames.com/pathfinding/a-star/implementation.html
#      https://www.geeksforgeeks.org/a-search-algorithm/
def Astar(matrix, start, end, pos):
    path = []
    visited = {}
    
    #                                x1              y1            x2           y2 
    heuristic = euclid_norm(pos[start][0], pos[start][1], pos[end][0], pos[end][1]) # calculate distance between start and end
    pq = {start: (0, None, heuristic)} # create priority queue (cost, node, heuristic)
    
    while (len(pq) != 0):
        node = min(pq.items(), key=lambda x: x[1][2])[0] # choose node with smallest cost to inspect
        visited[node] = pq[node][1] # mark that node is visited

        if (node == end):
            break

        for i in range(len(matrix)):
            if (matrix[node][i] != 0 and i not in visited):
                # calculate new heuristic
                dist = matrix[node][i] + pq.get(node)[0] + euclid_norm(pos[i][0], pos[i][1], pos[end][0], pos[end][1])
                final_cost = matrix[node][i] + pq.get(node)[0]

                if (i not in pq or dist < pq.get(i)[2]): # choose better option
                    pq[i] = (final_cost, node, dist) # update priority queue

        del pq[node] # remove from queue after an inspection

    trace(start, end, visited, path)

    return visited, path

# Ref: https://www.geeksforgeeks.org/best-first-search-informed-search/
#      https://en.wikipedia.org/wiki/Best-first_search
def GBFS(matrix, start, end):
    # TODO:
    path = []
    visited = {start: None}
    stack = [];	# using stack to store node
    stack.append(start)

    while (len(stack) != 0):
        node = stack[-1]
        stack.pop()

        if (node == end):
            break
        
        temp = []

        for i in range(len(matrix)):
            if (matrix[node][i] != 0 and i not in visited):
                temp.append(i)
                visited[i] = node

        temp = sorted(temp)

        # Get lowest cost path to forward
        for i in range(len(temp)):
            stack.append(temp[len(temp) - 1 - i])
    
    trace(start, end, visited, path)

    return visited, path