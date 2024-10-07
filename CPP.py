import itertools
import heapq

n = int(input())  # Number of vertices
e = int(input())  # Number of edges

graph = {}
adjacency_list = {i: [] for i in range(1, n + 1)}  # Adjacency list for graph traversal

def add_edge(edge_name, u, v, weight):
    # Check if there is an existing edge between u and v
    existing_edges = [(adj_v, w, e_name) for adj_v, w, e_name in adjacency_list[u] if adj_v == v and w > weight]
    
    if existing_edges:
        # If an existing edge is found, store its details before removal
        existing_edge_name = existing_edges[0][2]  
        existing_weight = existing_edges[0][1]     \
        
        # Remove the existing edge from the adjacency list and the graph
        adjacency_list[u] = [(adj_v, w, e_name) for adj_v, w, e_name in adjacency_list[u] if adj_v != v]
        adjacency_list[v] = [(adj_u, w, e_name) for adj_u, w, e_name in adjacency_list[v] if adj_u != u]
        del graph[existing_edge_name]  

        # Add the new edge
        graph[edge_name] = (u, v, weight) 
        adjacency_list[u].append((v, weight, edge_name))
        adjacency_list[v].append((u, weight, edge_name))

        # Re-add the previous edge (as a duplicate)
        graph[existing_edge_name] = (u, v, existing_weight)
        adjacency_list[u].append((v, existing_weight, existing_edge_name))
        adjacency_list[v].append((u, existing_weight, existing_edge_name))

    else:
        # No existing edge, just add the new edge
        graph[edge_name] = (u, v, weight)  
        adjacency_list[u].append((v, weight, edge_name))
        adjacency_list[v].append((u, weight, edge_name))

# Input edges
for _ in range(e):
    edge_info = input().split() 
    edge_name = int(edge_info[0])  
    u = int(edge_info[1])          
    v = int(edge_info[2])          
    weight = int(edge_info[3])      
    
    if u > v:
        tempU = u
        u = v
        v = tempU

    add_edge(edge_name, u, v, weight)

# 1. Degree Calculation: Find the degree of each vertex
def get_degrees(adjacency_list):
    degree = {i: 0 for i in range(1, n + 1)}
    for u in adjacency_list:
        degree[u] = len(adjacency_list[u])
    return degree

# 2. Find odd-degree vertices
def get_odd_degree_vertices(degree):
    odd_vertices = [v for v in degree if degree[v] % 2 != 0]
    return odd_vertices

# 3. Dijkstra's Algorithm to find shortest path between two vertices
def dijkstra(source, target):
    dist = {i: float('inf') for i in range(1, n + 1)}
    dist[source] = 0
    priority_queue = [(0, source)]
    prev = {i: None for i in range(1, n + 1)}
    path_edges = {i: None for i in range(1, n + 1)}  

    while priority_queue:
        current_dist, u = heapq.heappop(priority_queue)
        
        if current_dist > dist[u]:
            continue
        
        for v, weight, edge_name in adjacency_list[u]:
            distance = current_dist + weight
            if distance < dist[v]:
                dist[v] = distance
                prev[v] = u
                path_edges[v] = edge_name
                heapq.heappush(priority_queue, (distance, v))
    
    # Reconstruct the path from source to target
    path = []
    current = target
    while prev[current] is not None:
        path.append(path_edges[current])  # Append the edge used
        current = prev[current]
    
    path.reverse()  # Reverse the path to get it in the correct order
    return dist[target], path

# 4. Find minimum-cost matching of odd-degree vertices
def find_minimum_matching(odd_vertices):
    pairs = list(itertools.combinations(odd_vertices, 2))
    pairings = {}
    
    for u, v in pairs:
        distance, path = dijkstra(u, v)
        pairings[(u, v)] = (distance, path)
    
    return pairings

# 5. Add minimum matching edges to make the graph Eulerian
def add_minimum_matching_edges(odd_vertices, pairings):
    # Use a greedy algorithm to add edges for pairing odd-degree vertices
    total_cost = 0
    route = []
    
    while odd_vertices:
        u = odd_vertices.pop(0)
        best_pair = None
        best_cost = float('inf')
        best_path = None
        
        for v in odd_vertices:
            if (u, v) in pairings and pairings[(u, v)][0] < best_cost:
                best_pair = v
                best_cost = pairings[(u, v)][0]
                best_path = pairings[(u, v)][1]
        
        if best_pair is not None:
            odd_vertices.remove(best_pair)
            total_cost += best_cost
            route.extend(best_path)  # Append the path (edges) to the route
            # print(f"Adding duplicate edge between {u} and {best_pair} with cost {best_cost}")
    
    return total_cost, route

# 6. Modified Eulerian tour to select the lowest weight edge when traversing vertices with degree > 2
def find_eulerian_tour(start_vertex):
    stack = [start_vertex]  
    eulerian_tour = []      
    used_edges = set()     

    while stack:
        u = stack[-1]  # Look at the current vertex at the top of the stack

        # Get all available edges from the current vertex that have not been used
        available_edges = [(v, weight, edge_name) for v, weight, edge_name in adjacency_list[u] if edge_name not in used_edges]

        if available_edges:
            # Select the edge with the lowest weight
            v, min_weight, min_edge_name = min(available_edges, key=lambda x: x[1])
            used_edges.add(min_edge_name)  # Mark this edge as used
            stack.append(v)  # Move to the next vertex
        else:
            # No more available edges from u, backtrack and record the vertex
            stack.pop()
            if len(stack) > 0:
                eulerian_tour.append(u)

    return eulerian_tour

# Main function to solve Chinese Postman Problem
def solve_cpp():
    degree = get_degrees(adjacency_list)
    odd_vertices = get_odd_degree_vertices(degree)
    
    additional_cost = 0
    additional_route = []

    if not odd_vertices:
        # print("Graph is already Eulerian.")
        pass
    else:
        # print(f"Odd degree vertices: {odd_vertices}")
        
        pairings = find_minimum_matching(odd_vertices)
        additional_cost, additional_route = add_minimum_matching_edges(odd_vertices, pairings)
        # print(f"Additional cost to make the graph Eulerian: {additional_cost}")

    # At this point, the graph should be Eulerian (or made Eulerian).
    # The total cost for the CPP will be the sum of the original edges plus any additional cost for matching odd vertices.
    total_weight = sum(weight for _, (_, _, weight) in graph.items())
    total_cost = total_weight + additional_cost

    # Find the Eulerian tour starting from any vertex (assuming graph is connected)
    start_vertex = start  
    eulerian_route = find_eulerian_tour(start_vertex)

    # Include the names of the additional edges in the final route
    eulerian_route = list(graph.keys()) + additional_route  # Simplified Eulerian tour generation
    print(f"Cost: {total_cost}")
    print("Route:", ', '.join(f"{x}" for x in eulerian_route))

# Call the function to solve the Chinese Postman Problem
start = int(input())
solve_cpp()
