# Function to parse the input
def parse_input():
    n = int(input())  
    e = int(input())

    # Initialize the distance matrix with large numbers (representing infinity)
    dist = [[float('inf')] * (n + 1) for _ in range(n + 1)]

    # Set distance to self as 0
    for i in range(n + 1):
        dist[i][i] = 0

    # Fill the distance matrix based on input edges
    for _ in range(e):
        edge_id, node1, node2, cost = map(int, input().split())
        dist[node1][node2] = cost
        dist[node2][node1] = cost

    start_node = int(input())

    return n, dist, start_node


# Memoization table for TSP
def initialize_memo(n):
    return [[-1] * (1 << (n + 1)) for _ in range(n + 1)]


# TSP function (recursive with memoization)
def tsp(i, mask, dist, memo, n, parent):
    # Base case: If all nodes are visited (except starting node)
    if mask == ((1 << i) | 3):
        return dist[1][i]

    # Check if already solved (memoization)
    if memo[i][mask] != -1:
        return memo[i][mask]

    # Initialize result with a large number (infinity)
    res = 10**9

    # Try all possible nodes to visit
    for j in range(1, n + 1):
        if (mask & (1 << j)) != 0 and j != i and j != 1:
            temp_res = tsp(j, mask & (~(1 << i)), dist, memo, n, parent) + dist[j][i]
            if temp_res < res:
                res = temp_res
                parent[i][mask] = j

    memo[i][mask] = res
    return res


# Function to reconstruct the route
def find_route(parent, start, mask, n):
    route = [start]
    curr_node = start

    while True:
        next_node = parent[curr_node][mask]
        if next_node is None:
            break
        route.append(next_node)
        mask = mask & (~(1 << curr_node))
        curr_node = next_node

    return route


# Main function to solve TSP
def solve_tsp():
    n, dist, start_node = parse_input()

    # Initialize memoization and parent tables
    memo = initialize_memo(n)
    parent = [[None] * (1 << (n + 1)) for _ in range(n + 1)]

    # Find the minimum cost of the most efficient tour
    ans = 10**9
    best_start = None

    for i in range(1, n + 1):
        temp_ans = tsp(i, (1 << (n + 1)) - 1, dist, memo, n, parent) + dist[i][1]
        if temp_ans < ans:
            ans = temp_ans
            best_start = i

    # Reconstruct the route from the parent table
    mask = (1 << (n + 1)) - 1
    route = find_route(parent, best_start, mask, n)
    route.append(1)

    # Output the result
    print(f"Cost = {ans}")
    print(f"Route: {route}")


# Call the function
solve_tsp()
