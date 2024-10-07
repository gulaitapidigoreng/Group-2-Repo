# Function to parse the input
def parse_input():
    n = int(input())  # number of nodes
    e = int(input())  # number of edges

    # Initialize the distance matrix with large numbers (representing infinity)
    dist = [[float('inf')] * (n + 1) for _ in range(n + 1)]

    # Set distance to self as 0
    for i in range(n + 1):
        dist[i][i] = 0

    # Fill the distance matrix based on input edges
    for _ in range(e):
        edge_id, node1, node2, cost = map(int, input().split())
        dist[node1][node2] = cost
        dist[node2][node1] = cost  # Since it's an undirected graph

    start_node = int(input())  # Start and end node for TSP

    return n, dist, start_node


# Memoization table for TSP
def initialize_memo(n):
    return [[-1] * (1 << (n + 1)) for _ in range(n + 1)]


# TSP function (recursive with memoization)
def tsp(i, mask, dist, memo, n):
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
            res = min(res, tsp(j, mask & (~(1 << i)), dist, memo, n) + dist[j][i])

    memo[i][mask] = res
    return res


# Main function to solve TSP
def solve_tsp():
    n, dist, start_node = parse_input()
    
    # Initialize memoization table
    memo = initialize_memo(n)
    
    # Find the minimum cost of the most efficient tour
    ans = 10**9
    for i in range(1, n + 1):
        ans = min(ans, tsp(i, (1 << (n + 1)) - 1, dist, memo, n) + dist[i][1])

    # Output the result
    print("The cost of most efficient tour = " + str(ans))


# Call the function to run the TSP solver
solve_tsp()
