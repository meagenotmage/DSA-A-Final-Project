import heapq
import math
import networkx as nx
import matplotlib.pyplot as plt

# Ensure symmetric graph
def make_graph_symmetric(graph):
    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            if node not in graph[neighbor]:
                graph[neighbor][node] = weight
    return graph

# Example graph for a small building with rooms as nodes and paths as edges with weights
building_graph = {
    'Entrance': {'Hallway1': 5, 'Lobby': 3},
    'Lobby': {'Elevator': 4, 'Hallway1': 2},
    'Hallway1': {'Room101': 2, 'Room102': 3, 'Staircase': 4},
    'Room101': {'Hallway1': 2},
    'Room102': {'Hallway1': 3},
    'Staircase': {'Exit': 5, 'Hallway1': 4, 'Elevator': 3},
    'Elevator': {'Exit': 6, 'Lobby': 4},
    'Exit': {}
}

building_graph = make_graph_symmetric(building_graph)

# Example positions for nodes (used in the heuristic function)
positions = {
    'Entrance': (0, 0),
    'Lobby': (1, 1),
    'Hallway1': (2, 0),
    'Room101': (3, 0),
    'Room102': (2, -1),
    'Staircase': (2, 1),
    'Elevator': (1, 2),
    'Exit': (2, 2)
}

# Heuristic function (Euclidean distance)
def heuristic(node1, node2):
    x1, y1 = positions[node1]
    x2, y2 = positions[node2]
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# A* algorithm implementation
def a_star(graph, start, end):
    open_set = [(0, start)]
    came_from = {}
    g_score = {node: float('infinity') for node in graph}
    g_score[start] = 0
    f_score = {node: float('infinity') for node in graph}
    f_score[start] = heuristic(start, end)
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == end:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path from start to end
        
        for neighbor, weight in graph[current].items():
            tentative_g_score = g_score[current] + weight
            
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return "No path found"

# Visualization function using NetworkX and Matplotlib
def visualize_graph(graph, path=None):
    G = nx.Graph()
    
    # Add nodes and edges with weights
    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            G.add_edge(node, neighbor, weight=weight)
    
    # Draw the graph
    pos = nx.spring_layout(G)  # Positions for all nodes
    nx.draw(G, pos, with_labels=True, node_size=500, node_color="skyblue", font_size=10, font_weight="bold")
    
    # Draw edge labels
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)
    
    # Highlight the shortest path if provided
    if path and path != "No path found":
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="orange", width=2)
    elif path == "No path found":
        print("Cannot visualize a graph without a valid path.")
    
    plt.show()

# Main program logic
def main():
    start = input("Enter the starting location: ")
    end = input("Enter the destination location: ")

    # Check if inputs are valid nodes in the graph
    if start not in building_graph or end not in building_graph:
        print("Invalid start or end location. Please enter locations available in the building.")
        return
    
    # Find and print the shortest path
    shortest_path = a_star(building_graph, start, end)
    print(f"The shortest path from {start} to {end} is: {shortest_path}")
    
    # Visualize the graph with the shortest path highlighted
    if shortest_path == "No path found":
        print("No valid path to visualize.")
    else:
        visualize_graph(building_graph, path=shortest_path)

# Run the program
if __name__ == "__main__":
    main()
