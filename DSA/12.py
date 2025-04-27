import heapq
import math
import networkx as nx
import matplotlib.pyplot as plt

# Ensure symmetric graph (graph is undirected)
def make_graph_symmetric(graph):
    for node, edges in graph.items():
        for neighbor in edges:
            if node not in graph[neighbor]:
                graph[neighbor].append(node)
    return graph

# Example graph for a small building with rooms as nodes and paths as edges with weights
building_graph = {
    'A': ['B', 'D'],
    'B': ['A', 'C', 'E'],
    'C': ['B'],
    'D': ['A', 'E', 'R'],
    'E': ['B', 'D', 'F', 'Q'],
    'F': ['E', 'G', 'P'],
    'G': ['F', 'H', 'N'],
    'H': ['G', 'I', 'M'],
    'I': ['H', 'J'],
    'J': ['I', 'K'],
    'K': ['J', 'L'],
    'L': ['K'],
    'M': ['H', 'N'],
    'N': ['G', 'M', 'O'],
    'O': ['N', 'P'],
    'P': ['O', 'F', 'Q', 'R'],
    'Q': ['P', 'E', 'R'],
    'R': ['D', 'P', 'Q', 'S', 'W'],
    'S': ['R', 'T'],
    'T': ['S'],
    'W': ['R', 'X', 'V'],
    'X': ['W', 'Y'],
    'Y': ['X', 'Z'],
    'Z': ['Y', 'A1'],
    'A1': ['Z', 'A2'],
    'A2': ['A1', 'A3'],
    'A3': ['A2', 'A4'],
    'A4': ['A3', 'A5'],
    'A5': ['A4', 'A6', 'A7'],
    'A6': ['A5'],
    'A7': ['A5', 'A8'],
    'A8': ['A7', 'V'],
    'V': ['A8', 'W']
}

# Now change the list to a dictionary with weights
def add_weights(graph):
    for node, neighbors in graph.items():
        graph[node] = {neighbor: 1 for neighbor in neighbors}  # Default weight = 1
    return graph

# Make the graph symmetric and add weights
building_graph = make_graph_symmetric(building_graph)
building_graph = add_weights(building_graph)

# Example positions for nodes (used in the heuristic function)
positions = {
     'A': (1, 7), 'B': (2, 8), 'C': (3, 9), 'D': (2, 6), 'E': (3, 7),
        'F': (4, 8), 'G': (5, 7), 'H': (5, 6), 'I': (4, 5), 'J': (3, 4),
        'K': (2, 3), 'L': (1, 2), 'M': (6, 5), 'N': (6, 6), 'O': (7, 7),
        'P': (8, 6), 'Q': (8, 5), 'R': (7, 4), 'S': (6, 3), 'T': (5, 2),
        'W': (7, 5), 'X': (8, 7), 'Y': (9, 8), 'Z': (10, 7), 'A1': (11, 6),
        'A2': (12, 5), 'A3': (11, 4), 'A4': (10, 3), 'A5': (9, 2),
        'A6': (8, 1), 'A7': (7, 2), 'A8': (6, 4), 'V': (7, 6)
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
    
    # Draw the graph with a fixed layout (this is the static part)
    nx.draw(G, positions, with_labels=True, node_size=500, node_color="skyblue", font_size=10, font_weight="bold")
    
    # Draw edge labels
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, positions, edge_labels=edge_labels, font_size=8)
    
    # Highlight the shortest path if provided
    if path and path != "No path found":
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, positions, edgelist=path_edges, edge_color="orange", width=2)
    elif path == "No path found":
        print("Cannot visualize a graph without a valid path.")
    
    plt.show()

# Main program logic
def main():
    # Create the graph and plot it once (this is fixed)
    visualize_graph(building_graph)
    
    # User input for start and end locations
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
