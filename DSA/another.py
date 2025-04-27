import heapq
import math
import networkx as nx
import matplotlib.pyplot as plt

# Example campus graph with locations and distances
campus_graph = {
    'Main Gate': {'Security/Post Office': 2, 'Cultural Center': 4, 'MPS Commercial Building': 5},
    'Security/Post Office': {'Main Gate': 2, 'Cultural Center': 3, 'Audio Visual Hall': 6},
    'Cultural Center': {'Main Gate': 4, 'Security/Post Office': 3, 'MPS Commercial Building': 2, 'College of Communications': 7},
    'MPS Commercial Building': {'Main Gate': 5, 'Cultural Center': 2, 'College of Communications': 3, 'Audio Visual Hall': 4},
    'College of Communications': {'Cultural Center': 7, 'MPS Commercial Building': 3, 'Administration Building': 6},
    'Audio Visual Hall': {'Security/Post Office': 6, 'MPS Commercial Building': 4, 'Administration Building': 2},
    'Administration Building': {'Audio Visual Hall': 2, 'College of Communications': 6, 'Lopez Jaena Hall': 5},
    'Lopez Jaena Hall': {'Administration Building': 5, 'Rizal Hall': 4, 'College of Nursing': 3},
    'Rizal Hall': {'Lopez Jaena Hall': 4, 'College of Nursing': 2, 'Back Gate': 7},
    'College of Nursing': {'Lopez Jaena Hall': 3, 'Rizal Hall': 2, 'Research and Extension': 6},
    'Research and Extension': {'College of Nursing': 6, 'Binhi': 4},
    'Binhi': {'Research and Extension': 4, 'Back Gate': 5},
    'Back Gate': {'Rizal Hall': 7, 'Binhi': 5, 'Gray Building': 8},
    'Gray Building': {'Back Gate': 8, 'Coop': 3, 'College of ICT': 4},
    'Coop': {'Gray Building': 3, 'College of ICT': 5, 'Medicine Gym': 7},
    'College of ICT': {'Gray Building': 4, 'Coop': 5, 'Jubilee Park': 6},
    'Medicine Gym': {'Coop': 7, 'Miniforest': 2},
    'Miniforest': {'Medicine Gym': 2, 'Jubilee Park': 4},
    'Jubilee Park': {'College of ICT': 6, 'Miniforest': 4, 'Diamond Park': 5},
    'Diamond Park': {'Jubilee Park': 5, 'Grandstand': 3},
    'Grandstand': {'Diamond Park': 3, 'Field': 2},
    'Field': {'Grandstand': 2, 'Quezon Hall': 6},
    'Quezon Hall': {'Field': 6, 'Roxas Hall': 3},
    'Roxas Hall': {'Quezon Hall': 3, 'Hometel and Cafeteria': 2},
    'Hometel and Cafeteria': {'Roxas Hall': 2, 'Clinic': 3, 'Supply Office': 5},
    'Clinic': {'Hometel and Cafeteria': 3, 'Supply Office': 2},
    'Supply Office': {'Hometel and Cafeteria': 5, 'Clinic': 2, 'Cafeteria': 4},
    'Cafeteria': {'Supply Office': 4, 'College of Pescar': 3},
    'College of Pescar': {'Cafeteria': 3, 'College of Dentistry': 2},
    'College of Dentistry': {'College of Pescar': 2, 'College of Business and Management': 5},
    'College of Business and Management': {'College of Dentistry': 5, 'Center for Teaching Excellence': 3},
    'Center for Teaching Excellence': {'College of Business and Management': 3, 'Teachers Center': 4},
    'Teachers Center': {'Center for Teaching Excellence': 4, 'New Academic Building': 6},
    'New Academic Building': {'Teachers Center': 6, '2nd Gate': 4},
    '2nd Gate': {'New Academic Building': 4, 'Main Gate': 7}
}

# Ensure symmetric graph
def make_graph_symmetric(graph):
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            if neighbor not in graph or node not in graph[neighbor]:
                if neighbor not in graph:
                    graph[neighbor] = {}
                graph[neighbor][node] = weight
    return graph

campus_graph = make_graph_symmetric(campus_graph)

# Heuristic function (Euclidean distance approximation)
def heuristic(node1, node2):
    return abs(hash(node1) - hash(node2)) % 10  # Simple hash-based approximation

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
    
    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            G.add_edge(node, neighbor, weight=weight)
    
    pos = nx.spring_layout(G)
    nx.draw(G, pos, with_labels=True, node_size=500, node_color="skyblue", font_size=10, font_weight="bold")
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)
    
    if path and path != "No path found":
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="orange", width=2)
    
    plt.show()

# Main program logic
def main():
    start = input("Enter the starting location: ")
    end = input("Enter the destination location: ")

    if start not in campus_graph or end not in campus_graph:
        print("Invalid start or end location. Please enter locations available in the campus.")
        return

    shortest_path = a_star(campus_graph, start, end)
    print(f"The shortest path from {start} to {end} is: {shortest_path}")
    
    if shortest_path == "No path found":
        print("No valid path to visualize.")
    else:
        visualize_graph(campus_graph, path=shortest_path)

if __name__ == "__main__":
    main()