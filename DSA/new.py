import networkx as nx
import folium
from queue import PriorityQueue
import math

# Define the nodes (buildings/locations) with their names
CAMPUS_NODES = {
    # Top row (curved path, left to right)
    'K': (1, 10),
    'J': (2, 9.8),
    'I': (3, 9.5),
    'H': (4, 9.3),
    'G': (5, 9),
    'F': (6, 8.7),
    'E': (7, 8.5),
    'D': (8, 8.2),
    'C': (9, 8),
    'B': (10, 7.8),
    'A': (11, 7.5),
    
    # Middle section (including slight curves)
    'L': (1.5, 7),
    'M': (4.2, 6.5),
    'N': (5, 6.4),
    'O': (5.3, 6.2),
    'P': (6, 6),
    'Q': (6.8, 5.8),
    'R': (7.5, 5.5),
    'S': (8.3, 5.3),
    'T': (9, 5),
    
    # Bottom section (left cluster with slight curve)
    'X': (5, 4),
    'W': (5.7, 3.8),
    'V': (6.5, 3.5),
    'U': (7.2, 3.2),
    
    # Bottom cluster (tight grouping near the bottom center)
    'Y': (5.7, 2.5),
    'Z': (5.8, 2.3),
    'A1': (5.9, 2),
    
    # Bottom right cluster (slightly spread out)
    'A2': (6.5, 2.3),
    'A3': (7.3, 1.8),
    'A4': (7.5, 1.5),
    'A5': (8.2, 1.8),
    'A6': (8.4, 2.2),
    'A7': (9, 2),
    'A8': (9.5, 1.5)
}


def create_campus_graph():
    """Create a graph based on the exact map structure shown in the image."""
    G = nx.Graph()
    
    # Add all nodes
    for name, coords in CAMPUS_NODES.items():
        G.add_node(name, pos=coords)
    
    # Add edges 
    edges = [
    # Top curved paths
    ('K', 'J'), ('J', 'I'), ('I', 'H'), ('H', 'G'), ('G', 'F'),
    ('F', 'E'), ('E', 'D'), ('D', 'C'), ('C', 'B'), ('B', 'A'),
    
    # Vertical and diagonal connections from top row
    ('K', 'L'), ('L', 'M'), ('I', 'M'),
    ('M', 'N'), ('N', 'O'), ('O', 'P'),
    ('P', 'Q'), ('Q', 'R'), ('R', 'S'),
    ('S', 'T'),
    
    # Middle section curved paths
    ('M', 'O'), ('N', 'P'), ('Q', 'W'),
    
    # Bottom cluster connections
    ('X', 'W'), ('W', 'V'), ('V', 'U'),
    ('Y', 'Z'), ('Z', 'A1'), ('A1', 'W'),
    
    # Bottom right connections (curved group)
    ('A2', 'A3'), ('A2', 'A4'), ('A4', 'A5'),
    ('A5', 'A6'), ('A6', 'A7'), ('A7', 'A8'),
    
    # Cross connections
    ('W', 'A4'), ('R', 'W'), ('V', 'A8')
]

    G.add_edges_from(edges)
    return G

def astar_search(G, start, end):
    """A* pathfinding algorithm implementation."""
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    
    while not frontier.empty():
        current = frontier.get()[1]
        
        if current == end:
            break
        
        for next_node in G.neighbors(current):
            new_cost = cost_so_far[current] + 1  # Using uniform edge costs
            
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                # Using Manhattan distance as heuristic
                start_pos = G.nodes[next_node]['pos']
                end_pos = G.nodes[end]['pos']
                priority = new_cost + abs(start_pos[0] - end_pos[0]) + abs(start_pos[1] - end_pos[1])
                frontier.put((priority, next_node))
                came_from[next_node] = current
    
    # Reconstruct path
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = came_from.get(current)
    path.reverse()
    return path

def main():
    # Create the campus graph
    G = create_campus_graph()
    
    # Display available locations
    print("\nAvailable campus locations:")
    locations = sorted(CAMPUS_NODES.keys())
    for i, location in enumerate(locations, 1):
        print(f"{i}. {location}")
    
    try:
        # Get user input
        start_idx = int(input("\nEnter number for starting location: ")) - 1
        end_idx = int(input("Enter number for destination: ")) - 1
        
        start_location = locations[start_idx]
        end_location = locations[end_idx]
        
        # Find shortest path
        path = astar_search(G, start_location, end_location)
        print(f"\nShortest path found:")
        print(" → ".join(path))
        
        # Create map visualization
        center_lat = sum(pos[0] for pos in CAMPUS_NODES.values()) / len(CAMPUS_NODES)
        center_lon = sum(pos[1] for pos in CAMPUS_NODES.values()) / len(CAMPUS_NODES)
        m = folium.Map(location=[center_lat, center_lon], zoom_start=18)
        
        # Add all nodes as markers
        for name, coords in CAMPUS_NODES.items():
            color = 'green' if name == start_location else 'red' if name == end_location else 'gray'
            folium.Marker(
                coords,
                popup=name,
                icon=folium.Icon(color=color)
            ).add_to(m)
        
        # Draw all edges
        for edge in G.edges():
            coords = [CAMPUS_NODES[edge[0]], CAMPUS_NODES[edge[1]]]
            color = 'gray'
            weight = 2
            opacity = 0.5
            
            # Highlight the path
            if edge[0] in path and edge[1] in path:
                if abs(path.index(edge[0]) - path.index(edge[1])) == 1:
                    color = 'blue'
                    weight = 4
                    opacity = 1.0
            
            folium.PolyLine(
                coords,
                weight=weight,
                color=color,
                opacity=opacity
            ).add_to(m)
        
        # Save map
        output_file = "wvsu_campus_path.html"
        m.save(output_file)
        print(f"\nMap saved as {output_file}")
        
    except IndexError:
        print("Invalid location number selected.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()