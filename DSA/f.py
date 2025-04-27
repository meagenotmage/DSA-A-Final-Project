import networkx as nx
import folium

CAMPUS_NODES = {
    'Cultural Center': (10.716457, 122.565858),
    'Admin Building': (10.714402, 122.562236),
    'Lopez Hall': (10.715623, 122.563740),
    'RH': (10.715428, 122.563210),
    'COE Info': (10.715966, 122.564182),
    'CICT': (10.714774, 122.561347),
    'PE': (10.715612, 122.563471),
    'Research': (10.714238, 122.562014),
    'Graduate': (10.715248, 122.563012),
    'Gazebo': (10.714376, 122.562135),
    'Supply': (10.715815, 122.563140),
    'Post Office': (10.716326, 122.564563),
    'Registrar': (10.714776, 122.561951),
    'Teachers Center': (10.712822, 122.563553),
    'CTE': (10.714236, 122.562244),
    'Science': (10.714428, 122.562346),
    'ROTC': (10.717078, 122.565858),
    'Business': (10.715319, 122.563645),
    'CAS': (10.714428, 122.562346)
}

def create_campus_graph():
    """Create a graph based on the exact WVSU campus building coordinates."""
    G = nx.Graph()
    for name, coords in CAMPUS_NODES.items():
        G.add_node(name, pos=coords)

    edges = [
        ('Admin Building', 'Lopez Hall'),
        ('Lopez Hall', 'RH'),
        ('RH', 'COE Info'),
        ('COE Info', 'CICT'),
        ('CICT', 'PE'),
        ('PE', 'Research'),
        ('Research', 'Graduate'),
        ('Graduate', 'Gazebo'),
        ('Gazebo', 'Supply'),
        ('Supply', 'Post Office'),
        ('Post Office', 'Registrar'),
        ('Registrar', 'Teachers Center'),
        ('Teachers Center', 'CTE'),
        ('CTE', 'Science'),
        ('Science', 'Business'),
        ('Business', 'CAS'),
        ('Cultural Center', 'ROTC'),
        ('COE Info', 'PE'),
        ('PE', 'Graduate'),
        ('Graduate', 'Supply'),
        ('Supply', 'Post Office'),
        ('Post Office', 'Business'),
        ('Business', 'CTE'),
        ('CTE', 'Teachers Center')
    ]
    G.add_edges_from(edges)
    return G

def main():
    G = create_campus_graph()
    print("\nAvailable WVSU campus locations:")
    locations = sorted(CAMPUS_NODES.keys())
    for i, location in enumerate(locations, 1):
        print(f"{i}. {location}")

    try:
        start_idx = int(input("\nEnter number for starting location: ")) - 1
        end_idx = int(input("Enter number for destination: ")) - 1

        start_location = locations[start_idx]
        end_location = locations[end_idx]

        # Find shortest path
        path = nx.shortest_path(G, source=start_location, target=end_location, weight=None)
        print(f"\nShortest path from {start_location} to {end_location}:")
        print(" → ".join(path))

        # Create map visualization
        center_lat = sum(pos[0] for pos in CAMPUS_NODES.values()) / len(CAMPUS_NODES)
        center_lon = sum(pos[1] for pos in CAMPUS_NODES.values()) / len(CAMPUS_NODES)
        m = folium.Map(location=[center_lat, center_lon], zoom_start=19)

        for name, coords in CAMPUS_NODES.items():
            color = 'green' if name == start_location else 'red' if name == end_location else 'blue'
            folium.Marker(
                coords,
                popup=name,
                icon=folium.Icon(color=color)
            ).add_to(m)

        for edge in G.edges():
            coords = [CAMPUS_NODES[edge[0]], CAMPUS_NODES[edge[1]]]
            color = 'red' if edge[0] in path and edge[1] in path else 'blue'
            folium.PolyLine(coords, weight=3, color=color).add_to(m)

        m.save("wvsu_campus_path.html")
        print("\nMap saved as 'wvsu_campus_path.html'.")

    except IndexError:
        print("Invalid location number selected.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
