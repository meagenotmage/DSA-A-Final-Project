import heapq
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import FancyArrowPatch

# Coordinates for heuristic function and visualization
coordinates = {
    "Admin Building": (0, 0),
    "Library": (2, 1),
    "Skills Lab": (5, 2),
    "Gymnasium": (1, 5),
    "College of Education": (3, 0),
    "College of Medicine": (2, 3),
    "College of Engineering": (6, 3),
    "Cafeteria": (4, 5),
    "Cultural Center": (1, -1),
    "Dormitory": (5, 6),
}

# Define the graph (nodes and edges with weights)
graph = {
    "Admin Building": {
        "Library": 200, 
        "Gymnasium": 500, 
        "Skills Lab": 300, 
        "College of Education": 350,
        "Cultural Center": 400,
    },
    "Library": {
        "Admin Building": 200, 
        "Skills Lab": 150, 
        "College of Medicine": 250,
        "Cultural Center": 500,
    },
    "Skills Lab": {
        "Library": 150, 
        "Gymnasium": 400, 
        "College of Engineering": 300,
        "Admin Building": 300,
    },
    "Gymnasium": {
        "Admin Building": 500, 
        "Skills Lab": 400, 
        "Cafeteria": 450,
        "Dormitory": 300,
    },
    "College of Education": {
        "Admin Building": 350, 
        "College of Medicine": 300,
        "Cafeteria": 400,
    },
    "College of Medicine": {
        "Library": 250, 
        "College of Education": 300, 
        "College of Engineering": 350,
    },
    "College of Engineering": {
        "Skills Lab": 300, 
        "Cafeteria": 350,
        "College of Medicine": 350,
    },
    "Cafeteria": {
        "Gymnasium": 450, 
        "College of Engineering": 350, 
        "Dormitory": 200,
        "College of Education": 400,
    },
    "Cultural Center": {
        "Admin Building": 400, 
        "Library": 500,
    },
    "Dormitory": {
        "Cafeteria": 200, 
        "Gymnasium": 300,
    },
}

# Heuristic function (Euclidean distance)
def heuristic(node, goal):
    x1, y1 = coordinates[node]
    x2, y2 = coordinates[goal]
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# A* Algorithm for shortest pathfinding
def a_star(graph, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start, goal)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            # Reconstruct the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return the path from start to goal

        for neighbor, weight in graph[current].items():
            tentative_g_score = g_score[current] + weight
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                if neighbor not in [item[1] for item in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # In case no path is found

# Visualization function: Highlight path on campus map
def visualize_pathway_on_image(image_path, coordinates, path):
    # Load the campus map image
    img = mpimg.imread(image_path)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(img)

    # Plot the nodes on the map
    for node, (x, y) in coordinates.items():
        ax.scatter(x * 100, y * 100, color='blue')  # Scale for visual clarity
        ax.text(x * 100 + 10, y * 100 + 10, node, color='black', fontsize=10)

    # Plot the path
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        x1, y1 = coordinates[start]
        x2, y2 = coordinates[end]
        
        # Create an arrow for the path
        arrow = FancyArrowPatch((x1 * 100, y1 * 100), (x2 * 100, y2 * 100),
                                mutation_scale=15, color='red', arrowstyle='->')
        ax.add_patch(arrow)

    plt.title("Campus Building Navigator: Shortest Path")
    plt.axis('off')  # Hide axes for a clean map view
    plt.show()

# User interface
def main():
    print("Welcome to Campus Building Navigator: WVSU Easy Navigation")
    print("\nAvailable Locations:")
    for location in graph.keys():
        print(f" - {location}")

    start = input("\nEnter your starting location: ").strip()
    goal = input("Enter your destination: ").strip()

    if start not in graph or goal not in graph:
        print("Error: Invalid locations entered. Please try again.")
        return

    print("\nCalculating the shortest path...")
    path = a_star(graph, start, goal)

    if path:
        print(f"\nThe shortest path from {start} to {goal} is:")
        print(" -> ".join(path))
        # Specify the image path to the campus map
        image_path = 'map.png'  # Replace with the path to your image
        visualize_pathway_on_image(image_path, coordinates, path)
    else:
        print(f"No path found from {start} to {goal}.")

if __name__ == "__main__":
    main()
