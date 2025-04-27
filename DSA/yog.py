import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from heapq import heappush, heappop
import os

class WVSUCampusPathfinder:
    def __init__(self, map_path, building_names=None, speed=1.4):
        # Load and convert map
        if not os.path.exists(map_path):
            raise FileNotFoundError(f"Map file '{map_path}' not found.")
        
        self.original_map = cv2.imread(map_path)
        if self.original_map is None:
            raise ValueError(f"Failed to load the map image '{map_path}'. Make sure it is a valid image file.")
        
        self.original_map_rgb = cv2.cvtColor(self.original_map, cv2.COLOR_BGR2RGB)
        self.waypoints = self.detect_yellow_points()
        self.building_names = building_names or self.default_building_names()
        self.speed = speed
        self.road_mask = self.create_refined_road_mask()
        self.building_mask = self.create_building_mask()

    def detect_yellow_points(self):
        """Detect yellow landmarks."""
        hsv = cv2.cvtColor(self.original_map, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        waypoints = []
        for contour in contours:
            if cv2.contourArea(contour) > 20: 
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    waypoints.append((cx, cy))
        
        if len(waypoints) == 0:
            raise ValueError("No yellow landmarks (waypoints) detected on the map.")
        
        return np.array(waypoints)
    
    def create_refined_road_mask(self):
        """Create a refined road mask, where roads are walkable (thin black lines)."""
        gray = cv2.cvtColor(self.original_map, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 100, 200)
        refined_road_mask = cv2.bitwise_not(edges)
        return refined_road_mask

    def create_building_mask(self):
        """Create a mask for building areas to ensure they are non-walkable."""
        hsv = cv2.cvtColor(self.original_map, cv2.COLOR_BGR2HSV)

        lower_building = np.array([0, 0, 0])   # Low saturation for dark colors
        upper_building = np.array([180, 255, 70])  # Lightness threshold to filter buildings

        building_mask = cv2.inRange(hsv, lower_building, upper_building)
        
        kernel = np.ones((5,5), np.uint8)
        refined_building_mask = cv2.morphologyEx(building_mask, cv2.MORPH_CLOSE, kernel)

        return refined_building_mask

    def is_walkable(self, point):
        """Check if a point is within a walkable area."""
        x, y = point
        if 0 <= x < self.road_mask.shape[1] and 0 <= y < self.road_mask.shape[0]:
            road_walkable = self.road_mask[y, x] == 255  # White walkable area 
            building_non_walkable = self.building_mask[y, x] == 255  # Black building area
            return road_walkable and not building_non_walkable
        return False

    def a_star(self, start, end):
        """A* pathfinding algorithm with diagonal movement."""
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))  # Euclidean distance

        open_list = []
        heappush(open_list, (0 + heuristic(start, end), 0, start))
        came_from = {start: None}
        g_score = {start: 0}
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]
        explored_nodes = []

        while open_list:
            _, current_g, current = heappop(open_list)

            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                if self.is_walkable((nx, ny)):
                    tentative_g_score = current_g + 1
                    if (nx, ny) not in g_score or tentative_g_score < g_score[(nx, ny)]:
                        came_from[(nx, ny)] = current
                        g_score[(nx, ny)] = tentative_g_score
                        f_score = tentative_g_score + heuristic((nx, ny), end)
                        heappush(open_list, (f_score, tentative_g_score, (nx, ny)))
                        explored_nodes.append((nx, ny))

        return None

    def find_path(self, start_idx, end_idx):
        """Find path between two waypoints using A*."""
        start = tuple(self.waypoints[start_idx])
        end = tuple(self.waypoints[end_idx])
        
        if not self.is_walkable(start) or not self.is_walkable(end):
            print("Selected points are not in a walkable area.")
            return None, None

        path = self.a_star(start, end)
        if path:
            total_cost = sum(np.linalg.norm(np.array(path[i]) - np.array(path[i + 1])) for i in range(len(path) - 1))
            return path, total_cost
        return None, None

    def calculate_time(self, total_distance):
        """Calculate the time in seconds and minutes.""" 
        time_seconds = total_distance / self.speed
        minutes = int(time_seconds // 60)
        seconds = int(time_seconds % 60)
        return minutes, seconds

    def visualize_path(self, start_idx, end_idx):
        """Visualize the path on the map."""
        plt.figure(figsize=(15, 12))
        ax_map = plt.subplot2grid((5, 1), (0, 0), rowspan=4)
        
        path, total_cost = self.find_path(start_idx, end_idx)
        
        if path:
            vis_map = self.original_map_rgb.copy()
            for i in range(len(path) - 1):
                pt1 = path[i]
                pt2 = path[i + 1]
                cv2.line(vis_map, pt1, pt2, (50, 205, 50), 3)  # Thicker green line for the path
                
            for point in path:
                cv2.circle(vis_map, point, 5, (135, 206, 235), -1)  # Sky blue dots
                start_point = tuple(map(int, self.waypoints[start_idx]))
                end_point = tuple(map(int, self.waypoints[end_idx]))
            cv2.circle(vis_map, start_point, 15, (255, 50, 50), -1)  # Red start point
            cv2.circle(vis_map, end_point, 15, (50, 50, 255), -1)  # Blue end point
            
            total_distance = total_cost
            minutes, seconds = self.calculate_time(total_distance)
            
            start_name = self.building_names.get(start_idx, f'Building {start_idx}')
            end_name = self.building_names.get(end_idx, f'Building {end_idx}')
            
            plt.suptitle(f"WVSU Campus Navigation", fontsize=16, y=0.96, fontweight='bold')
            
            ax_map.imshow(vis_map)
            ax_map.axis('off')
            
            ax_info = plt.subplot2grid((5, 1), (4, 0))
            ax_info.axis('off')
            
            # Add trip information 
            info_text = f"""
            From: {start_name}
            To: {end_name}
            Distance: {total_distance:.1f} meters
            Estimated Time: {minutes}m {seconds}s
            """
            ax_info.text(0.5, 0.5, info_text,
                        horizontalalignment='center',
                        verticalalignment='center',
                        fontsize=12,
                        fontfamily='sans-serif',
                        bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=15))
            
            plt.tight_layout()
            plt.show()
        else:
            print("No valid path found between selected points.")

def run_pathfinder(map_path):
    building_names = {
         0: 'College of Business and Management',
        1: 'General Macario Peralta Hall',
        2: 'Printing and Souveneir Shop',
        3: 'Center for Teaching Excellence',
        4: 'Adriano Hernandez Hall',
        5: 'BINHI',
        6: 'College of Dentistry',
        7: 'Proposed Covered Gym',
        8: 'Alumni Hall',
        9: 'Apolinario Mabini Hall',
        10: 'Covered Gym',
        11: 'Supply Office Warehouse',
        12: 'Teachers Center',
        13: 'COOP Building',
        14: 'Martin Delgado Hall',
        15: 'PSWF Building',
        16: 'College of PESCAR',
        17: 'Drop In Center',
        18: 'Building 1',
        19: 'Tomas Confessor Hall',
        20: 'Reasearch and Extension Building 2',
        21: 'College of Medicine',
        22: 'Hometel',
        23: 'Multi Purpose Cooperative Building',
        24: 'Grounds and Building Office',
        25: 'University Clinic',
        26: 'Building 2',
        27: 'Research and Extension Building 2',
        28: 'New Academic Building',
        29: 'Benigno Aquino Jr. Hall',
        30: 'Quezon Hall',
        31: 'College of Information and Communications Technology',
        32: 'GSO/Motopool',
        33: 'College of Nursing',
        34: 'Gabaldon Building',
        35: 'GSO Electrical/Plumbing',
        36: 'Proposed Residential Area',
        37: 'Elementary Playground',
        38: 'Extension Office',
        39: 'Mini Forest',
        40: 'Medicine Gym',
        41: 'Rizal Hall',
        42: 'Second Gate',
        43: 'Jubilee Park',
        44: 'Dormitory Building',
        45: 'Grandstand',
        46: 'Diamond Park',
        47: 'WVSU Football Field',
        48: 'Lopez Jaena Hall',
        49: 'Administration Building',
        50: 'Audio Visual Hall',
        51: 'Main Gate',
        52: 'College of Communication',
        53: 'Cultural Center',
        54: 'WVSU Coop',
    }
    pathfinder = WVSUCampusPathfinder(map_path, building_names)
    points = []

    def onclick(event):
        if event.xdata and event.ydata:
            click_point = np.array([event.xdata, event.ydata])
            distances = cdist([click_point], pathfinder.waypoints)
            nearest_idx = np.argmin(distances)
            points.append(nearest_idx)
            
            # Provide feedback on selection
            building_name = building_names.get(nearest_idx, f'Building {nearest_idx}')
            if len(points) == 1:
                plt.title(f"Start: {building_name}\nClick to select destination", color='green')
            
            if len(points) == 2:
                plt.close()
                pathfinder.visualize_path(points[0], points[1])

    # Create selection interface
    plt.figure(figsize=(15, 10))
    plt.imshow(pathfinder.original_map_rgb)
    plt.title("Click to select starting point", color='black', pad=20, fontweight='bold')
    
    # Add usage instructions
    plt.figtext(0.5, 0.02, 
                "Click on any building to set start and end points",
                ha='center', color='darkgrey')
    
    plt.connect('button_press_event', onclick)
    plt.show()

if __name__ == "__main__":
    run_pathfinder("maps1.png")