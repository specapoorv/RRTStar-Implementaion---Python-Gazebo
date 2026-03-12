import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
import cv2 as cv
from scipy.spatial import KDTree
import math


class RRTStarPlanner():

    def __init__(self):
        self.max_dist = 10
        self.max_iterations = 100
        self.dilation_radius = 1
        self.search_radius = 20
        self.goal_threshold = 3
        self.line_thickness = 3


    def get_map_metadata(self, yaml_file="/home/ananth/gazebos_ros_ws/src/rrt_star/map_data/warehouse_map.yaml"):
        with open(yaml_file, "r")as yam:
            map_metadata = yaml.safe_load(yam) # This is a dictionary

        # Extracting metadata
        pgm_img_path = os.path.join(os.path.dirname(yaml_file), map_metadata["image"])
        map_metadata["image"] = pgm_img_path

        return map_metadata
    

    def get_occupancy_grid(self):
        map_metadata = self.get_map_metadata()

        pgm_map = cv.imread(map_metadata["image"], cv.IMREAD_UNCHANGED)    

        # Make it binary
        _, binary = cv.threshold(pgm_map, 250, 255, cv.THRESH_BINARY)

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        cleaned_morph = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel)

        # Dilation
        cleaned_inv = cv.bitwise_not(cleaned_morph)
        dilated = cv.dilate(cleaned_inv, kernel, iterations=1)
        cleaned = cv.bitwise_not(dilated)

        # Lets try some contour jugaad
        contours, _ = cv.findContours(cleaned, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        visual = cv.cvtColor(cleaned, cv.COLOR_GRAY2BGR)
        largest_contour = contours[1]
        cv.drawContours(visual, [largest_contour], -1, (0, 255, 0), 2)

        # contours[1] is the largest contour
        # Get bounding rect
        x, y, w, h = cv.boundingRect(largest_contour)
        cv.rectangle(visual, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # Crop image till bounding rect
        cropped_map = cleaned[y: y + h, x: x + w]

        
        # This occupacny grid is too detailed to downsample
        block_size = 3
        downsampled = cv.resize(
            cropped_map,
            (cropped_map.shape[1] // block_size, cropped_map.shape[0] // block_size)
        )
        occupancy_grid = np.where(downsampled == 0, 1, 0)
        # downsampled = cv.resize(occupancy_grid, (occupancy_grid.shape[1] // block_size, occupancy_grid.shape[0] // block_size), interpolation=cv.INTER_NEAREST)
        return occupancy_grid  # change it later if you want lists

    
    def rrt_star(self, occ_grid: np.ndarray, root: tuple[int, int], goal: tuple[int, int]):
        # Time to implement RRT*
        
        # This is the main function of the class

        # So first step get the 
        # First step initialization
        # Each node is going to be stored as [(x, y), cost, parent]
        nodes = [[root, 0, None]]
        col, row = occ_grid.shape
        for i in range(self.max_iterations):   
            # Sample a random point
            random_point = (
                np.random.randint(0, row),
                np.random.randint(0, col)
            )

            # Find the nearest node
            distances = [math.dist(random_point, node[0]) for node in nodes]
            least_distance_index = np.argmin(distances)
            closest_node = nodes[least_distance_index]

            # Check the distance threshold and get the new node
            new_node_distance = int(math.dist(random_point, closest_node[0]))
            new_node_cost = closest_node[1] + new_node_distance
            new_node = [random_point, new_node_cost, closest_node] if new_node_distance <= self.max_dist else [(
                self.max_dist * (random_point[0] - closest_node[0][0]) / math.dist(random_point, closest_node[0]) + closest_node[0][0],
                self.max_dist * (random_point[1] - closest_node[0][1]) / math.dist(random_point, closest_node[0]) + closest_node[0][1]
            ), new_node_cost, closest_node]

            # Check if the new node is in free space
            if self.obstacle_free(occ_grid, closest_node, new_node):
                # Or we can use KDTree to find nearest neighbour
                rrt_tree = KDTree([
                    node[0] for node in nodes
                ])
                indices  = rrt_tree.query_ball_point(new_node[0], r=self.search_radius)
                near = [nodes[i] for i in indices]

                min_node = closest_node
                min_cost = closest_node[1] + new_node_distance

                # Iterate through all the near nodes to see if any of them minimize the cost
                for node in near:
                    if self.obstacle_free(occ_grid, node, new_node) and (new_min_cost := node[1] + int(math.dist(node[0], new_node[0]))) < min_cost:
                        min_node = node
                        min_cost = node[1] + new_min_cost

                # Add the new_node to nodes
                new_node[2] = min_node
                new_node[1] = min_cost
                nodes.append(new_node)

                # Apparently I have to rewire the tree (REWIRE YOUR BRAIN JUMPSCARE)
                for node in near:
                    if self.obstacle_free(occ_grid, node, new_node):
                        cost = new_node[1] + int(math.dist(node[0], new_node[0]))
                        if cost < node[1]:
                            node[2] = new_node
                            node[1] = cost

                # Check if we reached the goal
                if int(math.dist(new_node[0], goal)) <= self.goal_threshold:
                    print(f"Minimum cost to goal so far: {new_node[1]}")
                    return self.construct_path(nodes)

        # If we reach here then the max iterations are less
        return KeyError("The max iterations are not enough")
                

    # def construct_path(self, nodes):
    #     path = []
    #     current_node = nodes[-1]
    #     while current_node is not None:
    #         path.append(current_node[0])
    #         current_node = current_node[2]
    #     path.reverse()
    #     return path


    # def obstacle_free(self, occ_grid, node1, node2):
    #     # So we have to check if the line joining node 1 to node 2 is free from obstacles.
    #     # The bresenham function here returns all the points that lie on the line from node1 to node2
    #     line = self.thick_bresenham_line(node1, node2)

    #     # Check if line is safe
    #     is_line_safe = all(occ_grid[y, x] == 0 for x, y in line)

    #     # Return True only if the line is obstacle free
    #     return is_line_safe and occ_grid[node1[0][1], node1[0][0]] == 0 and occ_grid[node2[0][1], node2[0][0]] == 0
    

    # def get_bresenham_line(self, node1, node2):
    #     """Return a list of integer coordinates on the line from (x0, y0) to (x1, y1)."""
    #     x0, y0 = node1[0]
    #     x1, y1 = node2[0]
    #     points = []

    #     dx = abs(x1 - x0)
    #     dy = abs(y1 - y0)
    #     x, y = x0, y0
    #     sx = 1 if x1 > x0 else -1
    #     sy = 1 if y1 > y0 else -1
    #     if dx > dy:
    #         err = dx / 2.0
    #         while x != x1:
    #             points.append((x, y))
    #             err -= dy
    #             if err < 0:
    #                 y += sy
    #                 err += dx
    #             x += sx
    #     else:
    #         err = dy / 2.0
    #         while y != y1:
    #             points.append((x, y))
    #             err -= dx
    #             if err < 0:
    #                 x += sx
    #                 err += dy
    #             y += sy
    #     points.append((x, y))
    #     return points
    

    # def thick_bresenham_line(self, node1, node2):
    #     # Get bresenham line
    #     line = self.get_bresenham_line(node1, node2)
    #     thick_points = set()

    #     for x, y in line:
    #         for dx in range(-self.line_thickness, self.line_thickness + 1):
    #             for dy in range(-self.line_thickness, self.line_thickness + 1):
    #                 if dx ** 2 + dy ** 2 <= self.line_thickness ** 2:
    #                     thick_points.add((x + dx, y + dy))

    #     return thick_points



    # def visualize(self, occupancy_grid: np.ndarray, rrt_points=None):
    #     plt.figure(figsize=(6,6))
    #     plt.imshow(occupancy_grid, cmap="gray_r", origin="lower")
    #     plt.title("Binary occupancy grid")
    #     plt.xlabel('X (grid)')
    #     plt.ylabel('Y (grid)')
    #     plt.show()




    # def load_map(self, package_name="rrt_star", yaml_file="warehouse_map.yaml"):
    #     # Get absolute paths
    #     package_share = get_package_share_directory(package_name=package_name)
    #     yaml_path = os.path.join(package_share, "maps", yaml_file)

    #     with open(yaml_path, "r") as f:
    #         map_metadata = yaml.safe_load(f)

    #     # Extract metadata
    #     image_path = os.path.join(os.path.dirname(yaml_path), map_metadata['image'])
    #     resolution = map_metadata['resolution']
    #     origin = map_metadata['origin']
    #     negate = map_metadata.get('negate', 0)
    #     occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
    #     free_thresh = map_metadata.get('free_thresh', 0.196)

    #     # Load the PGM image, which is a grayscale image of the map generated by slam toolbox
    #     map_img = cv.imread(image_path, cv.IMREAD_UNCHANGED)
    #     if map_img is None:
    #         return FileNotFoundError(f"Map image not found at location: {image_path}")
        
    #     # The map apparently can be in a negate state:
    #     if negate: 
    #         map_img = 255 - map_img

    #     # Normalization
    #     map_img = map_img.astype(np.float32) / 255.0

    #     # Create the occupancy grid
    #     occupancy_grid = np.full(map_img.shape, 1, dtype=np.int8)  
    #     occupancy_grid[map_img > free_thresh] = 0                   # Free space
    #     occupancy_grid[map_img < occupied_thresh] = 1               # Occupied space

    #     # Return useful stuff
    #     return {
    #         "occupancy_grid": occupancy_grid,
    #         "resolution": resolution,           # Will help for conversions i suppose, lets see
    #         "origin": origin,                   # Will be required later for transformations   
    #         "width": map_img.shape[0],
    #         "height": map_img.shape[1]
    #     }


    #######
    # cv.imshow("binary map", binary)
        # cv.imshow("og map", pgm_map)
        # cv.imshow("cleaned", cleaned)
        # cv.imshow("visual", visual)
        # cv.imshow("Cropped", cropped_map)
        # cv.waitKey(0)


        # def dilate_obstacles(self, occ_grid: np.ndarray):
    #     dilated_size = 2 * self.dilation_radius + 1
    #     struct_element = np.ones((dilated_size, dilated_size), dtype=bool)
    #     dilated_obstacles_grid = binary_dilation(
    #         occ_grid.astype(bool), structure=struct_element
    #     ).astype(np.int8)
    #     return dilated_obstacles_grid

def main():
    obj = RRTStarPlanner()
    occupancy_grid = obj.get_occupancy_grid()
    print(occupancy_grid)
