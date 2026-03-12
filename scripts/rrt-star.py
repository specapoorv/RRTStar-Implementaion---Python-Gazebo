import math 
import random
import time
import matplotlib.pyplot as plt
import numpy as np
# from rrt_star_planner_copy import RRTStarPlanner

class Node:
    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]
        self.parent = None
        self.cost = 0
        

class RRT:
    def __init__(self, start_pos, goal_pos, map_size, step_size, search_radius, goal_radius, occupancy_grid):
        self.start_node = Node(start_pos)
        self.goal_node = Node(goal_pos)
        self.map_size = map_size
        self.step_size = step_size
        self.search_radius = search_radius
        self.goal_radius = goal_radius
        self.occupancy_grid = occupancy_grid

        self.debug = False
        self.goal_reached = False
        self.node_tree = [self.start_node]

    def distance(self, n1, n2):
        return (dx := (n2.x - n1.x), dy := (n2.y - n1.y)), np.linalg.norm((dx, dy)) #returns dx, dy
    

    def sample_point(self):
        if self.debug:
            time.sleep(0.25)
            print("sampling point")
        width = self.map_size[0]
        height = self.map_size[1]
        rand_x = random.randint(0, width)
        rand_y = random.randint(0, height)
        rand_node_pos = (rand_x, rand_y)
        rand_node = Node(rand_node_pos)
        return rand_node
    

    def find_nearest_node(self, node):
        if self.debug:
            time.sleep(0.25)
            print("finding nearest node")
        nearest = self.node_tree[0]
        (_, _), min_dist = self.distance(nearest, node)

        for n in self.node_tree[1:]:
            (_, _), euclidean_distance = self.distance(n, node)

            if euclidean_distance < min_dist:
                nearest = n
                min_dist = euclidean_distance

        return nearest
    
    
    def find_neighbour_in_search_radius(self, new_node):

        nearby_nodes = []
        for node in self.node_tree:
            (_, _), dist = self.distance(node, new_node)

            if dist <= self.step_size:
                nearby_nodes.append(node)

        return nearby_nodes
    

    def rewire_tree(self, new_node):
    
        nearby_nodes = self.find_neighbour_in_search_radius(new_node)

        for n in nearby_nodes:
            if not self.is_valid_node(n):  # <- Prevent rewiring to a bad node
                continue

            cost = new_node.cost + np.linalg.norm(
                [n.x - new_node.x, n.y - new_node.y]
            )

            if self.debug:
                time.sleep(1)
                print(f"checking if ({n.x},{n.y}) collison free with ({new_node.x},{new_node.y})")

            if cost < n.cost and self.check_if_collision_free(new_node, n) == True:
                (_, _), dist = self.distance(new_node, n)

                if dist <= self.step_size:  
                    n.parent = new_node
                    n.cost = cost


    def brancher(self, from_node, to_node):
        if self.debug:
            time.sleep(0.25)
            print("branching")
        (dx, dy), _ = self.distance(from_node, to_node)

        # distance = math.sqrt(dx**2 + dy**2)
        # if distance <= self.step_size:
            # return to_node  # already close enough
        #above snippet gives too many nodes without parents so we might be facing issues in creating path or smtg

        theta = math.atan2(dy, dx)
        new_x = int(from_node.x + self.step_size * math.cos(theta))
        new_y = int(from_node.y + self.step_size * math.sin(theta))

        new_node = Node((new_x, new_y))
        # now the parent is decided based on cost and step size
        # modularise this and put it outside brancher
        nearby_nodes = self.find_neighbour_in_search_radius(new_node)
        min_cost = from_node.cost + self.step_size
        best_parent = from_node
        
        for n in nearby_nodes:
            current_cost = n.cost + self.step_size
            if current_cost < min_cost:
                min_cost = current_cost
                best_parent = n
        
        new_node.parent = best_parent
        new_node.cost = min_cost
        return new_node 
          

    def bresenham_line(self, x0, y0, x1, y1):
        #chatgpted but theoritically correct
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx // 2
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy // 2
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        cells.append((x1, y1))

        if self.debug == True:
            time.sleep(0.5)
            print(f"[debug] returned cells fro  bresenham line = {cells}")
        return cells
    

    def check_if_collision_free(self, from_node, to_node):
        if self.debug:
            time.sleep(0.25)
            print('checking if collsion free')
        
        x0, y0 = int(round(from_node.x)), int(round(from_node.y))
        x1, y1 = int(round(to_node.x)), int(round(to_node.y))

        cells = self.bresenham_line(x0, y0, x1, y1)
        # Write code here to get values from occupancy grid and check if 1 or 0 exist in the bresenham line cells
        # return a boolean True if no collision
        for x, y in cells:
        # Bounds check
            if x < 0 or x >= len(self.occupancy_grid[0]) or y < 0 or y >= len(self.occupancy_grid) or self.occupancy_grid[y, x] == 1:
                return False  # Collision detected

        return True  # All clear


    def check_if_goal_reached(self):
        if self.debug:
            time.sleep(0.25)
            print("checking if goal reached")
        self.nearest_to_goal_node = self.find_nearest_node(self.goal_node)
        (_, _), dist = self.distance(self.nearest_to_goal_node, self.goal_node)

        if dist < self.goal_radius:
            self.goal_reached = True
            self.goal_node.parent = self.nearest_to_goal_node
            self.goal_node.cost = self.nearest_to_goal_node.cost + dist

            self.node_tree.append(self.goal_node)
            if self.debug:
                time.sleep(1)
                for i in self.node_tree:
                    print(i.x, i.y)
                    time.sleep(0.5)
                

    def is_valid_node(self, node: Node):
        x, y = int(node.x), int(node.y)
        if x < 0 or x >= self.map_size[0] or y < 0 or y >= self.map_size[1]:
            return False
        if self.occupancy_grid[y][x] == 1:
            return False
        return True
    

    def check_if_node_doesnt_exists(self, node: Node):
        for n in self.node_tree:
            if node.x == n.x and node.y == n.y:
                return False
        
        return True        
        
            
    def develop_tree(self):
        
        if self.debug:
            time.sleep(0.25)
            print("developing tree")
        if not self.goal_reached:
            rand_node = self.sample_point()
            nearest_node = self.find_nearest_node(rand_node)

            new_node = self.brancher(nearest_node, rand_node)

            no_collision_path = self.check_if_collision_free(nearest_node, new_node)
            is_valid_node = self.is_valid_node(new_node)
            check_if_node_exists = self.check_if_node_doesnt_exists(new_node)

            if no_collision_path == True and is_valid_node == True and check_if_node_exists == True:
                self.node_tree.append(new_node)
                self.rewire_tree(new_node)

        else:
            # self.path_generator()
            pass

    def path_generator(self):
        print('generating path')
        path = []
        current = self.nearest_to_goal_node

        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        path.reverse()
        return path 
    



def visualize_path(occupancy_grid, path, start, goal):
    grid = np.array(occupancy_grid)

    plt.figure(figsize=(6, 6))
    plt.imshow(grid, cmap='Greys', origin='upper')

    # Extract x and y from path
    if path:
        x_coords, y_coords = zip(*path)
        plt.plot(x_coords, y_coords, color='blue', linewidth=2, label='RRT Path')

    # Mark start and goal
    plt.scatter(*start, color='green', s=100, label='Start')
    plt.scatter(*goal, color='red', s=100, label='Goal')

    plt.title("RRT Path on Occupancy Grid")
    plt.legend()
    plt.grid(True)
    plt.xticks(np.arange(0, len(occupancy_grid[0]), 1))
    plt.yticks(np.arange(0, len(occupancy_grid), 1))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def main():

    # planner = RRTStarPlanner()
    # occupancy_grid = planner.get_occupancy_grid()

    start_pos = (0, 0)
    goal_pos = (13, 2)
    map_size = (20, 17)
    step_size = 10.0
    search_radius = 20.0
    goal_radius = 5.0
    # occupancy_grid = [
    #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
    #     [0, 0, 1, 1, 1, 0, 1, 0, 1, 0],
    #     [0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
    #     [1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    #     [0, 0, 1, 0, 1, 0, 0, 0, 1, 0],
    #     [0, 0, 1, 0, 1, 1, 1, 1, 1, 0],
    #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    #     ]

    occupancy_grid = [
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0],
        [0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0],
        [1,1,1,0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0],
        [0,0,1,1,1,0,1,0,1,0,0,0,1,1,1,0,1,0,1,0],
        [0,0,0,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0],
        [0,0,1,0,1,0,0,0,1,0,0,1,1,0,1,0,0,0,1,0],
        [0,0,1,0,1,1,1,1,1,0,0,1,0,0,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0],
        [0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0],
        [0,0,1,1,1,0,1,0,1,0,0,0,1,1,1,0,1,0,1,0],
        [0,0,0,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0],
        [1,1,1,0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0],
        [0,0,1,0,1,0,0,0,0,0,1,1,1,0,0,0,0,1,1,1]]


    code = RRT(start_pos, goal_pos, map_size, step_size, search_radius, goal_radius, occupancy_grid)
    # code.sample_point()
    path = None

    if not code.is_valid_node(code.start_node):
        print("start node itself is invalid bro")
        return

    if not code.is_valid_node(code.goal_node):
        print('r u dumb, give me a goal thats not obstacle!')
        return
        
    code.develop_tree()
    max_iterations = 100000
    print("iterating now")
    for i in range(max_iterations):
        print(f"iteration : {i}")

        code.develop_tree()
        code.check_if_goal_reached()

    if code.goal_reached:
        path = code.path_generator()
    else:
        print("sorry couldnt find an optimal path, increase iterations or goal radius")

    print("\nPath:", path)
    if path is not None:
        visualize_path(occupancy_grid, path, start_pos, goal_pos)


if __name__ == "__main__":
    main()
    

