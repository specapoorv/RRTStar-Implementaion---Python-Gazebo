import math 
import random
import time

class node:
    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]
        self.parent = None
        self.cost = 0
        

class rrt():
    def __init__(self, start_pos, goal_pos, map_size, step_size, search_radius, goal_radius, occupancy_grid):
        self.start_node = node(start_pos)
        self.goal_node = node(goal_pos)
        self.map_size = map_size
        self.step_size = step_size
        self.search_radius = search_radius
        self.goal_radius = goal_radius
        self.occupancy_grid = occupancy_grid

        self.debug = False
        self.goal_reached = False
        self.node_tree = [self.start_node]

    def distance(self, n1, n2):
        return (n2.x - n1.x, n2.y - n1.y) #returns dx, dy
    

    def sample_point(self):
        if self.debug:
            time.sleep(0.25)
            print("sampling point")
        width = self.map_size[0]
        height = self.map_size[1]
        rand_x = random.randint(0, width)
        rand_y = random.randint(0, height)
        rand_node_pos = (rand_x, rand_y)
        rand_node = node(rand_node_pos)
        # print(rand_node.x)
        
        return rand_node

    def find_nearest_node(self, node):
        if self.debug:
            time.sleep(0.25)
            print("finding nearest node")
        nearest = self.node_tree[0]
        dx, dy = self.distance(nearest, node)
        min_dist = math.sqrt(dx**2 + dy**2)

        for n in self.node_tree[1:]:
            dx, dy = self.distance(n, node)
            euclidean_distance = math.sqrt(dx**2 + dy**2)

            if euclidean_distance < min_dist:
                nearest = n
                min_dist = euclidean_distance
        # print(nearest)
        return nearest
    
    def find_neighbour_in_search_radius(self, new_node):

        nearby_nodes = []
        for node in self.node_tree:
            dx, dy = self.distance(node, new_node)
            dist = math.sqrt(dx**2 + dy**2)
            if dist <= self.search_radius:
                nearby_nodes.append(node)
        return nearby_nodes


    def brancher(self, from_node, to_node):
        if self.debug:
            time.sleep(0.25)
            print("branching")
        dx, dy = self.distance(from_node, to_node)

        # distance = math.sqrt(dx**2 + dy**2)
        # if distance <= self.step_size:
            # return to_node  # already close enough
        #above snippet gives too many nodes without parents so we might be facing issues in creating path or smtg

        theta = math.atan2(dy, dx)
        new_x = int(from_node.x + self.step_size * math.cos(theta))
        new_y = int(from_node.y + self.step_size * math.sin(theta))


        new_node = node((new_x, new_y))
        # now the parent is decided based on cost and step size
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
        #write code here to get values from occupancy grid and check if 1 or 0 exist in the bresenham line cells
        # return a boolean True if no collision
        for x, y in cells:
        # Bounds check
            if x < 0 or x >= len(self.occupancy_grid[0]) or y < 0 or y >= len(self.occupancy_grid):
                return False
            if self.occupancy_grid[y][x] == 1: 
                return False  # Collision detected

        return True  # All clear

    def check_if_goal_reached(self):
        if self.debug:
            time.sleep(0.25)
            print("checking if goal reached")
        self.nearest_to_goal_node = self.find_nearest_node(self.goal_node)
        dx, dy = self.distance(self.nearest_to_goal_node, self.goal_node)
        dist = math.sqrt(dx**2 + dy**2)
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
                
            self.nearest_to_goal_node = self.goal_node

    def is_valid_node(self, node):
        x, y = int(node.x), int(node.y)
        if x < 0 or x >= self.map_size[0] or y < 0 or y >= self.map_size[1]:
            return False
        if self.occupancy_grid[y][x] == 1:
            return False
        return True

    def check_if_node_exists(self, node):
        for n in self.node_tree:
            if node.x == n.x and node.y == n.y:
                return False
        
        return True


    def rewire_tree(self):
        '''
        write code here to rewire the nodes
        '''
            

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
            check_if_node_exists = self.check_if_node_exists(new_node)

            if no_collision_path == True and is_valid_node == True and check_if_node_exists == True:
                self.node_tree.append(new_node)

            
            else:
                pass

        else:
            self.path_generator()

    def path_generator(self):
        print('generating path')
        path = []
        current = self.nearest_to_goal_node

        #fill code here

        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        path.reverse()

        return path 



def main():
    start_pos = (0, 0)
    goal_pos = (13, 5)
    map_size = (20, 17)
    step_size = 1.0
    search_radius = 2.0
    goal_radius = 1.0
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


    code = rrt(start_pos, goal_pos, map_size, step_size, search_radius, goal_radius, occupancy_grid)
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
        #if i % 5 == 0:
         #   print('sleeping')
            
            # time.sleep(0.1)
            
        code.check_if_goal_reached()
        if code.goal_reached:
            path = code.path_generator()
            break
        code.develop_tree()

    print("\nPath:", path)
    if path is not None:
        visualize_path(occupancy_grid, path, start_pos, goal_pos)

import matplotlib.pyplot as plt
import numpy as np

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


if __name__ == "__main__":
    main()
    



