import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
import cv2 as cv
from scipy.spatial import KDTree
import math


class Map_getter:

    def __init__(self):
        self.max_dist = 10
        self.max_iterations = 100
        self.dilation_radius = 1
        self.search_radius = 20
        self.goal_threshold = 3
        self.line_thickness = 3


    def get_map_metadata(self, yaml_file="/home/adityabhatewara/issac_rover_urdf/src/warehouse_map.yaml"):
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
        largest_contour = contours[1] 					### Largest countour is hardcoded change this later ###
        cv.drawContours(visual, [largest_contour], -1, (0, 255, 0), 2)

        # contours[1] is the largest contour
        # Get bounding rect
        x, y, w, h = cv.boundingRect(largest_contour)
        cv.rectangle(visual, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # Crop image till bounding rect
        cropped_map = cleaned[y: y + h, x: x + w]

        
        # This occupacny grid is too detailed to downsample
        block_size = 3
        new_width = max(1, cropped_map.shape[1] // block_size)
        new_height = max(1, cropped_map.shape[0] // block_size)

        downsampled = cv.resize(
            cropped_map,
            (new_width, new_height)
        )

        occupancy_grid = np.where(downsampled == 0, 1, 0)
        # downsampled = cv.resize(occupancy_grid, (occupancy_grid.shape[1] // block_size, occupancy_grid.shape[0] // block_size), interpolation=cv.INTER_NEAREST)
        return occupancy_grid  # change it later if you want lists

    
    

# def main():
#     obj = Map_getter()
#     occupancy_grid = obj.get_occupancy_grid()
#     print(occupancy_grid)
#     # obj = Map_getter()
#     # occupancy_grid = obj.get_occupancy_grid()

#     # print(f"[INFO] Occupancy Grid Shape: {occupancy_grid.shape}")
#     # plt.imshow(occupancy_grid, cmap='gray')
#     # plt.title("Preprocessed Occupancy Grid")
#     # plt.show()
# main()
