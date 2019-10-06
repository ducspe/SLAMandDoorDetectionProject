import re
import numpy as np
import os
import cv2


class Door_Position:

    f_x = 1.0569738651289942e+03
    f_y = 1.0565911865677338e+03
    c_x = 9.4051983105015063e+02
    c_y = 5.5005616989245004e+02

    def __init__(self, bounding_box, rgb_image, range):
        self.bounding_box = bounding_box
        self.rgb_image = rgb_image
        self.range = range
        # self.rgb_to_array = cv2.imread(rgb_image)
        self.rgb_to_array = rgb_image

    def cast_boundaries_to_int(self):
        max_clip = np.max(self.rgb_to_array.shape)
        return np.clip(self.bounding_box, a_min=0, a_max=max_clip).astype(np.uint16)

    def calculate_depth_distance(self):
        depth_len = len(self.range)
        img_row, img_height, _ = self.rgb_to_array.shape

        bounding_box_to_int = self.cast_boundaries_to_int()

        if depth_len == img_row:
            raise Exception('Number of depth rows do not match image rows')
            return None

        left_door_idx = bounding_box_to_int[1]
        right_door_idx = bounding_box_to_int[3]

        left_depth = self.range[left_door_idx]
        right_depth = self.range[right_door_idx]

        average_distance = (left_depth + right_depth)/2

        return average_distance

    def calculate_door_position(self):
        depth_distance = self.calculate_depth_distance()
        bounding_box_to_int = self.cast_boundaries_to_int()

        mid_x = int((bounding_box_to_int[3] - bounding_box_to_int[1])/2) + bounding_box_to_int[1]
        mid_y = int((bounding_box_to_int[2] - bounding_box_to_int[0])/2) + bounding_box_to_int[0]

        x_world = (mid_x - self.c_x) * depth_distance / self.f_x
        y_world = (mid_y - self.c_y) * depth_distance / self.f_y

        return x_world, y_world
