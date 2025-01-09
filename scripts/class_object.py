import numpy as np
# Ball class to save the detection results
class Ball:
    def __init__(self, center_point, radius, color):
        self.center_point = center_point
        self.radius = radius
        self.color = color

# Cube class to save the detection results
class Cube:
    def __init__(self, center_point, side_length, color):
        self.center_point = center_point
        self.side_length = side_length
        self.color = color

# Dictionary with every color in hsv (is going to be adjusted when using real images)
color_dic_hsv = {
    "red1": [(0, 120, 70),(10, 255, 255)],
    "red2": [(170, 120,70),(180,255,255)],
    "orange":[(15,120,70),(25,255,255)],
    "yellow":[(20,120,70),(35,255,255)],
    "green":[(45,120,70),(70,255,255)],
    "blue":[(90,120,70),(120,255,255)],
    "purple":[(135,120,70),(145,255,255)],
}