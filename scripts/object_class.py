class Ball:
    def __init__(self, center_point, radius, color):
        self.center_point = center_point
        self.radius = radius
        self.color = color

class Cube:
    def __init__(self, center_point, side_length, color):
        self.center_point = center_point
        self.side_length = side_length
        self.color = color

# example
kugel = Ball((3,4),5,"red")