import math

class Grid:
    def __init__(self, width, height):
        self.width = height
        self.height = width
        self.grid = [[50] * height for _ in range(width)]

    def update(self, lidar_reading):
        inc = lidar_reading.angle_increment
        cur_angle = lidar_reading.angle_min
        for i in range(len(lidar_reading.ranges)):
            x = lidar_reading.ranges[i] * math.sin(cur_angle)
            y = lidar_reading.ranges[i] * math.cos(cur_angle)

            # if abs(x - x // 1) > abs(y - y // 1): # shift x
            
            # TODO

            cur_angle += inc

        



