import math
from math import floor

class Grid:
    def __init__(self, width, height):
        self.updates = 0
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
        
        self.updates += 1

    # Source: https://stackoverflow.com/questions/35807686/find-cells-in-array-that-are-crossed-by-a-given-line-segment
    def sign(self, n):
        return (n > 0) - (n < 0)

    def dist(self, A, B):
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)

    # Source: https://stackoverflow.com/questions/35807686/find-cells-in-array-that-are-crossed-by-a-given-line-segment
    def raytrace(self, A, B):
        """ Return all cells of the unit grid crossed by the line segment between
            A and B.
        """

        (xA, yA) = A
        (xB, yB) = B
        (dx, dy) = (xB - xA, yB - yA)
        (sx, sy) = (self.sign(dx), self.sign(dy))

        grid_A = (floor(A[0]), floor(A[1]))
        grid_B = (floor(B[0]), floor(B[1]))
        (x, y) = grid_A
        traversed=[grid_A + (0, )]
        intersection = (0, 0)

        tIx = dy * (x + sx - xA) if dx != 0 else float("+inf")
        tIy = dx * (y + sy - yA) if dy != 0 else float("+inf")
        print(tIx, tIy)
        print(dx, dy)

        while (x,y) != grid_B:
            # NB if tIx == tIy we increment both x and y
            (movx, movy) = (tIx <= tIy, tIy <= tIx)

            if movx:
                # intersection is at (x + sx, yA + tIx / dx^2)
                intersection = (float(x + sx), yA + tIx / abs(dx))
                x += sx
                tIx = dy * (x + sx - xA)

            if movy:
                # intersection is at (xA + tIy / dy^2, y + sy)
                intersection = (xA + tIy / abs(dy), float(y + sy))
                y += sy
                tIy = dx * (y + sy - yA)
            
            print(intersection)
            traversed.append((x,y, self.dist(A, intersection)))

        return traversed
        
if __name__ == "__main__":
    grid = Grid(10, 10)
    print(grid.raytrace((0, 0), (1, -2)))
