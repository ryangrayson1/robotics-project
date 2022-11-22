import math
from math import floor

class Grid:
    def __init__(self, width, height):
        self.updates = 0
        self.width = width
        self.height = height
        self.grid = [[50] * width for _ in range(height)]

    def update(self, drone_position, lidar_reading):
        inc = lidar_reading.angle_increment
        cur_angle = lidar_reading.angle_min
        for i in range(len(lidar_reading.ranges)):
            x1 = drone_position.x
            y1 = drone_position.y
            x2 = lidar_reading.range_max * math.sin(cur_angle)
            y2 = lidar_reading.range_max * math.cos(cur_angle)

            # shift each x and y value half a cell up and to the right
            x1 += 0.5
            y1 += 0.5
            x2 += 0.5
            y2 += 0.5

            cells_crossed = self.raytrace((x1, y1), (x2, y2), max_dist=lidar_reading.range_max)
            j = 0
            white_cells = []
            while j < len(cells_crossed) and cells_crossed[j][2] < lidar_reading.ranges[i]:
                white_cells.append(cells_crossed[j])
                j += 1
            black_cell = None
            if (j < len(cells_crossed)):
                black_cell = cells_crossed[j]
            
            # mark the white cells as white
            for white_cell in white_cells:
                grid_x = white_cell[0] + math.floor(self.width / 2)
                grid_y = white_cell[1] + math.floor(self.height / 2)
                self.grid[grid_y][grid_x] -= 5
                self.grid[grid_y][grid_x] = max(self.grid[grid_y][grid_x], 0)

            
            # mark the black cells as black
            grid_x = black_cell[0] + math.floor(self.width / 2)
            grid_y = black_cell[1] + math.floor(self.height / 2)
            self.grid[grid_y][grid_x] += 5
            self.grid[grid_y][grid_x] = min(self.grid[grid_y][grid_x], 100)

            cur_angle += inc
        
        self.updates += 1

    # Source: https://stackoverflow.com/questions/35807686/find-cells-in-array-that-are-crossed-by-a-given-line-segment
    def sign(self, n):
        return (n > 0) - (n < 0)

    # Source: https://stackoverflow.com/questions/35807686/find-cells-in-array-that-are-crossed-by-a-given-line-segment
    def raytrace(self, A, B, max_dist=5):
        """ Return all cells of the unit grid crossed by the line segment between
            A and B.
        """
        dx = B[0] - A[0]
        dy = B[1] - A[1]

        direction_x = self.sign(dx)
        direction_y = self.sign(dy)

        direction_modifier_x = 0 if direction_x < 0 else direction_x
        direction_modifier_y = 0 if direction_y < 0 else direction_y

        currentCell = {"x": math.floor(A[0]), "y": math.floor(A[1])}
        targetCell = {"x": math.floor(B[0]), "y": math.floor(B[1])}

        traversed = [(currentCell["x"], currentCell["y"], 0)]
        intersect = (0, 0)

        calcIntersectionDistanceX = lambda: abs(dy * (currentCell["x"] + direction_modifier_x - A[0]))
        calcIntersectionDistanceY = lambda: abs(dx * (currentCell["y"] + direction_modifier_y - A[1]))

        intersection_distance_x = float("+inf") if dx == 0 else calcIntersectionDistanceX()
        intersection_distance_y = float("+inf") if dy == 0 else calcIntersectionDistanceY()

        while ((targetCell["x"] != currentCell["x"] or targetCell["y"] != currentCell["y"]) and math.dist(A, intersect) < max_dist):
            movx = intersection_distance_x <= intersection_distance_y
            movy = intersection_distance_y <= intersection_distance_x
        
            if movx:
                currentCell["x"] += direction_x
                intersect = (currentCell["x"] + 1 - direction_modifier_x, A[1] + intersection_distance_x / dx)
                intersection_distance_x = calcIntersectionDistanceX()
            
            if movy:
                currentCell["y"] += direction_y
                intersect = (A[0] + intersection_distance_y / dy, currentCell["y"] + 1 - direction_modifier_y)
                intersection_distance_y = calcIntersectionDistanceY()
            
            traversed.append((currentCell["x"], currentCell["y"], math.dist(A, intersect)))

        return traversed
        
if __name__ == "__main__":
    grid = Grid(10, 10)
    print(grid.raytrace((0, 0), (-10, 1)))
