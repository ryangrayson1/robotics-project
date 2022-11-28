import math
from tf.transformations import euler_from_quaternion

class MismatchedLengthsError(Exception):
    """Raised when you attempt to find the distance between two points of different dimensions"""
    pass

class Grid:
    def __init__(self, width, height):
        self.updates = 0
        self.width = width
        self.height = height
        self.grid = [[50] * width for _ in range(height)]

    def update(self, drone_pose, lidar_reading):
        inc = lidar_reading.angle_increment
        cur_angle = lidar_reading.angle_min
        for i in range(len(lidar_reading.ranges)):
            x1 = drone_pose.position.x
            y1 = drone_pose.position.y
            x2 = lidar_reading.range_max * math.cos(cur_angle)
            y2 = lidar_reading.range_max * math.sin(cur_angle)

            # shift each set of coords half a cell up and to the right since drone is starting in the middle of a cell
            # at (0, 0), but we want coordinates to indicate the bottom right of a cell.
            x1 += 0.5
            y1 += 0.5
            x2 += 0.5
            y2 += 0.5

            # cells crossed by line segment of length range_max (default 5) at current angle
            cells_crossed = self.raytrace((x1, y1), (x2, y2), max_dist=lidar_reading.range_max)

            # of the cells crossed, some will be white, exactly one or fewer will be black, and the remainder are unknown
            j = 0
            white_cells = []
            while j < len(cells_crossed) and cells_crossed[j][2] < lidar_reading.ranges[i]:
                white_cells.append(cells_crossed[j])
                j += 1
            
            # if the measured distance is only slightly greater than the distance to some cell, then that cell is probably black
            if j < len(cells_crossed) and abs(cells_crossed[j][2] - lidar_reading.ranges[i]) > abs(cells_crossed[j-1][2] - lidar_reading.ranges[i]):
                white_cells.pop(-1)
                j -= 1

            # black cell only exists if lidar was interrupted before range_max
            black_cell = None
            if j < len(cells_crossed):
                black_cell = cells_crossed[j]
            
            # mark the white cells as white (make them whiter)
            for white_cell in white_cells:
                grid_x = int(math.floor(self.width / 2)) + white_cell[0]
                grid_y = int(math.floor(self.height / 2)) - white_cell[1]
                self.grid[grid_y][grid_x] -= 5
                self.grid[grid_y][grid_x] = max(self.grid[grid_y][grid_x], 0)

            
            # mark the black cell as black (make it blacker)
            if black_cell:
                grid_x = int(math.floor(self.width / 2)) + black_cell[0]
                grid_y = int(math.floor(self.height / 2)) - black_cell[1]
                self.grid[grid_y][grid_x] += 5
                self.grid[grid_y][grid_x] = min(self.grid[grid_y][grid_x], 100)

            print("Finished update!")
            print("Drone position:  " + str(drone_pose.position))
            print("Angle min:       " + str(lidar_reading.angle_min * 57.2958))
            print("Cur angle:       " + str(cur_angle * 57.2958))
            print("Lidar distance:  " + str(lidar_reading.ranges[i]))
            print("Cells crossed:   " + str(cells_crossed))
            print("White cells:     " + str(white_cells))
            print("Black cell:      " + str(black_cell))
            self.print_grid()

            cur_angle += inc
        
        self.updates += 1

    # Source: https://stackoverflow.com/questions/35807686/find-cells-in-array-that-are-crossed-by-a-given-line-segment
    def sign(self, n):
        return (n > 0) - (n < 0)

    def dist(self, p1, p2):
        if len(p1) != len(p2):
            raise MismatchedLengthsError

        total = 0
        for i in range(len(p1)):
            total += (p1[i] - p2[i]) ** 2
        return math.sqrt(total)

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

        currentCell = {"x": int(math.floor(A[0])), "y": int(math.floor(A[1]))}
        targetCell = {"x": int(math.floor(B[0])), "y": int(math.floor(B[1]))}

        traversed = [(currentCell["x"], currentCell["y"], 0)]
        intersect = (0, 0)

        calcIntersectionDistanceX = lambda: abs(dy * (currentCell["x"] + direction_modifier_x - A[0]))
        calcIntersectionDistanceY = lambda: abs(dx * (currentCell["y"] + direction_modifier_y - A[1]))

        intersection_distance_x = float("+inf") if dx == 0 else calcIntersectionDistanceX()
        intersection_distance_y = float("+inf") if dy == 0 else calcIntersectionDistanceY()

        while ((targetCell["x"] != currentCell["x"] or targetCell["y"] != currentCell["y"]) and self.dist(A, intersect) < max_dist):
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
            
            traversed.append((currentCell["x"], currentCell["y"], self.dist(A, intersect)))

        return traversed

    def print_grid(self):
        grid_string = ""
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i][j] > 50:
                    grid_string = grid_string + "#"
                elif self.grid[i][j] < 50:
                    grid_string = grid_string + " "
                else:
                    grid_string = grid_string + "-"
            grid_string = grid_string + "\n"
        print(grid_string)
        
if __name__ == "__main__":
    grid = Grid(10, 10)
    print(grid.raytrace((0.5, 0.5), (1.5, 1.1)))
