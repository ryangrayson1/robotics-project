import math
import copy
import numpy as np
from collections import deque
from itertools import chain
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3, Pose, Point
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData

class MismatchedLengthsError(Exception):
    """Raised when you attempt to find the distance between two points of different dimensions"""
    pass

class Grid:
    def __init__(self, width, height):
        self.updates = 0
        self.width = width
        self.height = height
        self.grid = [[50] * width for _ in range(height)]
        self.last_measures = [[None] * width for _ in range(height)]
        self.current_measures = None
        self.times_diff_measured = [[0] * width for _ in range(height)]
        self.average_diffs = [[0] * width for _ in range(height)]
        self.free_threshold = 70
        self.door_threshold = 0.04
        self.doors = []
    
    # assumes fully raw position as input, such as from the dog position
    def world_to_grid(self, world_pos):
        world_x_shifted = round(world_pos.x + 0.5, 4)
        world_y_shifted = round(world_pos.y + 0.5, 4)
        grid_x = int(self.width / 2) + int(math.floor(world_x_shifted))
        grid_y = int(self.height / 2) - int(math.floor(world_y_shifted))
        return grid_x, grid_y
    
    # assumes integer input coordinates like those returned from world_to_grid
    def grid_to_world(self, grid_pos):
        world_x_shifted = int(grid_pos[0]) - int(self.width / 2)
        world_y_shifted = int(self.height / 2) - int(grid_pos[1])
        world_x = world_x_shifted - 0.5
        world_y = world_y_shifted - 0.5
        return world_x, world_y
    
    def set_cell(self, x, y, val):
        self.grid[y][x] = val

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height
    
    def can_travel(self, x, y):
        if self.in_bounds(x, y): # if less than 0, it is a door or the dog
            return -3 <= self.grid[y][x] <= self.free_threshold
        return False

    def is_closed_door(self, x, y):
        # return True
        return self.grid[y][x] == -1

    def update(self, drone_pose, lidar_reading):
        inc = lidar_reading.angle_increment
        cur_angle = lidar_reading.angle_min
        self.current_measures = [[None] * self.width for _ in range(self.height)]
        for i in range(len(lidar_reading.ranges)):
            x1 = drone_pose.position.x
            y1 = drone_pose.position.y
            x2 = x1 + lidar_reading.range_max * math.cos(cur_angle)
            y2 = y1 + lidar_reading.range_max * math.sin(cur_angle)

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
            if (lidar_reading.ranges[i] < lidar_reading.range_max 
                and (j == len(cells_crossed) 
                or abs(cells_crossed[j][2] - lidar_reading.ranges[i]) > abs(cells_crossed[j-1][2] - lidar_reading.ranges[i]))):
                white_cells.pop(-1)
                j -= 1

            # black cell only exists if lidar was interrupted before range_max
            black_cell = None
            if lidar_reading.ranges[i] < lidar_reading.range_max:
                black_cell = cells_crossed[j]
            
            # mark the white cells as white (make them whiter)
            for white_cell in white_cells:
                grid_x = int(math.floor(self.width / 2)) + white_cell[0]
                grid_y = int(math.floor(self.height / 2)) - white_cell[1]
                if self.grid[grid_y][grid_x] >= 0:
                    self.grid[grid_y][grid_x] -= 3
                    self.grid[grid_y][grid_x] = max(self.grid[grid_y][grid_x], 0)

            # mark the black cell as black (make it blacker)
            if black_cell:
                grid_x = int(math.floor(self.width / 2)) + black_cell[0]
                grid_y = int(math.floor(self.height / 2)) - black_cell[1]
                if (self.grid[grid_y][grid_x] >= 0):
                    self.grid[grid_y][grid_x] += 3
                    self.grid[grid_y][grid_x] = min(self.grid[grid_y][grid_x], 100)

                # Door detection logic
                # Observe and record the average difference in distance between this measurement and the previous measurement of the same tile.
                if self.last_measures[grid_y][grid_x] and self.last_measures[grid_y][grid_x][i]:
                    # relative_position = x2 - 
                    diff = abs(self.last_measures[grid_y][grid_x][i] - lidar_reading.ranges[i])
                    self.average_diffs[grid_y][grid_x] = ((self.average_diffs[grid_y][grid_x] * self.times_diff_measured[grid_y][grid_x] + diff) 
                        / (self.times_diff_measured[grid_y][grid_x] + 1))

                    if (self.grid[grid_y][grid_x] >= self.free_threshold 
                        and self.average_diffs[grid_y][grid_x] > self.door_threshold 
                        and self.grid[grid_y][grid_x] != -4
                        and self.grid[grid_y][grid_x] != -2
                        and self.times_diff_measured[grid_y][grid_x] >= 10):
                        self.grid[grid_y][grid_x] = -1
                    elif self.grid[grid_y][grid_x] == -1 and self.average_diffs[grid_y][grid_x] < self.door_threshold:
                        self.grid[grid_y][grid_x] = 100

                    self.times_diff_measured[grid_y][grid_x] += 1
                
                if self.current_measures[grid_y][grid_x] is None:
                    self.current_measures[grid_y][grid_x] = [None] * len(lidar_reading.ranges)
                self.current_measures[grid_y][grid_x][i] = lidar_reading.ranges[i]

            cur_angle += inc
        
        self.last_measures = copy.deepcopy(self.current_measures)
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
        intersect = (A[0], A[1])

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

    def print_average_diffs(self):
        grid_string = "  "
        for i in range(len(self.average_diffs[1])):
            grid_string += str(i) + " " * (5 - len(str(i)))
        grid_string += "\n"
        for i in range(len(self.average_diffs)):
            grid_string += str(i) + " "
            for j in range(len(self.average_diffs[1])):
                if self.grid[i][j] > self.free_threshold:
                    grid_string += "{:.2f}".format(self.average_diffs[i][j])
                else:
                    grid_string += "----"
                grid_string += " "
            grid_string = grid_string + "\n"
        print(grid_string)
    
    def get_grid_to_publish(self):
        m = MapMetaData()
        m.width = self.width
        m.height = self.height
        m.resolution = 1
        pos = np.array([-self.width * m.resolution / 2 + .5, -self.height * m.resolution / 2 + .5, 0])
        m.origin = Pose()
        m.origin.position.x, m.origin.position.y = pos[:2]
        og = OccupancyGrid()
        og.info = m
        
        for c in range(self.width):
            for r in range(self.height-1, -1, -1):
                if self.grid[r][c] == -4:
                    og.data.append(100)
                elif self.grid[r][c] == -5:
                    og.data.append(0)
                else:
                    og.data.append(self.grid[r][c])

        return og
    
    def get_shortest_path(self, dog_pos):
        start_x, start_y = int(self.width / 2), int(self.height / 2)
        dog_x, dog_y = self.world_to_grid(dog_pos)
        came_from = {(start_x, start_y): None}
        q = deque([(start_x, start_y)])
        door_num = 0
        while q:
            x, y = q.popleft()
            if door_num < 3 and (x, y) == self.doors[door_num]:
                door_num += 1
                q.clear()
            if x == dog_x and y == dog_y:
                break
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nxt_x, nxt_y = x + dx, y + dy
                if (nxt_x, nxt_y) not in came_from and (self.can_travel(nxt_x, nxt_y) or (self.in_bounds(nxt_x, nxt_y) and self.grid[nxt_y][nxt_x] == -5)):
                    came_from[(nxt_x, nxt_y)] = (x, y)
                    q.append((nxt_x, nxt_y))
        
        grid_path = [(dog_x, dog_y)]
        while grid_path[-1] != (start_x, start_y):
            grid_path.append(came_from[grid_path[-1]])
        grid_path.reverse()

        # print("shortest path (grid coords):")
        # print(grid_path)

        world_path = [(x - int(self.width / 2), int(self.height / 2) - y) for x, y in grid_path]

        # print("shortest path (world coords):")
        # print(world_path)

        ima = Int32MultiArray()
        ima.data = list(chain(*world_path))
        return ima
