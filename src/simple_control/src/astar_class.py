import math, heapq, random

class AStar:

  def __init__(self, grid):
    self.grid = grid
    self.times_planned_from = [[0] * grid.width for _ in range(grid.height)]
    self.STRAIGHT_LINE_MAX = 5

  # Get the nodes that current node can go to
  def get_neighbors(self, node_x, node_y):
    neighbors = []
    for x_o, y_o in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        x, y = node_x + x_o, node_y + y_o
        if self.grid.can_travel(x, y):
            neighbors.append((x, y))

    return neighbors
  
  def get_next_move(self, drone_pos, dog_pos):
    
    drone_x, drone_y = self.grid.world_to_grid(drone_pos)
    
    self.grid.visited[drone_y][drone_x] = True
    self.times_planned_from[drone_y][drone_x] += 1
    if self.times_planned_from[drone_y][drone_x] > 5:
      self.STRAIGHT_LINE_MAX = max(self.STRAIGHT_LINE_MAX - 1, 1)

    for ix, iy in [(0, 1), (1, 0), (-1, 0), (0, -1)]:
      if self.grid.is_closed_door(drone_x + ix, drone_y + iy):
        self.grid.visited[drone_y + iy][drone_x + ix] = True
        self.grid.visited[drone_y + iy + iy][drone_x + ix + ix] = True
        return drone_x + ix, drone_y + iy

    dog_x, dog_y = self.grid.world_to_grid(dog_pos)

    pq = [(0, drone_x, drone_y)]
    cost_so_far = {(drone_x, drone_y): 0}
    came_from = {(drone_x, drone_y): None}
    done = set()

    while pq:

      cur_cost, cur_x, cur_y = heapq.heappop(pq)

      if cur_x == dog_x and cur_y == dog_y:
        break

      for next_x, next_y in self.get_neighbors(cur_x, cur_y):
        nxt = (next_x, next_y)
        if nxt in done:
          continue

        new_cost = cost_so_far[(cur_x, cur_y)] + math.sqrt((next_x - cur_x)**2 + (next_y - cur_y)**2)

        if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
          cost_so_far[nxt] = new_cost
          priority = new_cost + math.sqrt((next_x - dog_x)**2 + (next_y - dog_y)**2)
          if nxt != (dog_x, dog_y):
            heapq.heappush(pq, (priority, next_x, next_y))
          came_from[nxt] = (cur_x, cur_y)

      done.add((cur_x, cur_y))

    path = [(dog_x, dog_y)]
    cur = (dog_x, dog_y)
    while cur in came_from and came_from[cur] is not None:
      cur = came_from[cur]
      path.append(cur)
      
    path.reverse()

    if len(path) < 2 or path[0] != (drone_x, drone_y) or path[-1] != (dog_x, dog_y):
      print("No path found, making default move")
      default_moves = [(0, 1), (1, 0), (-1, 0), (0, -1)]
      random.shuffle(default_moves)
      for ix, iy in default_moves:
        if self.grid.can_travel(drone_x + ix, drone_y + iy):
          return drone_x + ix, drone_y + iy
      print("Could not make default move")
      return None

    sli = self.get_straight_line_index(path)
    for x, y in path[:sli+1]:
      self.grid.visited[y][x] = True

    return path[sli]

  def get_straight_line_index(self, path):
    if len(path) <= 2:
      return 1
    x, y = path[0]
    i = 2
    if x == path[1][0]:
      while i < min(self.STRAIGHT_LINE_MAX, len(path)) and path[i][0] == x and not self.grid.is_closed_door(x, path[i][1]):
        i += 1
      return i - 1
    
    while i < min(self.STRAIGHT_LINE_MAX, len(path)) and path[i][1] == y and not self.grid.is_closed_door(path[i][0], y):
      i += 1
    return i - 1
