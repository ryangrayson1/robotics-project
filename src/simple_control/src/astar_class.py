import math, copy, heapq

class AStar:

  def __init__(self, grid):
    self.grid = grid

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

    for ix, iy in [(0, 1), (1, 0), (-1, 0), (0, -1)]:
      if self.grid.is_closed_door(drone_x + ix, drone_y + iy):
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
      print("No path found")
      return None

    return path[self.get_straight_line_index(path)]

  def get_straight_line_index(self, path):
    if len(path) <= 2:
      return 1
    x, y = path[0]
    i = 2
    if x == path[1][0] and len(path):
      while path[i][0] == x and not self.grid.is_closed_door(x, path[i][1]) and i < 5:
        i += 1
      return i - 1
    else:
      while path[i][1] == y and not self.grid.is_closed_door(path[i][0], y) and i < 5:
        i += 1
      return i - 1