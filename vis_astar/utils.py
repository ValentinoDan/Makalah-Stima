import numpy as np
from collections import deque

# ubah jadi grid
def to_grid(msg):
    width  = msg.info.width
    height = msg.info.height
    data   = np.array(msg.data).reshape(height, width)

    grid = np.zeros((height, width), dtype=int)
    grid[data > 50] = 1        

    return grid.tolist()

def clear_map(grid):
    if not grid or not grid[0]:
        return []

    height = len(grid)
    width  = len(grid[0])

    clearance = [[float("inf")] * width for _ in range(height)]
    q = deque()

    for y in range(height):
        for x in range(width):
            if grid[y][x] == 1:
                clearance[y][x] = 0
                q.append((x, y))

    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    while q:
        x, y = q.popleft()
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            new_dist = clearance[y][x] + 1
            if new_dist < clearance[ny][nx]:
                clearance[ny][nx] = new_dist
                q.append((nx, ny))

    return clearance