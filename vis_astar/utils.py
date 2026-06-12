from collections import deque

# membantu vis_astar (indikator aman atau tidak)
def clear_map(grid):

    if not grid or not grid[0]:
        return []

    height = len(grid)
    width = len(grid[0])

    clearance = [[float("inf") for _ in range(width)] for _ in range(height)]

    q = deque()

    # semua obstacle jadi source BFS
    for y in range(height):
        for x in range(width):

            if grid[y][x] == 1:
                clearance[y][x] = 0
                q.append((x, y))

    dir = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    while q:
        x, y = q.popleft()
        for dx, dy in dir:
            nx = x + dx
            ny = y + dy

            if nx < 0 or nx >= width:
                continue

            if ny < 0 or ny >= height:
                continue

            new_dist = clearance[y][x] + 1

            if new_dist < clearance[ny][nx]:
                clearance[ny][nx] = new_dist
                q.append((nx, ny))

    return clearance