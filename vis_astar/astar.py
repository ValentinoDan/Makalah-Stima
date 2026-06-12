import heapq
import math

class AStar:
    def __init__(self):
        pass

    def heuristic(self, a, b):
        # Euclidean dist
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node, grid):
        x, y = node

        # 8 arah yang mungkin
        dir = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        neighbors = []

        height = len(grid)
        width = len(grid[0])

        for dx, dy in dir:
            nx = x + dx
            ny = y + dy

            if nx < 0 or nx >= width:
                continue

            if ny < 0 or ny >= height:
                continue

            # obstacle
            if grid[ny][nx] == 1:
                continue

            # mepet ama obstacle
            if dx != 0 and dy != 0:
                if(grid[y][nx] == 1 or grid[ny][x] == 1):
                    continue

            neighbors.append((nx, ny))

        return neighbors

    def cost(self, current, neighbor):
        dx = abs(current[0] - neighbor[0])
        dy = abs(current[1] - neighbor[1])

        if dx == 1 and dy == 1:
            return math.sqrt(2)
        
        return 1.0

    def reconstruct_path(self, before, current):
        path = [current]

        while current in before:
            current = before[current]
            path.append(current)

        path.reverse()

        return path
    
    def solve(self, grid, start, goal):
        if not grid or not grid[0]:
            return []

        opens = []
        heapq.heappush(opens, (0.0, start))

        before = {}
        g_score = {start: 0.0}
        closed = set()

        while opens:
            _, current = heapq.heappop(opens)

            if current in closed:
                continue

            if current == goal:
                return self.reconstruct_path(before, current)

            closed.add(current)
            neighbors = self.get_neighbors(current, grid)

            for neighbor in neighbors:
                g = (g_score[current] + self.cost(current, neighbor))

                if (neighbor not in g_score or g < g_score[neighbor]):
                    before[neighbor] = current
                    g_score[neighbor] = g
                    f_score = (g + self.heuristic(neighbor, goal))
                    heapq.heappush(opens, (f_score, neighbor))

        return []

