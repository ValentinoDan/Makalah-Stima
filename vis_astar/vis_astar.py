import heapq
from .astar import AStar
from .utils import clear_map

class VisAStar(AStar):
    def __init__(self, lambda_score = 2.0):
        super().__init__()
        self.lambda_score = lambda_score

    def vis_cost(self, clearance):
        return self.lambda_score / (clearance + 1.0)

    def solve_vis(self, grid, clearance_map, start, goal):

        if not grid or not grid[0]:
            return []

        if grid[start[1]][start[0]] == 1:
            return []

        if grid[goal[1]][goal[0]] == 1:
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
                    clearance = (clearance_map[neighbor[1]][neighbor[0]])

                    penalty = (self.vis_cost(clearance))

                    f_score = (g + self.heuristic(neighbor, goal) + penalty)
                    heapq.heappush(opens, (f_score, neighbor))

        return []
