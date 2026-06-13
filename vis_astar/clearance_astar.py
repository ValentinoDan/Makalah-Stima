import heapq
from .astar import AStar

class ClearanceAStar(AStar):
    """
    - lambda_score : semakin besar, path semakin menjauhi tembok
    - min_clearance: cell dengan clearance < nilai ini dianggap impassable
    """
    def __init__(self, lambda_score = 2.0, min_clearance = 0.0):
        super().__init__()
        self.lambda_score = lambda_score
        self.min_clearance = min_clearance 

    def clearance_penalty(self, clearance):
        return self.lambda_score / (clearance + 1.0)

    def solve(self, grid, clearance_map, start, goal):
        if not grid or not grid[0]:
            return []
        if grid[start[1]][start[0]] == 1:
            return []
        if grid[goal[1]][goal[0]] == 1:
            return []

        opens = []
        heapq.heappush(opens, (0.0, start))
        before    = {}
        g_score   = {start: 0.0}
        closed    = set()

        while opens:
            _, current = heapq.heappop(opens)

            if current in closed:
                continue
            if current == goal:
                return self.reconstruct_path(before, current)

            closed.add(current)

            for neighbor in self.get_neighbors(current, grid):
                d = clearance_map[neighbor[1]][neighbor[0]]
                if d < self.min_clearance:
                    continue

                g = g_score[current] + self.cost(current, neighbor)

                if neighbor not in g_score or g < g_score[neighbor]:
                    before[neighbor]  = current
                    g_score[neighbor] = g
                    penalty = self.clearance_penalty(d)
                    f = g + self.heuristic(neighbor, goal) + penalty
                    heapq.heappush(opens, (f, neighbor))

        return []