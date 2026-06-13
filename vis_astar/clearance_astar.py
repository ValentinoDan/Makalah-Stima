import heapq
from .astar import AStar

class ClearanceAStar(AStar):
    """
    Semakin kecil lambda, mirip A*, tapi semakin besar akan memprioritaskan clearance
    """

    def __init__(self, lambda_score: float = 2.0):
        super().__init__()
        self.lambda_score = lambda_score

    def clearance_penalty(self, clearance: float) -> float:
        return self.lambda_score / (clearance + 1.0)

    def solve(self, grid, clearance_map, start, goal):
        """
        grid : list[list[int]]: 2-D occupancy grid. 0 = aman, 1 = obstacle.
        clearance_map : list[list[float]]
        start : tuple[int, int]
        goal : tuple[int, int]

        -> list[tuple[int, int]]
        """


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

            for neighbor in self.get_neighbors(current, grid):
                g = g_score[current] + self.cost(current, neighbor)

                if neighbor not in g_score or g < g_score[neighbor]:
                    before[neighbor] = current
                    g_score[neighbor] = g

                    d = clearance_map[neighbor[1]][neighbor[0]]
                    penalty = self.clearance_penalty(d)
                    f = g + self.heuristic(neighbor, goal) + penalty

                    heapq.heappush(opens, (f, neighbor))

        return []