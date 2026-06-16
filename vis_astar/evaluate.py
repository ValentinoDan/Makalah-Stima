import math
import time
import sys
import os
from vis_astar.astar import AStar
from vis_astar.clearance_astar import ClearanceAStar
from vis_astar.utils import clear_map

_pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _pkg_root not in sys.path:
    sys.path.insert(0, _pkg_root)

SAFETY_THRESH = 3

def path_length(path):
    """Total Euclidean path length in grid cells"""
    total = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        total += math.sqrt(dx * dx + dy * dy)
    return total

def clearance_stats(path, cmap):
    vals  = [cmap[y][x] for x, y in path]
    min_c = min(vals)
    avg_c = sum(vals) / len(vals)
    safe  = sum(1 for v in vals if v >= SAFETY_THRESH) / len(vals)
    return min_c, avg_c, safe

def run_timed(fn, *args):
    t0 = time.perf_counter()
    result = fn(*args)
    return result, time.perf_counter() - t0

def u_shape_grid():
    W, H = 60, 60
    grid = [[0] * W for _ in range(H)]

    for y in range(15, 45):
        for x in range(15, 20):
            grid[y][x] = 1
            
    for y in range(15, 20):
        for x in range(20, 45):
            grid[y][x] = 1
            
    for y in range(40, 45):
        for x in range(20, 45):
            grid[y][x] = 1
            
    return grid

def narrow_corridor_grid():
    W, H = 60, 60
    grid = [[0] * W for _ in range(H)]
    
    for y in range(10, 25):
        for x in range(10, 50):
            grid[y][x] = 1
            
    for y in range(35, 50):
        for x in range(10, 50):
            grid[y][x] = 1

    return grid

def dense_obstacles_grid():
    W, H = 60, 60
    grid = [[0] * W for _ in range(H)]
    
    blocks = [
        (10, 20, 10, 20),
        (30, 40, 10, 30),
        (10, 25, 30, 40),
        (35, 50, 40, 50),
        (45, 55, 15, 25),
        (25, 30, 45, 55)
    ]
    
    for (x1, x2, y1, y2) in blocks:
        for y in range(y1, y2):
            for x in range(x1, x2):
                grid[y][x] = 1
                
    return grid

def gazebo_world_grid():
    W, H = 100, 100
    grid = [[0] * W for _ in range(H)]

    def fill(cx_m, cy_m, sx_m, sy_m):
        cx = int((cx_m + 10.0) / 0.2)
        cy = int((cy_m + 10.0) / 0.2)
        hw = int(sx_m / 0.2 / 2) + 1
        hh = int(sy_m / 0.2 / 2) + 1
        for y in range(max(0, cy - hh), min(H, cy + hh + 1)):
            for x in range(max(0, cx - hw), min(W, cx + hw + 1)):
                grid[y][x] = 1

    fill( 0.0,  4.0, 2.0, 4.0)
    fill( 5.0, -2.0, 2.0, 4.0)
    fill(-4.0, -5.0, 3.0, 3.0)
    fill(-3.0,  2.0, 1.5, 5.0)
    fill( 3.0,  6.0, 4.0, 1.5)
    fill(-7.0, -1.0, 2.0, 2.0)
    fill( 7.0,  3.0, 1.5, 3.0)
    fill( 2.0, -6.0, 2.5, 1.5)
    return grid

SCENARIOS = [
    {
        "name":  "Narrow Corridor",
        "grid":  narrow_corridor_grid(),
        "start": (0, 0),
        "goal":  (9, 9),
    },
    {
        "name":  "Dense Obstacles",
        "grid":  dense_obstacles_grid(),
        "start": (0, 0),
        "goal":  (30, 30),
    },
    {
        "name":  "U-Shape Trap",
        "grid":  u_shape_grid(),
        "start": (0, 0),
        "goal":  (3, 4),
    },
    {
        "name":  "Open World (100x100)",
        "grid":  gazebo_world_grid(),
        "start": (5, 5),
        "goal":  (95, 95),
    },
]

def run_benchmark(lambda_values=(2.0, 6.0, 10.0)):
    astar = AStar()
    W = 93

    for cfg in SCENARIOS:
        name  = cfg["name"]
        grid  = cfg["grid"]
        start = cfg["start"]
        goal  = cfg["goal"]
        cmap  = clear_map(grid)

        rows = len(grid)
        cols = len(grid[0]) if rows else 0
        obs_cells = sum(grid[y][x] for y in range(rows) for x in range(cols))
        obs_pct   = 100.0 * obs_cells / (rows * cols) if rows * cols else 0

        print(f"\n{'─' * W}")
        print(f"  Scenario : {name}")
        print(f"  Grid     : {cols}x{rows}  |  Obstacles: {obs_cells} cells "
              f"({obs_pct:.1f}%)  |  Start: {start}  Goal: {goal}")
        print(f"{'─' * W}")
        header = (f"  {'Algorithm':<24} {'Length':>8} {'MinClr':>8} "
                  f"{'AvgClr':>8} {'Safe%':>8} {'Time(ms)':>10}"
                  f"  {'dLen':>8}  {'dMinClr':>8}")
        print(header)
        print("  " + "-" * (len(header) - 2))

        # A*
        path_a, t_a = run_timed(astar.solve, grid, start, goal)
        if path_a:
            la, aa, sa = clearance_stats(path_a, cmap)
            la_len = path_length(path_a)
            print(f"  {'A* (baseline)':<24} {la_len:>8.2f} "
                  f"{la:>8.2f} {aa:>8.2f} {sa*100:>7.1f}% "
                  f"{t_a*1000:>10.2f}  {'---':>8}  {'---':>8}")
        else:
            la, la_len = 0, 0
            print(f"  {'A* (baseline)':<24}  *** NO PATH FOUND ***")

        # ClearanceAStar for each lambda
        for lam in lambda_values:
            planner = ClearanceAStar(lambda_score=lam)
            path_v, t_v = run_timed(planner.solve, grid, cmap, start, goal)
            label = f"ClearA* lambda={lam:.1f}"
            if path_v:
                lv, av, sv = clearance_stats(path_v, cmap)
                lv_len = path_length(path_v)
                d_len  = lv_len - la_len
                d_minc = lv - la
                print(f"  {label:<24} {lv_len:>8.2f} "
                      f"{lv:>8.2f} {av:>8.2f} {sv*100:>7.1f}% "
                      f"{t_v*1000:>10.2f}  {d_len:>+8.2f}  {d_minc:>+8.2f}")
            else:
                print(f"  {label:<24}  *** NO PATH FOUND ***")

    print(f"\n{'=' * W}")
    print(f"  Legend:")
    print(f"    Length  = total Euclidean path length [grid cells]")
    print(f"    MinClr  = minimum clearance to obstacle along path [cells]")
    print(f"    AvgClr  = average clearance along path [cells]")
    print(f"    Safe%   = % of waypoints with clearance >= {SAFETY_THRESH} cells")
    print(f"    Time    = planning computation time [ms]")
    print(f"    dLen    = ClearA* length - A* length  (+ = longer but safer)")
    print(f"    dMinClr = ClearA* min clearance - A* min clearance  (+ = safer)")
    print("=" * W + "\n")

if __name__ == "__main__":
    run_benchmark()