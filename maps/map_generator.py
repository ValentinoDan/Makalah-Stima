def u_shape_grid():
    W, H = 60, 60
    # Init empty map
    grid = [[0] * W for _ in range(H)]

    # Left wall
    for y in range(15, 45):
        for x in range(15, 20):
            grid[y][x] = 1
            
    # Upper wall
    for y in range(15, 20):
        for x in range(20, 45):
            grid[y][x] = 1
            
    # Lower wall
    for y in range(40, 45):
        for x in range(20, 45):
            grid[y][x] = 1
            
    return grid

def narrow_corridor_grid():
    W, H = 60, 60
    grid = [[0] * W for _ in range(H)]
    
    # Upper wall
    for y in range(10, 25):
        for x in range(10, 50):
            grid[y][x] = 1
            
    # Lower wall
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

def export_to_ros_map(grid, map_name, resolution=0.05):
    height = len(grid)
    width = len(grid[0])
    
    pgm_filename = f"maps/{map_name}.pgm"
    yaml_filename = f"maps/{map_name}.yaml"
    pgm = f"{map_name}.pgm"
    
    with open(pgm_filename, 'w') as f:
        f.write("P2\n")
        f.write(f"{width} {height}\n")
        f.write("255\n")
        for row in grid:
            for cell in row:
                # 1 = obstacle, 0 = empty
                val = 0 if cell == 1 else 255
                f.write(f"{val} ")
            f.write("\n")
            
    with open(yaml_filename, 'w') as f:
        f.write(f"image: {pgm}\n")
        f.write(f"resolution: {resolution}\n")
        f.write("origin: [0.0, 0.0, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        
    print(f"Berhasil membuat {pgm_filename} dan {yaml_filename}!")

export_to_ros_map(u_shape_grid(), "ushape_map")
export_to_ros_map(narrow_corridor_grid(), "narrow_map")
export_to_ros_map(dense_obstacles_grid(), "dense_map")
export_to_ros_map(gazebo_world_grid(), "gazebo_map")