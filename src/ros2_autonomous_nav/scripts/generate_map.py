#!/usr/bin/env python3
"""
Generate a 2D occupancy grid map (PGM + YAML) from the known wall geometry
of nav_world.sdf. This lets you run Nav2 without doing SLAM first.

Usage:
    python3 generate_map.py
"""

import numpy as np
import os
import math

# --- Map parameters ---
RESOLUTION = 0.05  # meters per pixel
ORIGIN_X = -6.0    # bottom-left corner in world coords
ORIGIN_Y = -6.0
MAP_WIDTH_M = 12.0  # total map width in meters
MAP_HEIGHT_M = 12.0
WIDTH = int(MAP_WIDTH_M / RESOLUTION)   # 240 pixels
HEIGHT = int(MAP_HEIGHT_M / RESOLUTION)  # 240 pixels

FREE = 254
OCCUPIED = 0
UNKNOWN = 205


def world_to_pixel(wx, wy):
    """Convert world coordinates to pixel coordinates."""
    px = int((wx - ORIGIN_X) / RESOLUTION)
    py = int((wy - ORIGIN_Y) / RESOLUTION)
    return px, py


def draw_box(grid, cx, cy, sx, sy, yaw=0.0):
    """Draw an axis-aligned or rotated box obstacle on the grid."""
    half_x = sx / 2.0
    half_y = sy / 2.0
    cos_a = math.cos(yaw)
    sin_a = math.sin(yaw)

    # Compute bounding box in world coords
    corners = []
    for dx in [-half_x, half_x]:
        for dy in [-half_y, half_y]:
            rx = cx + dx * cos_a - dy * sin_a
            ry = cy + dx * sin_a + dy * cos_a
            corners.append((rx, ry))

    min_x = min(c[0] for c in corners)
    max_x = max(c[0] for c in corners)
    min_y = min(c[1] for c in corners)
    max_y = max(c[1] for c in corners)

    # Rasterize with point-in-rotated-rect check
    px_min, py_min = world_to_pixel(min_x, min_y)
    px_max, py_max = world_to_pixel(max_x, max_y)

    for py in range(max(0, py_min), min(HEIGHT, py_max + 1)):
        for px in range(max(0, px_min), min(WIDTH, px_max + 1)):
            # Pixel center in world coords
            wx = ORIGIN_X + (px + 0.5) * RESOLUTION
            wy = ORIGIN_Y + (py + 0.5) * RESOLUTION
            # Transform to box-local coords
            lx = (wx - cx) * cos_a + (wy - cy) * sin_a
            ly = -(wx - cx) * sin_a + (wy - cy) * cos_a
            if abs(lx) <= half_x and abs(ly) <= half_y:
                grid[py, px] = OCCUPIED


def draw_cylinder(grid, cx, cy, radius):
    """Draw a circular obstacle on the grid."""
    px_c, py_c = world_to_pixel(cx, cy)
    r_px = int(radius / RESOLUTION) + 1

    for py in range(max(0, py_c - r_px), min(HEIGHT, py_c + r_px + 1)):
        for px in range(max(0, px_c - r_px), min(WIDTH, px_c + r_px + 1)):
            wx = ORIGIN_X + (px + 0.5) * RESOLUTION
            wy = ORIGIN_Y + (py + 0.5) * RESOLUTION
            if (wx - cx)**2 + (wy - cy)**2 <= radius**2:
                grid[py, px] = OCCUPIED


def main():
    # Start with free space
    grid = np.full((HEIGHT, WIDTH), FREE, dtype=np.uint8)

    # --- Outer walls ---
    # North wall: center (0, 5), size 10.2 x 0.2
    draw_box(grid, 0.0, 5.0, 10.2, 0.2)
    # South wall: center (0, -5), size 10.2 x 0.2
    draw_box(grid, 0.0, -5.0, 10.2, 0.2)
    # East wall: center (5, 0), size 0.2 x 10.2
    draw_box(grid, 5.0, 0.0, 0.2, 10.2)
    # West wall: center (-5, 0), size 0.2 x 10.2
    draw_box(grid, -5.0, 0.0, 0.2, 10.2)

    # --- Internal walls ---
    # wall_i1: center (-1.5, -3.0), size 0.15 x 4.0
    draw_box(grid, -1.5, -3.0, 0.15, 4.0)
    # wall_i2: center (1.5, 2.75), size 0.15 x 4.5
    draw_box(grid, 1.5, 2.75, 0.15, 4.5)
    # wall_i3: center (-3.75, 1.5), size 2.5 x 0.15
    draw_box(grid, -3.75, 1.5, 2.5, 0.15)
    # wall_i4: center (3.5, -1.5), size 3.0 x 0.15
    draw_box(grid, 3.5, -1.5, 3.0, 0.15)

    # --- Pillars ---
    draw_cylinder(grid, -3.0, -3.0, 0.25)
    draw_cylinder(grid, 3.0, 3.0, 0.25)

    # --- Crates ---
    # crate_1: center (-3.0, 3.2), size 0.5 x 0.5, yaw 0.3
    draw_box(grid, -3.0, 3.2, 0.5, 0.5, yaw=0.3)
    # crate_2: center (3.7, -3.7), size 0.6 x 0.4, yaw -0.5
    draw_box(grid, 3.7, -3.7, 0.6, 0.4, yaw=-0.5)

    # Mark outside the enclosure as unknown (optional, cleaner look)
    for py in range(HEIGHT):
        for px in range(WIDTH):
            wx = ORIGIN_X + (px + 0.5) * RESOLUTION
            wy = ORIGIN_Y + (py + 0.5) * RESOLUTION
            if abs(wx) > 5.05 or abs(wy) > 5.05:
                if grid[py, px] != OCCUPIED:
                    grid[py, px] = UNKNOWN

    # --- Write PGM (P5 binary format) ---
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps')
    os.makedirs(out_dir, exist_ok=True)

    pgm_path = os.path.join(out_dir, 'nav_world.pgm')
    with open(pgm_path, 'wb') as f:
        header = f'P5\n{WIDTH} {HEIGHT}\n255\n'
        f.write(header.encode())
        # PGM row 0 is top of image = max Y in world coords
        for row in range(HEIGHT - 1, -1, -1):
            f.write(grid[row].tobytes())

    # --- Write YAML ---
    yaml_path = os.path.join(out_dir, 'nav_world.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f'image: nav_world.pgm\n')
        f.write(f'mode: trinary\n')
        f.write(f'resolution: {RESOLUTION}\n')
        f.write(f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: 0.65\n')
        f.write(f'free_thresh: 0.25\n')

    print(f'Map saved to:')
    print(f'  {os.path.abspath(pgm_path)}')
    print(f'  {os.path.abspath(yaml_path)}')
    print(f'  Size: {WIDTH}x{HEIGHT} pixels, {MAP_WIDTH_M}x{MAP_HEIGHT_M} m')


if __name__ == '__main__':
    main()
