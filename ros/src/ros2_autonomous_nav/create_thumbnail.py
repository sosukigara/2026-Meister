#!/usr/bin/env python3
"""Generate YouTube thumbnail for ROS2 - Autonomous Navigation with Nav2.

Visual concept:
  - Dark navy tech background with radial gradient
  - Subtle grid overlay
  - Occupancy grid map panel with robot path visualization
  - Top-down robot glyph with planned path (green) + local path (blue)
  - Bold title: "NAV2 NAVIGATION" + subtitle + tech badge
  - Channel logo overlay (top-left)
  - Cyan corner brackets

Output: /home/robocon/youtube_channel/thumbnails/ros_06_autonomous_navigation.png
        (1280x720, PNG)
"""

from PIL import Image, ImageDraw, ImageFont, ImageFilter
import os
import math
import random

random.seed(42)

WIDTH, HEIGHT = 1280, 720

# Colours
BG_DARK = (6, 10, 20)
BG_MID = (14, 22, 44)

FREE_WHITE = (235, 238, 245)
WALL_GREY = (70, 80, 100)
WALL_EDGE = (180, 210, 240)

ELECTRIC_CYAN = (60, 220, 255)
ELECTRIC_BLUE = (55, 140, 255)
PATH_GREEN = (25, 255, 80)
PATH_BLUE = (0, 150, 255)
HOT_ORANGE_RED = (255, 68, 0)
WARM_YELLOW = (255, 215, 70)
GOAL_RED = (255, 50, 50)
TEXT_WHITE = (255, 255, 255)


def load_font(size, bold=True, black=False):
    if black:
        paths = [
            "/usr/share/fonts/truetype/lato/Lato-Black.ttf",
            "/usr/share/fonts/truetype/lato/Lato-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        ]
    elif bold:
        paths = [
            "/usr/share/fonts/truetype/lato/Lato-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        ]
    else:
        paths = [
            "/usr/share/fonts/truetype/lato/Lato-Regular.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]
    for p in paths:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def draw_text_shadow(draw_obj, xy, text, font, fill):
    x, y = xy
    for dx, dy in [(6, 6), (4, 4), (2, 2)]:
        draw_obj.text((x + dx, y + dy), text, font=font, fill=(0, 0, 0, 180))
    draw_obj.text((x, y), text, font=font, fill=fill)


def make_background():
    bg = Image.new("RGB", (WIDTH, HEIGHT), BG_DARK)
    px = bg.load()
    cx, cy = WIDTH // 2, HEIGHT // 2
    max_d = math.hypot(WIDTH, HEIGHT)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            d = math.hypot(x - cx, y - cy) / max_d
            t = max(0.0, 1.0 - d * 1.55)
            r = int(BG_DARK[0] + (BG_MID[0] - BG_DARK[0]) * t)
            g = int(BG_DARK[1] + (BG_MID[1] - BG_DARK[1]) * t + 2 * t)
            b = int(BG_DARK[2] + (BG_MID[2] - BG_DARK[2]) * t + 10 * t)
            px[x, y] = (r, g, b)
    return bg


def draw_grid(base):
    overlay = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    step = 40
    for x in range(0, WIDTH, step):
        d.line([(x, 0), (x, HEIGHT)], fill=(60, 130, 180, 15), width=1)
    for y in range(0, HEIGHT, step):
        d.line([(0, y), (WIDTH, y)], fill=(60, 130, 180, 15), width=1)
    return Image.alpha_composite(base.convert("RGBA"), overlay)


def draw_map_panel(base):
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)

    mx1, my1 = 80, 80
    mx2, my2 = WIDTH - 80, HEIGHT - 80

    d.rounded_rectangle([mx1, my1, mx2, my2], radius=12,
                        fill=(12, 16, 28, 255),
                        outline=(60, 100, 180, 255), width=3)

    inset = 30
    fx1, fy1 = mx1 + inset, my1 + inset
    fx2, fy2 = mx2 - inset, my2 - inset

    # Explored region blob
    explored_poly = []
    cx = (fx1 + fx2) / 2
    cy = (fy1 + fy2) / 2
    rx = (fx2 - fx1) / 2
    ry = (fy2 - fy1) / 2
    N = 80
    for i in range(N):
        ang = (i / N) * math.tau
        jitter = 0.88 + random.random() * 0.14
        px = cx + math.cos(ang) * rx * jitter
        py = cy + math.sin(ang) * ry * jitter
        explored_poly.append((px, py))
    d.polygon(explored_poly, fill=FREE_WHITE + (255,))

    # Cell texture
    cell = 14
    for gy in range(int(fy1), int(fy2), cell):
        for gx in range(int(fx1), int(fx2), cell):
            v = random.randint(-6, 4)
            col = (max(0, FREE_WHITE[0] + v),
                   max(0, FREE_WHITE[1] + v),
                   max(0, FREE_WHITE[2] + v), 70)
            d.rectangle([gx, gy, gx + cell - 1, gy + cell - 1], fill=col)

    # Wall lines
    wall_coords = [
        (fx1 + 20, fy1 + 40, fx1 + 340, fy1 + 40),
        (fx1 + 400, fy1 + 40, fx2 - 20, fy1 + 40),
        (fx1 + 20, fy2 - 40, fx1 + 260, fy2 - 40),
        (fx1 + 340, fy2 - 40, fx2 - 20, fy2 - 40),
        (fx1 + 20, fy1 + 40, fx1 + 20, fy2 - 40),
        (fx2 - 20, fy1 + 40, fx2 - 20, fy2 - 40),
        (fx1 + 280, fy1 + 80, fx1 + 280, fy1 + 260),
        (fx1 + 280, fy1 + 260, fx1 + 520, fy1 + 260),
        (fx1 + 520, fy2 - 220, fx2 - 60, fy2 - 220),
        (fx2 - 220, fy1 + 100, fx2 - 220, fy2 - 220),
        (fx1 + 120, fy2 - 180, fx1 + 340, fy2 - 180),
    ]

    # Glow behind walls
    for (x1, y1, x2, y2) in wall_coords:
        glow = Image.new("RGBA", base.size, (0, 0, 0, 0))
        gd = ImageDraw.Draw(glow)
        gd.line([(x1, y1), (x2, y2)],
                fill=(100, 150, 210, 180), width=24)
        glow = glow.filter(ImageFilter.GaussianBlur(radius=4))
        layer = Image.alpha_composite(layer, glow)

    d = ImageDraw.Draw(layer)
    for (x1, y1, x2, y2) in wall_coords:
        d.line([(x1, y1), (x2, y2)], fill=WALL_GREY + (255,), width=10)
        d.line([(x1, y1), (x2, y2)], fill=WALL_EDGE + (255,), width=2)

    # Pillars
    for (ppx, ppy, r) in [(fx1 + 170, fy2 - 100, 14), (fx2 - 140, fy1 + 160, 14)]:
        d.ellipse([ppx - r, ppy - r, ppx + r, ppy + r],
                  fill=WALL_GREY + (255,), outline=WALL_EDGE + (255,), width=2)

    # Corner brackets on panel
    corner_col = (80, 240, 140, 255)
    cl = 28
    for (cx0, cy0, dx, dy) in [
        (mx1, my1, +1, +1), (mx2, my1, -1, +1),
        (mx1, my2, +1, -1), (mx2, my2, -1, -1),
    ]:
        d.line([(cx0, cy0), (cx0 + cl * dx, cy0)], fill=corner_col, width=4)
        d.line([(cx0, cy0), (cx0, cy0 + cl * dy)], fill=corner_col, width=4)

    # HUD labels
    hud_font = load_font(18, bold=True)
    d.text((mx1 + 20, my1 + 6), "NAV2 COSTMAP + PATH",
           font=hud_font, fill=(120, 220, 255, 230))
    d.ellipse([mx2 - 120, my1 + 8, mx2 - 106, my1 + 22],
              fill=HOT_ORANGE_RED + (255,), outline=(255, 200, 140, 255), width=1)
    d.text((mx2 - 96, my1 + 6), "LIVE",
           font=hud_font, fill=(255, 170, 90, 255))

    base = Image.alpha_composite(base, layer)
    return base, (fx1, fy1, fx2, fy2)


def draw_path_and_robot(base, map_bounds):
    fx1, fy1, fx2, fy2 = map_bounds
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)

    # Robot position
    rx = int(fx1 + 100)
    ry = int(fy2 - 100)

    # Goal position
    gx = int(fx2 - 100)
    gy = int(fy1 + 120)

    # Global path (green, curved through the maze)
    path_points = [
        (rx, ry),
        (rx + 60, ry - 40),
        (rx + 120, ry - 100),
        (int((fx1 + fx2) / 2) - 60, int((fy1 + fy2) / 2) + 40),
        (int((fx1 + fx2) / 2), int((fy1 + fy2) / 2) - 20),
        (int((fx1 + fx2) / 2) + 80, int((fy1 + fy2) / 2) - 80),
        (gx - 120, gy + 80),
        (gx - 40, gy + 20),
        (gx, gy),
    ]

    # Draw global path with glow
    path_glow = Image.new("RGBA", base.size, (0, 0, 0, 0))
    pgd = ImageDraw.Draw(path_glow)
    for i in range(len(path_points) - 1):
        pgd.line([path_points[i], path_points[i + 1]],
                 fill=PATH_GREEN + (120,), width=12)
    path_glow = path_glow.filter(ImageFilter.GaussianBlur(radius=6))
    layer = Image.alpha_composite(layer, path_glow)

    d = ImageDraw.Draw(layer)
    for i in range(len(path_points) - 1):
        d.line([path_points[i], path_points[i + 1]],
               fill=PATH_GREEN + (220,), width=4)

    # Local path segment (blue, first few points)
    for i in range(min(3, len(path_points) - 1)):
        d.line([path_points[i], path_points[i + 1]],
               fill=PATH_BLUE + (220,), width=6)

    # Goal marker (red pulsing circle)
    for r, a in [(30, 40), (22, 70), (14, 120)]:
        d.ellipse([gx - r, gy - r, gx + r, gy + r],
                  outline=GOAL_RED + (a,), width=3)
    d.ellipse([gx - 8, gy - 8, gx + 8, gy + 8], fill=GOAL_RED + (255,))

    # Goal flag
    flag_font = load_font(16, bold=True)
    d.text((gx + 18, gy - 12), "GOAL", font=flag_font,
           fill=GOAL_RED + (255,))

    # Costmap gradient around walls (orange/yellow inflate zones)
    inflate_layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    id_draw = ImageDraw.Draw(inflate_layer)
    # Add some inflation blobs near wall intersections
    for (ix, iy) in [(fx1 + 280, fy1 + 260), (fx2 - 220, fy2 - 220),
                     (fx1 + 340, fy2 - 180)]:
        id_draw.ellipse([ix - 35, iy - 35, ix + 35, iy + 35],
                        fill=(255, 140, 0, 50))
    inflate_layer = inflate_layer.filter(ImageFilter.GaussianBlur(radius=15))
    layer = Image.alpha_composite(layer, inflate_layer)

    # Robot body
    d = ImageDraw.Draw(layer)

    # Drop shadow
    shadow = Image.new("RGBA", base.size, (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow)
    sd.ellipse([rx - 40, ry + 14, rx + 40, ry + 34], fill=(0, 0, 0, 180))
    shadow = shadow.filter(ImageFilter.GaussianBlur(radius=8))
    layer = Image.alpha_composite(layer, shadow)

    d = ImageDraw.Draw(layer)
    bw, bh = 64, 44
    d.rounded_rectangle([rx - bw // 2, ry - bh // 2,
                         rx + bw // 2, ry + bh // 2],
                        radius=8, fill=(45, 110, 220, 255),
                        outline=(130, 200, 255, 255), width=3)
    # Wheels
    d.rounded_rectangle([rx - bw // 2 - 6, ry - 20, rx - bw // 2 + 2, ry + 20],
                        radius=3, fill=(14, 14, 18, 255),
                        outline=(80, 90, 110, 255), width=2)
    d.rounded_rectangle([rx + bw // 2 - 2, ry - 20, rx + bw // 2 + 6, ry + 20],
                        radius=3, fill=(14, 14, 18, 255),
                        outline=(80, 90, 110, 255), width=2)
    # Lidar puck
    d.ellipse([rx - 13, ry - 13, rx + 13, ry + 13],
              fill=(18, 20, 26, 255), outline=(120, 220, 255, 255), width=2)
    d.ellipse([rx - 7, ry - 7, rx + 7, ry + 7],
              fill=(80, 220, 255, 255), outline=(200, 245, 255, 255), width=1)
    # Forward arrow
    d.polygon([(rx + bw // 2 + 2, ry - 6), (rx + bw // 2 + 12, ry),
               (rx + bw // 2 + 2, ry + 6)],
              fill=WARM_YELLOW + (255,))

    base = Image.alpha_composite(base, layer)
    return base


def draw_logo(base):
    logo_path = "/home/robocon/youtube_channel/assets/logo_200.png"
    if not os.path.exists(logo_path):
        return base
    try:
        logo = Image.open(logo_path).convert("RGBA")
        target_h = 80
        ratio = target_h / logo.height
        new_w = int(logo.width * ratio)
        logo = logo.resize((new_w, target_h), Image.LANCZOS)
        alpha = logo.split()[3]
        alpha = alpha.point(lambda a: int(a * 0.85))
        logo.putalpha(alpha)
        logo_layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
        logo_layer.paste(logo, (20, 14), logo)
        base = Image.alpha_composite(base, logo_layer)
    except Exception as e:
        print(f"Warning: could not load logo: {e}")
    return base


def draw_title(base):
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)

    # Badge
    badge_font = load_font(22, bold=True)
    badge_text = "ROS2 JAZZY  \u2022  GAZEBO  \u2022  NAV2"
    bb = d.textbbox((0, 0), badge_text, font=badge_font)
    bw = bb[2] - bb[0]
    bx = (WIDTH - bw) // 2
    by = 36
    d.rounded_rectangle([bx - 22, by - 8, bx + bw + 22, by + 34],
                        radius=18, fill=(20, 30, 60, 220),
                        outline=ELECTRIC_CYAN + (255,), width=2)
    d.text((bx, by), badge_text, font=badge_font, fill=(140, 220, 255, 255))

    # Main title
    title_font = load_font(86, bold=True, black=True)
    title = "NAV2 NAVIGATION"
    tb = d.textbbox((0, 0), title, font=title_font)
    tw = tb[2] - tb[0]
    th = tb[3] - tb[1]
    tx = (WIDTH - tw) // 2
    ty = HEIGHT - th - 100
    draw_text_shadow(d, (tx, ty), title, title_font, TEXT_WHITE + (255,))

    # Underline
    ux1 = tx + 10
    ux2 = tx + tw - 10
    uy = ty + th + 12
    d.line([(ux1, uy), (ux2, uy)], fill=ELECTRIC_CYAN + (255,), width=4)

    # Subtitle
    sub_font = load_font(30, bold=True)
    sub = "AUTONOMOUS ROBOT PATH PLANNING"
    sbb = d.textbbox((0, 0), sub, font=sub_font)
    sw = sbb[2] - sbb[0]
    sx = (WIDTH - sw) // 2
    sy = uy + 12
    d.text((sx + 2, sy + 2), sub, font=sub_font, fill=(0, 0, 0, 200))
    d.text((sx, sy), sub, font=sub_font, fill=WARM_YELLOW + (255,))

    return Image.alpha_composite(base, layer)


def draw_corner_brackets(base):
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    col = ELECTRIC_CYAN[:3] + (220,)
    cl = 42
    margin = 24
    thickness = 5
    for (cx, cy, dx, dy) in [
        (margin, margin, +1, +1), (WIDTH - margin, margin, -1, +1),
        (margin, HEIGHT - margin, +1, -1),
        (WIDTH - margin, HEIGHT - margin, -1, -1),
    ]:
        d.line([(cx, cy), (cx + cl * dx, cy)], fill=col, width=thickness)
        d.line([(cx, cy), (cx, cy + cl * dy)], fill=col, width=thickness)
    return Image.alpha_composite(base, layer)


def main():
    random.seed(42)
    print("Building Nav2 navigation thumbnail...")

    img = make_background()
    img = draw_grid(img)
    img, bounds = draw_map_panel(img)
    img = draw_path_and_robot(img, bounds)
    img = draw_logo(img)
    img = draw_title(img)
    img = draw_corner_brackets(img)

    out_path = "/home/robocon/youtube_channel/thumbnails/ros_06_autonomous_navigation.png"
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    final = img.convert("RGB")
    final.save(out_path, "PNG")

    size_kb = os.path.getsize(out_path) / 1024
    print(f"Thumbnail saved: {out_path}")
    print(f"Dimensions: {final.size[0]}x{final.size[1]}")
    print(f"Size: {size_kb:.0f} KB")

    local_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "thumbnail.png"
    )
    final.save(local_path, "PNG")
    print(f"Local copy: {local_path}")


if __name__ == "__main__":
    main()
