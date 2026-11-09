#!/usr/bin/env python3
"""Render code-snippet PNG cards for the Nav2 autonomous navigation tutorial.

Each snippet is a 1920x1080 image with a dark editor look.
Used as overlays on the demo footage.
"""

import os
from io import BytesIO
from pygments import highlight
from pygments.lexers import PythonLexer, BashLexer, YamlLexer
from pygments.formatters import ImageFormatter
from PIL import Image, ImageDraw, ImageFont

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snippets")
os.makedirs(OUT_DIR, exist_ok=True)

CARD_W, CARD_H = 1920, 1080
BG = (20, 22, 32)
CHROME = (32, 36, 50)
CHROME_LINE = (70, 90, 140)
TITLE_FG = (220, 225, 240)
TITLE_DIM = (140, 150, 175)


def load_font(size, bold=True):
    paths = [
        "/usr/share/fonts/truetype/lato/Lato-Bold.ttf" if bold
        else "/usr/share/fonts/truetype/lato/Lato-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for p in paths:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def render_code_image(code, lexer):
    fmt = ImageFormatter(
        font_name="DejaVu Sans Mono",
        font_size=34,
        line_numbers=True,
        line_number_bg="#202234",
        line_number_fg="#5a6580",
        line_number_separator=False,
        line_pad=10,
        image_pad=24,
        style="monokai",
    )
    png_bytes = highlight(code, lexer, fmt)
    return Image.open(BytesIO(png_bytes)).convert("RGB")


def make_card(title_left, title_right, code, lexer, output_path):
    card = Image.new("RGB", (CARD_W, CARD_H), BG)
    draw = ImageDraw.Draw(card)

    chrome_h = 80
    draw.rectangle([0, 0, CARD_W, chrome_h], fill=CHROME)
    draw.line([(0, chrome_h), (CARD_W, chrome_h)], fill=CHROME_LINE, width=4)

    for i, color in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
        cx = 32 + i * 40
        draw.ellipse([cx, 26, cx + 28, 54], fill=color)

    title_font = load_font(30, bold=True)
    sub_font = load_font(24, bold=False)
    draw.text((180, 24), title_left, font=title_font, fill=TITLE_FG)
    bbox = draw.textbbox((0, 0), title_right, font=sub_font)
    tw = bbox[2] - bbox[0]
    draw.text((CARD_W - tw - 36, 28), title_right, font=sub_font, fill=TITLE_DIM)

    code_img = render_code_image(code, lexer)
    avail_w = CARD_W - 80
    avail_h = CARD_H - chrome_h - 80
    cw, ch = code_img.size
    scale = min(avail_w / cw, avail_h / ch, 1.6)
    if scale != 1.0:
        code_img = code_img.resize((int(cw * scale), int(ch * scale)),
                                   Image.LANCZOS)
        cw, ch = code_img.size

    cx = (CARD_W - cw) // 2
    cy = chrome_h + (avail_h - ch) // 2 + 30
    shadow = Image.new("RGB", (cw + 16, ch + 16), (10, 12, 18))
    card.paste(shadow, (cx - 8, cy - 4))
    card.paste(code_img, (cx, cy))

    border_color = (60, 100, 180)
    draw.rectangle([cx - 2, cy - 2, cx + cw + 2, cy + ch + 2],
                   outline=border_color, width=3)

    accent_h = 6
    for x in range(CARD_W):
        t = x / CARD_W
        r = int(60 + (255 - 60) * t)
        g = int(120 + (60 - 120) * abs(2 * t - 1))
        b = int(220 + (40 - 220) * t)
        for dy in range(accent_h):
            card.putpixel((x, CARD_H - accent_h + dy), (r, g, b))

    card.save(output_path, "PNG")
    print(f"  -> {os.path.basename(output_path)}")


SNIPPETS = [
    {
        "name": "step1_nav2_params.png",
        "title_left": "STEP 1 — Nav2 stack parameters",
        "title_right": "config/nav2_params.yaml",
        "lexer": YamlLexer(),
        "code": """amcl:
  ros__parameters:
    base_frame_id: "base_footprint"
    laser_model_type: "likelihood_field"
    max_particles: 2000
    set_initial_pose: true
    initial_pose: {x: -4.0, y: -4.0, yaw: 0.0}

controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.3
      max_vel_theta: 1.0
""",
    },
    {
        "name": "step2_gpu_detect.png",
        "title_left": "STEP 2 — GPU / CPU auto-detection",
        "title_right": "launch/robot_nav.launch.py",
        "lexer": PythonLexer(),
        "code": """def detect_gpu():
    try:
        result = subprocess.run(
            ['nvidia-smi'], capture_output=True, timeout=5
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False

has_gpu = detect_gpu()
urdf_name = 'my_robot_gpu.urdf' if has_gpu \\
            else 'my_robot_cpu.urdf'
""",
    },
    {
        "name": "step3_launch_nav2.png",
        "title_left": "STEP 3 — Launch the Nav2 stack",
        "title_right": "launch/robot_nav.launch.py",
        "lexer": PythonLexer(),
        "code": """# Delay Nav2 to let Gazebo + bridge start first
TimerAction(period=8.0, actions=[
    map_server,
    amcl,
    controller_server,
    planner_server,
    behavior_server,
    bt_navigator,
    lifecycle_manager,
])
""",
    },
    {
        "name": "step4_costmaps.png",
        "title_left": "STEP 4 — Costmap configuration",
        "title_right": "config/nav2_params.yaml",
        "lexer": YamlLexer(),
        "code": """global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.05
      robot_radius: 0.18
      plugins: ["static_layer", "obstacle_layer",
                "inflation_layer"]
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
""",
    },
    {
        "name": "step5_waypoint_follower.png",
        "title_left": "STEP 5 — Waypoint follower",
        "title_right": "scripts/waypoint_follower.py",
        "lexer": PythonLexer(),
        "code": """navigator = BasicNavigator()
navigator.waitUntilNav2Active(localizer='amcl')

waypoints = [
    (-3.0, -1.5, 'bottom-left area'),
    (-0.5, -0.5, 'center'),
    (-3.5,  3.5, 'top-left room'),
    ( 3.5,  1.0, 'right corridor'),
    (-4.0, -4.0, 'start position'),
]

for x, y, label in waypoints:
    goal = create_pose(navigator, x, y, 0.0)
    navigator.goToPose(goal)
""",
    },
    {
        "name": "step6_run_commands.png",
        "title_left": "STEP 6 — Run it (3 terminals)",
        "title_right": "terminal",
        "lexer": BashLexer(),
        "code": """# Terminal 1 — Gazebo + robot + full Nav2 stack
$ bash run.sh
# Auto-detects GPU/CPU at startup

# Terminal 2 — RViz with Nav2 displays
$ bash rviz.sh

# Terminal 3 — Send navigation goals
$ bash navigate.sh 3.5 3.5 1.57   # single goal
$ bash waypoints.sh                # maze tour
# Or click "2D Nav Goal" in RViz
""",
    },
    {
        "name": "step7_github.png",
        "title_left": "GET THE CODE",
        "title_right": "github.com/Ahmed-m-abbas",
        "lexer": BashLexer(),
        "code": """# Clone the repository
$ git clone https://github.com/Ahmed-m-abbas/\\
      ros2-autonomous-nav.git

# Build with colcon
$ cd ~/ros2_ws
$ colcon build --packages-select \\
      ros2_autonomous_nav --symlink-install

# Source and launch
$ source install/setup.bash
$ bash run.sh
""",
    },
]


def main():
    print(f"Rendering {len(SNIPPETS)} code snippet cards into {OUT_DIR}")
    for s in SNIPPETS:
        out = os.path.join(OUT_DIR, s["name"])
        make_card(s["title_left"], s["title_right"], s["code"],
                  s["lexer"], out)
    print("Done.")


if __name__ == "__main__":
    main()
