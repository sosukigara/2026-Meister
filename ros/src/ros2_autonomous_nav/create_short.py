#!/usr/bin/env python3
"""
Create a YouTube Short (9:16, ~45s) for the Nav2 autonomous navigation project.

Selects key clips from the screencast, adds on-screen text overlays,
watermark, and background music. Outputs 1080x1920 vertical video.
"""

import subprocess
import os
import sys
import json

BASE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(BASE))

SCREENCAST = "/home/robocon/Videos/Screencasts/2026-04-09 08-30-34.mkv"
OUTPUT = os.path.join(REPO, "shorts", "ros_06_autonomous_navigation_short.mp4")
LOGO_PATH = os.path.join(REPO, "assets", "logo_200.png")

# Short duration target
TARGET_DUR = 45.0

# Clip segments from the screencast (start_time, duration)
# These should capture the most visually interesting navigation moments
CLIPS = [
    (35.0, 10.0),   # Navigation in progress — robot moving through maze
    (55.0, 8.0),    # Path planning visible in RViz
    (80.0, 8.0),    # Robot reaching a goal / turning
    (120.0, 8.0),   # Multi-waypoint navigation
    (150.0, 8.0),   # Final navigation + terminals visible
]

# On-screen text for the short (time_in_short, duration, text)
TEXTS = [
    (0.5, 4.0,  "Autonomous Navigation\\nwith Nav2"),
    (5.0, 3.5,  "ROS2 Jazzy + Gazebo"),
    (10.0, 4.0, "Auto-detects GPU / CPU"),
    (16.0, 4.0, "AMCL Localization\\n+ Path Planning"),
    (22.0, 4.0, "DWB Controller\\nAvoids obstacles in real-time"),
    (28.0, 4.0, "Waypoint Navigation\\nVisits every room"),
    (34.0, 4.0, "Full code on GitHub\\ngithub.com/Ahmed-m-abbas"),
    (40.0, 4.0, "Subscribe for more!"),
]

GITHUB_URL = "github.com/Ahmed-m-abbas"


def run_ffmpeg(cmd, desc=""):
    if desc:
        print(f"  -> {desc}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        err = result.stderr[-1000:] if result.stderr else "(no stderr)"
        print(f"  [ERROR] {err}", file=sys.stderr)
        raise RuntimeError(f"ffmpeg failed: {desc}")


def get_duration(filepath):
    result = subprocess.run(
        ["ffprobe", "-v", "quiet", "-print_format", "json",
         "-show_format", filepath],
        capture_output=True, text=True,
    )
    return float(json.loads(result.stdout)["format"]["duration"])


def find_font():
    for fp in [
        "/usr/share/fonts/truetype/lato/Lato-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    ]:
        if os.path.exists(fp):
            return fp
    return None


def main():
    print("=" * 50)
    print("  Nav2 — YouTube Short Creator")
    print("=" * 50)

    if not os.path.exists(SCREENCAST):
        print(f"[ERROR] Screencast not found: {SCREENCAST}")
        sys.exit(1)

    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)

    raw_dur = get_duration(SCREENCAST)
    print(f"Source: {raw_dur:.1f}s")

    font = find_font()

    # Step 1: Extract and concat clips
    clip_files = []
    for i, (start, dur) in enumerate(CLIPS):
        if start + dur > raw_dur:
            dur = max(0, raw_dur - start)
        if dur <= 0:
            continue
        clip_path = os.path.join(BASE, f"_short_clip_{i}.mp4")
        clip_files.append(clip_path)

        run_ffmpeg([
            "ffmpeg", "-y",
            "-ss", f"{start:.1f}",
            "-i", SCREENCAST,
            "-t", f"{dur:.1f}",
            "-vf", "scale=1080:1920:force_original_aspect_ratio=decrease,"
                   "pad=1080:1920:(ow-iw)/2:(oh-ih)/2,setsar=1",
            "-c:v", "libx264", "-preset", "fast", "-crf", "22",
            "-pix_fmt", "yuv420p",
            "-an",
            clip_path,
        ], f"Extracting clip {i+1} ({start:.0f}s-{start+dur:.0f}s)")

    # Step 2: Create concat list
    concat_list = os.path.join(BASE, "_short_concat.txt")
    with open(concat_list, "w") as f:
        for clip in clip_files:
            f.write(f"file '{clip}'\n")

    # Step 3: Concat clips
    concat_raw = os.path.join(BASE, "_short_concat.mp4")
    run_ffmpeg([
        "ffmpeg", "-y",
        "-f", "concat", "-safe", "0",
        "-i", concat_list,
        "-c", "copy",
        concat_raw,
    ], "Concatenating clips")

    concat_dur = get_duration(concat_raw)
    total = min(concat_dur, TARGET_DUR)
    fade_out = total - 1.5

    # Step 4: Add background music
    music_path = os.path.join(REPO, "music", "lofi_bg_180s_seed42.wav")
    if not os.path.exists(music_path):
        music_path = os.path.join(REPO, "music", "lofi_bg_2min.wav")

    # Step 5: Build text overlay filters
    vf_parts = [
        # Crossfade transitions between clips (subtle brightness pulse)
        f"fade=t=in:st=0:d=0.5",
        f"fade=t=out:st={fade_out:.1f}:d=1.5",
    ]

    if font:
        # On-screen text overlays
        for start, dur, text in TEXTS:
            if start >= total:
                continue
            escaped = text.replace("'", "'\\''").replace(":", "\\:")
            fade_d = 0.4
            end = start + dur
            vf_parts.append(
                f"drawtext=text='{escaped}'"
                f":fontfile='{font}'"
                f":fontsize=48:fontcolor=white"
                f":x=(w-text_w)/2:y=(h-text_h)/2-100"
                f":box=1:boxcolor=black@0.55:boxborderw=16"
                f":alpha='if(lt(t,{start}),0,"
                f"if(lt(t,{start + fade_d}),(t-{start})/{fade_d},"
                f"if(lt(t,{end - fade_d}),1,"
                f"if(lt(t,{end}),({end}-t)/{fade_d},0))))'"
            )

        # Watermark at bottom
        vf_parts.append(
            f"drawtext=text='{GITHUB_URL}'"
            f":fontfile='{font}'"
            f":fontsize=28:fontcolor=white@0.7"
            f":x=(w-text_w)/2:y=h-80"
            f":box=1:boxcolor=black@0.3:boxborderw=8"
        )

        # DIY branding top-right
        vf_parts.append(
            f"drawtext=text='DIY'"
            f":fontfile='{font}'"
            f":fontsize=24:fontcolor=white@0.5"
            f":x=w-tw-20:y=20"
        )

    vf = ",".join(vf_parts)

    # Step 6: Final encode with music + text
    if os.path.exists(music_path):
        run_ffmpeg([
            "ffmpeg", "-y",
            "-i", concat_raw,
            "-i", music_path,
            "-vf", vf,
            "-filter_complex",
            f"[1:a]atrim=0:{total},volume=0.25,"
            f"afade=t=in:d=1,afade=t=out:st={total-2}:d=2[music]",
            "-map", "0:v", "-map", "[music]",
            "-c:v", "libx264", "-preset", "medium", "-crf", "20",
            "-pix_fmt", "yuv420p",
            "-c:a", "aac", "-b:a", "128k",
            "-t", f"{total:.1f}",
            "-movflags", "+faststart",
            OUTPUT,
        ], "Encoding final short with music + overlays")
    else:
        run_ffmpeg([
            "ffmpeg", "-y",
            "-i", concat_raw,
            "-vf", vf,
            "-c:v", "libx264", "-preset", "medium", "-crf", "20",
            "-pix_fmt", "yuv420p",
            "-an",
            "-t", f"{total:.1f}",
            "-movflags", "+faststart",
            OUTPUT,
        ], "Encoding final short")

    # Cleanup
    for f in clip_files + [concat_list, concat_raw]:
        if os.path.exists(f):
            os.remove(f)

    size_mb = os.path.getsize(OUTPUT) / (1024 * 1024)
    final_dur = get_duration(OUTPUT)
    print(f"\nDone -> {OUTPUT}")
    print(f"Duration: {final_dur:.1f}s | Size: {size_mb:.1f} MB")
    print(f"Format: 1080x1920 (9:16 vertical)")


if __name__ == "__main__":
    main()
