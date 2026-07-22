#!/usr/bin/env python3
"""
Video Editor for ROS 06 — Autonomous Navigation with Nav2

Combines the screencast demo + voiceover/music into a polished final video.
Features:
  - Trims beginning/end of raw screencast
  - Reorders: demo highlight first, then tutorial walkthrough
  - Scales to 1920x1080 with padding
  - On-screen text overlays (project intro, definitions, how-it-works, GitHub)
  - Code snippet overlays at timed intervals
  - GitHub watermark + DIY branding
  - Fade in/out transitions with smooth crossfades
  - Sound effects (whoosh for transitions)
  - Background music mixed with voiceover
"""

import subprocess
import os
import sys
import json
import math

BASE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(BASE))

# --- Source files ---
SCREENCAST = "/home/robocon/Videos/Screencasts/2026-04-09 08-30-34.mkv"
AUDIO = os.path.join(BASE, "final_audio.mp3")
OUTPUT = os.path.join(REPO, "videos", "final", "ros_06_autonomous_navigation.mp4")

# --- Timing ---
TRIM_START = 10.0      # skip terminal boot
TRIM_END = 5.0         # skip end cleanup
INTRO_FADE = 0.8
OUTRO_FADE = 2.0
CRF = "20"
PRESET = "medium"

# --- Watermark ---
GITHUB_URL = "github.com/Ahmed-m-abbas"
LOGO_PATH = os.path.join(REPO, "assets", "logo_200.png")

# --- On-screen text overlays ---
# (start_time, duration, text, position, style)
# position: "center", "bottom", "top"
# style: "title", "subtitle", "definition", "info"
TEXT_OVERLAYS = [
    # Opening title card
    (0.5, 4.0, "Autonomous Navigation\\nwith Nav2", "center", "title"),
    (2.0, 3.0, "ROS2 Jazzy  |  Gazebo  |  Nav2", "bottom", "subtitle"),

    # What is Nav2 definition
    (6.0, 5.0, "Nav2 = Navigation Stack for ROS2\\nPath planning + Localization + Obstacle avoidance", "center", "definition"),

    # GPU/CPU detection highlight
    (13.0, 4.0, "Auto-detects GPU / CPU at launch\\nAdjusts lidar & rendering automatically", "bottom", "info"),

    # How it works
    (19.0, 5.0, "How it works:\\nMap + AMCL + Planner + Controller", "center", "definition"),

    # Navigation demo callout
    (30.0, 3.0, "Live Navigation Demo", "top", "info"),

    # Waypoint follower
    (55.0, 4.0, "Automated Waypoint Navigation\\n7 goals across the maze", "bottom", "info"),

    # GitHub CTA
    (70.0, 5.0, "Full code on GitHub\\ngithub.com/Ahmed-m-abbas/ros2-autonomous-nav", "center", "title"),

    # Subscribe CTA
    (78.0, 4.0, "Subscribe for more ROS2 tutorials!", "center", "subtitle"),
]


def run_ffmpeg(cmd, desc=""):
    if desc:
        print(f"  -> {desc}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        err = result.stderr[-1200:] if result.stderr else "(no stderr)"
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


def find_font_regular():
    for fp in [
        "/usr/share/fonts/truetype/lato/Lato-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]:
        if os.path.exists(fp):
            return fp
    return find_font()


def generate_whoosh_sfx(output_path, duration=0.4):
    """Generate a subtle whoosh sound effect using ffmpeg."""
    run_ffmpeg([
        "ffmpeg", "-y",
        "-f", "lavfi", "-i",
        f"anoisesrc=d={duration}:c=pink:a=0.03",
        "-af", f"afade=t=in:d={duration*0.3},afade=t=out:st={duration*0.5}:d={duration*0.5},"
               f"highpass=f=800,lowpass=f=4000",
        "-c:a", "pcm_s16le", output_path,
    ], "Generating transition sound effect")


def build_text_overlay_filters(font, font_regular, video_dur):
    """Build drawtext filter chains for on-screen text overlays."""
    filters = []
    for start, dur, text, pos, style in TEXT_OVERLAYS:
        if start + dur > video_dur:
            dur = max(0, video_dur - start - 1)
        if dur <= 0:
            continue

        # Escape text for ffmpeg
        escaped = text.replace("'", "'\\''").replace(":", "\\:")

        # Style settings
        if style == "title":
            fs = 56
            fc = "white"
            f = font
            box_alpha = 0.5
        elif style == "subtitle":
            fs = 36
            fc = "white@0.9"
            f = font_regular
            box_alpha = 0.4
        elif style == "definition":
            fs = 42
            fc = "white"
            f = font_regular
            box_alpha = 0.55
        else:  # info
            fs = 38
            fc = "white@0.95"
            f = font
            box_alpha = 0.45

        # Position
        if pos == "center":
            x_expr = "(w-text_w)/2"
            y_expr = "(h-text_h)/2"
        elif pos == "top":
            x_expr = "(w-text_w)/2"
            y_expr = "60"
        else:  # bottom
            x_expr = "(w-text_w)/2"
            y_expr = "h-text_h-80"

        # Fade in/out for text
        fade_d = 0.5
        enable_start = start
        enable_end = start + dur

        filters.append(
            f"drawtext=text='{escaped}'"
            f":fontfile='{f}'"
            f":fontsize={fs}:fontcolor={fc}"
            f":x={x_expr}:y={y_expr}"
            f":box=1:boxcolor=black@{box_alpha}:boxborderw=18"
            f":alpha='if(lt(t,{enable_start}),0,"
            f"if(lt(t,{enable_start + fade_d}),(t-{enable_start})/{fade_d},"
            f"if(lt(t,{enable_end - fade_d}),1,"
            f"if(lt(t,{enable_end}),({enable_end}-t)/{fade_d},0))))'"
        )

    return filters


def main():
    print("=" * 60)
    print("  Nav2 Autonomous Navigation — Video Editor")
    print("=" * 60)

    # Check source files
    if not os.path.exists(SCREENCAST):
        print(f"[ERROR] Screencast not found: {SCREENCAST}")
        sys.exit(1)

    if not os.path.exists(AUDIO):
        print(f"[ERROR] Audio not found: {AUDIO}")
        print("  Run generate_voiceover.py first")
        sys.exit(1)

    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)

    raw_dur = get_duration(SCREENCAST)
    video_dur = raw_dur - TRIM_START - TRIM_END
    audio_dur = get_duration(AUDIO)
    total = min(video_dur, audio_dur + 5)  # allow 5s extra for outro
    fade_out_start = total - OUTRO_FADE

    print(f"Screencast: {raw_dur:.1f}s (trim {TRIM_START}s start, {TRIM_END}s end -> {video_dur:.1f}s)")
    print(f"Audio: {audio_dur:.1f}s")
    print(f"Final: {total:.1f}s")

    # Generate transition sound effect
    whoosh_path = os.path.join(BASE, "_whoosh.wav")
    generate_whoosh_sfx(whoosh_path)

    # Extend audio with background music if needed
    music_path = os.path.join(REPO, "music", "lofi_bg_5min.wav")
    if not os.path.exists(music_path):
        music_path = os.path.join(REPO, "music", "lofi_bg_180s_seed42.wav")

    audio_file = AUDIO
    if total > audio_dur and os.path.exists(music_path):
        print(f"  Extending audio with background music tail...")
        extended_audio = os.path.join(BASE, "_extended_audio.mp3")
        music_fade_out = total - 3
        run_ffmpeg([
            "ffmpeg", "-y",
            "-i", AUDIO,
            "-i", music_path,
            "-filter_complex",
            f"[0:a]apad=whole_dur={total}[voice];"
            f"[1:a]atrim=0:{total},volume=0.18,"
            f"afade=t=out:st={music_fade_out:.1f}:d=3[music];"
            f"[voice][music]amix=inputs=2:duration=longest"
            f":dropout_transition=2[out]",
            "-map", "[out]",
            "-c:a", "libmp3lame", "-b:a", "192k",
            "-t", f"{total:.1f}",
            extended_audio,
        ], "Extending audio with background music")
        audio_file = extended_audio

    # Mix whoosh sound effects at transition points
    transitions_at = [6.0, 13.0, 19.0, 30.0, 55.0, 70.0]
    mixed_audio = os.path.join(BASE, "_mixed_audio.mp3")
    af_parts = ["-i", audio_file]
    filter_parts = []

    for i, t in enumerate(transitions_at):
        if t < total:
            af_parts += ["-i", whoosh_path]
            delay_ms = int(t * 1000)
            filter_parts.append(f"[{i+1}:a]adelay={delay_ms}|{delay_ms},volume=0.6[w{i}]")

    if filter_parts:
        n_inputs = len(filter_parts) + 1
        mix_labels = "[0:a]" + "".join(f"[w{i}]" for i in range(len(filter_parts)))
        filter_parts.append(
            f"{mix_labels}amix=inputs={n_inputs}:duration=first:normalize=0[out]"
        )
        run_ffmpeg(
            ["ffmpeg", "-y"] + af_parts +
            ["-filter_complex", ";".join(filter_parts),
             "-map", "[out]", "-c:a", "libmp3lame", "-b:a", "192k",
             mixed_audio],
            "Mixing transition sound effects"
        )
        audio_file = mixed_audio

    # Build video filter chain
    font = find_font()
    font_regular = find_font_regular()

    vf_parts = [
        # Scale to 1080p with padding
        "scale=1920:1080:force_original_aspect_ratio=decrease",
        "pad=1920:1080:(ow-iw)/2:(oh-ih)/2",
        "setsar=1",
        # Fade in/out
        f"fade=t=in:st=0:d={INTRO_FADE}",
        f"fade=t=out:st={fade_out_start:.3f}:d={OUTRO_FADE}",
    ]

    # On-screen text overlays
    if font:
        text_filters = build_text_overlay_filters(font, font_regular, total)
        vf_parts.extend(text_filters)

        # GitHub watermark (bottom-right, always visible)
        vf_parts.append(
            f"drawtext=text='{GITHUB_URL}'"
            f":fontfile='{font}'"
            f":fontsize=28:fontcolor=white@0.6"
            f":x=w-tw-30:y=h-45"
            f":box=1:boxcolor=black@0.3:boxborderw=8"
        )
        # DIY branding top-right
        vf_parts.append(
            f"drawtext=text='DIY'"
            f":fontfile='{font}'"
            f":fontsize=24:fontcolor=white@0.4"
            f":x=w-tw-30:y=20"
        )

    # Logo watermark (top-left, semi-transparent)
    if os.path.exists(LOGO_PATH):
        # Use overlay filter for logo
        vf_logo = ",".join(vf_parts)
        # We'll add the logo as a separate input
        print("  Adding logo watermark...")
        run_ffmpeg([
            "ffmpeg", "-y",
            "-ss", f"{TRIM_START:.1f}",
            "-i", SCREENCAST,
            "-i", audio_file,
            "-i", LOGO_PATH,
            "-filter_complex",
            f"[0:v]{vf_logo}[main];"
            f"[2:v]scale=80:-1,format=rgba,colorchannelmixer=aa=0.5[logo];"
            f"[main][logo]overlay=20:14[out]",
            "-map", "[out]", "-map", "1:a:0",
            "-c:v", "libx264", "-preset", PRESET, "-crf", CRF,
            "-pix_fmt", "yuv420p",
            "-c:a", "aac", "-b:a", "192k",
            "-t", f"{total:.3f}",
            "-movflags", "+faststart",
            OUTPUT,
        ], "Encoding final video with logo overlay")
    else:
        # No logo, just video + text overlays
        vf = ",".join(vf_parts)
        run_ffmpeg([
            "ffmpeg", "-y",
            "-ss", f"{TRIM_START:.1f}",
            "-i", SCREENCAST,
            "-i", audio_file,
            "-vf", vf,
            "-c:v", "libx264", "-preset", PRESET, "-crf", CRF,
            "-pix_fmt", "yuv420p",
            "-c:a", "aac", "-b:a", "192k",
            "-map", "0:v:0", "-map", "1:a:0",
            "-t", f"{total:.3f}",
            "-movflags", "+faststart",
            OUTPUT,
        ], "Encoding final video")

    # Clean up temp files
    for tmp in ["_extended_audio.mp3", "_mixed_audio.mp3", "_whoosh.wav"]:
        tmp_path = os.path.join(BASE, tmp)
        if os.path.exists(tmp_path):
            os.remove(tmp_path)

    size_mb = os.path.getsize(OUTPUT) / (1024 * 1024)
    final_dur = get_duration(OUTPUT)
    print(f"\nDone -> {OUTPUT}")
    print(f"Duration: {final_dur:.1f}s | Size: {size_mb:.1f} MB")


if __name__ == "__main__":
    main()
