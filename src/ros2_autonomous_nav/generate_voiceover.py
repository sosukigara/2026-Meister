#!/usr/bin/env python3
"""Generate timed voiceover for ROS2 - Autonomous Navigation with Nav2.

Voice: en-GB-RyanNeural (British), -10% rate.
Sized to fit a ~150s demo screencast.
"""

import edge_tts
import asyncio
import subprocess
import os
import json

VOICE = "en-GB-RyanNeural"
RATE = "-10%"
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
TOTAL_DURATION = 155
SEGMENT_GAP = 0.35

SEGMENT_TEXTS = [
    # Hook — plays over the cold open (robot navigating through the maze)
    "Watch a ROS2 robot navigate through a maze completely on its own, "
    "planning paths, avoiding walls, and reaching goals autonomously.",

    # Concept — what Nav2 does
    "Nav2 is the navigation stack for ROS2. It takes a map, localises "
    "the robot with AMCL, plans a global path, and tracks it with a local "
    "controller, all while recovering from stuck situations automatically.",

    # GPU/CPU detection
    "This project auto-detects your hardware at launch. If an NVIDIA GPU "
    "is found, it runs a high-resolution lidar at ten hertz. On CPU-only "
    "systems, it drops to a lighter configuration so everything stays smooth.",

    # Map — pre-built or SLAM
    "Navigation needs a map. We include a pre-built occupancy grid generated "
    "from the known wall geometry, or you can build your own map using SLAM "
    "mode, just like the previous tutorial.",

    # Nav2 params
    "The nav2 params YAML configures the full stack. AMCL for localisation, "
    "NavfnPlanner for global path planning, the DWB local controller for "
    "trajectory tracking, and recovery behaviors like spin and backup.",

    # Launch file
    "The launch file brings up Gazebo, the robot, the ROS bridge, and the "
    "entire Nav2 stack. It detects the GPU at startup and picks the matching "
    "URDF and rendering settings automatically.",

    # RViz config
    "In RViz we display the map, both costmaps, the global and local paths, "
    "the AMCL particle cloud, and the laser scan. You can send goals by "
    "clicking the 2D Nav Goal button and clicking anywhere on the map.",

    # Running it
    "To run it, bash run dot sh launches everything. Bash rviz dot sh opens "
    "the navigation display. Then click a goal in RViz, or use navigate dot sh "
    "from the terminal, or run waypoints dot sh for an automated maze tour.",

    # Demo — plays during the navigation footage
    "And there it goes. The planner finds a path through the corridors, the "
    "DWB controller steers the robot along it, and when it encounters a tight "
    "turn or obstacle, the behavior tree triggers a recovery spin or backup "
    "before replanning.",

    # Waypoint follower
    "The waypoint follower script uses nav2 simple commander to send the robot "
    "through seven goals across the entire maze, visiting every room and "
    "corridor, then returning to the start.",

    # Outro
    "Full code is on GitHub, link in the description. Subscribe for more ROS2 "
    "tutorials.",
]


async def generate_segment(text, output_path):
    communicate = edge_tts.Communicate(text, VOICE, rate=RATE)
    await communicate.save(output_path)


def probe_duration(path):
    r = subprocess.run(
        ["ffprobe", "-v", "quiet", "-print_format", "json",
         "-show_format", path],
        capture_output=True, text=True,
    )
    return float(json.loads(r.stdout)["format"]["duration"])


async def main():
    print("Generating voice segments...")
    segment_files = []
    for i, text in enumerate(SEGMENT_TEXTS):
        seg_path = os.path.join(OUTPUT_DIR, f"_seg_{i:02d}.mp3")
        segment_files.append(seg_path)
        print(f"  {i+1}/{len(SEGMENT_TEXTS)}: {text[:60]}...")
        await generate_segment(text, seg_path)

    print("\nMeasuring segment durations and laying them out sequentially...")
    durations = [probe_duration(p) for p in segment_files]
    starts = []
    cursor = 0.0
    for d in durations:
        starts.append(cursor)
        cursor += d + SEGMENT_GAP

    voice_total = cursor - SEGMENT_GAP
    print(f"  Total speech: {voice_total:.2f}s  (budget: {TOTAL_DURATION}s)")
    if voice_total > TOTAL_DURATION:
        print(f"  WARNING: speech overruns the {TOTAL_DURATION}s budget by "
              f"{voice_total - TOTAL_DURATION:.2f}s — shorten some segments.")
    for i, (s, d) in enumerate(zip(starts, durations)):
        print(f"    seg {i+1}: start={s:6.2f}s  dur={d:5.2f}s  end={s+d:6.2f}s")

    print("\nCombining segments with timing...")
    silent_path = os.path.join(OUTPUT_DIR, "_silent.wav")
    subprocess.run([
        "ffmpeg", "-y", "-f", "lavfi", "-i", "anullsrc=r=44100:cl=mono",
        "-t", str(TOTAL_DURATION), "-c:a", "pcm_s16le", silent_path
    ], capture_output=True)

    inputs = ["-i", silent_path]
    filter_parts = []
    for i, (start_time, seg_path) in enumerate(zip(starts, segment_files)):
        inputs += ["-i", seg_path]
        delay_ms = int(start_time * 1000)
        filter_parts.append(f"[{i+1}:a]adelay={delay_ms}|{delay_ms}[s{i}]")

    mix_inputs = "[0:a]" + "".join(f"[s{i}]" for i in range(len(segment_files)))
    n = len(segment_files) + 1
    filter_parts.append(
        f"{mix_inputs}amix=inputs={n}:duration=first:normalize=0[voice]"
    )

    voice_path = os.path.join(OUTPUT_DIR, "voiceover.wav")
    subprocess.run(
        ["ffmpeg", "-y"] + inputs +
        ["-filter_complex", ";".join(filter_parts),
         "-map", "[voice]", "-c:a", "pcm_s16le", "-ar", "44100", voice_path],
        capture_output=True
    )
    print("Voiceover generated: voiceover.wav")

    print("\nAdding background music...")
    music_candidates = [
        os.path.join(OUTPUT_DIR, "..", "..", "music", "lofi_ros06_nav_155s.wav"),
        os.path.join(OUTPUT_DIR, "..", "..", "music", "lofi_ros05_slam_135s.wav"),
        os.path.join(OUTPUT_DIR, "..", "..", "music", "lofi_ros04_camera_130s.wav"),
        os.path.join(OUTPUT_DIR, "..", "..", "music", "lofi_ros03_lidar_98s.wav"),
    ]
    music_path = next((p for p in music_candidates if os.path.exists(p)),
                      music_candidates[-1])
    print(f"  using music: {os.path.basename(music_path)}")

    final_path = os.path.join(OUTPUT_DIR, "final_audio.mp3")
    fade_out_start = TOTAL_DURATION - 3

    result = subprocess.run([
        "ffmpeg", "-y", "-i", voice_path, "-i", music_path,
        "-filter_complex",
        f"[0:a]volume=1.0[voice];"
        f"[1:a]volume=0.18,afade=t=in:st=0:d=2,"
        f"afade=t=out:st={fade_out_start}:d=3[music];"
        f"[voice][music]amix=inputs=2:duration=first:dropout_transition=2[out]",
        "-map", "[out]", "-c:a", "libmp3lame", "-b:a", "192k",
        "-t", str(TOTAL_DURATION), final_path
    ], capture_output=True, text=True)

    if result.returncode == 0:
        size_mb = os.path.getsize(final_path) / (1024 * 1024)
        print(f"Final audio saved: final_audio.mp3 ({size_mb:.1f} MB)")
    else:
        print(f"Error: {result.stderr[:500]}")

    for seg_path in segment_files:
        os.remove(seg_path)
    os.remove(silent_path)
    print("\nDone!")


if __name__ == "__main__":
    asyncio.run(main())
