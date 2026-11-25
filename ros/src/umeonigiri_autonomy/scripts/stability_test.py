#!/usr/bin/env python3
"""ナビの安定性テスト: 固定ゴールを順に投げ、成否・最終誤差・所要時間を記録する。

画像認識を止めないため、走行中も Web UI の認識モニターは ON のままにする。
"""
import argparse
import json
import math
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8088"
GOALS = [
    (-0.65, 2.60),
    (-0.60, 0.90),
    (0.50, -2.90),
    (0.30, -0.90),
    (-3.20, -0.55),
    (3.20, 1.05),
]
TIMEOUT_S = 180.0


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.load(r)


def post(path, payload):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default=BASE, help="Web UI の URL")
    parser.add_argument("--out", default="/tmp/stability.json")
    parser.add_argument("--goals", help="[[x, y], ...] の JSON ファイル")
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S)
    parser.add_argument("--post-delay", type=float, default=2.0,
                        help="ゴール間の待ち時間 [s]")
    args = parser.parse_args(argv)
    base = args.base.rstrip("/")
    goals = GOALS
    if args.goals:
        with open(args.goals, encoding="utf-8") as handle:
            goals = [tuple(goal) for goal in json.load(handle)]

    def get(path):
        with urllib.request.urlopen(base + path, timeout=5) as response:
            return json.load(response)

    def post(path, payload):
        data = json.dumps(payload).encode()
        req = urllib.request.Request(base + path, data=data,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as response:
            return json.load(response)

    results = []
    for idx, (gx, gy) in enumerate(goals):
        start = time.monotonic()
        started_at = time.time()
        post("/api/nav", {"waypoints": [{"x": gx, "y": gy, "yaw": 0.0}]})
        state = "unknown"
        while time.monotonic() - start < args.timeout:
            time.sleep(1.0)
            st = get("/api/status")
            state = st.get("state", "unknown")
            if state not in ("sending", "navigating", "starting"):
                break
        elapsed = time.monotonic() - start
        pose = get("/api/pose")
        err = None
        if pose.get("has_pose"):
            err = math.hypot(pose["x"] - gx, pose["y"] - gy)
        results.append({"goal": [gx, gy], "state": state, "seconds": round(elapsed, 1),
                        "started_at": round(started_at, 3),
                        "ended_at": round(time.time(), 3),
                        "final": [round(pose.get("x", 0), 2), round(pose.get("y", 0), 2)],
                        "err_m": None if err is None else round(err, 3)})
        print(json.dumps(results[-1], ensure_ascii=False), flush=True)
        post("/api/cancel", {})
        time.sleep(args.post_delay)

    ok = sum(1 for r in results if r["state"] == "succeeded")
    errs = [r["err_m"] for r in results if r["err_m"] is not None]
    # 姿勢が 1 度も取れないと errs は空になる。max([]) と 0 除算で落ちると、
    # 走らせた全部の結果を書く前に落ちて JSON が残らない。
    summary = "err 不明 (姿勢が一度も取れませんでした)"
    if errs:
        summary = f"err max {max(errs):.3f} m  mean {sum(errs) / len(errs):.3f} m"
    print(f"--- success {ok}/{len(results)}  {summary}")
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=1)
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    # 以前は常に 0 で終わっていたため、失敗しても呼び出し側から見えなかった。
    sys.exit(main())
