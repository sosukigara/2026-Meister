#!/usr/bin/env python3
"""ナビの安定性テスト: 固定ゴールを順に投げ、成否・最終誤差・所要時間を記録する。

画像認識を止めないため、走行中も Web UI の認識モニターは ON のままにする。
"""
import json
import math
import subprocess
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


def main():
    results = []
    for idx, (gx, gy) in enumerate(GOALS):
        start = time.monotonic()
        post("/api/nav", {"waypoints": [{"x": gx, "y": gy, "yaw": 0.0}]})
        state = "unknown"
        while time.monotonic() - start < TIMEOUT_S:
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
                        "final": [round(pose.get("x", 0), 2), round(pose.get("y", 0), 2)],
                        "err_m": None if err is None else round(err, 3)})
        print(json.dumps(results[-1], ensure_ascii=False), flush=True)
        post("/api/cancel", {})
        time.sleep(2)

    ok = sum(1 for r in results if r["state"] == "succeeded")
    errs = [r["err_m"] for r in results if r["err_m"] is not None]
    print(f"--- success {ok}/{len(results)}  "
          f"err max {max(errs):.3f} m  mean {sum(errs)/len(errs):.3f} m")
    with open("/tmp/stability.json", "w") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
