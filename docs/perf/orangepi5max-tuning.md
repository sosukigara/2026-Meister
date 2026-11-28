# Orange Pi 5 Max (RK3588) 向け自律走行・手先認識 最適化ガイド

## 1. 起動モード

```bash
# マッピング（10Hz LiDAR + 差分IMU/エンコーダ + Smac Hybrid-A* + 軽量 Ackermann MPPI）
ros2 launch umeonigiri_autonomy real_robot.launch.py \
  nav_mode:=mapping controller_profile:=mppi

# 事前地図での自律走行（AMCL 非ホロノミック設定）
ros2 launch umeonigiri_autonomy real_robot.launch.py \
  nav_mode:=localization localization_backend:=amcl \
  map:=/path/to/map.yaml

# 事前地図での自律走行（SLAM Toolbox localization モード）
ros2 launch umeonigiri_autonomy real_robot.launch.py \
  nav_mode:=localization localization_backend:=slam_toolbox \
  map:=/path/to/map_posegraph

# 超低CPUフォールバック（切り返し対応 RPP）
ros2 launch umeonigiri_autonomy real_robot.launch.py \
  nav_mode:=localization controller_profile:=rpp map:=/path/to/map.yaml
```

エンコーダ実測を `FB_STATE` 経由で有効にする場合は `use_encoder_feedback:=true` を付与する。

## 2. RK3588 big.LITTLE コア割当（既定 `rk3588_affinity:=true`）

- **Cortex-A76 (CPU 4-7)**: `controller_server` (MPPI), `planner_server` (Smac Hybrid-A*), `slam_toolbox` / `amcl`, `yolo_grasp_node`
- **Cortex-A55 (CPU 0-3)**: `d500_node`, `umeonigiri_serial_bridge`, `robot_state_publisher`, `velocity_smoother`, `behavior_server`, `bt_navigator`
- `OMP_NUM_THREADS=2` / `OPENBLAS_NUM_THREADS=2` / `opencv_threads:=2` により OpenCV・Eigen・Ceres のスレッド競合を抑止。

## 3. 手先 25FPS 物体検出モデル変換（RK3588 3 NPU コア並列）

```bash
# INT8（例: yolov8m / yolo11m）
python3 tools/rknn/convert_yolov8.py \
  --onnx /tmp/yolov8m.onnx --dataset /tmp/coco_eval/calib.txt \
  --quant i8 --out ros/src/umeonigiri_vision/models/yolov8m-zoo.rknn

# FP16（小物体 Recall 重視・3 NPU コア並列で 25FPS 維持）
python3 tools/rknn/convert_yolov8.py \
  --onnx /tmp/yolo11s.onnx --quant fp16 \
  --out ros/src/umeonigiri_vision/models/yolo11s-zoo.rknn
```

`librga` が導入済みの環境では CMake が自動検出して `HAVE_LIBRGA` を有効化する。未導入時は OpenCV パスへ自動フォールバックする。
