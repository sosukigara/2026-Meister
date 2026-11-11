#!/bin/bash
# 手骨格モデルを取得して assets に置く。モデルは git 管理外 (*.task)。
# 使い方: ./tools/fetch_model.sh
# 出典: MediaPipe 公式配布 (https://storage.googleapis.com/mediapipe-models/...)。2026-11-11 に HEAD で到達確認済み。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ASSETS="$HERE/../app/src/main/assets"
URL="https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"

mkdir -p "$ASSETS"
if [ -f "$ASSETS/hand_landmarker.task" ]; then
    echo "already present: $ASSETS/hand_landmarker.task"
    exit 0
fi
curl -L --fail -o "$ASSETS/hand_landmarker.task" "$URL"
ls -lh "$ASSETS/hand_landmarker.task"
