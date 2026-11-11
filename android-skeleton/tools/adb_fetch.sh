#!/bin/bash
# 実機の状態を ADB で一括取得する。PC に USB 接続 + USB デバッグ ON が前提。
# 使い方: ./tools/adb_fetch.sh [出力先]
# 取得物: devices / logcat(MeisterSkeleton) / screen.png / battery
set -euo pipefail

OUT="${1:-/tmp/meister-skeleton-adb-$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$OUT"

if ! adb devices | grep -q $'\tdevice$'; then
    echo "ERROR: adb device が無い。USB 接続と USB デバッグを確認。" >&2
    adb devices | tee "$OUT/devices.txt"
    exit 1
fi
adb devices -l > "$OUT/devices.txt"

# アプリの要約ログ (1Hz) を抜く。無ければ空になる。
adb logcat -d MeisterSkeleton:I '*:S' > "$OUT/logcat.txt" 2>/dev/null || true

# 画面そのまま。PNG なのでそのまま確認できる。
adb exec-out screencap -p > "$OUT/screen.png"

# 発熱の当たりを付ける (充電・温度)。
adb shell dumpsys battery > "$OUT/battery.txt" 2>/dev/null || true

echo "saved to $OUT"
ls -lh "$OUT"
