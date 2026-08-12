"""YOLOv8n ONNX モデルのダウンロードスクリプト。

https://github.com/ultralytics/assets/releases/latest/download/yolov8n.onnx
からモデルを models/ ディレクトリへ保存する。既にダウンロード済みなら
何もしない (冪等)。サイズの簡易チェックで途中終了を検出する。

使い方:
    python3 src/meister_vision/scripts/download_model.py
    ros2 run meister_vision download_model
"""
from __future__ import annotations

import argparse
import sys
import tempfile
import urllib.request
from pathlib import Path

__all__ = ["MODEL_URL", "MODEL_FILENAME", "download_model",
           "default_output_dir", "main"]

MODEL_URL = "https://github.com/ultralytics/assets/releases/latest/download/yolov8n.onnx"
MODEL_FILENAME = "yolov8n.onnx"
# サイズの簡易チェック。yolov8n.onnx は実測およそ 12.3 MiB。
# 不完全なダウンロード (数KB) を検出するための緩い範囲。
MIN_BYTES = 5 * 1024 * 1024   # 5 MiB
MAX_BYTES = 30 * 1024 * 1024  # 30 MiB


def default_output_dir() -> Path:
    """モデルの保存先ディレクトリを返す。

    1. インストール済みなら共有ディレクトリ (install/share/meister_vision/models)
    2. それ以外はソースツリーの src/meister_vision/models
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("meister_vision")) / "models"
        if share.parent.exists():
            return share
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / "models"


def download_model(output_dir: Path, url: str = MODEL_URL) -> Path:
    """モデルを output_dir にダウンロードする (冪等)。

    Returns:
        保存先のモデルファイルパス
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    dest = output_dir / MODEL_FILENAME

    if dest.exists():
        size = dest.stat().st_size
        if MIN_BYTES <= size <= MAX_BYTES:
            print(f"[download_model] 既に存在するためスキップ: {dest} "
                  f"({size / 1024 / 1024:.1f} MiB)")
            return dest
        print(f"[download_model] 既存ファイルのサイズ異常 ({size} bytes) のため再取得")
        dest.unlink()

    print(f"[download_model] ダウンロード中: {url}")
    # urllib は 302 リダイレクトを自動追跡する
    with urllib.request.urlopen(url, timeout=60) as resp, \
            tempfile.NamedTemporaryFile(
                dir=output_dir, prefix=".yolov8n.", delete=False) as tmp:
        while True:
            chunk = resp.read(1024 * 1024)
            if not chunk:
                break
            tmp.write(chunk)
        tmp_path = Path(tmp.name)
        size = tmp_path.stat().st_size

    if not (MIN_BYTES <= size <= MAX_BYTES):
        tmp_path.unlink(missing_ok=True)
        raise RuntimeError(
            f"モデルのサイズが想定範囲外です: {size} bytes "
            f"(期待: {MIN_BYTES}〜{MAX_BYTES} bytes)。ダウンロードが途中終了した可能性があります。")

    tmp_path.rename(dest)
    print(f"[download_model] 保存完了: {dest} ({size / 1024 / 1024:.1f} MiB)")
    return dest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="YOLOv8n ONNX モデルを models/ にダウンロードする")
    parser.add_argument(
        "--output-dir", type=Path, default=default_output_dir(),
        help="保存先ディレクトリ (既定: パッケージ共有ディレクトリの models/)")
    args = parser.parse_args(argv)

    try:
        download_model(args.output_dir)
    except Exception as exc:  # noqa: BLE001
        print(f"[download_model] エラー: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
