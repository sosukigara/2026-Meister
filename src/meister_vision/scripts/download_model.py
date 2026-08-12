#!/usr/bin/env python3
"""download_model のスタンドアロンラッパー。

インストール前でもソースツリーから直接実行できるようにする:
    python3 src/meister_vision/scripts/download_model.py
"""
import sys
from pathlib import Path

# src/meister_vision を sys.path に追加して meister_vision パッケージを解決
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from meister_vision.download_model import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
