#!/bin/bash
# Meister — Build script
#
# Usage:
#   ./build.sh                    # 通常ビルド（symlink-install）
#   ./build.sh --deps             # 依存パッケージもインストールしてからビルド
#   ./build.sh --release          # Release ビルド（最適化有効）
#   ./build.sh --clean            # 完全クリーンビルド
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# ── Flags ──────────────────────────────────────────────────────────────────────
INSTALL_DEPS=false
RELEASE=false
CLEAN=false
for arg in "$@"; do
    case "$arg" in
        --deps)    INSTALL_DEPS=true ;;
        --release) RELEASE=true ;;
        --clean)   CLEAN=true ;;
        --help|-h)
            echo "Usage: $0 [--deps] [--release] [--clean]"
            echo ""
            echo "  --deps     依存パッケージをインストールしてからビルド"
            echo "  --release  Release ビルド（最適化有効）"
            echo "  --clean    完全クリーンビルド（build/ install/ log/ 削除）"
            exit 0
            ;;
    esac
done

# ── Banner ─────────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║           Meister — Build                        ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── ROS2 check ─────────────────────────────────────────────────────────────────
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy"
    echo "  Install ROS2 Jazzy first:"
    echo "  https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
echo "→ ROS_DISTRO: $ROS_DISTRO"
echo "→ Workspace:  $WORKSPACE_DIR"
echo ""

# ── Clean ──────────────────────────────────────────────────────────────────────
if [ "$CLEAN" = true ]; then
    echo "→ Cleaning build/ install/ log/ ..."
    rm -rf "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/install" "$WORKSPACE_DIR/log"
    echo "  Cleaned."
    echo ""
fi

# ── Install dependencies ───────────────────────────────────────────────────────
if [ "$INSTALL_DEPS" = true ]; then
    echo "→ Installing system dependencies..."
    sudo apt update -qq

    # rosdep
    if ! command -v rosdep &>/dev/null; then
        sudo apt install -y python3-rosdep
        sudo rosdep init --rosdistro jazzy 2>/dev/null || true
    fi
    rosdep update --rosdistro jazzy 2>/dev/null || true

    # パッケージ依存関係を rosdep で解決
    cd "$WORKSPACE_DIR"
    rosdep install --from-paths src --ignore-src -r -y 2>&1 | tail -5

    # 明示的に必要なパッケージも確実に
    sudo apt install -y \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-navigation2 \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-laser-filters \
        ros-jazzy-nav2-smac-planner \
        python3-yaml \
        python3-scipy \
        python3-numpy \
        2>&1 | tail -5

    echo "  Dependencies installed."
    echo ""
fi

# ── Build ──────────────────────────────────────────────────────────────────────
cd "$WORKSPACE_DIR"

COLCON_OPTS=("--symlink-install")
if [ "$RELEASE" = true ]; then
    COLCON_OPTS+=("--cmake-args" "-DCMAKE_BUILD_TYPE=Release")
fi

echo "→ Running: colcon build ${COLCON_OPTS[*]}"
echo ""

colcon build "${COLCON_OPTS[@]}"

echo ""
echo "✓ Build succeeded."
echo ""

# ── Source hint ────────────────────────────────────────────────────────────────
echo "──────────────────────────────────────────────────"
echo "  Source the workspace:"
echo "    source $WORKSPACE_DIR/install/setup.bash"
echo ""
echo "  Launch:"
echo "    ./launch.sh maze explore"
echo "──────────────────────────────────────────────────"
