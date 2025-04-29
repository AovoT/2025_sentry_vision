#!/usr/bin/env bash
# 一键构建脚本：默认 Release，可选 debug/release，构建成功后自动 source install/setup.zsh

set -euo pipefail

# 脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# workspace 根目录（auto_aim 的上一级）
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 解析参数
BUILD_TYPE="Release"
if [[ $# -gt 0 ]]; then
  case "${1,,}" in
    (debug)   BUILD_TYPE="Debug"   ;;
    (release) BUILD_TYPE="Release" ;;
    (-h|--help)
      echo "Usage: $0 [debug|release]"
      exit 0
      ;;
    (*)
      echo "Invalid option: $1"
      echo "Usage: $0 [debug|release]"
      exit 1
      ;;
  esac
fi

echo "🔨  切换到工作目录：$WS_ROOT"
cd "$WS_ROOT"

echo "🔧  开始构建 (Build Type = $BUILD_TYPE)…"
if colcon build --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE"; then
  echo "✅  构建成功，正在 source install/setup.zsh…"
  # 注意：如果你的 shell 是 bash 或 zsh，都可以用 source 或 “.” 
  # shellcheck disable=SC1091
  source install/setup.zsh
  echo "🚀  完成。"
else
  echo "❌  构建失败，退出。"
  exit 1
fi
