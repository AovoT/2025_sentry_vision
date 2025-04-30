#!/bin/bash
set -e

RULE_FILE="/etc/udev/rules.d/99-serial.rules"

echo "🔧 写入 udev 规则到: $RULE_FILE"

sudo tee "$RULE_FILE" > /dev/null <<EOF
# 串口左侧设备（serial: 3057358A3034）
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="3057358A3034", SYMLINK+="ttyACM_RIGHT", MODE="0666"

# 串口右侧设备（serial: 205837735948）
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="205837735948", SYMLINK+="ttyACM_LEFT", MODE="0666"
EOF

echo "✅ 规则写入完成"

echo "🔄 重新加载 udev 规则并触发"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "🧪 当前串口设备（如软链接正确应出现 ttyHIK_LEFT/RIGHT）："
ls -l /dev/ttyACM_*

echo "🔒 添加当前用户到 dialout 组（如未添加）"
if groups $USER | grep -qv dialout; then
  sudo usermod -aG dialout $USER
  echo "✅ 用户 $USER 已加入 dialout，需重新登录终端生效"
else
  echo "✅ 当前用户已在 dialout 组中"
fi

echo "🚀 完成。请重新插拔串口测试 /dev/ttyHIK_LEFT 和 /dev/ttyHIK_RIGHT 是否稳定"
