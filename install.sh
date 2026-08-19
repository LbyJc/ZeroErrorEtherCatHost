#!/bin/bash
# install.sh —— 安装 EtherCAT Joint Control 到系统。
#
# 安装后：应用菜单里出现 "EtherCAT Joint Control"，双击即可运行，
# 正常实验全程不需要打开终端。
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX=/opt/ethercat-joint-control
CONFDIR=/etc/ethercat-joint-control
DATADIR=/var/lib/ethercat-joint-control
GROUP=ethercat
PYTHON="${ECJC_PYTHON:-/home/tyy/miniconda3/envs/zeroError/bin/python}"

red()   { echo -e "\033[31m$*\033[0m"; }
green() { echo -e "\033[32m$*\033[0m"; }
info()  { echo -e "\033[36m==>\033[0m $*"; }

[[ $EUID -eq 0 ]] || { red "请用 root 运行：  pkexec $0   或   sudo $0"; exit 1; }

REAL_USER="${SUDO_USER:-${PKEXEC_UID:+$(id -nu "$PKEXEC_UID")}}"
REAL_USER="${REAL_USER:-$(logname 2>/dev/null || echo "")}"

# ── 1. 依赖检查 ──────────────────────────────────────────────────────────
info "检查系统依赖"
missing=()
[[ -f /usr/local/include/ecrt.h ]] || missing+=("IgH EtherCAT Master (ecrt.h)")
[[ -f /usr/include/yaml-cpp/yaml.h ]] || missing+=("libyaml-cpp-dev")
ls /usr/include/hdf5/serial/hdf5.h >/dev/null 2>&1 || missing+=("libhdf5-dev")
command -v cmake >/dev/null || missing+=("cmake")
command -v g++ >/dev/null || missing+=("g++")
if [[ ${#missing[@]} -gt 0 ]]; then
    red "缺少依赖："
    printf '  - %s\n' "${missing[@]}"
    echo "  安装命令: apt-get install -y cmake g++ libyaml-cpp-dev libhdf5-dev"
    exit 1
fi

if [[ ! -x "$PYTHON" ]]; then
    red "找不到 Python 解释器: $PYTHON"
    echo "  请用 ECJC_PYTHON=/path/to/python $0 指定，"
    echo "  或先创建 conda 环境: conda create -n zeroError python=3.11 && pip install pyside6 pyqtgraph numpy h5py pyyaml"
    exit 1
fi
"$PYTHON" -c "import PySide6, pyqtgraph, numpy, h5py, yaml" 2>/dev/null || {
    red "Python 环境缺少依赖"
    echo "  $PYTHON -m pip install pyside6 pyqtgraph numpy h5py pyyaml"
    exit 1
}
green "依赖齐备"

# ── 2. 编译 Backend ─────────────────────────────────────────────────────
info "编译实时后端"
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$SRC/build" -j"$(nproc)"
green "编译完成"

info "运行单元测试"
(cd "$SRC/build" && ctest --output-on-failure) || { red "单元测试未通过，安装中止"; exit 1; }

# ── 3. 用户组 ───────────────────────────────────────────────────────────
info "配置用户组 $GROUP"
getent group "$GROUP" >/dev/null || groupadd --system "$GROUP"
if [[ -n "$REAL_USER" ]]; then
    usermod -aG "$GROUP" "$REAL_USER"
    green "已把用户 $REAL_USER 加入 $GROUP 组（需重新登录生效）"
else
    red "无法确定当前用户，请手动执行: usermod -aG $GROUP <你的用户名>"
fi

# ── 4. 安装文件 ─────────────────────────────────────────────────────────
info "安装到 $PREFIX"
install -d "$PREFIX"/{bin,gui,docs,share,tools,experiments}
install -m 755 "$SRC/build/ecjc-backend" "$PREFIX/bin/"
install -m 755 "$SRC/system/ecjc-helper" "$PREFIX/bin/"
cp -r "$SRC/gui/." "$PREFIX/gui/"
cp -r "$SRC/docs/." "$PREFIX/docs/" 2>/dev/null || true
[[ -d "$SRC/tools" ]] && cp -r "$SRC/tools/." "$PREFIX/tools/" || true
# 实验配置（一键实验的配置列表）与工况轨迹文件——GUI 按 <root>/experiments/presets 找
[[ -d "$SRC/experiments" ]] && cp -r "$SRC/experiments/." "$PREFIX/experiments/" || true

# GUI 启动器：把 conda 环境固化进去，用户不需要手动 activate
cat > "$PREFIX/bin/ecjc-gui" <<EOF
#!/bin/bash
exec "$PYTHON" "$PREFIX/gui/main.py" "\$@"
EOF
chmod 755 "$PREFIX/bin/ecjc-gui"

# 图标
cat > "$PREFIX/share/ecjc.svg" <<'EOF'
<svg xmlns="http://www.w3.org/2000/svg" width="64" height="64" viewBox="0 0 64 64">
  <rect width="64" height="64" rx="10" fill="#0277bd"/>
  <circle cx="32" cy="32" r="17" fill="none" stroke="#fff" stroke-width="4"/>
  <circle cx="32" cy="32" r="5" fill="#fff"/>
  <path d="M32 15 L32 6 M32 58 L32 49 M15 32 L6 32 M58 32 L49 32"
        stroke="#fff" stroke-width="4" stroke-linecap="round"/>
</svg>
EOF

# ── 5. 配置（已存在则保留，不覆盖用户改过的参数）──────────────────────
info "安装配置到 $CONFDIR"
install -d "$CONFDIR"
for f in "$SRC"/config/*.yaml; do
    base=$(basename "$f")
    if [[ -f "$CONFDIR/$base" ]]; then
        install -m 644 "$f" "$CONFDIR/$base.new"
        echo "  保留已有 $base（新版本存为 $base.new）"
    else
        install -m 644 "$f" "$CONFDIR/$base"
    fi
done

install -d -m 775 -g "$GROUP" "$DATADIR"/{data,logs}
chmod g+ws "$DATADIR/data" "$DATADIR/logs"
# 让 Backend 写到系统数据目录
sed -i "s|^  data_dir:.*|  data_dir: \"$DATADIR/data\"|" "$CONFDIR/app.yaml"
sed -i "s|^  log_dir:.*|  log_dir: \"$DATADIR/logs\"|" "$CONFDIR/app.yaml"

# ── 6. systemd / polkit / desktop ───────────────────────────────────────
info "安装 systemd 服务"
install -m 644 "$SRC/system/ethercat-joint-control.service" /etc/systemd/system/
systemctl daemon-reload

info "安装 polkit 策略"
install -m 644 "$SRC/system/com.zeroerr.ecjc.policy" \
    /usr/share/polkit-1/actions/ 2>/dev/null || \
    echo "  （polkit 目录不存在，跳过）"

info "安装桌面入口"
install -m 644 "$SRC/system/ethercat-joint-control.desktop" \
    /usr/share/applications/
update-desktop-database /usr/share/applications 2>/dev/null || true

# ── 7. 实时优先级限制 ───────────────────────────────────────────────────
info "配置实时优先级限制"
cat > /etc/security/limits.d/99-ethercat-joint-control.conf <<EOF
# EtherCAT Joint Control：允许 $GROUP 组取得实时优先级与锁内存
@$GROUP  -  rtprio  95
@$GROUP  -  memlock unlimited
EOF

echo
green "安装完成"
cat <<EOF

后续步骤：
  1. 重新登录（或执行 newgrp $GROUP）使用户组生效
  2. 在应用菜单里找到 "EtherCAT Joint Control" 双击启动
  3. 首次使用请在 GUI 的 System Configuration 页核对：
       - 网卡名（当前 $(grep -oP '(?<=interface:\s)\S+' "$CONFDIR/ethercat.yaml" 2>/dev/null || echo '?')）
       - 编码器分辨率与减速比

无硬件试用：
  $PREFIX/bin/ecjc-gui --mock

EOF

# 编码器分辨率是否已验证 —— 按实际配置说话，不要硬编码一句可能已经过时的警告
if grep -q 'encoder_resolution_verified:\s*true' "$CONFDIR/scaling.yaml" 2>/dev/null; then
    echo "✓ 编码器分辨率已通过物理转角验证，标定可信。"
else
    cat <<'EOF2'
⚠ 注意：encoder_resolution_verified 为 false，输出侧编码器分辨率尚未经物理转角验证。
   若实为 2^18，所有 rpm 数值需翻倍。验证脚本：tests/verify_encoder.py
EOF2
fi
