#!/bin/bash
# uninstall.sh —— 卸载。
# 默认**保留**实验数据、配置和日志（任务书第四十七节），
# 除非明确加 --purge。实验数据是不可再生的，删除必须是显式意图。
set -euo pipefail

PREFIX=/opt/ethercat-joint-control
CONFDIR=/etc/ethercat-joint-control
DATADIR=/var/lib/ethercat-joint-control
GROUP=ethercat

PURGE=0
[[ "${1:-}" == "--purge" ]] && PURGE=1

[[ $EUID -eq 0 ]] || { echo "请用 root 运行：pkexec $0"; exit 1; }

echo "==> 停止服务"
systemctl stop ethercat-joint-control.service 2>/dev/null || true
systemctl disable ethercat-joint-control.service 2>/dev/null || true

echo "==> 删除程序文件"
rm -f /etc/systemd/system/ethercat-joint-control.service
rm -f /usr/share/applications/ethercat-joint-control.desktop
rm -f /usr/share/polkit-1/actions/com.zeroerr.ecjc.policy
rm -f /etc/security/limits.d/99-ethercat-joint-control.conf
rm -rf "$PREFIX"
systemctl daemon-reload
update-desktop-database /usr/share/applications 2>/dev/null || true

if [[ $PURGE -eq 1 ]]; then
    echo "==> --purge：删除配置、数据与日志"
    read -rp "确认删除 $DATADIR 下的全部实验数据？(输入 yes 确认) " ans
    if [[ "$ans" == "yes" ]]; then
        rm -rf "$CONFDIR" "$DATADIR"
        groupdel "$GROUP" 2>/dev/null || true
        echo "已全部删除"
    else
        echo "已取消数据删除，仅移除了程序文件"
    fi
else
    cat <<EOF

==> 已保留：
      配置  $CONFDIR
      数据  $DATADIR/data
      日志  $DATADIR/logs
      用户组 $GROUP

    如需一并删除，执行：  pkexec $0 --purge
EOF
fi
