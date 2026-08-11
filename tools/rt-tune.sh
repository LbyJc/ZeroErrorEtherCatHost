#!/bin/bash
# rt-tune.sh —— 实时性调优（无需重启内核）。
#
# 针对本机实测到的三个抖动来源：
#   1. 调频策略是 powersave，CPU 会掉到 800MHz，变频有延迟
#   2. C-state 开到 C10，深睡唤醒延迟在短周期下是致命的
#   3. enp3s0 的中断和实时线程抢同一个核
#
# 需要 root。要更进一步（isolcpus / nohz_full / PREEMPT_RT）必须改内核启动参数并重启。
set -uo pipefail

RT_CORE=${RT_CORE:-2}      # 实时线程独占的核
IRQ_CORE=${IRQ_CORE:-1}    # 网卡中断集中到这个核
IFACE=${IFACE:-enp3s0}

echo "=== 1. 调频策略 → performance ==="
n=0
for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [ -w "$g" ] && echo performance > "$g" && n=$((n+1))
done
echo "  已设置 $n 个核"
echo -n "  确认 cpu$RT_CORE: "; cat /sys/devices/system/cpu/cpu$RT_CORE/cpufreq/scaling_governor

echo "=== 2. 抑制深度 C-state ==="
# 写 /dev/cpu_dma_latency 并**保持文件打开**才有效，关闭即失效。
# 所以这里起一个常驻的小守护进程持有它。
if pgrep -f "cpu_dma_latency_holder" >/dev/null 2>&1; then
    echo "  已有守护进程在持有 /dev/cpu_dma_latency"
else
    setsid bash -c 'exec -a cpu_dma_latency_holder sh -c "
        exec 9<>/dev/cpu_dma_latency
        printf \"\\0\\0\\0\\0\" >&9
        while :; do sleep 3600; done"' </dev/null >/dev/null 2>&1 &
    sleep 1
    echo "  已启动守护进程持有 /dev/cpu_dma_latency = 0"
fi
# 另外把实时核上的深睡状态直接禁掉（立即生效，重启后失效）
d=0
for s in /sys/devices/system/cpu/cpu$RT_CORE/cpuidle/state[2-9]/disable; do
    [ -w "$s" ] && echo 1 > "$s" && d=$((d+1))
done
echo "  cpu$RT_CORE 已禁用 $d 个深睡状态"

echo "=== 3. 网卡中断迁到 cpu$IRQ_CORE，让开实时核 ==="
mask=$(printf '%x' $((1 << IRQ_CORE)))
m=0
for irq in $(grep "$IFACE" /proc/interrupts | awk -F: '{gsub(/ /,"",$1); print $1}'); do
    if echo "$mask" > /proc/irq/$irq/smp_affinity 2>/dev/null; then
        m=$((m+1))
    fi
done
echo "  已迁移 $m 个中断到 cpu$IRQ_CORE (mask=0x$mask)"
# irqbalance 会把中断再挪回去，必须停掉
if systemctl is-active --quiet irqbalance 2>/dev/null; then
    systemctl stop irqbalance
    echo "  已停止 irqbalance（否则它会把中断挪回去）"
else
    echo "  irqbalance 未运行"
fi

echo "=== 4. 把非实时任务挪出实时核 ==="
# 构造"除实时核以外"的核列表，把现有进程都赶过去。
# 注意不能写成 taskset -pc 0-7 —— 那是"允许用所有核"，等于什么也没赶走，
# 反而会把别处已有的绑定一并冲掉。
ncpu=$(nproc)
others=$(seq 0 $((ncpu-1)) | grep -vx "$RT_CORE" | paste -sd,)
moved=0; skipped=0
for pid in $(ps -eo pid --no-headers); do
    if taskset -pc "$others" "$pid" >/dev/null 2>&1; then moved=$((moved+1))
    else skipped=$((skipped+1)); fi
done
echo "  已迁移 $moved 个进程到 cpu[$others]，$skipped 个迁不动（多为内核线程）"
echo "  cpu$RT_CORE 现在基本空着，留给实时线程独占"

echo
echo "=== 当前状态 ==="
echo -n "  governor: "; cat /sys/devices/system/cpu/cpu$RT_CORE/cpufreq/scaling_governor
echo -n "  cpu$RT_CORE 频率: "; awk '{printf "%.0f MHz\n", $1/1000}' \
    /sys/devices/system/cpu/cpu$RT_CORE/cpufreq/scaling_cur_freq 2>/dev/null || echo "?"
echo "  网卡中断分布:"
grep "$IFACE" /proc/interrupts | awk '{printf "    %s %s\n", $1, $NF}' | head -6
echo
echo "注意：这些设置重启后全部失效。要持久化 + 进一步降低抖动，需要改内核启动参数："
echo "  isolcpus=$RT_CORE nohz_full=$RT_CORE rcu_nocbs=$RT_CORE intel_idle.max_cstate=1 processor.max_cstate=1"
echo "并考虑换 PREEMPT_RT 内核。"
