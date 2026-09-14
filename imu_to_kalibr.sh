#!/bin/bash
# ==============================================================================
# 优化版 imu_to_kalibr.sh (修复 %YAML:1.0 解析异常)
# 自动兼容 imu_utils 非标头 | 自动截断回放 | 真实采样率写入
# ==============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

WORKSPACE="/home/siyi/imu_ws"
DEFAULT_ROSBAG="/home/siyi/imu_ws/data/rosbag/realsense_imu_20260907_203928_2026-09-07-20-39-28.bag"
DEFAULT_LAUNCH="${WORKSPACE}/src/imu_utils/launch/d435i_imu_an.launch"
DEFAULT_KALIBR_DIR="${WORKSPACE}/src/imu_utils/data/kalibr"
DEFAULT_PLAY_RATE=200

SKIP_CALIBRATION=""

usage() {
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  -b, --rosbag FILE       rosbag 文件路径 (默认: ${DEFAULT_ROSBAG})"
    echo "  -l, --launch FILE       roslaunch 文件路径 (默认: ${DEFAULT_LAUNCH})"
    echo "  -o, --output DIR        Kalibr 配置输出目录 (默认: ${DEFAULT_KALIBR_DIR})"
    echo "  -r, --rate RATE         rosbag 播放速率倍数 (默认: ${DEFAULT_PLAY_RATE})"
    echo "  -s, --skip-calibration  跳过标定步骤，直接从已有 YAML 转换"
    echo "  -y, --yaml FILE         指定已有的 imu_utils 输出 YAML"
    echo "  -h, --help              显示帮助信息"
    exit 0
}

ROSBAG_FILE="${DEFAULT_ROSBAG}"
LAUNCH_FILE="${DEFAULT_LAUNCH}"
KALIBR_DIR="${DEFAULT_KALIBR_DIR}"
PLAY_RATE="${DEFAULT_PLAY_RATE}"
IMU_UTILS_YAML=""

while [ $# -gt 0 ]; do
    case "$1" in
        -b|--rosbag) ROSBAG_FILE="$2"; shift 2 ;;
        -l|--launch) LAUNCH_FILE="$2"; shift 2 ;;
        -o|--output) KALIBR_DIR="$2"; shift 2 ;;
        -r|--rate) PLAY_RATE="$2"; shift 2 ;;
        -s|--skip-calibration) SKIP_CALIBRATION="yes"; shift ;;
        -y|--yaml) IMU_UTILS_YAML="$2"; shift 2 ;;
        -h|--help) usage ;;
        *) echo -e "${RED}错误: 未知参数 '$1'${NC}" >&2; usage ;;
    esac
done

check_prerequisites() {
    echo -e "${CYAN}[检查] 环境依赖...${NC}"
    if [ -z "${ROS_DISTRO:-}" ]; then
        if [ -f "/opt/ros/noetic/setup.bash" ]; then
            source "/opt/ros/noetic/setup.bash"
        fi
    fi
    if [ -f "${WORKSPACE}/devel/setup.bash" ]; then
        source "${WORKSPACE}/devel/setup.bash"
    fi

    for cmd in roslaunch rosbag rospack python3; do
        if ! command -v $cmd &>/dev/null; then
            echo -e "${RED}  ✗ 找不到 $cmd 命令${NC}"
            exit 1
        fi
    done
    echo -e "${GREEN}  ✓ 基础环境检测通过${NC}\n"
}

get_param_from_launch() {
    local param_name="$1"
    local default_val="$2"
    local val
    val=$(grep "${param_name}" "${LAUNCH_FILE}" | sed -n 's/.*value="\([^"]*\)".*/\1/p' | head -n 1)
    echo "${val:-$default_val}"
}

run_imu_calibration() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN} 阶段 1: 运行 imu_utils Allan 方差标定${NC}"
    echo -e "${CYAN}========================================${NC}"

    [ ! -f "${LAUNCH_FILE}" ] && { echo -e "${RED}错误: 找不到 launch 文件: ${LAUNCH_FILE}${NC}"; exit 1; }
    [ ! -f "${ROSBAG_FILE}" ] && { echo -e "${RED}错误: 找不到 rosbag 文件: ${ROSBAG_FILE}${NC}"; exit 1; }

    IMU_NAME=$(get_param_from_launch "imu_name" "d435i")
    IMU_TOPIC=$(get_param_from_launch "imu_topic" "/camera/imu")
    MAX_TIME_MIN=$(get_param_from_launch "max_time_min" "120")
    IMU_UTILS_YAML="$(rospack find imu_utils)/data/${IMU_NAME}_imu_param.yaml"

    rm -f "${IMU_UTILS_YAML}"

    echo -e "  Launch 文件 : ${LAUNCH_FILE}"
    echo -e "  Rosbag 文件 : ${ROSBAG_FILE}"
    echo -e "  IMU 话题    : ${IMU_TOPIC}"
    echo -e "  配置时长    : ${MAX_TIME_MIN} 分钟\n"

    echo -e "${CYAN}[步骤 1/3] 启动 imu_utils 节点...${NC}"
    roslaunch "${LAUNCH_FILE}" &
    LAUNCH_PID=$!

    local wait_count=0
    while ! rosnode list 2>/dev/null | grep -q "imu_an"; do
        sleep 1
        wait_count=$((wait_count + 1))
        if [ ${wait_count} -ge 30 ]; then
            echo -e "${RED}错误: 等待 imu_an 节点超时${NC}"
            kill ${LAUNCH_PID} 2>/dev/null || true
            exit 1
        fi
    done
    echo -e "${GREEN}  ✓ imu_an 节点已就绪${NC}\n"

    echo -e "${CYAN}[步骤 2/3] 播放 rosbag (速率: ${PLAY_RATE}x)...${NC}"
    rosbag play "${ROSBAG_FILE}" --rate="${PLAY_RATE}" --wait-for-subscribers >/dev/null 2>&1 &
    ROSBAG_PID=$!

    echo -e "${CYAN}[步骤 3/3] 数据回放与计算监听中...${NC}"
    while true; do
        if [ -f "${IMU_UTILS_YAML}" ]; then
            echo -e "\n${GREEN}  ✓ 标定计算完成，文件已生成！立即切断播放。${NC}"
            kill -9 ${ROSBAG_PID} 2>/dev/null || true
            break
        fi

        if ! kill -0 ${ROSBAG_PID} 2>/dev/null; then
            echo -e "\n${YELLOW}  ! Rosbag 播放完毕，等待算法最终收尾计算 (最多等待 30 秒)...${NC}"
            for i in $(seq 1 30); do
                if [ -f "${IMU_UTILS_YAML}" ]; then
                    echo -e "${GREEN}  ✓ 计算完成！${NC}"
                    break 2
                fi
                sleep 1
            done
            echo -e "${RED}错误: Rosbag 播放完毕但未生成 YAML。Bag 时长可能不足 ${MAX_TIME_MIN} 分钟。${NC}"
            kill ${LAUNCH_PID} 2>/dev/null || true
            exit 1
        fi
        sleep 2
    done

    kill ${LAUNCH_PID} 2>/dev/null || true
    wait ${LAUNCH_PID} 2>/dev/null || true
    echo -e "${GREEN}[完成] imu_utils 标定结果: ${IMU_UTILS_YAML}${NC}\n"
}

generate_kalibr_config() {
    local yaml_in="$1"
    local out_dir="$2"

    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN} 阶段 2: 生成 Kalibr 配置文件${NC}"
    echo -e "${CYAN}========================================${NC}"

    mkdir -p "${out_dir}"
    local out_file="${out_dir}/${IMU_NAME:-d435i}_kalibr_imu.yaml"

    python3 - <<EOF
import yaml
import sys
import subprocess
import re

yaml_path = "${yaml_in}"
bag_path = "${ROSBAG_FILE}"
topic = "${IMU_TOPIC:-/camera/imu}"
out_path = "${out_file}"

# 核心修复：滤除 imu_utils 生成的 OpenCV 非标头 "%YAML:1.0"
try:
    with open(yaml_path, 'r', encoding='utf-8') as f:
        lines = [line for line in f if not line.strip().startswith('%')]
        clean_yaml = "".join(lines)
    doc = yaml.safe_load(clean_yaml)
except Exception as e:
    sys.exit(f"YAML 读取失败: {e}")

gyr_n = float(doc['Gyr']['avg-axis']['gyr_n'])
gyr_w = float(doc['Gyr']['avg-axis']['gyr_w'])
acc_n = float(doc['Acc']['avg-axis']['acc_n'])
acc_w = float(doc['Acc']['avg-axis']['acc_w'])

calc_freq = 200.0
try:
    cmd = f"rosbag info {bag_path}"
    res = subprocess.check_output(cmd, shell=True).decode()
    
    # 提取话题消息总数
    match_msg = re.search(rf'{topic}\s+([0-9]+)\s+msgs', res)
    match_dur = re.search(r'duration:\s+([0-9.]+)\s*s', res)
    if match_msg and match_dur:
        msgs = int(match_msg.group(1))
        dur = float(match_dur.group(1))
        calc_freq = round(msgs / dur, 1)
except Exception:
    calc_freq = 200.0

content = f"""# ==============================================================================
# Kalibr IMU Configuration for ${IMU_NAME:-d435i}
# Generated from imu_utils
# ==============================================================================

rostopic: {topic}
update_rate: {calc_freq}   # 实际实测数据频率 [Hz]

accelerometer_noise_density: {acc_n:.16e}
accelerometer_random_walk: {acc_w:.16e}

gyroscope_noise_density: {gyr_n:.16e}
gyroscope_random_walk: {gyr_w:.16e}
"""

with open(out_path, 'w', encoding='utf-8') as f:
    f.write(content)

print(f"  陀螺仪白噪声   (gyr_n): {gyr_n:.6e}")
print(f"  陀螺仪随机游走 (gyr_w): {gyr_w:.6e}")
print(f"  加速度白噪声   (acc_n): {acc_n:.6e}")
print(f"  加速度随机游走 (acc_w): {acc_w:.6e}")
print(f"  写入实测采样频率      : {calc_freq} Hz")
EOF

    echo -e "\n${GREEN}[完成] Kalibr IMU 配置已成功写入: ${out_file}${NC}"
}

check_prerequisites

if [ -n "${SKIP_CALIBRATION}" ]; then
    [ -z "${IMU_UTILS_YAML}" ] && { echo -e "${RED}错误: 需指定 -y 参数${NC}"; exit 1; }
    generate_kalibr_config "${IMU_UTILS_YAML}" "${KALIBR_DIR}"
else
    run_imu_calibration
    generate_kalibr_config "${IMU_UTILS_YAML}" "${KALIBR_DIR}"
fi