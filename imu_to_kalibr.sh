#!/bin/bash
# ==============================================================================
# imu_to_kalibr.sh
# 完整 IMU 标定流程：roslaunch 启动 imu_utils → rosbag play → 等待标定完成 →
# 提取 Allan 方差参数 → 生成 Kalibr IMU 标定配置文件
#
# 用法: ./imu_to_kalibr.sh [rosbag_file] [launch_file] [output_dir]
# 示例: ./imu_to_kalibr.sh /home/siyi/imu_ws/data/rosbag/my_imu.bag
# ==============================================================================

set -euo pipefail

# ---------- 颜色输出 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ---------- 默认参数 ----------
WORKSPACE="/home/siyi/imu_ws"
DEFAULT_ROSBAG="/home/siyi/imu_ws/data/rosbag/realsense_imu_20260907_203928_2026-09-07-20-39-28.bag"
DEFAULT_LAUNCH="${WORKSPACE}/src/imu_utils/launch/d435i_imu_an.launch"
DEFAULT_KALIBR_DIR="${WORKSPACE}/src/imu_utils/data/kalibr"

# 可选：指定已有的 imu_utils 输出 YAML 跳过标定步骤（仅做转换）
SKIP_CALIBRATION=""

# ---------- 帮助信息 ----------
usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "完整标定流程: roslaunch → rosbag play → 生成 Kalibr 配置"
    echo ""
    echo "选项:"
    echo "  -b, --rosbag FILE       rosbag 文件路径"
    echo "                          (默认: ${DEFAULT_ROSBAG})"
    echo "  -l, --launch FILE       roslaunch 文件路径"
    echo "                          (默认: ${DEFAULT_LAUNCH})"
    echo "  -o, --output DIR        Kalibr 配置输出目录"
    echo "                          (默认: ${DEFAULT_KALIBR_DIR})"
    echo "  -s, --skip-calibration  跳过标定步骤，直接从已有 YAML 转换"
    echo "  -y, --yaml FILE         与 --skip-calibration 配合，指定 imu_utils 输出 YAML"
    echo "  -h, --help              显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  # 完整标定流程"
    echo "  $0 -b /path/to/imu.bag"
    echo ""
    echo "  # 仅从已有标定结果转换"
    echo "  $0 -s -y /path/to/imu_utils_output.yaml"
    exit 0
}

# ---------- 参数解析 ----------
ROSBAG_FILE="${DEFAULT_ROSBAG}"
LAUNCH_FILE="${DEFAULT_LAUNCH}"
KALIBR_DIR="${DEFAULT_KALIBR_DIR}"
IMU_UTILS_YAML=""

while [ $# -gt 0 ]; do
    case "$1" in
        -b|--rosbag)
            ROSBAG_FILE="$2"; shift 2 ;;
        -l|--launch)
            LAUNCH_FILE="$2"; shift 2 ;;
        -o|--output)
            KALIBR_DIR="$2"; shift 2 ;;
        -s|--skip-calibration)
            SKIP_CALIBRATION="yes"; shift ;;
        -y|--yaml)
            IMU_UTILS_YAML="$2"; shift 2 ;;
        -h|--help)
            usage ;;
        *)
            echo -e "${RED}错误: 未知参数 '$1'${NC}" >&2
            usage ;;
    esac
done

# ---------- 环境检查 ----------
check_prerequisites() {
    local ok=1
    echo -e "${CYAN}[检查] 环境依赖...${NC}"

    # 检查 ROS 环境
    if [ -z "${ROS_DISTRO:-}" ]; then
        echo -e "${YELLOW}[警告] ROS_DISTRO 未设置，尝试 source /opt/ros/*/setup.bash${NC}"
        # 尝试自动 source
        for distro in noetic melodic kinetic; do
            if [ -f "/opt/ros/${distro}/setup.bash" ]; then
                source "/opt/ros/${distro}/setup.bash"
                echo -e "${GREEN}  ✓ 已 source ROS ${distro}${NC}"
                break
            fi
        done
        if [ -z "${ROS_DISTRO:-}" ]; then
            echo -e "${RED}  ✗ 未找到 ROS 环境${NC}"
            ok=0
        fi
    else
        echo -e "${GREEN}  ✓ ROS ${ROS_DISTRO}${NC}"
    fi

    # 检查工作空间
    if [ -f "${WORKSPACE}/devel/setup.bash" ]; then
        source "${WORKSPACE}/devel/setup.bash"
        echo -e "${GREEN}  ✓ 工作空间已 source${NC}"
    else
        echo -e "${YELLOW}[警告] 未找到 ${WORKSPACE}/devel/setup.bash，请先编译工作空间${NC}"
    fi

    # 检查 roslaunch
    if ! command -v roslaunch &>/dev/null; then
        echo -e "${RED}  ✗ 找不到 roslaunch 命令${NC}"
        ok=0
    else
        echo -e "${GREEN}  ✓ roslaunch 可用${NC}"
    fi

    # 检查 rosbag
    if ! command -v rosbag &>/dev/null; then
        echo -e "${RED}  ✗ 找不到 rosbag 命令${NC}"
        ok=0
    else
        echo -e "${GREEN}  ✓ rosbag 可用${NC}"
    fi

    # 检查 imu_utils 包
    if ! rospack find imu_utils &>/dev/null; then
        echo -e "${RED}  ✗ 找不到 imu_utils 包${NC}"
        ok=0
    else
        echo -e "${GREEN}  ✓ imu_utils 包: $(rospack find imu_utils)${NC}"
    fi

    if [ "${ok}" -eq 0 ]; then
        echo -e "${RED}环境检查失败，请先解决上述问题${NC}"
        exit 1
    fi
    echo ""
}

# ---------- 从 launch 文件提取 imu_name ----------
get_imu_name_from_launch() {
    local launch_file="$1"
    local name
    name=$(grep 'imu_name' "${launch_file}" | sed 's/.*value="\([^"]*\)".*/\1/')
    if [ -z "${name}" ]; then
        echo "d435i"
    else
        echo "${name}"
    fi
}

# ---------- 从 launch 文件提取 imu_topic ----------
get_imu_topic_from_launch() {
    local launch_file="$1"
    local topic
    topic=$(grep 'imu_topic' "${launch_file}" | sed 's/.*value="\([^"]*\)".*/\1/')
    if [ -z "${topic}" ]; then
        echo "/camera/imu"
    else
        echo "${topic}"
    fi
}

# ---------- 阶段1: 运行 imu_utils 标定 ----------
run_imu_calibration() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN} 阶段 1: 运行 imu_utils Allan 方差标定${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    # 检查文件
    if [ ! -f "${LAUNCH_FILE}" ]; then
        echo -e "${RED}错误: 找不到 launch 文件: ${LAUNCH_FILE}${NC}"
        exit 1
    fi
    if [ ! -f "${ROSBAG_FILE}" ]; then
        echo -e "${RED}错误: 找不到 rosbag 文件: ${ROSBAG_FILE}${NC}"
        exit 1
    fi

    # 获取 imu 名称和 topic
    IMU_NAME=$(get_imu_name_from_launch "${LAUNCH_FILE}")
    IMU_TOPIC=$(get_imu_topic_from_launch "${LAUNCH_FILE}")
    IMU_UTILS_YAML="$(rospack find imu_utils)/data/${IMU_NAME}_imu_param.yaml"

    echo -e "  Launch 文件 : ${LAUNCH_FILE}"
    echo -e "  Rosbag 文件 : ${ROSBAG_FILE}"
    echo -e "  IMU 名称    : ${IMU_NAME}"
    echo -e "  IMU Topic   : ${IMU_TOPIC}"
    echo -e "  输出 YAML   : ${IMU_UTILS_YAML}"
    echo ""

    # 估算 rosbag 时长
    echo -e "${CYAN}[信息] 分析 rosbag 文件...${NC}"
    BAG_INFO=$(rosbag info "${ROSBAG_FILE}" 2>/dev/null || true)
    BAG_DURATION=$(echo "${BAG_INFO}" | grep "Duration:" | sed 's/.*Duration: \([0-9.]*\).*/\1/')
    if [ -n "${BAG_DURATION}" ]; then
        BAG_MIN=$(echo "${BAG_DURATION} / 60" | bc 2>/dev/null || echo "?")
        echo -e "  时长: 约 ${BAG_MIN} 分钟"
    fi
    echo ""

    # ---- 启动 imu_utils 节点（后台） ----
    echo -e "${CYAN}[步骤 1/3] 启动 imu_utils 节点...${NC}"
    roslaunch "${LAUNCH_FILE}" &
    LAUNCH_PID=$!
    echo -e "  roslaunch PID: ${LAUNCH_PID}"

    # 等待节点就绪
    echo -e "${CYAN}[步骤 2/3] 等待 imu_an 节点就绪...${NC}"
    local wait_count=0
    while ! rosnode list 2>/dev/null | grep -q "imu_an"; do
        sleep 1
        wait_count=$((wait_count + 1))
        if [ ${wait_count} -ge 30 ]; then
            echo -e "${RED}错误: 等待 imu_an 节点超时 (30s)${NC}"
            kill ${LAUNCH_PID} 2>/dev/null || true
            exit 1
        fi
    done
    echo -e "${GREEN}  ✓ imu_an 节点已就绪${NC}"
    echo ""

    # ---- 播放 rosbag ----
    echo -e "${CYAN}[步骤 3/3] 播放 rosbag 文件...${NC}"
    echo -e "  播放中...（请耐心等待，期间不要中断）"
    echo ""

    # 使用 rosbag play，--wait-for-subscribers 确保 imu_an 已订阅
    rosbag play "${ROSBAG_FILE}" \
        --wait-for-subscribers \
        --rate=1.0 \
        2>&1 | while IFS= read -r line; do
            # 每隔一段时间打印进度
            if [[ "${line}" == *"Processed"* ]] || [[ "${line}" == *"Bag End"* ]]; then
                echo -e "  ${line}"
            fi
        done

    echo ""
    echo -e "${GREEN}  ✓ rosbag 播放完成${NC}"

    # ---- 等待 imu_utils 完成处理 ----
    echo -e "${CYAN}[等待] imu_utils 正在处理数据...${NC}"
    local process_count=0
    while rosnode list 2>/dev/null | grep -q "imu_an"; do
        # 检查节点是否仍在运行
        if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
            break
        fi
        sleep 5
        process_count=$((process_count + 1))
        # 每30秒打印一次状态
        if [ $((process_count % 6)) -eq 0 ]; then
            echo -e "  仍在处理中... (${process_count}x5s)"
        fi
    done

    # 停止 roslaunch
    kill ${LAUNCH_PID} 2>/dev/null || true
    wait ${LAUNCH_PID} 2>/dev/null || true

    echo ""
    echo -e "${GREEN}  ✓ imu_utils 处理完成${NC}"
    echo ""

    # 检查输出文件
    if [ ! -f "${IMU_UTILS_YAML}" ]; then
        echo -e "${RED}错误: 标定完成但未找到输出文件: ${IMU_UTILS_YAML}${NC}"
        echo -e "${YELLOW}请检查 imu_utils 节点日志${NC}"
        exit 1
    fi

    echo -e "${GREEN}[完成] imu_utils 标定结果: ${IMU_UTILS_YAML}${NC}"
    echo ""
}

# ---------- 阶段2: 从 imu_utils YAML 提取参数 ----------
extract_avg_value() {
    local key="$1"
    local section="$2"
    local yaml_file="$3"
    local value
    value=$(sed -n "/^${section}:/,/^   [xA-Z]/p" "${yaml_file}" \
            | sed -n '/avg-axis/,/x-axis\|z-axis\|^$/p' \
            | grep "${key}:" \
            | awk -F':' '{print $NF}' \
            | tr -d ' ')
    if [ -z "${value}" ]; then
        echo -e "${RED}错误: 无法从 ${yaml_file} 的 ${section} avg-axis 中提取 ${key}${NC}" >&2
        exit 1
    fi
    echo "${value}"
}

# ---------- 阶段3: 生成 Kalibr 配置文件 ----------
generate_kalibr_config() {
    local imu_utils_yaml="$1"
    local kalibr_dir="$2"

    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN} 阶段 2: 生成 Kalibr IMU 配置文件${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    echo -e "${CYAN}[提取] 从 ${imu_utils_yaml} 提取 IMU 参数...${NC}"

    # 提取参数
    GYR_N=$(extract_avg_value "gyr_n" "Gyr" "${imu_utils_yaml}")
    GYR_W=$(extract_avg_value "gyr_w" "Gyr" "${imu_utils_yaml}")
    ACC_N=$(extract_avg_value "acc_n" "Acc" "${imu_utils_yaml}")
    ACC_W=$(extract_avg_value "acc_w" "Acc" "${imu_utils_yaml}")

    # 提取 IMU 名称
    IMU_NAME=$(grep "name:" "${imu_utils_yaml}" | head -1 | awk -F':' '{print $2}' | tr -d ' ')
    IMU_NAME="${IMU_NAME:-imu}"

    # 从 launch 文件获取 rostopic（如果可用）
    if [ -n "${IMU_TOPIC:-}" ]; then
        ROSTOPIC="${IMU_TOPIC}"
    else
        ROSTOPIC="/camera/imu"
    fi

    echo -e "  陀螺仪白噪声 (gyr_n) : ${GYR_N}"
    echo -e "  陀螺仪随机游走 (gyr_w): ${GYR_W}"
    echo -e "  加速度计白噪声 (acc_n) : ${ACC_N}"
    echo -e "  加速度计随机游走 (acc_w): ${ACC_W}"
    echo -e "  IMU 名称              : ${IMU_NAME}"
    echo -e "  ROS Topic             : ${ROSTOPIC}"
    echo ""

    # imu_utils 参数单位已符合 Kalibr 要求，无需转换
    GYROSCOPE_NOISE_DENSITY="${GYR_N}"
    GYROSCOPE_RANDOM_WALK="${GYR_W}"
    ACCELEROMETER_NOISE_DENSITY="${ACC_N}"
    ACCELEROMETER_RANDOM_WALK="${ACC_W}"

    # 生成 Kalibr 配置文件
    mkdir -p "${kalibr_dir}"
    OUTPUT_FILE="${kalibr_dir}/${IMU_NAME}_kalibr_imu.yaml"

    echo -e "${CYAN}[生成] Kalibr 配置文件: ${OUTPUT_FILE}${NC}"

    cat > "${OUTPUT_FILE}" <<EOF
# ==============================================================================
# Kalibr IMU Configuration for ${IMU_NAME}
# Generated from imu_utils (Allan Variance) Calibration Results
# Source: ${imu_utils_yaml}
# Date: $(date '+%Y-%m-%d %H:%M:%S')
# ==============================================================================

# Sensor Model
rostopic: ${ROSTOPIC}
update_rate: 200.0  # 标称采样频率 [Hz]

# ------------------------------------------------------------------------------
# Accelerometer (加速度计参数)
# ------------------------------------------------------------------------------
# 白噪声谱密度 (VRW, Velocity Random Walk) [m / s^2 / sqrt(Hz)]
accelerometer_noise_density: ${ACCELEROMETER_NOISE_DENSITY}

# 零偏随机游走 (加速度偏置漂移率) [m / s^3 / sqrt(Hz)]
accelerometer_random_walk: ${ACCELEROMETER_RANDOM_WALK}

# ------------------------------------------------------------------------------
# Gyroscope (陀螺仪参数)
# ------------------------------------------------------------------------------
# 白噪声谱密度 (ARW, Angle Random Walk) [rad / s / sqrt(Hz)]
gyroscope_noise_density: ${GYROSCOPE_NOISE_DENSITY}

# 零偏随机游走 (角速度偏置漂移率) [rad / s^2 / sqrt(Hz)]
gyroscope_random_walk: ${GYROSCOPE_RANDOM_WALK}
EOF

    echo -e "${GREEN}[完成] Kalibr IMU 配置已写入: ${OUTPUT_FILE}${NC}"
    echo ""
    echo "--- 生成的文件内容 ---"
    cat "${OUTPUT_FILE}"
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN} 标定流程全部完成！${NC}"
    echo -e "${GREEN} imu_utils 结果 : ${imu_utils_yaml}${NC}"
    echo -e "${GREEN} Kalibr 配置    : ${OUTPUT_FILE}${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# ==============================================================================
# 主流程
# ==============================================================================

echo -e "${CYAN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║      IMU Allan 方差标定 → Kalibr 配置生成       ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════════╝${NC}"
echo ""

if [ -n "${SKIP_CALIBRATION}" ]; then
    # ---- 仅转换模式 ----
    if [ -z "${IMU_UTILS_YAML}" ]; then
        echo -e "${RED}错误: 使用 --skip-calibration 时必须通过 --yaml 指定 imu_utils 输出文件${NC}"
        exit 1
    fi
    if [ ! -f "${IMU_UTILS_YAML}" ]; then
        echo -e "${RED}错误: 找不到 imu_utils YAML 文件: ${IMU_UTILS_YAML}${NC}"
        exit 1
    fi

    echo -e "${YELLOW}[跳过] 标定步骤已跳过，仅执行转换${NC}"
    echo ""
    generate_kalibr_config "${IMU_UTILS_YAML}" "${KALIBR_DIR}"
else
    # ---- 完整标定流程 ----
    check_prerequisites
    run_imu_calibration
    generate_kalibr_config "${IMU_UTILS_YAML}" "${KALIBR_DIR}"
fi
