#!/bin/bash
#
# OpenArmX Docker 环境启动脚本
#
# 用法:
#   ./run_docker.sh              # 构建镜像 + 进入容器 bash
#   ./run_docker.sh --build      # 强制重新构建镜像
#   ./run_docker.sh --sim-vr     # 直接启动模拟 + VR
#   ./run_docker.sh --clean      # 清理旧编译产物后进入容器
#

set -e
cd "$(dirname "$0")"

IMAGE_NAME="openarmx:humble"
CONTAINER_NAME="openarmx_vr"
ACTION="shell"
FORCE_BUILD=""
DO_CLEAN=""

for arg in "$@"; do
    case $arg in
        --build)    FORCE_BUILD="yes" ;;
        --sim-vr)   ACTION="sim-vr" ;;
        --clean)    DO_CLEAN="yes" ;;
        --help|-h)
            echo "用法: ./run_docker.sh [选项]"
            echo ""
            echo "选项:"
            echo "  (无参数)     构建镜像并进入容器 bash"
            echo "  --build     强制重新构建 Docker 镜像"
            echo "  --sim-vr    进入容器后直接编译+启动模拟+VR"
            echo "  --clean     清理旧编译产物 (build/ install/ log/)"
            echo ""
            echo "进入容器后的常用命令:"
            echo "  colcon build --symlink-install          # 编译工作空间"
            echo "  source install/setup.bash               # 加载编译产物"
            echo "  ./shadow_mode/start.sh --sim --vr       # 模拟+VR模式"
            echo "  ./shadow_mode/start.sh --sim --vr --viz # 模拟+VR+可视化"
            exit 0
            ;;
    esac
done

xhost +local:docker 2>/dev/null || true

echo "========================================"
echo "  OpenArmX Docker 环境"
echo "========================================"
echo ""

EXISTING_IMAGE=$(sg docker -c "docker images -q $IMAGE_NAME" 2>/dev/null)
if [ -z "$EXISTING_IMAGE" ] || [ -n "$FORCE_BUILD" ]; then
    echo "[1/3] 构建 Docker 镜像 (首次需要几分钟)..."
    sg docker -c "docker build -t $IMAGE_NAME ./docker"
    echo ""
else
    echo "[1/3] 镜像已存在，跳过构建 (--build 强制重建)"
    echo ""
fi

if [ -n "$DO_CLEAN" ]; then
    echo "[2/3] 清理旧编译产物..."
    rm -rf build/ install/ log/
    echo "  已删除 build/ install/ log/"
    echo ""
else
    echo "[2/3] 保留现有编译产物 (--clean 清理)"
    echo ""
fi

sg docker -c "docker rm -f $CONTAINER_NAME" 2>/dev/null || true

DOCKER_RUN="docker run --rm -it \
    --name $CONTAINER_NAME \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v $(pwd):/ws:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -w /ws \
    $IMAGE_NAME"

if [ "$ACTION" = "sim-vr" ]; then
    echo "[3/3] 启动容器 → 编译 → 模拟+VR..."
    sg docker -c "$DOCKER_RUN bash -c '\
        colcon build --symlink-install 2>&1 && \
        source install/setup.bash && \
        echo \"\" && \
        echo \">>> 启动模拟+VR...\" && \
        python3 shadow_mode/start_shadow.py --sim --vr \
    '"
else
    echo "[3/3] 进入容器 (bash)..."
    echo ""
    echo "  首次进入请运行:"
    echo "    colcon build --symlink-install"
    echo "    source install/setup.bash"
    echo ""
    echo "  然后启动 VR 模拟:"
    echo "    python3 shadow_mode/start_shadow.py --sim --vr"
    echo ""
    sg docker -c "$DOCKER_RUN"
fi
