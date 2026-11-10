#!/usr/bin/env bash
# ros1.sh —— ROS Noetic Docker 管理脚本（多终端）
#
# 用法: bash <工作区目录>/ros1.sh
#       工作区 = 脚本所在目录，与执行命令时的 $PWD 无关。
#
# 默认行为:
#   工作区     = 脚本所在目录（$SCRIPT_DIR）
#   挂载       = 脚本所在目录 -> 容器内 ~/<同名目录>（bind mount，实时双向）
#   镜像       = ros1-noetic:local（由同目录 Dockerfile 构建，普通用户 rosuser/uid 1000）
#   容器内路径 = 按镜像默认用户推断（rosuser -> /home/rosuser/<目录名>）
#                如果容器内没有这个目录，会自动创建同名目录
#
# 可覆盖的环境变量:
#   ROS_WS        已忽略（工作区固定为脚本所在目录；保留仅为兼容旧命令）
#   ROS_MOUNT     已忽略（固定挂载整个工作区；保留仅为兼容旧命令）
#   ROS_IMAGE     换镜像，如 osrf/ros:noetic-desktop-full（同时设 ROS_CWS=/root/<目录名>）
#   ROS_CWS       容器内工作区路径（默认 ~/<脚本目录名>；一般不用改）
#   ROS_CONTAINER 容器名（默认 ros1）
#   ROS_SHM       共享内存大小（默认 1g，Gazebo 建议 1g 以上）
#   ROS_USER      以指定用户运行容器，如 1000:1000（默认用镜像里的用户）
#   ROS_RECREATE  设为 1 时强制删除旧容器并重建
#   ROS_AUTO_RECREATE 挂载不匹配时自动重建（默认 1；设 0 只报错、不删除）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${ROS_IMAGE:-ros1-noetic:local}"
NAME="${ROS_CONTAINER:-ros1}"
SHM="${ROS_SHM:-1g}"
AUTO_RECREATE="${ROS_AUTO_RECREATE:-1}"

# 工作区固定为脚本所在目录，与执行命令时的 $PWD 无关。
WS="$SCRIPT_DIR"
WS_NAME="$(basename "$WS")"

# 兼容旧变量：ROS_WS / ROS_MOUNT 不再改变挂载行为。
if [ -n "${ROS_WS:-}" ] && [ "$(cd "$ROS_WS" 2>/dev/null && pwd -P || true)" != "$WS" ]; then
    echo "提示：ROS_WS=$ROS_WS 已忽略；工作区固定为脚本所在目录 $WS" >&2
fi
if [ -n "${ROS_MOUNT:-}" ] && [ "${ROS_MOUNT}" != "ws" ]; then
    echo "提示：ROS_MOUNT=${ROS_MOUNT} 已忽略；本脚本固定挂载整个工作区。" >&2
fi

# 主工作区（唯一真源）：脚本所在目录。可用 ROS_MAIN_WS 覆盖（仅影响同步）。
MAIN_WS="${ROS_MAIN_WS:-$SCRIPT_DIR}"
MAIN_WS="$(cd "$MAIN_WS" && pwd -P)"
# 可与主工作区双向同步的镜像目录。设 ROS_SYNC_PEER= 关闭同步。
SYNC_PEER="${ROS_SYNC_PEER-$MAIN_WS/../Workspace}"

# ---------- 0. 双向同步（可选，默认开启）----------
# 背景：Docker bind mount 要求挂载源必须已存在，否则会以 root 身份自动建一个空目录挂进去，
# 把容器内容全部遮住。此前的故障正是“在空目录启动 → 挂了个空目录”造成的。
# 若同时存在两个工作区副本，这里把 src/ 双向对齐，保证两边一致。
#
# 同步策略（rsync 的 --update 只能单向优先，双向同时用会互相覆盖丢文件）：
#   先按“较新优先”把双方增量合并到主工作区，再把主工作区整体镜像到对端。
#   删除操作不传播（保护误删），仅做新增/更新。
SYNC_SRC="${SYNC_SRC:-src}"        # 只同步源码树，build/devel 不入同步
if [ -n "${SYNC_PEER}" ] && [ -d "$SYNC_PEER" ]; then
    if ! command -v rsync >/dev/null 2>&1; then
        echo "警告：未安装 rsync，跳过同步（apt-get install rsync）" >&2
    else
        echo "同步：$MAIN_WS/$SYNC_SRC <-> $SYNC_PEER/$SYNC_SRC"
        mkdir -p "$MAIN_WS/$SYNC_SRC"
        if [ -d "$SYNC_PEER/$SYNC_SRC" ]; then
            # 1) 对端较新的文件先拉回主工作区
            rsync -a --update "$SYNC_PEER/$SYNC_SRC/" "$MAIN_WS/$SYNC_SRC/"
            # 2) 主工作区整体镜像到对端（不删多余文件）
            rsync -a "$MAIN_WS/$SYNC_SRC/" "$SYNC_PEER/$SYNC_SRC/"
            echo "同步完成（两边内容已对齐）"
        else
            echo "提示：$SYNC_PEER 下没有 $SYNC_SRC，跳过同步"
        fi
    fi
fi

# ---------- 1. 工作区 = 脚本所在目录 ----------
echo "工作区：$WS（目录名：$WS_NAME）"

# 防止把空目录挂进去（会遮住容器内已有内容，且 Docker 会以 root 创建挂载源）
if [ ! -d "$WS/src" ]; then
    echo "错误：$WS 下没有 src/，不是 catkin 工作区。" >&2
    echo "      请把 ros1.sh 放在工作区根目录（与 src/ 同级）。" >&2
    exit 1
fi

# ---------- 2. 镜像（没有就按同目录 Dockerfile 构建，或拉取）----------
ensure_image() {
    docker image inspect "$IMAGE" >/dev/null 2>&1 && return 0
    if [ -f "${SCRIPT_DIR}/Dockerfile" ]; then
        echo "本地没有镜像 ${IMAGE}，用 ${SCRIPT_DIR}/Dockerfile 构建（首次较慢）..."
        docker build -t "$IMAGE" "$SCRIPT_DIR"
    else
        echo "本地没有镜像 ${IMAGE}，开始拉取..."
        docker pull "$IMAGE"
    fi
}
ensure_image

# ---------- 3. 容器内工作区路径：~/<脚本目录名> ----------
if [ -n "${ROS_CWS:-}" ]; then
    CWS="$ROS_CWS"
else
    img_user="$(docker image inspect -f '{{.Config.User}}' "$IMAGE" 2>/dev/null || true)"
    case "$img_user" in
        ""|root|0|0:0) CWS="/root/$WS_NAME" ;;
        *)             CWS="/home/${img_user%%:*}/$WS_NAME" ;;
    esac
fi
echo "容器内工作区：$CWS（不存在时创建）"

# ---------- 4. 挂载范围：整个工作区 ----------
HOST_MOUNT="$WS"
DEST_MOUNT="$CWS"
echo "挂载：$HOST_MOUNT -> $DEST_MOUNT（实时双向）"

# ---------- 5. GUI 授权（X11 / XWayland 均可）----------
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +si:localuser:root >/dev/null 2>&1 || true
    xhost +si:localuser:"$(id -un)" >/dev/null 2>&1 || true
fi

# ---------- 6. 容器通用参数（注释绝不能写在反斜杠同一行！）----------
COMMON_ARGS=(
    -e DISPLAY="${DISPLAY:-:0}"
    -e QT_X11_NO_MITSHM=1
    -e LIBGL_ALWAYS_INDIRECT=0
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "${HOST_MOUNT}:${DEST_MOUNT}:rw"
    -w "${CWS}"
    --shm-size="${SHM}"
    --net=host
)
if [ -n "${ROS_USER:-}" ]; then
    COMMON_ARGS+=(-u "$ROS_USER")
fi

# ---------- 7. 挂载一致性检查（挂载在创建时就固定，改不了）----------
check_mount() {
    local mounts cur
    mounts="$(docker inspect -f '{{range .Mounts}}{{.Destination}}={{.Source}}
{{end}}' "$NAME" 2>/dev/null || true)"
    if ! printf '%s\n' "$mounts" | grep -qxF "${DEST_MOUNT}=${HOST_MOUNT}"; then
        cur="$(printf '%s\n' "$mounts" | awk -F= -v d="${CWS}" 'index($1, d) == 1 {print $1" <- "$2}')"
        echo "警告：容器 ${NAME} 的挂载与当前设置不一致" >&2
        echo "      期望：${HOST_MOUNT} -> ${DEST_MOUNT}" >&2
        [ -n "$cur" ] && echo "      实际：${cur}" >&2
        return 1
    fi
    return 0
}

# 容器内没有目标工作区时，创建同名目录（新建容器时 Docker 会随 -v 自动创建）
ensure_container_ws() {
    if docker exec "$NAME" test -d "$CWS" >/dev/null 2>&1; then
        return 0
    fi
    echo "容器内没有 $CWS，创建同名 ws 目录..." >&2
    docker exec "$NAME" mkdir -p "$CWS"
}

# ROS_RECREATE=1：强制删除旧容器，按当前设置重建
if [ "${ROS_RECREATE:-0}" = "1" ] && [ -n "$(docker ps -aq -f "name=^/${NAME}$")" ]; then
    echo "ROS_RECREATE=1，删除容器 ${NAME} 并重建..."
    docker rm -f "$NAME" >/dev/null
fi

# 容器已存在但挂载与当前工作区不一致：默认自动重建
if [ -n "$(docker ps -aq -f "name=^/${NAME}$")" ] && ! check_mount; then
    if [ "${AUTO_RECREATE}" = "1" ]; then
        echo "自动重建容器 ${NAME}（容器内的 build/devel 会丢失）..." >&2
        docker rm -f "$NAME" >/dev/null
    else
        echo "错误：容器挂载与当前工作区不一致，未进入。" >&2
        echo "      自动重建: ROS_AUTO_RECREATE=1 bash $0" >&2
        echo "      保留旧容器: ROS_CONTAINER=ros1_${WS_NAME} bash $0" >&2
        echo "      强制重建: ROS_RECREATE=1 bash $0" >&2
        exit 1
    fi
fi

if [ "$(docker ps -q -f "name=^/${NAME}$")" ]; then
    echo "容器 ${NAME} 已在运行，正在创建新终端..."
elif [ "$(docker ps -aq -f "name=^/${NAME}$")" ]; then
    echo "发现已停止的容器 ${NAME}，正在启动..."
    docker start "$NAME" >/dev/null
else
    echo "首次创建容器 ${NAME}（容器内工作区：$CWS，由 -v 自动创建）..."
    docker run -dit \
        --name "$NAME" \
        "${COMMON_ARGS[@]}" \
        "$IMAGE" bash
    echo "容器 ${NAME} 已创建（后台常驻）。"
fi

ensure_container_ws

# ---------- 8. 进入终端 ----------
exec docker exec -it -w "$CWS" -e "ROS_WS_PATH=$CWS" "$NAME" bash -lc '
    source /opt/ros/noetic/setup.bash
    if [ -f "$ROS_WS_PATH/devel/setup.bash" ]; then
        source "$ROS_WS_PATH/devel/setup.bash"
    fi
    exec bash
'
