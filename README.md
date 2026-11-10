# VisionWork for DX27

> RoboMaster 装甲板视觉工作区 —— ROS Noetic + OpenCV + Docker

基于 HSV 颜色阈值的**红/蓝装甲板灯条检测**，配一套「一条命令进容器」的 ROS Noetic 开发环境。
代码只有两个节点，环境成本却被 Docker 完全封装：`git clone` 之后 `bash ros1.sh` 就能开始编译，宿主机不需要装 ROS。

---

## 功能

- **红色装甲板检测** —— `detection_node_red`，阈值区间偏橙红（`H 5–45`），针对赛场暖色灯光调过
- **蓝色装甲板检测** —— `detection_node_blue`，阈值区间 `H 80–115`
- **双路可视化输出** —— 二值 mask（`MONO8`）+ 叠加图（`BGR8`），后者画出灯条最小外接矩形、灯条中点与短轴中线
- **零配置环境** —— `Dockerfile` 内置 ROS Noetic 全套依赖，`ros1.sh` 负责建镜像、建容器、挂载工作区、转发 X11

## 目录结构

```
vision_main_ws/
├── Dockerfile                  # ROS Noetic 开发镜像（rosuser/uid 1000，rosdep 走 BFSU 镜像）
├── ros1.sh                     # 容器管理脚本：一条命令构建镜像 + 启动 + 进入终端
├── src/
│   └── detection_node/         # 唯一的 catkin 包
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── src/
│           ├── detection_hsv_red.cpp
│           └── detection_hsv_blue.cpp
├── test_img/                   # 检测效果截图（rqt_image_view）
├── bag/                        # 测试 rosbag（不入库，见下文）
└── README.md
```

## 环境要求

| 依赖 | 版本 / 说明 |
| --- | --- |
| Docker | 20.10+，需要能挂载 `/tmp/.X11-unix`（GUI 显示） |
| 宿主机系统 | Linux + X11 / XWayland（WSL2 需自行配 X server） |
| 镜像 | `ros1-noetic:local`，基于 `osrf/ros:noetic-desktop-full` |
| 编译工具 | 镜像内已含 `catkin_make` 与 `catkin_tools` |

宿主机**不需要**安装 ROS。所有编译与运行都在容器里完成。

## 快速开始

```bash
# 1) 克隆
git clone git@github.com:bllx0019/VisionWork_for_DX27.git
cd VisionWork_for_DX27

# 2) 首次运行：镜像不存在时自动 docker build（约 5–10 分钟），然后进入容器
bash ros1.sh

# 3) 容器内编译
cd ~/VisionWork_for_DX27
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# 4) 运行检测节点（二选一，原因见「已知问题」）
rosrun detection_node detection_node_red
rosrun detection_node detection_node_blue
```

容器内的路径由脚本自动推断为 `~/<工作区目录名>`，所以 clone 下来叫什么名字，容器里就是什么路径。
重新进容器只要再执行一次 `bash ros1.sh`，它会在已有容器里开一个新终端。

### 查看结果

```bash
rqt_image_view
# Topic 选 /hk_camera/image_hsv_debug（叠加图）或 /hk_camera/image_hsv_mask（二值图）
```

### 用 rosbag 回放

`bag/` 目录不入库（体积原因），需要自己放入。回放时注意话题名要和节点订阅的一致：

```bash
rosbag play bag/red_4.bag
# 若 bag 里的话题不是 /hk_camera/image_raw/compressed，用 remap：
rosbag play bag/red_4.bag /your_camera/image_raw/compressed:=/hk_camera/image_raw/compressed
```

## 节点与话题

| 项 | 内容 |
| --- | --- |
| 订阅 | `/hk_camera/image_raw/compressed`（`sensor_msgs/CompressedImage`） |
| 发布 | `/hk_camera/image_hsv_mask`（`sensor_msgs/Image`, `MONO8`） |
| 发布 | `/hk_camera/image_hsv_debug`（`sensor_msgs/Image`, `BGR8`） |
| 可执行文件 | `detection_node_red` / `detection_node_blue` |

话题名硬编码在 `main()` 里，换相机最直接的办法是改源码；临时适配别的 bag 可以用命令行重映射：

```bash
rosrun detection_node detection_node_red /hk_camera/image_raw/compressed:=/my_cam/image_raw/compressed
```

## 算法流程

两个节点结构完全一致，只有 `inRange` 的阈值不同：

1. `cv_bridge::toCvCopy` 解压 bag/相机的压缩图 → `BGR8`
2. `cvtColor` → HSV
3. `GaussianBlur` 5×5 去噪
4. `inRange` 取色
5. 形态学链：`OPEN(3×3, iter=2)` → `CLOSE(5×5, iter=2)` → `ERODE(3×3, iter=1)`
6. `findContours(RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)`
7. 过滤：轮廓点数 `<5`、面积 `<50` 或 `>50%` 画面、短边 `<2 px` 的一律丢弃
8. `minAreaRect` 求最小外接旋转矩形 → 画四条边
9. 取**较短的两条对边**的中点，连成短轴中线（用于判断灯条朝向、给后续配对装甲板留接口）

### 阈值

| 目标 | H | S | V |
| --- | --- | --- | --- |
| 红（偏橙） | 5 – 45 | 75 – 255 | 75 – 255 |
| 蓝 | 80 – 115 | 50 – 255 | 75 – 255 |

红色节点里保留了一段被注释掉的 `H 170–180` 分支：赛场灯光下装甲板偏橙，两段合并没有收益，所以只留了一段。
如果换到冷白光环境，把 `170–180` 那段打开并 `bitwise_or` 合并会更稳。

## 效果

灯条框选（`/hk_camera/image_hsv_debug`）：

![红蓝灯条框选](test_img/2026-11-02%2023-30-11%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

加上中点与短轴中线后：

![灯条中点与短轴中线](test_img/2026-11-02%2023-57-44%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

更多截图见 [`test_img/`](test_img)。

## 调参建议

- **先看 mask 再调阈值**：`rqt_image_view` 选 `image_hsv_mask`，调 `inRange` 上下界直到灯条是实心块、背景是纯黑
- **白天/夜晚差很大**：`V` 下限（现在都是 75）是主要旋钮，暗环境可降到 50，强光下要提到 100+
- **形态学迭代次数**比核大小更敏感：`OPEN` 迭代多了会把远处小灯条吃掉
- 所有参数都在源码里写死，改完要重新 `catkin_make`

## 已知问题

1. **两个节点默认不能同时运行。** 两个 `main()` 里都写了 `ros::init(argc, argv, "delection_hsv")`（拼写是 `delection`，原始代码如此），节点名相同且发布同一组话题，同时跑会被 ROS master 顶掉一个。

   不想改代码的话，用 `__name` 和话题重映射给第二个节点改名即可并行：

   ```bash
   rosrun detection_node detection_node_red  __name:=hsv_red  \
       /hk_camera/image_hsv_mask:=/red/mask  /hk_camera/image_hsv_debug:=/red/debug
   rosrun detection_node detection_node_blue __name:=hsv_blue \
       /hk_camera/image_hsv_mask:=/blue/mask /hk_camera/image_hsv_debug:=/blue/debug
   ```

   长期还是建议把节点名和话题名改成各自的默认值。
2. **`CMakeLists.txt` 没有 `find_package(OpenCV)`**，只用了 `${OpenCV_LIBS}`。目前靠 `cv_bridge` 间接带入，能编过；换环境若链接失败，在 `find_package(catkin ...)` 后面补一行 `find_package(OpenCV REQUIRED)`。
3. **阈值不可动态调参**，没有 ROS 参数服务器、没有 `dynamic_reconfigure`，现场调参必须重编译。
4. **截图文件名带空格和中文**（`test_img/2026-11-02 23-30-11屏幕截图.png`），某些工具链下引用不便，后续建议改成 ASCII 命名。

## 版本库与数据

`bag/`、`build/`、`devel/`、`logs/`、`.vscode/`、`.catkin_tools/` 均通过 [`.gitignore`](.gitignore) 排除：

- `bag/` 约 2.7 GB，单条 bag 最大约 450 MB，超过 GitHub 单文件 100 MB 硬限制，**永远不要提交**
- `.vscode/browse.vc.db` 单文件 1 GB，属 IDE 索引缓存

实际入库内容约 4.3 MB（源码 + `test_img/` 截图 + 两个脚本）。

## License

`package.xml` 里的 license 字段目前仍是 `TODO`。在正式发布或参赛提交前请补一个明确的许可证（RoboMaster 队伍通常用 MIT 或 Apache-2.0）。
