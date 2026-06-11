# RK3588 Full SITL 安装问题复盘

本文记录在 RK3588 开发板安装 full-sitl 环境时实际遇到过的问题、现象、原因和处理方式。

对应安装入口：

```bash
cd /home/jie/uav_swarm_system
./scripts/rk3588_install_full_sitl.sh full-sitl
```

相关完整安装记录：

```text
docs/debug/rk3588_install_log_2026-05-31.md
```

## 1. 不要用 root 用户直接运行安装脚本

现象：

```text
[RK3588 install][ERROR] Run this script as the normal user, not root. It will use sudo when needed.
```

原因：

- ROS2、PX4 Python 依赖、工作空间构建产物都应该落在普通用户环境下。
- root 运行会导致 `~/.local`、`ros2_ws/build`、`ros2_ws/install` 权限混乱。

处理：

```bash
cd /home/jie/uav_swarm_system
./scripts/rk3588_install_full_sitl.sh full-sitl
```

不要使用：

```bash
sudo ./scripts/rk3588_install_full_sitl.sh full-sitl
```

## 2. 架构和系统版本检查

现象：

```text
This installer is intended for RK3588 Ubuntu arm64.
This project targets Ubuntu 22.04.
```

原因：

- 当前安装脚本默认面向 RK3588 / Ubuntu 22.04 / arm64。
- x86、WSL2 或其他 Ubuntu 版本不属于默认安装目标。

处理：

确认：

```bash
dpkg --print-architecture
lsb_release -a
```

期望：

```text
arm64
Ubuntu 22.04
```

如果只是临时在非目标系统上验证脚本，可显式覆盖：

```bash
RK3588_INSTALL_FORCE=1 ./scripts/rk3588_install_full_sitl.sh full-sitl
```

但正式 RK3588 部署不建议跳过检查。

## 3. `set -u` 导致 source ROS 环境失败

现象：

安装脚本或环境脚本在 source ROS2 setup 时中途退出，可能表现为变量未定义相关错误。

原因：

- 安装脚本使用了：

```bash
set -Eeuo pipefail
```

- ROS2 的 setup 脚本内部可能访问一些未定义变量。
- 在 `set -u` 下，访问未定义变量会直接退出。

处理：

脚本中已加入 `source_with_nounset_disabled()`：

```bash
source_with_nounset_disabled "/opt/ros/humble/setup.bash"
```

如果后续新增脚本也要在 `set -u` 下 source ROS 环境，应采用同样方式：

```bash
set +u
source /opt/ros/humble/setup.bash
set -u
```

## 4. `gazebo --version` 有输出但返回非零

现象：

`scripts/setup_env.sh` 中执行：

```bash
gazebo --version
```

能看到 Gazebo 版本，但脚本仍然提前退出。

原因：

- RK3588 板端 GUI/OpenGL 环境不完整时，`gazebo --version` 可能打印版本后返回非零。
- `setup_env.sh` 使用 `set -e`，命令返回非零会导致整个脚本退出。

处理：

`setup_env.sh` 中已改为：

```bash
gazebo --version || true
```

原则：

- 环境加载脚本不能因为 Gazebo GUI 状态导致 ROS2/PX4 环境加载失败。
- 板端主线使用 headless `gzserver`，不依赖 `gzclient`。

## 5. MicroXRCEAgent 构建时 FastDDS tag 不可用

现象：

构建 Micro-XRCE-DDS-Agent 2.4.1 时，CMake 拉取 FastDDS 相关依赖失败，提示类似 tag/branch 不存在。

当时遇到的问题：

```text
FastDDS tag 2.11.x 不可用
```

原因：

- 上游依赖 tag 名称和实际 GitHub tag 不一致。
- 这属于外部项目依赖锁定问题。

处理：

当时处理方式是把上游依赖改到可用 tag：

```text
v2.11.3
```

验证：

```bash
which MicroXRCEAgent
MicroXRCEAgent --version
```

期望：

```text
/usr/local/bin/MicroXRCEAgent
2.4.1
```

说明：

- 当前项目固定 MicroXRCEAgent 2.4.1。
- 不要升级到 MicroXRCEAgent 3.x。

## 6. Gazebo Classic 11.10.2 在 arm64 上包不完整

现象：

在 RK3588 Ubuntu 22.04 arm64 上，直接通过 apt 安装 Gazebo Classic 11 时，可能拿不到完整的 `11.10.2` 目标包，或者缺少开发包/插件依赖。

当时处理：

- 从源码构建 Gazebo Classic 11.10.2 Debian 包。
- 构建目录：

```text
/home/jie/build_gazebo_classic
```

生成过的包：

```text
gazebo_11.10.2+dfsg-1_arm64.deb
libgazebo11_11.10.2+dfsg-1_arm64.deb
libgazebo-dev_11.10.2+dfsg-1_arm64.deb
gazebo-plugin-base_11.10.2+dfsg-1_arm64.deb
```

验证：

```bash
pkg-config --modversion gazebo
gazebo --version
gzserver --version
```

期望：

```text
11.10.2
```

注意：

- 当前 `scripts/rk3588_install_full_sitl.sh` 中优先尝试 apt 安装 `gazebo` 和 `libgazebo11-dev`。
- 如果 apt 源不能提供目标版本，需要参考 `docs/debug/rk3588_install_log_2026-05-31.md` 中的源码打包记录。
- 如果已经有构建好的 `.deb` 包，可直接在同款 RK3588 板子上安装。

## 7. 编译 Gazebo 时磁盘和内存压力较大

现象：

Gazebo 源码编译耗时长，占用空间大，板端可能出现内存不足或构建失败。

当时资源占用：

```text
/home/jie/build_gazebo_classic 约 7.5G
```

当时曾临时创建：

```text
/swapfile
```

用途：

- 防止 Gazebo 编译时内存不足。

处理建议：

构建前检查：

```bash
df -h /
free -h
```

如果内存不足，可临时启用 swap。构建完成后可以关闭并删除。

构建完成且不再重新编译 Gazebo 时，可以清理：

```bash
rm -rf /home/jie/build_gazebo_classic
sudo apt clean
```

## 8. PX4 编译失败：NuttX tag 缺失

现象：

执行：

```bash
cd /home/jie/PX4-Autopilot
DONT_RUN=1 HEADLESS=1 make px4_sitl gazebo-classic
```

第一次构建失败。

当时原因：

- shallow submodule 中缺少 NuttX 的 `nuttx-*` tag。
- `px_update_git_header.py` 生成版本头时找不到 NuttX tag。

处理：

```bash
git -C /home/jie/PX4-Autopilot/platforms/nuttx/NuttX/nuttx fetch --tags --force
```

然后重新构建：

```bash
cd /home/jie/PX4-Autopilot
DONT_RUN=1 HEADLESS=1 make px4_sitl gazebo-classic
```

验证产物：

```bash
ls -lh /home/jie/PX4-Autopilot/build/px4_sitl_default/bin/px4
ls /home/jie/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/*.so
```

## 9. `HEADLESS=1 make px4_sitl gazebo-classic` 被 timeout 截断

现象：

冒烟测试时用 `timeout` 启动 PX4/Gazebo，退出码为：

```text
124
```

原因：

- `124` 是 `timeout` 主动截断的退出码。
- 不代表 PX4/Gazebo 启动失败。

判断是否启动成功应看日志：

```text
Simulator connected on TCP port 4560.
PX4 startup script returned successfully
```

结论：

- 对长驻服务类命令，不要只看 timeout 退出码。
- 要结合启动日志判断。

## 10. 多机 SITL 默认启动 `gzclient` 导致板端崩溃

现象：

PX4 自带多机脚本默认会启动 `gzclient`。

板端没有可用显示环境时，可能出现：

```text
Can't open display
gzclient abort
```

原因：

- RK3588 板端常用 headless 运行。
- 没有桌面显示或 OpenGL 环境时，`gzclient` 不稳定。

处理：

- 本项目 `scripts/start_px4_multi_sitl.sh` 默认：

```bash
HEADLESS=1
```

- 当前脚本直接启动 `gzserver`，不启动 `gzclient`。

如果确实需要 GUI：

```bash
HEADLESS=0 ./scripts/start_px4_multi_sitl.sh 1 iris
```

但板端日常调试建议保持 headless。

## 11. Gazebo ROS 插件无关加载错误

现象：

启动 Gazebo 时可能出现 `gazebo_ros` 相关插件加载错误。

原因：

- 当前 PX4/Gazebo Classic SITL 主线不依赖 `gazebo_ros` 插件。
- 如果环境中 `ROS_VERSION=2`，部分 Gazebo 启动逻辑可能尝试加载 ROS Gazebo 插件。

处理：

当前 `scripts/start_px4_multi_sitl.sh` 启动 `gzserver` 时使用：

```bash
ROS_VERSION= gzserver ...
```

除非显式设置：

```bash
ENABLE_GAZEBO_ROS_PLUGINS=1
```

否则不加载 `libgazebo_ros_init.so` 和 `libgazebo_ros_factory.so`。

## 12. Anaconda / Python 环境可能干扰 ROS2 构建

现象：

`colcon build` 过程中找错 Python，或者 CMake 找到 Anaconda 的 Python。

原因：

- ROS2 Humble 和项目消息生成应使用系统 Python 3.10。
- Anaconda 改写 PATH 后可能让 `python3` 指向非系统 Python。

处理：

构建时显式指定系统 Python：

```bash
cd /home/jie/uav_swarm_system/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
  -DPYTHON_LIBRARY=/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/libpython3.10.so
```

构建前检查：

```bash
which python3
python3 --version
```

期望：

```text
/usr/bin/python3
Python 3.10.x
```

## 13. `px4_msgs` 版本必须匹配 PX4 v1.14

现象：

`px4_bridge_uxrce` 编译或运行时，PX4 消息字段对不上。

原因：

- `px4_msgs` 的 message 定义必须和 PX4 版本匹配。
- 当前 PX4 固定为 `v1.14.4`。

处理：

使用：

```text
px4_msgs release/1.14
```

检查：

```bash
git -C /home/jie/uav_swarm_system/ros2_ws/src/px4_msgs branch --show-current
git -C /home/jie/uav_swarm_system/ros2_ws/src/px4_msgs describe --tags --always
```

不要改成 PX4 main 对应的 `px4_msgs`。

## 14. `rosdep init` 重复执行不是错误

现象：

安装脚本执行：

```bash
sudo rosdep init
```

如果系统之前已经初始化过，会提示已存在。

处理：

脚本中已使用：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

所以重复初始化不是失败点。

真正要关注的是：

```bash
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
```

是否正常完成。

## 15. QGC 不在 RK3588 上，网络要单独确认

现象：

PX4/Gazebo/ROS2 都启动了，但 Windows QGC 看不到飞机。

原因：

- QGC 运行在 Windows，不在 RK3588。
- Windows 防火墙或 PX4 MAVLink remote host 配置可能不对。

检查：

```bash
ping -c 3 192.168.1.20
```

Windows PowerShell：

```powershell
netstat -ano -p udp | findstr 14550
```

当前项目中，PX4 GCS 链路应发往：

```text
Windows 192.168.1.20:14550
```

相关文件：

```text
/home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
/home/jie/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink
```

注意：

- 源码 `ROMFS` 文件用于后续重新构建。
- 当前运行时使用 `build/.../px4-rc.mavlink`。
- 只改源码但不重新构建，当前 SITL 不一定生效。

## 16. 安装完成后的最小验证顺序

新开终端：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
```

检查版本：

```bash
ros2 --version
MicroXRCEAgent --version
gazebo --version
git -C /home/jie/PX4-Autopilot describe --tags --always --dirty
```

检查 ROS2 包：

```bash
ros2 pkg list | grep -E 'swarm_bringup|px4_bridge_uxrce|formation_controller'
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py --show-args
```

启动顺序：

```text
1. MicroXRCEAgent
2. PX4/Gazebo SITL
3. ROS2 bridge / swarm nodes
4. arm/takeoff
```

不要一开始直接跑大规模多机。先按顺序验证：

```text
1 架 -> 3 架 -> 两板各 1 架 -> 更多实例
```
