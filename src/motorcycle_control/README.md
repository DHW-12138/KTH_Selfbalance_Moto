# 树莓派 ROS 2 速度主控

Waveshare USB-CAN-A 串口直连版本，独立于 IMU 包。假定用户所说的“SS停止”为通过 SSH 停止。
节点只发送目标速度，速度闭环和油门输出由 Teensy 实现。

## 配置

- 默认串口：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`；2,000,000 baud，8N1。
- 启动时发送适配器配置：500 kbit/s、标准帧、正常模式、可变长度串口协议、关闭自动重发；接收全部 ID，软件仅处理标准数据帧 0x201。
- 配置命令没有实现读回确认，串口写入成功不代表 CAN 已发送或 Teensy 已执行。
- 命令 ID 0x200，DLC 3，小端 `<BH>`；状态 ID 0x201，DLC 8，小端 `<HHHBB>`；错误 DLC 3，FF + 小端错误码。
- 波特率必须与 Teensy 和总线上其他节点一致；串口默认值可能被历史设置修改。
- 默认目标 0 km/h，上限 70 km/h（协议示例范围，需按实际车辆调整）。
- 每 100 ms 发送目标速度或 STOP；每 200 ms 请求状态。默认有效状态超时 1 秒。
- 每条正常状态发布为 `/motorcycle_master/status` 的 JSON String，包含速度、轮速、转速、油门、目标和本地主控 active 标志。active 不代表执行确认。

厂家协议：
https://www.waveshare.com/wiki/Secondary_Development_Serial_Conversion_Definition_of_CAN_Protocol
https://files.waveshare.com/wiki/USB-CAN-A/Demo/USB%20%28Serial%20port%29%20to%20CAN%20protocol%20defines.pdf

## 树莓派安装及运行

本包位于仓库的 `src/motorcycle_control`。在树莓派克隆仓库后，从仓库根目录构建。以下命令在树莓派执行；将 ROS_DISTRO 替换为已安装发行版名，或先设置该变量。

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/KTH_Selfbalance_Moto
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select motorcycle_control
source install/setup.bash
id
```

若用户不在 dialout 组，执行 `sudo usermod -aG dialout "$USER"`，然后退出 SSH 重新登录。不要同时打开厂商串口软件。

第一个 SSH 终端启动主控，启动后保持 STOP，不自动恢复先前运行状态：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/KTH_Selfbalance_Moto/install/setup.bash
ros2 run motorcycle_control master_node
```

可在启动时覆盖参数，例如 `--ros-args -p can_bitrate:=500000 -p max_speed_kmh:=30.0`。除目标速度外，参数在启动时配置。

第二个 SSH 终端（同样 source 上述两个 setup.bash），设置 5 km/h 并启动：

```bash
ros2 param set /motorcycle_master target_speed_kmh 5.0
ros2 service call /motorcycle_master/start std_srvs/srv/Trigger '{}'
```

启动要求：目标大于零，最近 1 秒收到合法状态，无当前错误。Teensy 尚未初始化时，拒绝启动是预期行为。
运行中修改目标立即在下一次周期发送时生效；目标设为 0 会撤销启动状态，后续正目标需重新调用 start。

停止（可从第三个已配置好 ROS 环境的 SSH 终端执行）：

```bash
ros2 service call /motorcycle_master/stop std_srvs/srv/Trigger '{}'
```

STOP 请求立即发送，后续每 100 ms 重复。含义为油门归零、积分复位、滑行，不是主动制动。服务 success 仅表示串口写入成功。

查看反馈：

```bash
ros2 topic echo /motorcycle_master/status
```

## 故障与退出行为

反馈超时、控制器错误、非法状态或串口故障会撤销 active，并尽力发送 STOP；恢复合法反馈后仍需明确调用 start。
正常 Ctrl+C 退出尽力发送 STOP。SSH 断开是否结束进程由会话管理方式决定，不能把关闭 SSH 当成停止命令。
进程被 SIGKILL、树莓派断电、USB/CAN 断线时软件无法保证 STOP 到达。Teensy 必须独立实现命令超时（例如连续 500 ms 没收到 STOP/SET_SPEED 即油门归零、积分复位；REQ_STATUS 不续期）。该行为尚待 Teensy 固件实现。

初次联调应先断开驱动输出或架空驱动轮，确认停止语义、反馈及超时后再接入实际驱动。需要外部 CAN 收发器、正确的 CANH/CANL/参考地连接及总线两端终端电阻。

## 本地验证

```bash
cd src/motorcycle_control
PYTHONPATH=. python3 -m pytest -q test
```

测试覆盖用户提供的速度和状态向量、串口分包/粘包/噪声、扩展/远程帧排除、错误及越界输入。不替代树莓派 ROS 2 和真实 CAN 硬件联调。
