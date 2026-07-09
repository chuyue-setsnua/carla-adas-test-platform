# 从零搭建 CARLA ADAS 仿真测试平台：Python 实现 AEB 场景自动化测试

> 作者：chuyue-setsnua | 项目地址：[carla-adas-test-platform](https://github.com/chuyue-setsnua/carla-adas-test-platform)

---

## 为什么要做这个

如果你在准备自动驾驶测试开发的校招岗位，大概率会看到 JD 里写着"熟悉 CARLA/CarMaker 仿真平台"。但大多数教程只会教你怎么在 CARLA 里手动开车——这和真实的台架测试开发完全是两回事。台架测试要做的是：自动化的场景运行、传感器数据采集、CAN 总线通信、批量参数回归测试。本文从零开始，带你搭建一个完整的 ADAS 仿真测试平台。

项目最终效果：一键启动 CARLA 服务端，运行 Python 脚本，Pygame 窗口实时显示上帝视角摄像头画面 + HUD 数据面板 + 速度/距离/TTC 实时曲线图，场景结束后自动生成四象限测试报告和 Vector .asc 格式的 CAN 日志文件。

---

## 环境准备

- CARLA 0.9.16（解压到 `E:\CARLA`，确保 `CarlaUE4.exe` 可用）
- Python 3.12（我用的 `C:\Users\...\Python312\python.exe`）
- Python 包：`carla`（CARLA 自带 wheel 安装）、`pygame`、`numpy`、`matplotlib`

安装 CARLA Python API：

```bash
pip install E:\CARLA\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl
pip install pygame numpy matplotlib
```

**关键细节**：启动 CARLA 服务端时**必须先 cd 到 CARLA 目录**，因为它的 DLL 依赖需要当前工作目录正确。直接双击 exe 或用绝对路径 start 会静默失败。

```batch
cd /d E:\CARLA
start CarlaUE4.exe
```

---

## 第一个场景：让车跑起来

CARLA 的核心操作模式是 Client-Server。Client 连接 `localhost:2000`，通过 `world` 对象操控一切。

```python
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(10.0)
world = client.get_world()

# 关键：开启同步模式，固定仿真步长 0.05s（20Hz）
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 生成车辆
spawn_points = world.get_map().get_spawn_points()
ego_bp = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
ego = world.spawn_actor(ego_bp, spawn_points[0])

# 踩油门
ego.apply_control(carla.VehicleControl(throttle=1.0))

# 主循环
while True:
    world.tick()  # 每帧推进 0.05s
```

这里有一个新手必踩的坑：**spawn_actor 之后立刻调用 get_transform().location 拿到的坐标是 (0, 0, 0)**。不是 bug，是 CARLA 的物理引擎需要约 10 个 tick 才能同步 transform。解决方案：

```python
for _ in range(10):
    world.tick()
    time.sleep(0.01)
# 现在 get_transform() 才能拿到正确坐标
```

---

## AEB 核心逻辑：TTC 分级制动

自动驾驶里最关键的指标是 TTC（Time-to-Collision，碰撞时间）。计算公式极其简单：

```
TTC = 相对距离 / 相对接近速度
```

其中相对接近速度 = ego 的纵向速度 - 目标物在 ego 前进方向上的速度分量。

```python
ego_vel = ego.get_velocity()
eyaw = ego.get_transform().rotation.yaw
fx = math.cos(math.radians(eyaw))
fy = math.sin(math.radians(eyaw))

ego_lon = ego_vel.x * fx + ego_vel.y * fy    # ego 纵向速度
target_lon = target_vel.x * fx + target_vel.y * fy  # 目标纵向速度
approach_speed = ego_lon - target_lon          # 相对接近速度

if approach_speed > 0:
    ttc = dist / approach_speed
else:
    ttc = float("inf")  # 目标在远离，TTC 无限大
```

得到 TTC 后做分级制动——这是 Euro NCAP 测试中 AEB 的标准评估逻辑：

| TTC 区间 | 制动策略 |
|---|---|
| TTC > 4s | 正常 ACC 巡航 |
| 2s < TTC ≤ 4s | 轻刹 0.3 |
| TTC ≤ 2s | 紧急制动 1.0 |
| 距离 < 安全距 + 3m | 满刹 1.0 |

---

## CAN 总线日志：从仿真到 X-in-the-Loop 的关键一步

台架测试和纯仿真的核心区别在于：**你要输出的不是 "brake=1.0" 这样的 Python 变量，而是真实的 CAN 报文**。

我设计了 7 个标准 CAN ID，对齐真实车辆的总线协议：

| CAN ID | 信号 | 分辨率 | 字节数 |
|---|---|---|---|
| 0x0C0 | 车速 | km/h × 100 | uint16 |
| 0x0C4 | 转向角 | deg × 10 | int16 |
| 0x1A0 | 制动压力 | bar × 10 | uint16 |
| 0x1A4 | 油门开度 | 0-255 | uint8 |
| 0x200 | 横摆角速度 | deg/s × 10 | int16 |
| 0x220 | 纵向加速度 | m/s² × 100 | int16 |
| 0x300 | 雷达目标 | 距 m×10 + 相对速 m/s×10 | 4 bytes |

输出格式是 Vector .asc，可以直接导入 CANalyzer/CANoe 做时序分析。这是面试时的核心亮点——"我不仅会跑仿真，我还会生成 CAN 报文导入专业工具"。20Hz 频率下每帧 7 个 CAN ID，50ms 一个周期，与实际车辆总线频率对齐。

---

## 批量测试：80 组参数自动跑

人工一个一个改参数跑场景是没有工程价值的。我写了一个无头（headless）批量测试框架，定义了参数矩阵：

- 5 档 ego 速度：40, 50, 60, 70, 80 km/h
- 4 档初始间距：20, 25, 30, 35 m
- 4 档前车减速度：3, 5, 7, 9 m/s²

共 80 组，全部自动执行，输出热力图报告 + CSV 数据表。跑完一套大约 20 分钟，直接看出 AEB 参数的安全边界。

---

## 项目代码结构

```
carla-adas-test-platform/
├── can_simulator.py          # CAN 总线模拟库（可复用）
├── scenario_aeb_test.py       # 两车 AEB 追尾场景
├── scenario_cutin.py          # 三车 Cut-in 切入场景
├── scenario_pedestrian.py     # 行人横穿 AEB 场景
├── batch_test.py              # 无头批量参数扫描框架
├── run_*.bat                  # Windows 启动脚本
└── test_report/               # 自动生成的测试报告
```

全部代码已开源：**[github.com/chuyue-setsnua/carla-adas-test-platform](https://github.com/chuyue-setsnua/carla-adas-test-platform)**。

如果你也在准备自动驾驶仿真方向的校招，希望这个项目对你有帮助。下一篇我会写项目复盘，聊聊架构设计思路和面试里怎么讲这些技术点。
