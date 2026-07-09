# 我做了一个 CARLA 自动驾驶仿真测试平台——项目复盘与技术架构

> 作者：chuyue-setsnua | 项目地址：[carla-adas-test-platform](https://github.com/chuyue-setsnua/carla-adas-test-platform)

---

## 项目背景

我是智能车辆工程专业大三学生，目标岗位是**台架测试开发工程师（自动驾驶仿真方向）**。这个岗位的核心要求就几个：Python/C 自动化测试、CAN 总线、HIL/SIL 测试框架、至少一个仿真平台（CARLA 或 CarMaker）。

网上能找到的 CARLA 教程基本是"怎么 spawn 一辆车然后按 WASD 开"，跟台架测试完全不搭边。真实台架测试要的是：自动化运行场景 → 采集传感器 + CAN 数据 → 批量回归测试 → 生成测试报告。所以我决定从零搭一个完整的 ADAS 仿真测试平台，既能练技术，又能写进简历。

---

## 系统架构

整个平台围绕 CARLA Client-Server 架构搭建，核心分层如下：

```
┌─────────────────────────────────────────┐
│         Pygame UI Layer                  │
│  Camera View | HUD Panel | Live Charts   │
├─────────────────────────────────────────┤
│      Scenario Controller                 │
│  State Machine (IDLE → RUNNING → DONE)  │
│  Keyboard Input (SPACE/R/Q)             │
├──────────────────┬──────────────────────┤
│  AEB Controller  │  CAN Bus Logger       │
│  - TTC Graded    │  - 7 CAN IDs @ 20Hz   │
│  - Persistent    │  - Vector .asc format  │
│  - Lane Keeping  │  - CANalyzer compat   │
├──────────────────┴──────────────────────┤
│         CARLA Client API                 │
│  world.tick() | spawn_actor | sensors    │
├──────────────────────────────────────────┤
│      CARLA Server (CarlaUE4.exe)         │
│  Unreal Engine 4 | Physics | Rendering   │
└──────────────────────────────────────────┘
```

每一层做了一件事且只做一件事：场景控制器管状态机和用户输入，AEB 控制器只管跟车和制动逻辑，CAN 日志只负责数据编码和落盘。这种分层的好处是你换一个场景（比如从跟车 AEB 变成行人 AEB），只需要改场景控制器层，AEB 和 CAN 层完全不用动。

---

## 三个核心场景

### 1. AEB 跟车场景（scenario_aeb_test.py）

两车场景。Ego 以巡航速度跟随 Lead，Lead 在某个时刻踩死刹车。评估 ego 能否在碰撞前刹停。这是 Euro NCAP 的 Car-to-Car AEB 基础测试项。

### 2. Cut-in 切入场景（scenario_cutin.py）

三车场景，复杂度大幅提升。Ego 在自己的车道巡航，前方有 Lead，相邻车道有一辆 Cut-in 车比 ego 稍快，在设定的时间点变道切入 ego 前方。这个场景要同时跟踪两个目标（Lead 和 Cut-in），AEB 要判断哪个更近、哪个更危险。

三车放置是个技术难点——不是直接取三个 spawn_point 就行的。我的方案：ego 占 spawn_point，Lead 用几何投影在 ego 前方同车道，Cut-in 从 spawn_points 中根据 road_id 和 lane_id 动态筛选相邻车道的点。

### 3. 行人横穿 AEB（scenario_pedestrian.py）

CARLA 里行人和车辆是不同的 actor 类型。行人 spawn 需要 walker blueprint 和 WalkerController。这个场景最大的坑是**AEB 制动后 ACC 重新加速**——后面会详细讲。

所有场景都集成了 CAN 日志输出，每个 tick 记录 7 帧 CAN 报文到 .asc 文件。

---

## CAN 总线设计思路

面试官大概率会问"为什么选这 7 个 CAN ID"，提前准备一下我的设计逻辑：

- **0x0C0 车速 + 0x1A4 油门 + 0x1A0 制动**：这三者构成纵向控制闭环，任何一个场景都能据此评估控制质量
- **0x0C4 转向角 + 0x200 横摆角速度**：横向控制的输入和输出，评估车道保持能力
- **0x220 纵向加速度**：二重导数，能捕捉制动的瞬态响应（刹车踩下到实际减速之间的延迟）
- **0x300 雷达目标**：AEB 的核心感知数据，记录 ego 前方最近目标的距离和相对速度

选择 Vector .asc 格式是因为它是汽车电子行业的事实标准——CANalyzer/CANoe 不认 csv 和 json，只认 .asc/.blf。这也意味着你的测试数据可以直接交给标定工程师做进一步分析，这正是台架测试开发的日常工作流。

---

## 关键数据指标

以行人 AEB 场景为例，一次通过的测试结果：

| 指标 | 数值 |
|---|---|
| 触发 TTC | 2.8 s |
| 最小距离 | 4.2 m |
| 最大制动力 | 1.0（满刹） |
| 场景耗时 | 12.3 s |
| CAN 帧数 | ~1720 帧 |
| 测试结论 | PASS（无碰撞） |

批量测试框架跑 80 组参数，可以直接生成热力图，一眼看出哪些速度 + 间距 + 减速度组合会触发碰撞（FAIL），哪些安全通过（PASS）。这对调参和系统安全边界分析非常有用。

---

## 项目收获

做完这个项目，我对台架测试开发的理解从"大概知道"变成了"亲手做过"：

1. **仿真 ≠ 点一下 Play**。自动化场景运行要考虑状态机、重置逻辑、异常处理，以及 headless 模式下没有 UI 时怎么验证结果。
2. **CAN 总线是仿真和真实台架的桥梁**。理解 CAN ID 设计、信号分辨率、报文周期，这些知识在 HIL 测试中直接复用。
3. **踩坑是最快的学习方式**。spawn_actor 的 (0,0,0) 问题、AEB 制动后重新加速、CARLA server 损坏——每一个坑都让我对引擎机制的理解深了一层。

代码和完整文档都在 GitHub：**[github.com/chuyue-setsnua/carla-adas-test-platform](https://github.com/chuyue-setsnua/carla-adas-test-platform)**。下一篇我会专门写这个项目里踩过的坑和修复过程，那些才是面试时最能让面试官点头的内容。
