# CARLA 仿真踩坑实录：那些文档不会告诉你的 5 个致命问题

> 作者：chuyue-setsnua | 项目地址：[carla-adas-test-platform](https://github.com/chuyue-setsnua/carla-adas-test-platform)

---

## 前言

看 CARLA 官方 Demo 跑起来的时候你会觉得一切都很丝滑——点一下 spawn，车就出现在路上了。但当你真正开始写自动化测试场景时，你会遇到一堆"Google 搜不到、文档没写、只能靠自己 debug"的问题。这些问题每一个都花了我半小时到两小时排查，踩过之后回头看，本质上都是对 CARLA 内部机制不够了解导致的。

本文记录我在开发 ADAS 仿真测试平台过程中遇到的最痛的 5 个坑，以及每个坑的排查思路和最终解决方案。

---

## 坑 1：spawn_actor 后 get_transform().location 恒返回 (0, 0, 0)

**现象**：spawn 一辆车，立刻读取坐标，永远是 (0.0, 0.0, 0.0)。画面上车明明在正确位置，但坐标就是不对。更诡异的是基于这个错误坐标计算的前车位置也全部偏移，导致前车 spawn 到马路外面去。

**排查过程**：一开始我以为是 spawn_point 本身有问题，打印了所有 spawn_points 的坐标，完全正确。然后怀疑是 blueprint 的问题，换了几个 blueprint 都一样。最后怀疑是 get_transform() 的返回时机问题——是不是 CARLA 的 transform 不是立刻同步的？

**根因**：spawn_actor 只是向引擎提交了创建请求，真正的位置同步需要等物理引擎跑几个 tick。CARLA 在 synchronous mode 下，每调用一次 `world.tick()` 才会推进一帧。如果你 spawn 后立刻读 transform，引擎还没来得及把你的 actor 放到物理世界里。

**修复**：

```python
ego = world.spawn_actor(ego_bp, spawn_point)

# 等物理引擎同步——10 个 tick 足够
for _ in range(10):
    world.tick()
    time.sleep(0.01)

# 现在读到的坐标是正确的
ego_loc = ego.get_transform().location
```

**教训**：**永远不要假设 spawn_actor 是同步的**。更保险的做法是：如果你要基于某个 actor 的坐标去 spawn 另一个 actor，直接使用 spawn_point 的坐标做几何计算，不要依赖 get_transform()——后者是瞬时值，前者是你确定的值。

---

## 坑 2：AEB 制动后 ACC 重新加速，反复逼近最终碰撞

**现象**：行人横穿场景，ego 检测到行人后 AEB 触发，车辆开始减速。TTC 因为车速降低而恢复到安全范围，AEB 逻辑判定"已经安全了"于是退出制动。退出后 ACC 控制器接管，发现车速远低于巡航目标，于是开始加速。一加速 TTC 又降低，AEB 再次触发……如此反复，最终要么撞上，要么在行人面前反复"点头"。

**排查过程**：参数调了无数次——加大制动力度、降低 AEB 触发阈值、提前触发时机——都不管用。最后看时间序列数据才发现关键：**AEB 退出后 ACC 重新加速了**。这不是参数问题，是逻辑设计问题。

**根因**：AEB 和 ACC 是两个独立的控制器，AEB 退出条件判断的是"当前 TTC 是否安全"，但它没有考虑"一退出马上有人会踩油门"。在行人横穿场景下，行人还没完全通过车道，ego 就不应该恢复巡航。

**修复**：引入 **persistent AEB（持续制动状态锁）**：

```python
if ttc < AEB_TTC_THRESHOLD:
    aeb_active = True  # 一旦触发，锁住

if aeb_active:
    throttle = 0.0
    brake = 1.0 if dist < 6.0 else 0.8
    # 只有行人已经远离且车已停稳才解锁
    if dist > 20.0 and speed < 0.5:
        aeb_active = False
```

**教训**：**安全系统不能用瞬时状态机**。真实车辆的 AEB 也不会在 TTC 刚恢复的瞬间就松刹车——它有一个"确认安全"的滞回窗口。做仿真控制器要模拟的不是理想的数学控制律，而是真实的安全逻辑。

---

## 坑 3：CARLA Server 反复 spawn/destroy 后损坏

**现象**：批量测试跑到第 20 组左右，所有新 spawn 的车辆全部出现在 (0, 0, 0) 坐标上，而不在 spawn_point 指定位置。这不是前面那个"刚 spawn 读不到"的问题——这次是物理上确实停在了 (0,0,0)，所有车辆堆叠在一起。

**排查过程**：检查了 spawn_point、blueprint、world 状态，全正常。重启测试脚本，又正常了，跑到 20 组再次复现。怀疑是 CARLA Server 内部有 ghost actor（已经 destroy 但没彻底释放的 actor）占用了 spawn 资源。

**根因**：长期反复 spawn/destroy actor 后，Unreal Engine 的 actor 池可能出现泄漏或状态不一致。这是 CARLA 0.9.x 的已知问题，社区里叫 "ghost actor"。

**修复**：三种策略：

1. 最彻底：杀掉 `CarlaUE4.exe` 进程重启，先 load 不同地图再切回目标地图刷新状态
2. 折中：每组测试之间 sleep 0.5s + world.tick() 若干次
3. 预防：spawn 前先销毁同类型所有已有 actor，保持世界干净

```powershell
# 杀 CARLA 进程（bash 中 taskkill 编码有问题，用 PowerShell）
powershell Stop-Process -Name CarlaUE4 -Force
```

**教训**：**仿真平台的稳定性本身就是测试的一部分**。台架测试开发要关注的不仅是场景逻辑，还有测试框架的健壮性——如果跑到一半 server 挂了，你的自动化测试就没有工程价值。

---

## 坑 4：CarlaUE4.exe 用绝对路径 start 静默失败

**现象**：在任意目录下执行 `start "E:\CARLA\CarlaUE4.exe"`，没有报错，但 CARLA 服务端也没启动。双击 exe 就正常。

**排查过程**：以为是权限问题、杀毒软件拦截、防火墙……折腾一圈。最后发现是 DLL 加载路径的问题。

**根因**：CarlaUE4.exe 依赖很多 DLL 和资源文件，Unreal Engine 在启动时会从**当前工作目录**查找这些依赖。如果你不在 E:\CARLA 目录下 start，它找不到依赖，直接静默退出，不报任何错。

**修复**：

```batch
cd /d E:\CARLA
start CarlaUE4.exe
```

**教训**：这个坑本质上是对 Windows 进程启动机制的理解不足——很多程序依赖工作目录来定位资源，Unreal Engine 尤其如此。

---

## 坑 5：Sync Mode 下多车 Placement 不能用 waypoint 的 get_left_lane()

**现象**：想在相邻车道 spawn cut-in 车辆，用了 `waypoint.get_left_lane().transform` 做位置计算。结果在 Town04 高架路上，所有相邻车道 waypoint 的 transform 都投射到了同一个点。

**排查过程**：起初以为是 Town04 地图的特殊性——高架路有多层，waypoint 可能混淆了高度。check 了 z 坐标，排除了。又尝试了 `project_to_road()` 矫正，依然无效。

**根因**：`get_left_lane()` 在多层道路结构中的行为不一致，在某些道路段返回的 waypoint 位置不可靠。CARLA 的 waypoint API 设计更适用于导航路径规划，而不是精确的三维坐标计算。

**修复**：改用 spawn_points 筛选策略——从地图的所有 spawn_points 中，根据 road_id、lane_id 和方向角过滤出符合条件的点，而不是从 waypoint 推算：

```python
for sp in spawn_points:
    wp = world_map.get_waypoint(sp.location)
    if (wp.road_id == same_road and 
        wp.lane_id != ego_lane and 
        abs(sp.rotation.yaw - ego_yaw) < 90):
        candidates.append(sp)
```

**教训**：**API 文档说的"能用"和工程上"可靠"是两回事**。waypoint 的 get_left_lane 做 demo 没问题，做自动化测试的 placement 就不行。永远要验证你的核心逻辑在目标地图上是否稳定。

---

## 总结

这 5 个坑的共同特征：都不是代码逻辑错误，而是对平台机制的理解不足。spawn 不是同步的、AEB 不能按瞬时状态退出、server 会损坏、进程有工作目录依赖、API 在不同地图上行为不一致——这些问题不会在教程里出现，只会在你真正写自动化测试时冒出来。

这也正是面试时最有价值的回答。当面试官问"你遇到过什么技术难点"，你能把每个坑的**现象 → 排查路径 → 根因分析 → 修复方案**讲清楚，而不是泛泛地说"调了一些参数"。这种 debug 能力才是台架测试开发岗最看重的。

完整代码：[github.com/chuyue-setsnua/carla-adas-test-platform](https://github.com/chuyue-setsnua/carla-adas-test-platform)
