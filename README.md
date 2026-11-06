# Unicycle PID Control System

基于 Infineon AURIX **TC377TP** 的自平衡独轮车（unicycle）控制系统。实现了姿态自平衡、速度与转向的串级 PID 控制，以及 GNSS + IMU 组合导航下的自主路径跟踪。

> 本科期间（2024 年）完成的嵌入式控制项目，基于逐飞科技 SeekFree TC377 开源库开发。

---

## 1. 硬件平台

| 类别 | 器件 |
| --- | --- |
| MCU | Infineon AURIX TC377TP（TriCore，三核） |
| 九轴 IMU | IMU963RA |
| 六轴 IMU | IMU660RA |
| 卫星定位 | GNSS TAU1201 |
| 激光测距 | DL1B |
| 电机 | 3 路直流电机 + 正交编码器（驱动轮 / 平衡 / 转向） |
| 无线 | 无线串口（逐飞助手上位机）、LoRa 3A22 |
| 人机交互 | 点阵屏（dot matrix screen）、5 按键菜单 |
| 灯光 | TLD7002 LED driver |
| 开发环境 | AURIX Development Studio (ADS) v1.9.20 |

## 2. 目录结构

```
main_code/
├── user/                 # 三核入口与中断
│   ├── cpu0_main.c       # CPU0：主控循环（Go / 菜单 / 姿态稳定判断）
│   ├── cpu1_main.c       # CPU1：姿态融合（First_complement）
│   ├── cpu2_main.c       # CPU2：预留
│   └── isr.c, isr_config.h  # PIT 定时中断，各控制环的调度节拍
├── code/                 # 本项目自研应用层
│   ├── PID.c/.h              # 位置式 / 增量式 PID
│   ├── Control.c/.h          # 串级控制环（见第 3 节）
│   ├── Posture.c/.h          # 姿态解算：互补滤波 + 卡尔曼 + 低通
│   ├── Combined_Navigation.c # GNSS/IMU 组合导航、航点逼近判断、轨迹跟踪
│   ├── Path_Planning.c/.h    # 航点采集与远程启动导航
│   ├── Motor.c/.h            # 电机与编码器驱动
│   ├── Interact.c/.h         # 上位机 / LoRa 交互
│   ├── UI.c/.h               # 点阵屏菜单
│   ├── User_Flash.c/.h       # PID 参数掉电保存
│   ├── zf_device_*           # 点阵屏 / LoRa3A22 / TLD7002 设备驱动
│   └── TLD7002_driver/       # TLD7002 分层驱动（Service / Func / Control）
└── libraries/            # 逐飞 SeekFree 开源库 + Infineon iLLD（第三方）
```

## 3. 控制架构

独轮车在俯仰（pitch）方向靠驱动轮前后加速维持平衡，在滚转（roll）方向靠飞轮/转向维持平衡，属于典型的双向欠驱动系统。控制采用**串级 PID**：

```
                 ┌──────────── 外环（慢）────────────┐
距离环  distance_loop
   └→ 速度环  Speed_loop ────┐
                             ↓ 输出期望倾角
        动态零点补偿  Dynamic_Zeroing_Pitch / _roll
                             ↓
      角度环  Angle_Loop ────┐
                             ↓ 输出期望角速度
   角速度环  Angle_Speed_Loop ──→ PWM（内环，最快）
```

- **平衡方向**：`distance_loop` → `Speed_loop` → `Angle_Loop` → `Angle_Speed_Loop`
- **转向方向**：`Divert_Loop` / `Divert_Speed_Loop` / `Divert_Angle_Loop` / `Turn_Angle`
- **动态零点**：`Dynamic_Zeroing_Pitch(vec, angle)` 按当前速度在线修正机械零点，抑制稳态前倾/后仰
- **安全保护**：`Attitude_stability_judgment(pitch, roll, yaw)` 姿态超限时切断输出
- 各环节拍由 `isr.c` 中的 PIT 中断分频驱动，内环最快 1 ms

PID 结构体（`PID.h`）统一支持 Kp/Ki/Kd、积分限幅 `Integral_lim` 与输出限幅 `Out_lim`，提供位置式 `Pid_Position()` 与增量式 `Pid_Incre()` 两种形式。

## 4. 状态估计与导航

- **姿态**：陀螺仪零偏自标定（`IMU_gyro_Offset_Init`）→ 一阶互补滤波（`First_complement`，跑在 CPU1）→ 卡尔曼滤波 `KalmanFilter()` / 低通滤波
- **航向**：`IMU_YAW_integral()` 积分，并对 0°/180°/360° 跨越做连续化处理（`IMU_Handle_*`）
- **组合导航**：`Data_Collect()` 采集 GNSS 经纬度与 IMU 航向，`get_two_points_distance()` 计算两点球面距离，`Point_Approaching_Judgement()` 判断航点到达，`Track()` 执行轨迹跟踪
- **航点来源**：手动 `Auto_Collect()` 采集记录，或经 LoRa `remote_Start_Navigate()` 远程下发启动

## 5. 编译与烧录

1. 安装 **AURIX Development Studio v1.9.20 或更高版本**
2. `File → Import → Existing Projects into Workspace`，选择本仓库的 `main_code/` 目录
3. 选中工程 → `Build`（首次编译会重新生成 `Debug/`，该目录已被 `.gitignore` 排除）
4. 用 DAP/miniWiggler 连接 TC377 核心板，`Run → Debug` 烧录

## 6. 说明

- 源文件注释为 **GB2312/GBK 编码**（逐飞库惯例）。仓库已通过 `.gitattributes` 中的 `* -text` 关闭行尾与编码转换；若在 VS Code 中查看中文注释乱码，请将文件编码切换为 GB2312。
- `main_code/libraries/` 为第三方代码：逐飞科技 SeekFree TC377 开源库（GPL-3.0）与 Infineon iLLD，版权归原作者所有，许可证见 `main_code/libraries/doc/`。
- 引脚分配参考仓库内 `main_code/推荐IO分配.txt` 与 `main_code/尽量不要使用的引脚.txt`。
