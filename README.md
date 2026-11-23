# OpenNear — Clean-Slate NearLink-Class Deterministic Wireless Protocol Stack

OpenNear 是一个零历史包袱、从零设计的开源类星闪（NearLink-Class）无线协议栈与多平台真机实现，覆盖 **L1 抗干扰物理层原语 → L2 6 字节极简帧头与四段式动态超帧 → L3 亚微秒硬件锁相与 Sub-G 错峰多跳 → L4 确定性同步动作与 Backup_ID 零投票热备自愈**。

## 目录结构

```text
opennear/
├── build_and_test.ps1               # 一键编译运行：全量单测 + 10 节点多跳仿真 + L1 基准 + ESP32/nRF52 交叉编译检查
├── core/                            # 零外部依赖的纯 C11 可移植协议核（Host / ESP32 / nRF52 共用）
│   ├── include/
│   │   ├── on_types.h               # 12-bit Short-ID、四段式超帧时隙常量与角色定义
│   │   ├── on_frame.h               # 6 字节极简 MAC 帧头 + Hdr_CRC8 + 载荷 CRC16 编解码
│   │   ├── on_polar.h               # L1 控制面短码 CA-Polar (N=128, K=48, CA-SCL L=8)
│   │   ├── on_sync.h                # L3 亚微秒同步三步链：硬捕获 + 4时间戳 RTT + NCO TC-PLL
│   │   ├── on_superframe.h          # L2 四段式超帧排班（SYNC/MAP -> D-TDMA/Sub-G -> C-RA -> ACT/QUIET）
│   │   ├── on_sec.h                 # v3.1 安全层：AES-128-CCM + Beacon CBC-MAC 防伪造 + Session Salt + 密钥轮换
│   │   ├── on_ecdh.h                # v3.1 零预置密钥配网：恒定时间 X25519 (RFC 7748) + OOB PIN 密钥派生
│   │   ├── on_afh.h                 # v3.1 TSCH 自适应跳频 + Wi-Fi 干扰黑名单 + 原子倒计时 + 冷启动扫频
│   │   ├── on_amc.h                 # v3.1 L1 双波形速率自适应调制编码（2M MSK <-> 4M QPSK <-> 20M OFDM）
│   │   ├── on_sar.h                 # v3.1 轻量分片重组（最高 1 KB OTA/流）+ 3 级抢占式 QoS 队列
│   │   ├── on_ranging.h             # v4.0 星闪 2.0 通感一体：16 频点多载波相位差 (MCPD) + ToF 厘米级测距与防中继围栏
│   │   ├── on_ota.h                 # v5.0 节点多跳 OTA：AES-CCM 加密、CRC32+CBC-MAC 鉴权、32 块滑动窗口 NACK、同步激活
│   │   ├── on_gateway.h             # v5.0/v5.1 公网 Wi-Fi STA & BLE GATT 共存网关 + 无锁 SPSC 队列 + 管理令牌鉴权
│   │   ├── on_mesh.h                # v6.0 3~6 跳混合 Mesh 路由自愈 + 256 节点多周期 Epoch-TDMA 休眠调度器
│   │   ├── on_mqtt.h                # v6.0 纯 C11 OASIS MQTT 3.1.1 编解码器 + Home Assistant 自动发现
│   │   └── on_node.h                # 完整节点引擎（无主冷启动 / G / T / Sub-G / 集群拓扑快照 / OTA / Mesh）
│   └── src/
├── hal/
│   └── on_hal.h                     # 跨平台射频 / 高精定时器 / GPIO 硬件抽象层
├── sim/                             # 10 节点多跳物理仿真器（支持 CLI 混沌参数注入 + CSV + PCAP + 仪表盘数据）
│   ├── sim_world.h / sim_world.c
│   └── main_sim.c
├── tests/
│   └── test_opennear.c              # 32 组全链路自动化单测与集成断言套件（230+ 项断言）
├── tools/
│   ├── dashboard.html               # 交互式 HTML 协议可视化与极限破坏性压测工作台
│   ├── chaos_server.mjs             # 在线调用底层 C11 仿真引擎的交互式混沌压测本地服务器
│   └── opennear_dissector.lua       # Wireshark Lua 抓包解析插件（DLT_USER0 = 147）
└── ports/
    ├── esp32_wroom32/               # 路线 C：ESP32-WROOM-32 裸 0xD0 @ 54M + esp_timer 硬件 ACT + TRNG Salt
    ├── nrf52840_bare/               # 路线 A：nRF52840 裸 RADIO+PPI + RTC0 双时钟微安休眠 + 启动/链接脚本
    └── sdr_fpga_ref/                # 路线 B：CFO(+40ppm)+多径短前导 + 定点 Q4.4 CA-Polar + 可综合 Verilog RTL
        └── rtl/                     # opennear_preamble_correlator.v + opennear_polar_encoder/decoder_n128.v + tb
```

## 快速运行（一键编译与全量验证）

在 PowerShell 中直接执行：

```powershell
powershell -ExecutionPolicy Bypass -File d:\AIOS\opennear\build_and_test.ps1
```

该脚本会自动完成四项验证：
1. **核心协议自动化测试套件**（`build/test_opennear.exe`）：校验 6 字节短头与独立 `Hdr_CRC8` 拦截、CA-Polar 抗干扰解码、4 时间戳 RTT 与 40 ms 滑行锁相、10 节点含两堵墙 2 跳多跳同步与 `Backup_ID` 第 4 帧 `0 μs` 热备切换。
2. **10 节点全生命周期仿真**（`build/opennear_sim.exe`）：输出各节点 NCO 频偏锁定、RTT 标定、低功耗射频占空比与物理世界真实同步偏差，并导出 `build/opennear_sim_trace.csv`。
3. **路线 B L1 物理层基准**（`build/l1_phy_bench.exe`）：实测 `8 μs` 专用短前导相关峰捕获精度及 CA-Polar 在 `-3 dB ~ +2 dB` AWGN 下的 BLER 曲线。
4. **真机硬件端口交叉编译检查**：使用本地 `xtensa-esp32-elf-gcc/g++` 交叉编译 `ports/esp32_wroom32` 全部源码，并对 `ports/nrf52840_bare` 执行 C11 严格语法校验。

## 硬件烧录指南

### 路线 C：手头 10 块 ESP32-WROOM-32 实测同步闪灯
进入 `ports/esp32_wroom32/`，使用 PlatformIO 分别烧录主控与从机（或全部烧录 `esp32_node_t`，将其中一块的 `GPIO4` 接地即可自动作为 Primary `G` 启动）：
- 主控 G 节点：`pio run -e esp32_node_g -t upload`
- 从机 T / Sub-G / Backup 节点：`pio run -e esp32_node_t -t upload`
- **v6.0 万物互联全功能网关**（公网 Wi-Fi STA + HTTP `/api/topology` & `/api/data` + BLE GATT `0xFFE0` + MQTT/Home Assistant 自动发现 + A/B 分区 OTA + 无主冷启动）：
  - 网关/主控：`pio run -d .\ports\esp32_wroom32 -e esp32_iot_mesh_gw -t upload`
  - OTA/BLE 终端：`pio run -d .\ports\esp32_wroom32 -e esp32_node_t_ota -t upload`
  - 蓝牙 `0xFFE2` / HTTP `POST /api/cmd` 支持指令：`WIFI:<ssid>|<pass>|<cloud_url>`、`MQTT:<host>|<port>|<user>|<pass>`、`OTA:<target_hex>|<ver>|<http_url>`、`CMD:<target_hex>|<payload>`、`TOPO?`、`DATA?`（开启 `OPENNEAR_ADMIN_TOKEN` 时自动要求 `AUTH:<token>|...` 或 HTTP Header `X-OpenNear-Token`）。
- 节点使用 `GPIO2` 输出超帧中的 ACT 动作；串口 `115200` 下可查看角色、同步帧接收量、上行收发计数和接管次数。
- ESP32 的原始 Wi-Fi 发送由驱动排队，提交成功不代表已经发射，且没有可用于此端口 RTT 校准的实际发射时间戳。因此这个端口不启用空口 RTT 校准或基于提交时间的 PTP 跟进修正，备用节点须同时满足连续漏收至少 12 帧和本地时钟计时至少 150 ms 未收到有效同步帧才接管；仿真和其他端口默认仍为 3 帧，无额外时长门槛。GPIO 对齐精度需用示波器或逻辑分析仪独立测量。

- 主节点重启更换会话后，直连从节点收到认证的主同步帧即丢弃旧时隙并自动重新入网；若允许备用角色，主节点可以重新选它为备用。相同会话内的密钥轮换不触发重新入网。同一会话内，动态入网的直连从节点也会在主同步帧不再公布其时隙时重新入网。ESP32 将上行租约设为 250 个超帧（默认 4 ms 超帧下约 1 秒），避免短暂的 Wi-Fi 上行丢帧撤销时隙。串口的 `rejoins` 记录重新入网，`leases` 记录已回收的租约。

#### 双板主备真机回归

烧录主控和备用节点后，关闭占用两个串口的监视器，并在项目根目录运行：

```powershell
python -m pip install pyserial
python .\tests\test_esp32_failover_hw.py --primary COM3 --backup COM4
```

将端口号替换成实际值。这个测试会通过 RTS 多次复位主控（要求主控板的 USB 串口支持 RTS 拉住 EN），不会擦除 Flash 或重新烧录。默认先观察 180 秒，再做 12 轮主备切换，最后观察 30 秒。每轮检查备用接管、原主恢复后备用无需重启即可重新入网并再次获得备用资格。短地址是动态分配的，脚本不会将备用地址固定为某个值。前后观察阶段检查两板角色和接管计数、意外重启，以及同步帧和上行数据计数是否持续增长。

串口原始日志和 JSON 摘要写到 `build/hardware-esp32/`，失败时返回非零状态。串口状态约每秒输出一次，记录的时间是观测时间，不用于测量精确接管延迟或 GPIO 对齐精度。

#### 双板 GPIO 同步测量（无外置逻辑分析仪）

主控 ESP32 可通过两个 MCPWM 捕获通道，用同一个硬件计数器给两路 GPIO 边沿打时间戳。拔掉两板 USB 电源后接线（按 GPIO 编号，不是排针序号）：

| 信号 | 接法 |
| --- | --- |
| 共地 | 主控 GND ↔ 从机 GND |
| 主控 ACT | 主控 GPIO2 → 主控 GPIO32 |
| 从机 ACT | 从机 GPIO2 → 主控 GPIO33 |

不要把两块板的 GPIO2 并在一起，也不要互连 3V3 或 5V。两板接回原 USB 口后，只在主控上烧录测量环境，从机保留原固件。在项目根目录运行：

```powershell
pio run -d .\ports\esp32_wroom32 -e esp32_node_g_capture -t upload --upload-port COM3
python .\tests\measure_esp32_gpio_sync.py --port COM3 --seconds 60
```

脚本默认在 ±10 ms 内一对一配对同极性边沿。`build/gpio-sync/` 保存原始串口记录、配对 CSV 和 JSON；正的时间差表示从机在后。看 P95、P99、最大偏差时也要检查漏配数量，窗口外的边沿不参与统计。硬件计数器分辨率不是整套测量的准确度；如需估计通道间偏差，先用同一个主控 GPIO2 驱动 GPIO32 和 GPIO33，再恢复表中的双板接线。测量结束可烧回 `esp32_node_g`。

如需定位偏差来源，保持上述接线，分别烧录诊断环境 `esp32_node_g_trace`（主控）和 `esp32_node_t_trace`（从机）。待两板入网后，在项目根目录运行：

```powershell
python .\tests\capture_esp32_act_trace.py --primary COM3 --terminal COM4 --seconds 60
python .\tests\analyze_esp32_act_trace.py <主板日志路径> <从板日志路径>
```

诊断将真实 GPIO 边沿与固件内部的动作超帧、计划时间、写入时间和最后收到的同步帧关联。USB 串口到达时间只用于初步对齐记录，不用于计算偏差。报告中的「主板计划发帧时刻 → 从板 Wi-Fi 回调」包含发送调度、驱动排队、空口和接收驱动延迟；它不是纯空口时延。诊断环境会增加少量记录开销，不替代无诊断固件的基线测量。

#### 可选实验：提前广播 GPIO 翻转

两块直连 ESP32 分别烧录 `esp32_node_g_lookahead`（主板）和 `esp32_node_t_lookahead`（从板）；沿用上述接线、trace 采集和分析命令。对照组用 `esp32_node_g_trace` / `esp32_node_t_trace`。普通 `esp32_node_g` / `esp32_node_t` 仍保持原行为。

主板在翻转前 1–7 个超帧重复广播倒计时。从板将目标超帧的 ACT 时刻转换成本地截止时间，并取同一目标多次观测中最早的预测值；目标帧信标迟到或丢失时也可以执行。扩展占用主 `SYNC_MAP` 第 37 字节（从 0 编号为 36）的第 1–3 位，仍是 44 字节，保留当前电平字节、AFH/PHY 旗标和认证标签位置。此模式仅供直连二值 GPIO 演示，不扩展到二跳广播或任意动作负载，也不修改 TDMA 时钟。

启用后 trace 所记超帧是计划目标，不等于从板实际写入时所处的主板超帧。比较精度以捕获器的真实时间差为准；`gpio_terminal_late_by_sf` 统计从板真实边沿至少晚一个超帧的已诊断配对。trace 中最后收到的同步帧不一定是提供计划时间的那一帧。提前执行不等同于校准驱动排队造成的绝对时钟偏差。

### 路线 A：Nordic nRF52840 裸射频 + PPI（量产微安级低功耗）
进入 `ports/nrf52840_bare/`，使用 ARM GCC 工具链编译：
- `make MASTER=1`（Primary G） / `make MASTER=0`（Terminal / Sub-G / Backup）
- 硬件 PPI 通道直连：`RADIO->EVENTS_ADDRESS -> TIMER0->TASKS_CAPTURE[0]`（62.5 ns 分辨率硬件捕获）与 `TIMER0->EVENTS_COMPARE[2] -> GPIOTE->TASKS_OUT[0]`（零 CPU 抖动硬件动作触发）。

## 开源参考项目导航
按 L1 物理层、L2/L3 确定性同步与多跳、三条真机落地路线及原生星闪 SDK 整理的真实开源仓库清单见：[REFERENCES.md](./REFERENCES.md)。
