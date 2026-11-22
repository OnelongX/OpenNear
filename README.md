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
│   │   └── on_node.h                # 完整节点引擎（G / T / 双区 FDMA Sub-G / Lease GC / Backup_ID / Fast HARQ）
│   └── src/
├── hal/
│   └── on_hal.h                     # 跨平台射频 / 高精定时器 / GPIO 硬件抽象层
├── sim/                             # 10 节点多跳物理仿真器（支持 CLI 混沌参数注入 + CSV + PCAP + 仪表盘数据）
│   ├── sim_world.h / sim_world.c
│   └── main_sim.c
├── tests/
│   └── test_opennear.c              # 20 组全链路自动化单测与集成断言套件（125 项断言）
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
- 所有节点板载 `GPIO2` LED 将在每个超帧的 `ACT/QUIET` 静默区齐刷刷同步闪烁。

### 路线 A：Nordic nRF52840 裸射频 + PPI（量产微安级低功耗）
进入 `ports/nrf52840_bare/`，使用 ARM GCC 工具链编译：
- `make MASTER=1`（Primary G） / `make MASTER=0`（Terminal / Sub-G / Backup）
- 硬件 PPI 通道直连：`RADIO->EVENTS_ADDRESS -> TIMER0->TASKS_CAPTURE[0]`（62.5 ns 分辨率硬件捕获）与 `TIMER0->EVENTS_COMPARE[2] -> GPIOTE->TASKS_OUT[0]`（零 CPU 抖动硬件动作触发）。

## 开源参考项目导航
按 L1 物理层、L2/L3 确定性同步与多跳、三条真机落地路线及原生星闪 SDK 整理的真实开源仓库清单见：[REFERENCES.md](./REFERENCES.md)。
