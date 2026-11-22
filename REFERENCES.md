# OpenNear 开源参考项目导航（按协议层与落地路线映射）

以下均为已核验的真实开源仓库，精准对应 OpenNear 的每一层核心模块，可直接作为代码级参考或后续扩展组件：

## 1. 最接近 OpenNear 路线 A（nRF52840 裸射频 + PPI + 确定性 TDMA）

| 开源项目 | 仓库地址 | 可直接借鉴的核心实现 |
| :--- | :--- | :--- |
| **DotBots / Mari**（Inria 团队） | https://github.com/DotBots/mari | **最像“开源类星闪 SLE”的真机项目**：在 Nordic `nRF52840 / nRF5340` 上绕开完整 BLE 协议栈，直接在 2 Mbps 裸物理层上实现确定性 **TDMA / TSCH**（支持数百节点高密度机器人集群）。重点看 `firmware/` 下的时隙状态机与无线调度器。 |
| **nRF5-ble-timesync-demo** | https://github.com/nordic-auko/nRF5-ble-timesync-demo | **OpenNear `hal_nrf52_radio_ppi.c` 的权威对照**：演示如何用 Nordic `RADIO + PPI + TIMER` 在零 CPU 干预下做微秒级无线时钟同步（重点看 `time_sync.c` 的 PPI 通道配置与自由运行定时器捕获）。 |
| **ETHZ-TEC / Baloo**（苏黎世联邦理工） | https://github.com/ETHZ-TEC/Baloo | 著名的 **Glossy 同步传输（Synchronous Transmissions）** 多跳框架，支持 nRF52840，将底层的微秒级同步原语与上层调度完全解耦，非常适合参考多跳时钟同步与抗干扰重传。 |
| **Contiki-NG (TSCH)** | https://github.com/contiki-ng/contiki-ng | 工业级开源 **TSCH（时隙信道跳频）** 协议栈，内置无 SoftDevice 依赖的 `nRF52840` 完整移植（见 `doc/platforms/nrf52840.md`），适合参考动态跳频黑名单（AFH）与低功耗占空比管理。 |
| **OpenWSN (`openwsn-fw`)** | https://github.com/openwsn-berkeley/openwsn-fw | UC Berkeley / Inria 开源 6TiSCH 确定性时隙协议栈，适合参考超帧时隙表、多跳父节点选择与网络自愈逻辑。 |

## 2. 对应 OpenNear L1 物理层与路线 B（CA-Polar 编解码 + Zynq/AD9361 FPGA）

| 开源项目 | 仓库地址 | 可直接借鉴的核心实现 |
| :--- | :--- | :--- |
| **AFF3CT** | https://github.com/aff3ct/aff3ct | 开源性能最强的 C++ 信道编解码库，完整实现 5G NR **Polar / CA-SCL（CRC 辅助列表译码）** 与 **LDPC**，带 SIMD 加速；配套的 https://github.com/aff3ct/polar_decoder_gen 可自动生成展开的高速 SC/CA-SCL 源码，可直接升级 OpenNear `on_polar.c`。 |
| **openwifi**（`open-sdr`） | https://github.com/open-sdr/openwifi （硬件 Verilog 仓：https://github.com/open-sdr/openwifi-hw ） | **路线 B（Zynq SoC + AD9361 SDR）的标杆开源基带**：在 AntSDR E310 / ZedBoard / ZC706 等板卡上完整跑通 FPGA 物理层 OFDM、相关峰捕获、细频偏跟踪与低层 MAC（`xpu`），改掉它的前导与 MAC 即是 100% 物理层自定义 OpenNear。 |

## 3. 对应 OpenNear 路线 C（ESP32 802.11 裸帧 & 低时延无线控制）

| 开源项目 | 仓库地址 | 可直接借鉴的核心实现 |
| :--- | :--- | :--- |
| **ExpressLRS** | https://expresslrs.org （GitHub: `ExpressLRS/ExpressLRS`） | 开源超低时延 2.4 GHz 固定节拍跳频遥控链路（支持 ESP32 + SX1280/LR1121 FLRC/LoRa 芯片），它的锁定节拍状态机、丢包滑行同步与极简帧压缩思路极具工程参考价值（注：ELRS 主链路使用外置 SX12xx 射频芯片，ESP32 内置 Wi-Fi 仅作辅助，正好印证了我们选择裸射频芯片的架构判断）。 |
| **ESP-IDF `esp_wifi_80211_tx` 生态** | 官方头文件 `esp_wifi.h`（配合社区 `esp32free80211` 等裸帧实验仓） | 本项目 `ports/esp32_wroom32/src/hal_esp32_raw80211.c` 已直接落地 `0xD0` 裸帧 @ 54M + `rx_ctrl.timestamp` 64 位无回绕展开方案。 |

## 4. 原生星闪（NearLink / SLE）官方开源 SDK 对照

如果想同时把 OpenNear 和商用真星闪芯片（如海思 WS63 / BS21）做横向对比实测：

| 开源项目 | 仓库地址 | 说明 |
| :--- | :--- | :--- |
| **HiSpark / `fbb_ws63`** | https://gitcode.com/HiSpark/fbb_ws63 （Gitee 历史仓：https://gitee.com/HiSpark/fbb_ws63 ） | 海思官方开源 WS63/WS63E（Wi-Fi 6 + 星闪 SLE + BLE）LiteOS SDK，官方文档见 https://docs.hisilicon.com/repos/fbb_ws63/zh-CN/master/ 。 |
| **HiHopeORG / NearLink** | https://gitee.com/HiHopeORG/NearLink | 润和软件整理的 WS63 / BS21 星闪开发板 Demo、SLE 组网示例与烧录工具汇总。 |
