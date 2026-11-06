# 数显 USB PD3.2 充电器固件

本工程是基于 **CH32L103C8T6** 的 USB PD 数字电源固件。主控使用片上 USB PD PHY 完成 Type-C Source 协议通信，通过 **MP2980** 调节 Buck-Boost 功率级，通过 **CH211** 管理 Type-C 高压接口与 VBUS 通路，并使用两路 **INA226** 采集 Type-C、Type-A 端口的电压、电流和功率。

整机由两块同构功率板堆叠组成，每块板提供 1 路 Type-C 和 1 路 Type-A。两块 PCB 共用同一套源码，通过编译宏区分主板与副板：副板采集本地端口状态并通过 USART 上报，主板汇总四路数据，驱动 LCD 显示并根据总输出功率调节风扇。

---

# 功能概览

- **USB PD Source：** 支持 SPR Fixed、SPR PPS、SPR AVS、EPR Fixed 与 EPR AVS。
- **默认输出能力：** Fixed 5V/9V/12V/15V @ 3A、20V @ 5A，PPS 5\~21V，SPR AVS 9\~20V，EPR Fixed 28V @ 5A，EPR AVS 15\~28V，单路最高 140W。
- **线缆能力识别：** 通过 SOP' Discover Identity 读取 E-Marker，根据线缆电流和耐压能力动态限制 5A 与 EPR 档位；普通线缆按 3A 能力发布。
- **数字电源控制：** 根据 PD 请求异步配置 MP2980 输出电压与限流值，并结合 INA226 的 VBUS 反馈确认电源稳定后发送 `PS_RDY`。
- **功率监测：** 两路 INA226 分别采集本板 Type-C 与 Type-A 端口数据，供保护、板间通信、显示和散热控制使用。
- **故障管理：** 汇总 LM5069、LMR33630、CH217K、INA226、MP2980 与 CH211 的 GPIO/寄存器状态，完成消抖、延时屏蔽、协议层上报与界面提示。
- **四口状态显示：** 主板 LCD 显示 C1、A1、C2、A2 的电压、电流、功率、协议类型及故障状态。
- **功率联动散热：** 主板按四个端口的总功率生成风扇 PWM；100W 以下保持 30% 占空比，100\~300W 线性提升，300W 到达最大值 80%。

> 实际可发布的电流与 EPR 能力由线缆识别结果动态决定，并不等同于始终开放全部档位。

---

# 硬件架构设计

原理图、PCB、BOM 以及描述等见立创开源硬件平台链接 [基于CH32L103和MP2980的数显PD3.2充电器](https://oshwhub.com/luoxifnt/project_utfaikpd)。

---

# 软件架构设计

固件采用**中断提供时间基准、主循环协作调度**的架构。各协议与外设模块均以短时、非阻塞状态机运行，耗时的 I2C 和 SPI 传输交由 DMA 完成，避免功率芯片访问或整屏刷新阻塞 PD 报文处理。

```text
src/
├─ main.c, board_config.h        应用入口与板级引脚/常量配置
├─ platform/                     板级适配层：CH32L103 外设、系统与 port
│   ├─ ch32l103_init.*           时钟、GPIO 与外设初始化
│   ├─ ch32l103_it.*             中断向量实现（含 1 ms 系统节拍）
│   ├─ ch32_i2c_dma.*            共享 I2C DMA 队列
│   ├─ ch32_spi_dma.*            SPI DMA 传输
│   ├─ ch32l103_conf.h           外设配置开关
│   ├─ system_ch32l103.*         系统时钟与启动
│   ├─ {mp2980,ch211,ina226}_i2c_dma_port.*  各器件的 I2C DMA 端口
│   └─ st7735s_spi_dma_port.*    面板的 SPI/GPIO 端口
├─ drivers/                      器件驱动层：平台无关
│   ├─ mp2980.c/.h, ch211.c/.h, ina226.c/.h
│   └─ lcd_st7735s/              面板控制器与图形队列
│       └─ st7735s.c/.h, lcd.c/.h, lcdfont.h, lcd_pic.h
├─ pd/                           协议与控制层
│   └─ pd.c/.h, pd_internal.h, pd_phy, pd_cc, pd_cable,
│      pd_capability, pd_policy, pd_power
└─ service/                      功能服务层
    └─ display_service, ina226_service, fault_service, fan_service,
       led_service, board_link
```

分层与目录的对应关系：

| 层 | 目录 | 说明 |
| --- | --- | --- |
| 应用服务层 | `src/service` | 显示页面、测量、风扇、故障、板间同步、指示灯 |
| 协议与控制层 | `src/pd` | CC、PHY、线缆识别、能力、策略、电源时序 |
| 器件驱动层 | `src/drivers` | 平台无关驱动：三个器件 + 面板控制器与图形队列 |
| 板级适配层 | `src/platform`、`src/main.c` | MCU 外设、系统启动、板级配置与各器件 port |

更换 MCU 平台时，`src/platform` 是唯一需要重写的目录：寄存器级外设操作、中断向量、系统时钟，以及四个端口（MP2980/CH211/INA226 的 I2C DMA、面板的 SPI/GPIO）都在这一层。`src/drivers` 完全平台无关；`src/pd`、`src/service` 通过 `src/platform` 暴露的引脚宏、传输接口与板级配置访问硬件，自身不包含初始化代码。

新增目录时只需把目录加入 `.eide/eide.yml` 的 `incList`；`srcDirs` 会递归收集源文件。

## 1. 系统调度

`TIM1` 每 **1ms** 产生一次系统节拍（`BOARD_TICK_MS`），只在中断中累计 PD、显示、故障检测和板间通信的计时量。`main()` 持续推进各状态机，主要数据流如下：

```text
定时器累计时间
      ↓
PD 协议与电源状态机
      ↓
共享 I2C DMA 队列及设备驱动
      ↓
INA226 更新本地测量值
      ↓
故障检测 → 板间数据同步
      ↓
主板风扇控制与 LCD 刷新
```

I2C2 由 INA226、MP2980 和 CH211 共用。所有访问统一进入 DMA 队列；发生总线错误时，主循环会中止三个客户端的未完成操作、清除总线错误并重新进入各自的初始化或空闲状态，从而避免单个器件异常拖死系统。

## 2. PD 协议栈

PD 功能按职责拆分，公共接口集中在 `pd.h`，模块内部状态与时序定义集中在 `pd_internal.h`：

- `pd_cc.c`：CC 接入、方向检测和连接状态管理。
- `pd_phy.c`：USB PD 收发、GoodCRC、消息 ID、重试及中断配合。
- `pd_cable.c`：VCONN、SOP' 通信和 E-Marker 能力识别。
- `pd_capability.c`：PDO/APDO 编码、Source Capability 生成及 RDO 合法性校验。
- `pd_policy.c`：控制消息、扩展消息、Revision、PPS Status 和 EPR Mode 等策略响应。
- `pd_power.c`：MP2980/CH211 的非阻塞上电、调压、稳压确认、放电和关断时序。
- `pd.c`：协议上下文、顶层状态机、超时恢复及对外状态查询。

连接建立后，协议栈先以安全的 5V 输出发布能力，再根据接收端版本与线缆能力更新可用档位。有效 Request 经能力校验后进入电源切换状态机；只有目标电压通过 INA226 反馈确认后才完成供电握手。通信异常、软复位、硬复位、PPS/EPR 保活超时和端口故障均由同一顶层状态机收敛处理。

## 3. 主副板协作

主板与副板使用 `board_config.h` 中的编译角色选择，共用绝大部分业务代码：

| 功能 | 主板 `BOARD_MAIN` | 副板 `BOARD_SUB` |
| --- | --- | --- |
| 本地 Type-C PD 与功率控制 | 支持 | 支持 |
| 本地 Type-C/Type-A 功率采集 | 支持 | 支持 |
| 本地故障检测 | 支持 | 支持 |
| USART1 板间通信 | 接收 | 每 100ms 发送 |
| 四口 LCD 显示 | 支持 | 不编译 |
| 四口总功率风扇调速 | 支持 | 不编译 |

板间帧携带两路 INA226 原始数据、端口故障、Type-C 当前协议、聚合故障位和序号。主板接收时检查帧头、版本、长度与校验和，并保留通信错误和重同步计数，供调试时定位线路或数据异常。

---

# 编译与烧录

工程使用 **VS Code + Embedded IDE (EIDE)** 管理，按照 EIDE 提示完成开发环境配置后即可编译与烧录。

1. 使用 VS Code 打开 `ch32l103_pd3.2_140w.code-workspace`。
2. 安装工作区推荐的 EIDE 扩展，并完成WCH RISC-V开发环境配置（互联网上有很多参考，这里不再赘述）。
3. 在 `src/board_config.h` 中选择 `BOARD_MAIN` 或 `BOARD_SUB`，分别编译主板或副板固件。
4. 连接 WCH-Link，在 EIDE 中点击编译和烧录即可。

---

# 版本变更

## v2.0.0

**新增**

- PD 调试页：默认关闭，显示协议状态、失败原因与收发计数
- 板间通信显示远端板掉线与通信错误
- MP2980 / CH211 / INA226 驱动补齐寄存器读回接口

**修复**

- EPR 28 V / 140 W 长时间运行偶发掉压、对端硬复位：接收链路、保活应答、分块消息、重复帧
- I2C 总线错误恢复重复触发保护动作

**重构**

- 源码按 platform / drivers / pd / service 分层
- 显示栈拆分为控制器驱动、图形队列、显示服务，新增平台无关 SPI 端口
- 故障按供电拓扑分组，板间帧增加校验与重同步计数
- 统一 1 ms 系统时基与中断优先级编码
- 谓词与标志统一为 bool，命名统一为 CH32_ / CH32L103_ 两级前缀
- 清理重复防御与重复代码

## v1.0.0

首个开源版本，已归档。EPR 28 V / 140 W 长时间加热时可能出现掉压或对端硬复位，请使用 v2.0.0。