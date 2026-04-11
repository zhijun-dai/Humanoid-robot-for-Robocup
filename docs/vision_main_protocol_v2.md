# Vision <-> MainBoard 通讯协议 V2（建议稿）

## 1. 目标

本协议用于人形机器人比赛场景下的视觉与主控通讯，满足：
- 巡线低延迟、可丢包容忍。
- 二维码/障碍等事件可靠到达。
- 异常可检测（校验、超时、版本不一致）。
- 可从当前老协议平滑迁移。

## 2. 现状与问题

当前工程同时存在两种输出：
- 老协议：单字符方向码（"1"/"2"/"3"/"4"）。
- 简帧协议：3 字节 [0xAA, CMD, CHECKSUM]。

问题：
- 无序号与时间戳，无法判定“新旧包”。
- 无可靠事件重发机制。
- 校验过弱，抗干扰能力有限。
- 消息语义不足（只有方向码，不含置信度/失线状态）。

## 3. 物理层建议

- 物理接口：UART（保持 UART1）。
- 参数：115200, 8N1（若主控算力足，可升到 230400）。
- 字节序：Little Endian。

## 4. 帧格式（统一）

固定头 + 长度 + CRC16：

| 字段 | 长度(字节) | 说明 |
|---|---:|---|
| SOF1 | 1 | 0x55 |
| SOF2 | 1 | 0xAA |
| VER | 1 | 协议版本，当前 0x02 |
| MSG_TYPE | 1 | 消息类型 |
| FLAGS | 1 | 位标志（ACK_REQ/IS_ACK 等） |
| SEQ | 1 | 递增序号 0~255 |
| TS_MS | 4 | 发送端毫秒时间戳 |
| LEN | 1 | Payload 长度 |
| PAYLOAD | LEN | 负载 |
| CRC16 | 2 | CRC-16/CCITT-FALSE（从 VER 到 PAYLOAD） |

CRC 参数：
- poly=0x1021
- init=0xFFFF
- xorout=0x0000

## 5. 消息类型定义

### 5.1 高频流（允许丢包，不重发）

| MSG_TYPE | 方向 | 名称 | 周期 |
|---:|---|---|---|
| 0x01 | Vision->Main | HEARTBEAT | 10 Hz |
| 0x02 | Vision->Main | LINE_CTRL | 20~30 Hz |
| 0x81 | Main->Vision | ROBOT_STATE | 10 Hz |

LINE_CTRL payload（12 字节建议）：
- mode_u8: 0=idle,1=line_follow,2=lost_search
- conf_u8: 0~100
- lost_u8: 0/1
- route_u8: 1=go,2=left,3=right,4=slight_left
- ex_mm_i16: 横向偏差（毫米）
- ang_cdeg_i16: 航向误差（0.01 deg）
- v_cmd_mmps_i16: 线速度指令
- w_cmd_mradps_i16: 角速度指令

说明：
- 即使 route 仍保留，主控应优先使用 v_cmd/w_cmd。
- route 作为兼容字段与应急兜底。

### 5.2 事件流（必须可靠）

| MSG_TYPE | 方向 | 名称 | 可靠性 |
|---:|---|---|---|
| 0x10 | Vision->Main | QR_EVENT | ACK + 可重发 |
| 0x11 | Vision->Main | OBSTACLE_EVENT | ACK + 可重发 |
| 0x12 | Vision->Main | MODE_SWITCH_REQ | ACK + 可重发 |
| 0x80 | Main->Vision | ACK/NACK | - |
| 0xE0 | 双向 | ESTOP | ACK + 最高优先级 |

QR_EVENT payload（示例）：
- event_id_u16
- qr_id_u8 (1~6)
- stable_frames_u8
- conf_u8 (0~100)
- reserve_u8

ACK/NACK payload：
- ack_seq_u8
- ack_type_u8
- code_u8 (0=OK, 非0=错误)
- reserve_u8

## 6. 状态机与超时

主控侧建议：
- 连续 300 ms 未收到有效 HEARTBEAT 或 LINE_CTRL -> 进入 SAFE_STOP。
- 连续 800 ms 未恢复 -> 进入 LOST_RECOVERY（小步停转）。
- 收到 ESTOP -> 立即停机，需人工或明确 CLEAR 命令恢复。

视觉侧建议：
- 事件包发送后等待 ACK，超时 80 ms 重发。
- 重发上限 3 次；失败后上报 ERROR 事件并降级。

## 7. 优先级规则

消息优先级（高到低）：
1. ESTOP
2. OBSTACLE_EVENT
3. QR_EVENT
4. MODE_SWITCH_REQ
5. LINE_CTRL / HEARTBEAT

冲突时按高优先级覆盖低优先级。

## 8. 与当前工程兼容迁移

阶段 1（本周可做）：
- 保留老字符路由输出。
- 同时发送 V2 帧（双发）。
- 主控先“只读不控”V2，记录解析结果。

阶段 2：
- 主控切换为以 LINE_CTRL 为主。
- route 字段仅作 fallback。

阶段 3：
- 下线老字符协议，仅保留 V2。

## 9. 旧协议映射

| 旧字符 | 含义 | V2 route_u8 |
|---|---|---:|
| "1" | go | 1 |
| "2" | left | 2 |
| "3" | right | 3 |
| "4" | slight_left | 4 |

## 10. 最小联调清单

- 双方统一 CRC16 实现与测试向量。
- 主控打印解析后的 seq/ts/type/conf/lost。
- 验证 300 ms 超时停机行为。
- 验证 QR_EVENT 的 ACK 重发。
- 验证拔插串口与噪声下不会误触发动作。

## 11. 建议的保留参数

建议在配置中保留：
- output.uart.id
- output.uart.baud
- output.send_interval_ms
- output.route.*（兼容期保留）

新增建议：
- output.protocol.version = 2
- output.protocol.heartbeat_hz = 10
- output.protocol.ctrl_hz = 25
- output.protocol.ack_timeout_ms = 80
- output.protocol.retry_max = 3
- output.protocol.safety_timeout_ms = 300

## 12. 文档应写到多详细（工程分级）

### 12.1 L1（过粗，不建议）

只写“UART+命令字”。
问题：
- 无超时与重发规则，联调时互相甩锅。
- 无字段单位，主控和视觉容易解读不一致。
- 无版本策略，后续改字段必崩。

### 12.2 L2（工程常用，推荐）

写到“可直接联调”的程度：
- 物理层参数（串口号、波特率、校验）。
- 帧格式（头、长、序号、时间戳、CRC）。
- 消息字典（每字段类型/单位/范围）。
- 时序与超时（心跳、ACK、重发、降级）。
- 状态机（主控/视觉在异常下怎么退化）。
- 兼容迁移（新旧协议并行期方案）。

### 12.3 L3（偏重，比赛一般不必）

写到“形式化验证/完备证明”级别。
不建议当前阶段投入，性价比低。

## 13. 本比赛建议详细度

建议采用 L2 中等偏上，原因：
- 你们是人形双足，动作代价高，通信异常必须可控。
- 视觉任务有高频巡线和低频关键事件两种流，可靠性要求不同。
- 比赛环境干扰多，必须有超时停机与ACK重发机制。

具体做到：
1. 高频控制流可丢包（LINE_CTRL）。
2. 事件流必须可靠（QR/障碍，ACK+重发）。
3. 主控超时保护明确（300 ms 未收到有效控制即安全策略）。
4. 保留 route 兼容字段，但逐步转向 v/w 连续控制。

## 14. 对你们最小可交付（本周版）

1. 文档层：本文件 + 一页消息对照表（MSG_TYPE 与 payload 字段）。
2. 视觉层：双发（老字符 + V2帧）。
3. 主控层：先只解析V2打印，不接管控制；确认稳定后切换。
4. 安全层：实现通信超时停机。
5. 联调层：完成 5 个异常测试：断线、误码、重复包、乱序包、ACK丢失。

## 15. 什么叫“够详细”

满足以下条件即为够详细：
- 新同学不看口头说明，只看文档就能写出解析器。
- 现场串口异常时，双方都能依据文档定位责任边界。
- 协议升级时，旧版本机器人不会立刻失效。

## 16. 比赛必需输出清单（最小集）

按当前赛题（沿赛道走中间、识别二维码、识别红棍），视觉最小必须输出如下。

| 任务 | 消息 | 必需字段 | 频率/触发 | 可靠性要求 |
|---|---|---|---|---|
| 巡线居中 | LINE_CTRL | mode, route, conf, lost, ex, ang | 20~30 Hz | 可丢少量包，不重发 |
| 二维码动作触发 | QR_EVENT | event_id, qr_id(1~6), conf, stable_frames, ts | 稳定2~3帧后触发 | 必须ACK+重发 |
| 红棍检测/通过 | OBSTACLE_EVENT | event_id, state(detected/passed), conf, ts | 状态变化触发 | 必须ACK+重发 |
| 在线与安全 | HEARTBEAT | mode, seq, ts | 10 Hz | 主控超时保护 |

补充要求：
- 主控 300 ms 未收到有效 LINE_CTRL/HEARTBEAT，进入安全策略（减速/停机）。
- 事件消息 ACK 超时 80~120 ms 重发，重发上限 3 次。

## 17. 左右转精细度（重点）

左右转可做成 3 个工程等级：

### 17.1 等级A：离散方向码（当前兼容能力）

- 输出：route_u8（go/left/right/slight_left）。
- 优点：实现最简单，主控改动小。
- 缺点：弯中控制颗粒粗，容易内切或外摆。

### 17.2 等级B：离散增强（推荐过渡）

- 输出：route_u8 + turn_level_u8（例如 0~7）。
- 优点：比纯方向码细腻，联调成本可控。
- 适用：主控暂时不能接收连续量时。

### 17.3 等级C：连续控制（比赛建议目标）

- 输出：v_cmd + w_cmd（并保留 route 兜底）。
- 优点：直道和弯道都更平滑，内切可显著降低。
- 适用：主控可接受连续速度控制输入。

### 17.4 对当前项目可达到的精细度（实战口径）

在你们当前 OpenMV + 赛道条件下，合理目标：
- 直道：中心偏差稳定在约 2~5 px 波动。
- 常规弯道：偏差约 6~12 px，峰值主要出现在入弯/弯心。
- 控制链路端到端延迟：建议压到 120 ms 以内（含视觉计算+串口+主控执行）。

说明：
- 若只用 route 方向码，通常只能到等级A/B上限。
- 若输出连续 w_cmd，并保留 route 兜底，可达到等级C，弯道体验明显更好。

### 17.5 建议验收标准（围绕左右转）

建议把“看起来好不好”落为以下可执行标准：
1. 第一左弯外摆不超过跑道宽度的 1/3 且持续不超过 0.2 s。
2. 连续 3 圈内，无“单侧边线长时间贴边（>0.5 s）”现象。
3. 弯道中 route 或 w_cmd 不出现高频翻转（避免抖舵）。
