# 视觉输出如何指导机器人走路

## 视觉输出

LineDetector.process(bgr) 每帧返回四个值：

- dev_px：横向偏差（像素）。正数=轨道中心在图像右边，机器人偏左了，需要右转纠偏。负数=偏右了，需要左转。
- heading_deg：轨道朝向（度）。0°=正前方，正数=轨道向右弯，负数=向左弯。
- conf：置信度 0~1。越高说明检测越靠谱。
- vis：可视化图像（调试用）。

只要 dev_px 和 heading_deg 是零，机器人就直走。不为零就需要纠偏。

---

## 从视觉到电机的完整步骤

### 1. 误差归一化

视觉输出的 dev_px 和 heading_deg 是像素和度数，不能直接给 PID。先归一化到 [-1, +1] 区间，让不同分辨率下的参数通用。

```
near_norm = dev_px / 80.0          （80 是半图宽度，偏差占半图的比例）
heading_norm = heading_deg / 45.0   （45° 是最大期望朝向角）
fused_err = -near_norm              （主项：偏右需要左转，所以取负号）
fused_err += 0.06 * (-heading_norm) （朝向前馈：弯道提前打方向，权重很小，防抖）
```

### 2. EMA 平滑

直接拿误差算会导致方向盘抖动（每帧噪声不一样）。用指数移动平均让误差变化平滑。

```
低噪声时（conf > 0.4）：alpha = 0.65（30%旧 + 70%新，灵敏）
高噪声时（conf ≤ 0.4）：alpha = 0.85（85%旧 + 15%新，迟钝，滤噪）
晃动时：alpha = 0.88（更迟钝）
smoothed_err = alpha * 上一次的smoothed_err + (1 - alpha) * fused_err
```

### 3. 晃动检测（在平滑之前）

跟踪最近 5 帧的原始 dev_px 值，算相邻帧之间变化的 RMS。如果 RMS 超过 6px，说明视觉输出在抖动——可能是机器人走路晃动了相机。

晃动状态下：EMA 更迟钝（alpha=0.88）、更依赖底部对称锁（权重 ×1.5）。

### 4. PID 计算

三个分量分别处理不同时间尺度的误差，然后加在一起。

#### P（比例）：当前位置偏差 × KP
当前偏离中心越多，方向盘打得越猛。KP=0.35~0.5。

#### I（积分）：累积偏差 × KI  
如果一直偏向同一侧（比如机器人的重心偏了），I 会慢慢积累力量把方向盘掰回来。KI=0.004~0.012。

积分有上限（clamp），防止累积过头导致冲出去。低置信时积分缓慢衰减（×0.85），防止丢线期间积累错误信号。

#### D（微分）：偏差变化速度 × KD
偏差在快速收窄时，D 会提前减小方向盘角度，防止冲过头。KD=0.08~0.10。

```
P = KP * smoothed_err
I = clamp(I + KI * smoothed_err * dt, -I_CLAMP, +I_CLAMP)
D = KD * (smoothed_err - 上一次的smoothed_err) / dt
pid_out = P + I + D
```

### 5. 非线性映射

PID 直接输出的话，小偏差反应迟钝、大偏差转弯太猛。用 tanh 函数把它映射成平滑的 S 形曲线。

```
steer = STEER_SAT * tanh(pid_out / STEER_SCALE)
```

STEER_SAT=45 是方向盘最大值。STEER_SCALE=1.0 控制"膝点"位置——值越小，小偏差越灵敏。

每帧方向盘变化不能超过 14 个单位（防止突变）。

### 6. 差速驱动

左右轮不同速度实现转弯。向右转 = 左轮快、右轮慢。

```
delta = clamp(steer * STEER_W, -2.8, 2.8)
left_speed  = base_speed - delta
right_speed = base_speed + delta
```

STEER_W=0.04 把 steer 量纲换算成电机速度差。delta 被限制在 [-2.8, 2.8] 防止一边轮子倒转。

### 7. 速度控制

根据置信度调整前进速度——看不清楚就慢点。

```
conf > 0.3：全速（base_speed = 3.2）
conf 0.1~0.3：75% 速度
conf < 0.1：60% 速度
```

### 8. 丢线处理

当 conf < 0.08（完全找不到线）时进入丢线模式。

```
丢线前 6 帧：保持上一次的方向，但打 85% 的力度（可能是短暂遮挡）
6 帧之后：交替左右打满方向盘进行搜索（±26 个单位），希望能重新找到线
```

---

## 9. 步态控制（双足步行）

前面的步骤 1~8 输出的是连续的 steer（转向力）和 speed（前进速度），仿真里直接驱动轮子。但真实机器人是迈步的——每迈一步，需要决定这一步迈多远（前进速度）和迈多偏（转向角度）。

从连续信号到离散步态有两种办法：

#### 办法一：离散档位（老方案 protocol V2）

把 steer 量化为几个固定档位，每步只发一个档位号。

```
steer > +deadband → route = 3 (右转)
steer < -18       → route = 2 (左转)
steer 在中间      → route = 4 (微左)
abs(steer) ≤ deadband → route = 1 (直行)
```

简单可靠，但弯道精度不够——只有 3~4 个档位，转弯半径跳变。

#### 办法二：连续值（推荐）

每步直接告诉步态控制器两个连续值——这一步迈多大、迈多偏。视觉系统每 100ms 刷新一次（10Hz），刚好匹配步频。

**步长（前进速度）**：
```
step_length = speed * 0.01    （speed=3.2 → 步长 3.2cm）
```

弯道时适当缩短步长（慢了不容易摔）：
```
if abs(steer) > 20:
    step_length *= 0.7        （大转弯时步子缩小到 70%）
```

**步角（转向角度）**：
```
step_yaw = steer * 0.03       （steer=10 → 这一步向右偏 0.3°，微调）
                                （steer=40 → 这一步向右偏 1.2°，明显转弯）
```

系数 0.03 需要实机调试——太大画龙，太小转不过弯。典型范围 0.02~0.06。

**发送格式**（参考 protocol V2 的 line_ctrl 消息字段）：
- ex_mm_i16: 当前横向偏差转成毫米（dev_px × cm_per_px × 10）
- ang_cdeg_i16: 朝向角转成百分度（heading_deg × 100）
- v_mm_s: 建议前进速度 mm/s
- yaw_cdeg_per_step: 每步偏航角百分度

实际上 step_yaw 和 step_length 就够了——底盘单片机拿到后，对应算出这一步步态参数（落脚点、重心偏移），执行迈步。

---

## 参考控制器

| 文件 | 说明 |
|------|------|
| `Webots/controllers/jetson_bridge/jetson_bridge.py` | 最简洁模板——PID + 晃动检测 + 丢线搜索。适合 V3 |
| `Webots/controllers/jetson_bridge_v2/jetson_bridge_v2.py` | V2 专用版，参数微调 |
| `Webots/controllers/line_follow_transfer/line_follow_transfer.py` | 最完整——双模式 PID（直/弯道切换 KP/KI/KD），非线性转向 + 非对称修正，路重量化输出 |
