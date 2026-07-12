# 实车巡线控制 — 配置说明

## 需要给队友的文件

```
jetson_vision/
  run_real_car.py           # 主程序：摄像头→V1检测→PID→串口输出
  line_detector_v1_warp.py  # V1 检测器（IPM鸟瞰+黑帽+自适应阈值+两带检测）
  run_real_car.bat           # Windows 双击启动
  vision_main.py             # 调试用 demo（4窗口可视化，可选）
```

## 摄像头参数

| 参数 | 值 |
|------|-----|
| 分辨率 | 1280×720 |
| 视场角 | 100° 对角（约 49° 垂直 / 87° 水平，16:9） |
| 镜头类型 | 200万像素，无畸变 |
| 安装高度 | 约 40 cm（`CAM_HEIGHT_CM`） |
| 安装俯角 | 约 45°（`CAM_PITCH_DEG`） |
| 摄像头索引 | 0=内置，1=USB（`CAM_IDX`） |

## 运行环境

- Python 3.x + 虚拟环境 `.venv`
- pip install: `opencv-python`, `numpy`, `pyserial`

## 运行方式

**方式一：命令行**
```
.venv\Scripts\python.exe jetson_vision\run_real_car.py
```

**方式二：双击 `run_real_car.bat`**

## 可调参数（环境变量）

运行前设环境变量覆盖默认值：

| 环境变量 | 默认 | 说明 |
|----------|------|------|
| `CAM_IDX` | 0 | 摄像头索引 |
| `CAM_HEIGHT_CM` | 40.0 | 相机安装高度 |
| `CAM_PITCH_DEG` | 45.0 | 相机俯角 |
| `CAM_VFOV_DEG` | 49.0 | 相机垂直视场角 |
| `REAL_CAR_SPEED` | 35.0 | 基准速度 cm/s |
| `JETSON_STEER_SPEED_EXP` | 1.0 | 速度-转向缩放指数 |
| `SERIAL_PORT` | COM10 | 串口 |
| `SERIAL_BAUD` | 115200 | 波特率 |

PID 参数（直道 / 弯道）也可设，详见 `run_real_car.py` 第 42-48 行。

## 串口协议

波特率 115200，每帧 6 字节（相机帧率 ~20-30Hz）：

```
0xFF  FR  FL  RR  RL  0xEE
```

- FR/FL/RR/RL：四轮转速 (rad/s × 10)，int8_t 格式
- 正值 = 前进
- 例：`FF 64 64 64 64 EE` = 四轮均 10.0 rad/s 前进

## 控制器逻辑

1. 摄像头 → V1 检测器（IPM 鸟瞰 → 黑帽 → 自适应阈值 → 两带扫描 → 底部锁）
2. 检测器输出 `fused_err`（融合偏差）→ 双模式 PID（直道/弯道）
3. PID 输出 steer → 差速驱动 → 四轮转速 → 串口发给 MCU
4. 速度自适应：高速时增大转向、减缓积分、放宽限幅

## 4 个调试窗口

| 窗口 | 内容 |
|------|------|
| 1.Original | 原始画面 + 控制叠加 |
| 2.Warp (birdseye) | 鸟瞰灰度图 |
| 3.Adaptive (binary) | 自适应二值化 |
| 4.Close+Fit | 鸟瞰 + 检测标注（黄箭头=方向，绿点=偏差） |

按 `Q` 退出，按 `S` 切换串口。
