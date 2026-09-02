# gpu_vision — Jetson GPU 加速版视觉

目标硬件：**Jetson Orin Nano Super 8GB**（Ampere 1024 CUDA 核，JetPack 6.x）。
视觉预处理（二值化/形态学/透视变换）走 `cv2.cuda`，图卡分类走 ShapeCNN
（PyTorch，torch.cuda 可用时自动上 GPU）。几何算法（Hough/LSD/轮廓/连通域）
留 CPU——CUDA 无对应实现或收益小。

**cv2.cuda 不可用时自动 CPU 回退**（backend.py 的 HAS_CUDA 探测），回退实现
与 `jetson/` CPU 版逐算子等价——桌面无 GPU 跑本目录 = 验证正确逻辑。

## 文件

| 文件 | 说明 |
|---|---|
| `backend.py` | cv2.cuda 算子后端：找框二值链 / 巡线二值链 / warp / 核缓存 |
| `shape_detector_gpu.py` | 图卡找框（继承 CPU 版，预处理链 GPU 化）|
| `line_detector_gpu.py` | 巡线（process 预处理链 GPU 化）|
| `cnn 分类` | 复用 `jetson/shape_cnn.py`（共享定义），权重 `jetson/shape_cnn_best_v2.pt` |
| `vision_camera.py` | 相机源：Windows DSHOW / Linux V4L2 / 视频文件 |
| `run_robot.py` | 机器人控制器（协议 V2 LINE_CTRL + 一步前瞻）|
| `probe.py` | 板子到手第一步：CUDA/OpenCV 能力 + 算子计时 |
| `compare_cpu_gpu.py` | CPU 版 vs GPU 版同帧一致性 |
| `requirements-gpu.txt` | Jetson 依赖与装法（torch wheelhouse 说明）|

## Jetson 安装

```bash
# torch（按 JetPack 版本改 v61/v612…，见 requirements-gpu.txt）
pip install torch --index-url https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch
# OpenCV：探测系统自带是否带 CUDA
python probe.py
#   不带 → sudo apt install python3-opencv  或 自编译（CUDA_ARCH_BIN=8.7）
```

## 运行

```bash
python run_robot.py               # 相机 + 调试窗口
python run_robot.py --headless    # 板子无显示器（实车标准）
python run_robot.py --video x.mp4 # 视频回放调试
python run_robot.py --width-640   # CPU 紧张时降分辨率
python run_robot.py --no-serial   # 不开串口（纯视觉调试）
```

参数 env 覆盖：`STEP_LEN_CM`（机器人步长）、`PREVIEW_GAIN`（一步前瞻增益）、
`SHAPE_CNN_ENABLE/WEIGHT`、`SERIAL_PORT` 等，见 run_robot.py 头注释。

## 板子验收步骤

1. `python probe.py` → CUDA 设备、`cv2.cuda` 可用性、每算子计时表
2. `python compare_cpu_gpu.py` → GPU vs CPU 同帧对比
   （adaptiveThreshold 允许 ≤0.1% 像素差；形态学/阈值应完全一致）
3. `python run_robot.py --headless --no-serial` → 全链帧率（960×540 预处理 <5ms）
4. 接串口实车小跑

## 回退

任意环节（OpenCV 无 CUDA / 权重缺失 / torch 未装）→ 程序不崩：
backend CPU 回退 + 规则分类兜底（jetson/shape_detector 自带）。
