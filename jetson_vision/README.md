# Jetson Nano 视觉方案

## 与 OpenMV 对比

| 维度 | OpenMV H7+ | Jetson Nano + USB |
|------|-----------|-------------------|
| 分辨率 | QQVGA 160x120 | 640x480 起步 |
| QR 解码 | `find_qrcodes()` 慢且弱 | `cv2.QRCodeDetector` 或 `pyzbar` |
| 畸变校正 | 简化 `lens_corr(α)` 单参数 | 完整 Brown 模型 k1~k5 |
| 巡线输出 | 4 种离散 (go/left/right/slight) | 连续转向角 (0.1°精度) |
| 算力 | 单核 M7 480MHz | 四核 A57 + GPU (CUDA) |
| 开发 | MicroPython, 受限 | 完整 Python3 + OpenCV + numpy |
| 通信 | UART1 (P1/P0) | UART (ttyTHS1 或 USB转串口) |

## 目录

```
jetson_vision/
  vision_main.py     # 主循环
  qr_detector.py     # QR 检测模块
  line_detector.py   # 巡线模块
  protocol_v2.py     # 协议 V2 (与 STM32 通信)
  camera_config.py   # 相机配置/标定参数
  config.py          # 统一参数加载
```

## 通信

与 OpenMV 相同：UART 115200 8N1，Protocol V2 帧格式。
Jetson Nano 的 UART:
- `/dev/ttyTHS1` — 硬件 UART1 (引脚 8/10 on J41 header)
- `/dev/ttyUSB0` — USB 转串口适配器
