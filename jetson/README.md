# Jetson Nano + USB 摄像头 视觉方案

## 运行

### 方法1：双击（最简单）
双击 `jetson/run_qr_test.bat`

### 方法2：终端
```powershell
.venv\Scripts\python jetson\usb_cam_qr_test.py
```
输出在同一个终端窗口里。开着 OpenCV 窗口，QR 码放摄像头前，终端会打印 SEND，按 ESC 退出。

## 目录

```
jetson/
  usb_cam_qr_test.py   # USB 摄像头 QR 测试（Windows 验证用）
  run_qr_test.bat       # Windows 双击运行
  vision_main.py        # 完整主循环（QR + 巡线 + 串口，部署到 Jetson）
  qr_detector.py        # QR 检测模块（OpenCV QRCodeDetector）
  line_detector.py      # 巡线模块（鸟瞰变换法）
  protocol_v2.py        # 协议 V2（部署时从 openmv/ 复制）
```

## 与 OpenMV 代码的区分

| | OpenMV (`openmv/`) | Jetson (`jetson/`) |
|---|---|---|
| 相机 | OpenMV 板载 sensor | USB 摄像头 |
| 分辨率 | QQVGA 160×120 | 640×480 |
| 语言 | MicroPython | Python 3 + OpenCV |
| 算法 | 逐行扫描 + cm投影 | 鸟瞰变换 |
| QR | `find_qrcodes()` | `cv2.QRCodeDetector` |
| 部署 | 拷 3 个文件到 U 盘 | `python vision_main.py` |
