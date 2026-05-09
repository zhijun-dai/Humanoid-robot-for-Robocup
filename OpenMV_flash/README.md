# OpenMV 刷机目录（唯一入口）

本目录**只放需要拷到 OpenMV U 盘根目录的文件**。日常改参数、改 `main.py` 请以这里为准，避免在 `CVpart/main/` 里和一堆测试脚本混在一起。

## 拷贝到相机

把下面 **3 个文件** 复制到 U 盘根目录（与 `boot.py` 同级），覆盖同名文件：

| 文件 | 说明 |
|------|------|
| `main.py` | 上电自动运行；内容由 `main_webots_aligned` 巡线流水线对齐 Webots |
| `protocol_v2.py` | 与主控通讯 V2 |
| `line_follow_params.json` | **真机用**（`sim_opencv_distort: false` 等）；与仓库根目录 `line_follow_params.json`（仿真）对照同步时，只合并你需要上场的字段，勿把仿真专用开关原样刷下去 |

接线：`UART1`，**P1=TX → 主控 RX**，**P0=RX ← 主控 TX**，GND 共地；**115200 8N1**。

## 与仓库其它部分的关系

- **巡线算法源码母本**（便于 diff）：`CVpart/main/main_webots_aligned.py`  
  若你更新了母本，可把内容覆盖到本目录 `main.py`，或只在本目录改、再按需回拷到母本（团队约定一种即可）。
- **协议母本**：`CVpart/main/protocol_v2.py` — 更新后覆盖本目录 `protocol_v2.py`。
- **仿真用参数**：仓库根目录 `line_follow_params.json`（含 `sim_opencv_distort: true` 等）。  
  更新相机标定、ROI、PID 等后，建议：**先改根目录并通过 Webots**，再把需上场的键同步到本目录 `line_follow_params.json`。

## 二维码测试

本目录 **默认 `main.py` 不包含二维码**。仅测 QR+串口请用 `CVpart/main/qr_uart_test.py`（或自行把 QR 逻辑并入本目录 `main.py`）。路线图见仓库根目录 `task_plan.md`。
