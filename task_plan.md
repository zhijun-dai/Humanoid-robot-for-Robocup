# RoboCup CV / OpenMV / 仿真 — 任务计划

> 导师要求摘要：**大倾角鲁棒**、**低延迟（0.5 s 量级优于 1 s）**、**提前约 20 cm 能稳检**；测试是为了找问题，不是声称没问题。  
> 规则 PDF（`10.2赛事规则...2025...pdf`）**当前不在仓库内**；请放入 `docs/rules/` 后把与 QR/视觉相关的条款补进 `findings.md`。

---

## 阶段 A — 目录与刷机（已完成）

- [x] 建立 `OpenMV_flash/`，仅含 `main.py`、`protocol_v2.py`、`line_follow_params.json` + `README.md`
- [x] 旧 `CVpart/openmv_flash_package/` 改为指向新目录（见该目录 `README.txt`）

## 阶段 B — Webots 与参数

- [ ] 在你本机确认 Webots 安装路径；用 `Webots/worlds/Robocup.wbt` 跑通 `line_follow_transfer`
- [ ] 新相机标定已进根目录 `line_follow_params.json`；复核 `scripts/check_camera_consistency.py` → PASS
- [ ] 按需微调仿真侧 ROI / PID / `shake.*`（以短跑对照为主，不必从零重跑全部历史 sweep）
- [ ] 将**确定上场**的键同步到 `OpenMV_flash/line_follow_params.json`（保持 `sim_opencv_distort: false`）

**阻塞说明**：当前自动化环境未检测到 `webots` 可执行文件；阶段 B 需你在本机执行或提供 CLI 路径。

## 阶段 C — 二维码：指标与方案

**物理/相机（当前配置口径）**

- `line_follow_params.json`：`height_cm=40`，`pitch_deg=45`；标定分辨率 320×240；`hfov`/`vfov` 与内参见 `camera.calibration`
- OpenMV：QVGA 流程与 `lens_corr` 已在 `qr_uart_test.py` 使用；真机畸变与仿真 OpenCV 后畸变是两条线，以**真场测试**为准

**工程指标（对应导师质疑）**

| 维度 | 目标 | 验证方法 |
|------|------|----------|
| 倾角 | 斜持仍能稳定解码 | 固定距离，扫 tilt 0°~45°（或规则上限），统计成功率与耗时 |
| 延迟 | 识别到发出指令 **≤0.5 s**（力争；可先量 1 s 再压） | 打日志时间戳或 GPIO；多帧均值 |
| 提前量 | 较「贴到最近」再早 **~20 cm** 仍可检 | 钢卷尺/地贴标记；录视频对帧 |

**技术选项（逐项小步试）**

1. **分辩率与 ROI**：全程 QVGA 或窗口 ROI 放大码区；权衡 fps 与模块像素数
2. **`lens_corr` 强度**：与距离/倾角联合扫参
3. **状态机**：接近标志位 → 提高 QR 帧预算或短暂降巡线频率
4. **备选**：若 QR 仍不稳，评估 AprilTag / 更大数据字库（规则允许前提下）

**风险**：无法在理论上「保证」任意远+任意倾角必解；以统计通过率 + 最坏工况录像为交付。

## 阶段 D — 规则对齐

- [ ] PDF 入场后：摘录 QR/尺寸/距离/禁止事项 → `findings.md`
- [ ] 调整场地标定与评分相关的测试用例

---

## 当前优先级建议

1. 本机跑通 Webots + 短测，根目录参数与 `OpenMV_flash` 真机 JSON 对齐  
2. 建 QR 测试台（距离 × 倾角 × 光照），记录表格  
3. 再决定是否合并 QR 进 `OpenMV_flash/main.py`（与巡线分时）
