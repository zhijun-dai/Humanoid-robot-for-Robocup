# 研究笔记（工程约束）：**不是**项目进度的唯一真相源

仓库里 **`task_plan.md`、`progress.md`、`本文件`** 以及散落各处的说明，都**可能被某次会话写过但没续更**。  
判断「现在做到哪了」请优先看：

- **`git log`**（最近改了什么）
- **`OpenMV_flash/`**（当前约定要烧的三件东西）
- **根目录 `line_follow_params.json`**（仿真 / Webots 主参数）
- **`Webots/controllers/line_follow_transfer/`**、`**CVpart/main/main_webots_aligned.py**`（算法与控制器对齐关系）

---

## 赛事规则 PDF（官方条文）

- 你引用的：`10.2赛事规则.工程竞技类机器人-人形全能竞技项目.2025 中国机器人大赛暨RoboCup机器人世界杯中国赛.pdf`
- **在本工作区/仓库路径下仍未检索到该文件**（仅存在 `motor_protocol_v2_p1.pdf` 等其它 PDF）。若规则在你本机其它目录，请复制到 **`docs/rules/`**（见同目录 `PLACE_PDF_HERE.txt`）并提交，再在下面**用「条款编号 + 页」摘录**，避免口口相传误差。

### 待 PDF 入库后补全（占位）

- [ ] 是否与**二维码 / AprilTag / 其它标记**有关：**原文摘录**
- [ ] 与**场地尺寸、标记尺寸、识别距离**有关：**原文摘录**
- [ ] 与**违规、得分**有关（若影响视觉方案）：**原文摘录**

> 在 PDF 未入库前，**不在此臆造规则正文**。下方「二维码」仅为**工程现状与待办**。

---

## 二维码 — 工程侧现状（非规则摘录）

| 项 | 状态 |
|----|------|
| **`OpenMV_flash/main.py`（上场默认）** | **无** `find_qrcodes`；只有巡线 + 协议 V2 等 |
| **专项测试** | `CVpart/main/qr_uart_test.py`：白名单 `"1"~"6"`，UART 发 **uint8** `0x01~0x06`，**5 s** 内至多发了 1 次，`lens_corr` + QVGA |
| **历史参考** | `main_test0.py`、`main0.py` 含 QR + 旧发送逻辑；**未**合并进 `OpenMV_flash` |
| **与赛场对齐** | 若规则要求场上扫码：需把 QR **并入刷机 `main.py` 状态机**或改由**主控/其它传感器**承担；并做 **距离 × 倾角 × 端到端延迟** 实测 |

### 导师/自测关注点（工程指标，非规则）

- 大倾角、**~0.5 s 级**解码、较「最近」再 **~20 cm** 仍能检出 —— 需建测试表与失败样本，见 `task_plan.md` 阶段 C（**计划文件本身也可能过时**）。

---

## OpenMV 与 Webots

- 刷机 JSON：`OpenMV_flash/line_follow_params.json`（真机；`sim_opencv_distort: false` 等）
- 仿真 JSON：仓库根 `line_follow_params.json`（可开 `sim_opencv_distort` 等）
- Webots：**`D:\Webots\msys64\mingw64\bin\webots.exe`**（R2025a）；`scripts/run_shake_eval.py`、`auto_tune_webots_params.py` 可驱动批跑

## 相机标定（新头）

- 照片集：`calib_photos_manual_now/`；结果：`generated/camera_calibration_result_calib_photos_manual_now.json`（若 `generated/` 被 ignore，以提交记录与根 `line_follow_params` 为准）
- 已写入根 `line_follow_params`、`presets`、`OpenMV_flash/line_follow_params`（数值以当前 JSON 文件为准）
