# 进度日志

## 2026-05-08

- 新增 `OpenMV_flash/` 作为**唯一刷机工件目录**（`main.py` / `protocol_v2.py` / `line_follow_params.json`）。
- 初装：`main.py` ← `CVpart/main/main_webots_aligned.py`；`protocol_v2.py` ← `CVpart/main/protocol_v2.py`；`line_follow_params.json` ← 原 `openmv_flash_package` 真机版。
- 建立 `task_plan.md`（含 QR 指标与阶段 B–D）；`findings.md` 待用。
- **Webots**：自动化环境未找到 `webots.exe`，阶段 B 待本机执行。

## 2026-05-09

- 本机 Webots：**`D:\Webots\msys64\mingw64\bin\webots.exe`**（R2025a）。
- 已跑 **`run_shake_eval.py --run-seconds 12`** 冒烟三组全过；报告 `generated/shake_eval/smoke12_report_20260509_114613.json`。

## 待办（下一会话优先）

- [ ] 将赛事 PDF 放入仓库并更新 `findings.md`
- [ ] 若需大范围重调：跑 `auto_tune_webots_params.py` 或将 `run_shake_eval` 默认秒数拉长做统计
