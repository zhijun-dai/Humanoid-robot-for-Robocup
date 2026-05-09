# 进度日志

## 2026-05-08

- 新增 `OpenMV_flash/` 作为**唯一刷机工件目录**（`main.py` / `protocol_v2.py` / `line_follow_params.json`）。
- 初装：`main.py` ← `CVpart/main/main_webots_aligned.py`；`protocol_v2.py` ← `CVpart/main/protocol_v2.py`；`line_follow_params.json` ← 原 `openmv_flash_package` 真机版。
- 建立 `task_plan.md`（含 QR 指标与阶段 B–D）；`findings.md` 待用。
- **Webots**：自动化环境未找到 `webots.exe`，阶段 B 待本机执行。

## 待办（下一会话优先）

- [ ] 用户提供 Webots 路径或自行跑仿真后，把需合并的参数变更记入本文件
- [ ] 将赛事 PDF 放入仓库并更新 `findings.md`
