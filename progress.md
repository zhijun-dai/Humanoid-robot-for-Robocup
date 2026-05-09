# 进度日志（**辅助**，可能与现状不完全同步）

更可靠：**`git log`** + **`OpenMV_flash/`** + **`line_follow_params.json`**。

## 近期提交里能看出的轨迹（约 2026-05）

- `OpenMV_flash/` 成为唯一刷机工件目录；`CVpart/openmv_flash_package/` 仅保留跳转说明
- 新相机标定 `calib_photos_manual_now` → 根 / 预设 / 刷机 `line_follow_params` 已更新
- `qr_uart_test.py`：UART uint8 1–6、5 s 节流等迭代多轮
- Webots：**`D:\Webots\...`** R2025a；`run_shake_eval.py --run-seconds 12` 冒烟曾通过（见当时 `generated/shake_eval/`）

## 2026-05-10

- **主程序二维码**：`main_webots_aligned.py` / `OpenMV_flash/main.py` 已并入 **1～6** 识别；`line_follow_params.openmv_webots_aligned` 增加 `qr_*` 键；协议 **`MSG_QR_EVENT`** + `PendingAcks`；下架仓库内重复长文件名规则 PDF（保留 `rule.pdf`）。

- **目录清理**：删除 `CVpart/main/` 下早期测试 `main0.py`、`main_test0.py`、`main1test.py`、`autotune_main1test.py`、`autotune_best.json`；更新 `CVpart/README.md`、`findings.md`、`docs/line_follow_params_参数说明.md`。

## 未完成 / 需在代码与场地验证

- [ ] 规则 PDF **纳入 `docs/rules/`** 并在 `findings.md` **摘录正式条款**（含二维码若存在）
- [ ] QR：**是否**并入 `OpenMV_flash/main.py`（规则与赛程决定）
- [ ] 仿真：换畸变后可按需再跑长评测 / `auto_tune`，非必须从零重做
