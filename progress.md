# 进度日志（**辅助**，可能与现状不完全同步）

更可靠：**`git log`** + **`OpenMV_flash/`** + **`line_follow_params.json`**。

## 近期提交里能看出的轨迹（约 2026-05）

- `OpenMV_flash/` 成为唯一刷机工件目录；`CVpart/openmv_flash_package/` 仅保留跳转说明
- 新相机标定 `calib_photos_manual_now` → 根 / `config/presets` / 刷机 `line_follow_params` 已更新
- `qr_uart_test.py`：UART uint8 1–6、5 s 节流等迭代多轮
- Webots：**`D:\Webots\...`** R2025a；`run_shake_eval.py --run-seconds 12` 冒烟曾通过（见当时 `generated/shake_eval/`）

## 2026-05-10

- **主程序二维码**：`main_webots_aligned.py` / `OpenMV_flash/main.py` 已并入 **1～6** 识别；`line_follow_params.openmv_webots_aligned` 增加 `qr_*` 键；协议 **`MSG_QR_EVENT`** + `PendingAcks`；下架仓库内重复长文件名规则 PDF（保留 `rule.pdf`）。

- **目录清理**：删除 `CVpart/main/` 下早期测试 `main0.py`、`main_test0.py`、`main1test.py`、`autotune_main1test.py`、`autotune_best.json`；更新 `CVpart/README.md`、`findings.md`、`docs/line_follow_params_参数说明.md`。

## 2026-05-11 — QR 鲁棒性强化

- **QR 算法**：ROI 裁剪到下半区 55%、3x 放大（原 2x）、histeq 开、尺寸过滤（min 12px / max 300px / min area 60px²）、lens_corr + 无校正双管道重试、调试日志输出码尺寸/位置/校正标记
- **配置**：根 `line_follow_params.json` 与 `OpenMV_flash/line_follow_params.json` 同步新 QR 参数
- **仓库整理**：删除过时的 `line_follow.py`（763 行孤本）、空 `docs/rules/`；规划文件迁到根目录；创建 `CLAUDE.md`
- **同步**：`CVpart/main/main_webots_aligned.py` ↔ `OpenMV_flash/main.py` 保持对齐
- **QR 测试工具**：新建 `CVpart/main/qr_test_hardened.py`，独立测试脚本，管线与主程序完全一致，逐帧终端输出（检测/候选/发送/过滤符号），每秒统计摘要（命中率/发送数/拒绝数/平均延迟）

## 未完成 / 需在代码与场地验证

- [ ] 真场 QR 测试（距离 × 倾角 × 光照）验证新参数
- [ ] 仿真：换畸变后按需跑 Webots 长评测 / `auto_tune`
- [ ] 红色障碍检测：真机是否切 RGB565？（当前灰度下关闭）
