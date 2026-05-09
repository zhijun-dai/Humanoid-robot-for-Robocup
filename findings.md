# 研究发现与约束

## 赛事规则 PDF

- 用户引用：`10.2赛事规则.工程竞技类机器人-人形全能竞技项目.2025 中国机器人大赛暨RoboCup机器人世界杯中国赛.pdf`
- **当前仓库未包含该文件**。建议路径：`docs/rules/robocup_2025_humanoid_rules.pdf`  
- 入库后请补充：**二维码最小尺寸、允许距离、是否允许多码、违规判罚**等与视觉直接相关的条款。

## 导师反馈（聊天摘要）

- 大倾角扫码：手机都要扫一会 → **角度影响显著**，需专门扫参和失败样本。
- **0.5 s vs 1 s** 有区别 → 明确测端到端延迟（识别稳定 → UART 发出）。
- **提前 ~20 cm 能扫到** vs 贴顶才扫到 → 需在场地尺度下做距离曲线。
- 测试目的：**暴露问题**，需记录失败工况而非只报成功。

## OpenMV_flash 与仿真

- `OpenMV_flash/line_follow_params.json` 为真机导向（如 `sim_opencv_distort: false`）。
- 仓库根目录 `line_follow_params.json` 服务 Webots；二者 intentionally 不完全相同。
