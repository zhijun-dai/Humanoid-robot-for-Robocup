# Isaac 云端入门（给初学者）

## 1. 先回答你的问题
是的，你可以理解为：

1. 你本地电脑继续写代码（Windows + WSL）。
2. 云端再租一台可远程操作的 Linux 机器（带 NVIDIA GPU）。
3. 你通过 SSH 连接那台云端机器，在上面运行 Isaac Sim。

为什么要这样做：

1. Isaac Sim 对 GPU 要求高。
2. 本地电脑没有合适显卡时，云端更稳定、也更省时间。

---

## 2. 你可以把它想成两台机器配合

1. 本地机（你现在的电脑）
作用：写代码、改参数、看日志、提交代码。

2. 云端机（远程 Linux）
作用：跑 Isaac Sim 场景、跑相机仿真、后续跑运动仿真。

3. 工作流
本地改代码 -> 推到云端 -> 在云端运行仿真 -> 看结果 -> 继续改。

---

## 3. Isaac 相关名词，一次讲清

1. Isaac Sim
核心仿真平台。你做视觉仿真和后续运动仿真，主要都用它。

2. Isaac Gym
老一代大规模并行训练工具。新项目一般不再作为首选入口。

3. Isaac Lab
建立在 Isaac Sim 之上的训练框架。主要用于强化学习训练流程。

结论：

1. 你当前阶段（视觉仿真）先用 Isaac Sim。
2. 以后做训练，再考虑 Isaac Lab。

---

## 4. 你的项目建议路线（分两阶段）

### 阶段 A：视觉仿真先跑通
目标是验证：

1. 仿真相机能稳定出图。
2. 你的视觉算法能吃到图像并输出控制量。
3. 小车在简单赛道可以闭环行驶。

### 阶段 B：再做运动 sim
目标是加入更真实的动力学因素：

1. 质量、惯量、摩擦、驱动限制。
2. 弯道稳定性和速度上限评估。
3. 量化误差指标（横向误差、速度误差、恢复时间）。

---

## 5. 云端怎么搭（可操作版）

### 5.1 选云主机配置（视觉阶段）

1. GPU：A10 或 L4（24G 显存级别）。
2. CPU：8 核以上。
3. 内存：32G 以上。
4. 磁盘：100G 以上 SSD。
5. 系统：Ubuntu 22.04 或 24.04。

### 5.2 连接云主机

1. 在本地终端执行：

```bash
ssh 用户名@云主机公网IP
```

2. 能连上后，先检查 GPU：

```bash
nvidia-smi
```

如果看不到显卡信息，先别继续装 Isaac，先修驱动。

### 5.3 安装 Docker 与 NVIDIA 容器支持

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-plugin curl

distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
	sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
	sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
	sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

验证容器里也能看到 GPU：

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

### 5.4 拉起 Isaac Sim 容器

1. 先准备缓存目录：

```bash
mkdir -p ~/isaac/cache/kit ~/isaac/cache/ov ~/isaac/cache/pip ~/isaac/data
```

2. 拉起容器（示例）：

```bash
docker run --name isaac-sim -it --gpus all --network host \
	-e ACCEPT_EULA=Y \
	-e PRIVACY_CONSENT=Y \
	-v ~/isaac/cache/kit:/isaac-sim/kit/cache:rw \
	-v ~/isaac/cache/ov:/root/.cache/ov:rw \
	-v ~/isaac/cache/pip:/root/.cache/pip:rw \
	-v ~/isaac/data:/isaac-sim/data:rw \
	nvcr.io/nvidia/isaac-sim:4.5.0
```

说明：

1. 版本号按官方可用版本替换。
2. 首次启动下载较慢，耐心等。

### 5.5 启动后你先做三件事

1. 能打开基础场景。
2. 能看到仿真相机图像流。
3. 能给机器人发送控制并看到动作。

做到这三点，就算“云端基础搭建成功”。

---

## 6. 你现在最该做的最小闭环

先别一上来就做复杂运动学，按这个顺序：

1. 赛道 + 小车 + 相机先跑起来。
2. 把你现有视觉算法接进去，只做直道和单弯。
3. 稳定后再引入更真实动力学参数。

这会比“全都一起上”更快成功。

---

## 7. 常见坑（初学者高频）

1. 驱动正常，但容器看不到 GPU。
检查 nvidia-container-toolkit 是否安装并重启 docker。

2. 首次启动很慢。
是正常现象，镜像和缓存下载体量大。

3. 一开始就追求复杂场景。
先做最小闭环，确认链路通，再逐步加复杂度。

4. 把 Isaac Gym 和 Sim 混着用。
你当前目标是视觉仿真，优先 Isaac Sim 即可。

---

## 8. 一句话记住
你当前最优实践是：

本地写代码 + 云端跑 Isaac Sim + 先视觉闭环 + 再做运动 sim。

如果你愿意，我下一步可以在这份文档后面继续补一节：

1. 你这个 RoboCup 项目的最小话题接口定义（图像输入、控制输出）。
2. 一份可直接运行的“云端启动脚本模板”（含日志与自动重启）。

---

## 9. 是不是还要写“检验控制代码”？
是的，必须要有。至少需要两部分控制量：

1. 转向控制：偏了多少，就转多少。
2. 速度控制：直道快一点，弯道和误差大时慢一点。

一个最小可用控制律可以写成：

1. 横向误差：e_y（车偏离中线的距离）
2. 航向误差：e_psi（车头方向和赛道切线方向的夹角）
3. 转角命令：

   delta = clamp(Ky * e_y + Kpsi * e_psi + Kd * de_y, -delta_max, delta_max)

4. 速度命令：

   v = clamp(v_ref - Kv * |e_y| - Kc * |curvature|, v_min, v_max)

意思很直观：

1. 偏得越多、角度差越大，打方向越多。
2. 越接近弯道或误差越大，就自动降速。

你后续可以把这个控制律接在视觉输出后面，先跑通再细调参数。

### 9.1 最小 Python 代码模板（可直接改）

```python
import math
from dataclasses import dataclass


def clamp(x, lo, hi):
		return max(lo, min(hi, x))


@dataclass
class CtrlParam:
		ky: float = 1.2
		kpsi: float = 0.8
		kd: float = 0.05
		kv: float = 0.6
		kc: float = 0.4
		delta_max: float = math.radians(28.0)
		v_ref: float = 1.2
		v_min: float = 0.25
		v_max: float = 1.5


class VisionController:
	def __init__(self, p: CtrlParam):
		self.p = p
		self.prev_ey = 0.0

	def step(self, ey, epsi, curvature, dt):
		# 横向误差变化率
		dey = (ey - self.prev_ey) / max(dt, 1e-3)
		self.prev_ey = ey

		# 转向控制
		delta = self.p.ky * ey + self.p.kpsi * epsi + self.p.kd * dey
		delta = clamp(delta, -self.p.delta_max, self.p.delta_max)

		# 速度控制
		v = self.p.v_ref - self.p.kv * abs(ey) - self.p.kc * abs(curvature)
		v = clamp(v, self.p.v_min, self.p.v_max)

		return delta, v
```

---

## 10. Isaac 用什么语言？
你可以先记一个实用结论：

1. 日常开发主力是 Python。
2. 高性能插件和底层扩展可以用 C++。
3. 场景与资产核心格式是 USD（不是编程语言，是场景描述格式）。
4. 和 ROS 2 对接时，Python/C++ 节点都可以。

初学者建议：

1. 先用 Python + ROS 2 把流程跑通。
2. 以后性能瓶颈明显，再考虑 C++ 扩展。

---

## 11. Isaac 运行起来是什么样的？
常见有两种形态：

1. GUI 模式（像一个 3D 仿真软件）
你会看到场景窗口、时间轴、物体树、参数面板、相机视角。

2. Headless 模式（无桌面）
常用于云端，通过流媒体或日志和话题数据来观察结果。

你现在做云端时，大概率用第 2 种，再配一个远程预览页面。

---

## 12. 真实场地数据如何导入？
可以分成四层导入，逐步逼近真实：

1. 几何层（场地形状）
把 CAD/网格模型导入成 USD 资产。你仓库里已有 `el05.stp`，可以作为几何起点。

2. 外观层（纹理与光照）
把赛道纹理、地面材质、光照条件做近似，减少视觉域偏差。

3. 传感器层（相机参数）
把真实相机内参、畸变、安装位姿写入仿真相机。

4. 动力学层（车体参数）
把质量、惯量、轮胎摩擦、执行器限制逐步拟合到实车。

### 12.1 推荐导入顺序（不要一次全做）

1. 先导入简化赛道几何和机器人模型，跑通控制链路。
2. 再导入真实相机参数，校准视觉误差。
3. 最后再拟合动力学参数，做运动 sim 和性能评估。

### 12.2 你仓库里可直接利用的资产

1. `el05.stp`：场地几何源。
2. `pi_12dof/urdf/pi_12dof.urdf`：机器人 URDF。
3. `Qmini-main/urdf/Qmini.urdf`：另一套机器人 URDF。

这些都可以作为 Isaac 场景资产的输入起点。

---

## 13. AutoDL 按量计费怎么用（省钱且不丢环境）

你租到的不是“单独一张卡”，而是一个完整实例：

1. GPU 算力（跑 Isaac Sim）
2. 系统环境（你安装的软件就在这里）
3. 存储空间（系统盘和数据盘）

所以“能安装东西”是正常的，安装内容保存在实例的磁盘里，不是在显卡里。

### 13.1 计费状态怎么理解

1. 运行中：按量计费持续走。
2. 关机：算力计费停止，但实例数据通常会保留。
3. 释放实例：数据会被清理，环境和文件都不再可用。

注意：

1. 不同套餐和平台策略会影响“关机后能保留多久”。
2. 以控制台当前规则为准，尤其是“自动释放”与“存储计费”条款。

### 13.2 你的最佳使用方式（推荐）

1. 首次开机一次性装好环境：驱动、Docker、Isaac 依赖。
2. 把项目和数据放到数据盘，不要只放系统盘。
3. 每次收工先备份代码与结果，再关机省钱。
4. 长时间不用时，确认是否会自动释放；重要数据提前导出。

### 13.3 日常 SOP（开机到关机）

1. 开机后先做健康检查：

```bash
nvidia-smi
docker ps -a
```

2. 进入工作目录并同步代码：

```bash
cd ~/work/Robocup
git pull
```

3. 启动 Isaac 相关服务或容器，开始实验。
4. 实验结束保存结果到数据盘目录（如 `/root/autodl-tmp/` 或你自定义的数据盘路径）。
5. 提交代码或打包关键结果后再关机。

### 13.4 防丢数据最小清单

每次关机前至少做这 4 件事：

1. `git status` 确认代码改动是否已提交。
2. 将实验日志和模型文件复制到数据盘固定目录。
3. 关键结果额外同步一份到本地或远端仓库。
4. 在控制台确认实例状态变为“已关机”。

### 13.5 什么时候按量，什么时候包日

1. 按量：适合你这种“分段实验、间歇运行”的模式。
2. 包日/包周：适合连续多小时长训，避免反复开关机与排队抢卡。

简单判断：

1. 当天只跑几小时，按量更划算。
2. 连续重负载开发训练，包日通常更省心。

### 13.6 你现在就能执行的动作

1. 在控制台确认当前实例的“关机保留策略”和“自动释放规则”。
2. 建一个固定数据目录（例如 `/root/autodl-tmp/robocup_runs`）。
3. 以后每次实验结束都先备份再关机。

这样做的结果是：

1. 能继续按量省钱。
2. 又能最大化保留环境与实验成果。
