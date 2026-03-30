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
