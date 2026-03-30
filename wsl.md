# WSL 当前配置进度（可迁移到其他工作区）

更新日期：2026-03-30

## 1. WSL 总体状态

- 默认发行版：`Ubuntu-24.04`
- 默认版本：`WSL2`
- 当前发行版状态：可正常启动（`wsl -l -v` 显示 `Ubuntu-24.04  VERSION 2`）

## 2. Linux 用户与 Git

- 默认用户：`yuque`
- Git 全局用户名：`yuque`
- Git 全局邮箱：`3957054419@qq.com`
- Git 版本：`2.43.0`

## 3. SSH 状态（GitHub）

- 已生成 SSH key（ed25519）
- 直连 `github.com:22`：被拒绝（当前网络环境不可用）
- 备用 `ssh.github.com:443`：可正常认证（已测试通过）

建议在 `~/.ssh/config` 固定 443 方案：

```sshconfig
Host github.com
	HostName ssh.github.com
	Port 443
	User git
	IdentityFile ~/.ssh/id_ed25519
```

## 4. Python 学习环境（WSL 内）

- Miniconda 安装路径：`/home/yuque/miniconda3`
- conda 版本：`26.1.1`
- 已有环境：
	- `base`
	- `py-study`（Python 3.11）

常用命令：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate py-study
python --version
```

## 5. C++ 编译环境（WSL 内）

- g++：`13.3.0`
- cmake：`3.28.3`
- gdb：已安装

说明：WSL 内已具备基础 C/C++ 开发能力。

## 6. 在新工作区怎么复用

进入任意新工作区后，按下面 3 步即可：

1. 进入 WSL：
```powershell
wsl
```

2. 进入项目目录（示例）：
```bash
cd /mnt/d/用户/Lenovo/桌面/CS\ and\ AI
```

3. 激活 Python 环境：
```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate py-study
```

---

如果后续在新工作区遇到 Git 拉取失败，优先检查是否已经写入上述 `~/.ssh/config`（443 端口方案）。

## 7. 本次新增（2026-03-29）

- 已完成系统基础更新与常用依赖补齐（网络、压缩、调试、Python 工具链等）。
- 已安装容器基础环境：
  - `docker.io`（Docker Engine）
  - `docker-compose`（Compose v1）
- Docker 服务状态：`enabled + active`
- 当前用户组：`yuque sudo docker`（可直接使用 Docker）

## 8. SSH 配置现状

- 已创建 `~/.ssh/config`，并固定 GitHub 443 端口方案：

```sshconfig
Host github.com
	HostName ssh.github.com
	Port 443
	User git
	IdentityFile ~/.ssh/id_ed25519
```

## 9. 与 Isaac 相关的本机限制

- 当前本机（Windows + WSL）均未检测到可用 `nvidia-smi`。
- 结论：本机不适合直接运行 Isaac Sim 图形仿真。
- 建议：本机继续用于代码开发；Isaac Sim 放在 AutoDL GPU 实例运行。

## 10. WSL 日常使用说明（写清楚版）

### 10.1 进入与退出

在 Windows PowerShell 中：

```powershell
wsl
```

指定发行版进入：

```powershell
wsl -d Ubuntu-24.04
```

在 WSL 里退出：

```bash
exit
```

### 10.2 常用管理命令（在 Windows 里执行）

```powershell
wsl -l -v          # 查看发行版与版本
wsl --status       # 查看WSL总体状态
wsl --shutdown     # 完全重启WSL（改配置后常用）
```

### 10.3 进入项目并开发

```bash
cd /mnt/d/用户/Lenovo/桌面/Robocup
```

Python 环境：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate py-study
python --version
```

### 10.4 Git 与 SSH（当前网络推荐）

- 本机网络下优先使用 `ssh.github.com:443`。
- 配置文件位置：`~/.ssh/config`。
- 测试命令：

```bash
ssh -T git@github.com
```

### 10.5 Docker 使用

当前用户已在 `docker` 组，可直接使用：

```bash
docker --version
docker-compose --version
docker ps
```

如果刚改过用户组，先在 Windows 执行：

```powershell
wsl --shutdown
```

然后重新进入 WSL。

### 10.6 可视化（WSLg）

- Windows 11 自带 WSLg，可运行 Linux GUI 程序窗口。
- 快速自检：

```bash
sudo apt update
sudo apt install -y x11-apps gedit
xeyes   # 或 gedit
```

## 11. 一条命令快速自检

在 PowerShell 里执行：

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'whoami; python3 --version; git --version; docker --version; docker-compose --version; systemctl is-active docker; id -nG'
```

期望结果：
- 用户是 `yuque`
- Docker 服务是 `active`
- 用户组包含 `docker`

## 12. 常见坑与恢复（已实测）

### 现象

在 Ubuntu 终端里执行：
- `wsl -l -v`
- `wsl -d Ubuntu-24.04`

提示不是 Microsoft 的 WSL，而是：
- `WSL Wsman Shell commandLine, version 0.2.1`

### 原因

- 在 Ubuntu 内误执行了：`sudo apt install wsl`
- 该包会安装 `/usr/bin/wsl`，与 Windows 的 `wsl.exe` 同名，导致命令冲突。

### 正确用法

- Windows 管理命令（如 `wsl -l -v`）只能在 Windows PowerShell 里执行。
- Ubuntu 里如果确实要调用 Windows WSL 命令，使用：

```bash
/mnt/c/Windows/System32/wsl.exe -l -v
```

### 恢复步骤

在 Ubuntu 里执行：

```bash
sudo apt remove wsl
sudo apt autoremove
```

恢复后：
- Ubuntu 里 `wsl` 不再存在（避免冲突）
- PowerShell 中 `wsl -l -v` 正常
- Ubuntu 里可通过 `/mnt/c/Windows/System32/wsl.exe` 调用 Windows WSL 管理命令

## 13. zsh / oh-my-zsh 现状（2026-03-30）

- 已安装 `zsh`：`zsh 5.9`
- 已安装 `oh-my-zsh`
- 默认登录 shell：`/usr/bin/zsh`

当前采用“稳定开发配置”：
- 主题：`robbyrussell`
- oh-my-zsh 插件：`plugins=(git)`
- 额外插件（来自 apt）：
	- `zsh-autosuggestions`
	- `zsh-syntax-highlighting`

相关配置文件：`~/.zshrc`
