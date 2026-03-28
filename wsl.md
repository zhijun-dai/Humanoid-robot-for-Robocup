# WSL 当前配置进度（可迁移到其他工作区）

更新日期：2026-03-28

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
