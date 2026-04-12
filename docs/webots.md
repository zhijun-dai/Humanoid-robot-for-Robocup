# Webots 视觉仿真详细流程（按这个做就能跑）

## 1. 先回答：项目里有没有场地信息

有，但还不是开箱即用的 Webots 世界文件。

当前仓库可直接利用的场地相关资产：

1. `el05.stp`：场地 CAD 几何源。
2. `前人代码/前人摄像头1/a.bmp`：可作为黑白场地图候选纹理。
3. `CVpart/README.md` 与 `Task.md`：有赛道任务描述（可用于还原线路逻辑）。

结论：

1. 你有“场地源数据”。
2. 需要做一次“Webots 场景拼装”。

---

## 2. 基于 Webots 文档的关键点（R2025a）

本流程依赖这些官方规则：

1. `Robot.controller` 指向 `controllers/<控制器名>/` 目录下程序。
2. `Camera` 节点可直接设置 `width`、`height`、`fieldOfView`。
3. 控制器里通过 `camera.getWidth()`、`camera.getHeight()` 读取实际分辨率。
4. 轮式机器人可先用内置 e-puck，字段里可直接设 `camera_width`、`camera_height`。

---

## 3. 你要达到的目标（本次最小闭环）

1. 导入黑白场地图（赛道纹理）。
2. 放一个简单小车（用 e-puck 即可）。
3. 装摄像头并确认分辨率。
4. 跑测试控制器，让车能按简单指令前进并根据黑线偏差转向。

---

## 4. 推荐目录结构（先建好）

建议在仓库下新建：

```text
webots_project/
	worlds/
		lane_test.wbt
	controllers/
		line_follow_test/
			line_follow_test.py
	textures/
		track_bw.png
```

说明：

1. `track_bw.png` 可以由 `前人代码/前人摄像头1/a.bmp` 转出来。
2. 如果你已经有黑白图，也可以直接替换。

---

## 5. 黑白场地图怎么准备（边缘线怎么画）

### 5.1 最快做法（纹理法，推荐）

1. 新建一张图：`2048 x 2048`。
2. 背景填浅灰或白色。
3. 用黑色画两条边缘线（中间留赛道）。
4. 线宽建议先用 20 到 40 像素。
5. 保存为 `webots_project/textures/track_bw.png`。

### 5.2 与真实比例对齐建议

1. 先把地面设为 `4m x 4m`。
2. 如果赛道实际宽度约 `0.30m`，那纹理里赛道宽度占比约 `0.30/4 = 7.5%`。
3. 这样视觉阈值会更接近实物。

---

## 6. 场景搭建步骤（Webots GUI）

1. 打开 Webots，创建 `webots_project`。
2. 在 `worlds/lane_test.wbt` 中放地面。
3. 给地面贴 `track_bw.png`。
4. 插入 e-puck 机器人。
5. 配置 e-puck 控制器为 `line_follow_test`。
6. 设置相机分辨率（例如 320x240）。

### 6.0 纯 UI 点击版（10 分钟速通）

如果你不想手改 world 文本，按这个点击顺序直接做：

1. `File > New > New Project Directory...`，创建项目和 `lane_test.wbt`。
2. 打开 `Tools > Scene Tree`，后面所有参数都在左侧树和底部字段编辑器里改。
3. 选中场景树最后一个节点，点 `Add`，插入 `Solid` 并命名为 `track_floor`。
4. 在 `track_floor.children` 里 `Add > Shape`。
5. 在 `Shape.appearance` 里 `Add > PBRAppearance`。
6. 在 `PBRAppearance.baseColorMap` 里 `Add > ImageTexture`，`url` 填 `../textures/track_bw.png`。
7. 在 `Shape.geometry` 里 `Add > Box`，`size` 设为 `4 0.02 4`。
8. 在 `track_floor.boundingObject` 里也设 `Box`，`size` 同样为 `4 0.02 4`。
9. 再点一次 `Add`，插入 `PROTO nodes (Webots Projects) > robots > gctronic > e-puck > E-puck`。
10. 展开 `E-puck` 节点，把 `controller` 设为 `line_follow_test`。
11. 把 `camera_width` 设为 `320`，`camera_height` 设为 `240`，`camera_fieldOfView` 设为 `1.0`。
12. `File > New > New Robot Controller...`，创建 Python 控制器 `line_follow_test`。
13. 粘贴第 7 节代码，保存。
14. 运行前先 `Pause` 再 `Reset`，然后点 `Run real-time`。
15. 控制台看到 `camera width= 320 height= 240`，说明 UI 配置生效。

### 6.0A 你当前这个界面怎么做（左侧有 `Floor "floor"`）

你截图里的场地是内置 `Floor`，可以直接在它上面改，不用先删。

1. 先改尺寸：选中 `Floor "floor"`，把 `size` 设为 `4 4`（先做 4m x 4m 小场地）。
2. 再改贴图密度：把 `tileSize` 设为 `4 4`，先做到一张图只铺一次，便于校准线宽。
3. 展开 `appearance Parquetry`。
4. 把 `type` 从 `"chequered"` 改为纯色外观（例如木纹/纯色都可），目标是先去掉棋盘格干扰。
5. 如果你的界面支持替换节点：在 `appearance` 上右键或字段编辑器里用 `Replace`，把 `Parquetry` 换成 `PBRAppearance`。
6. 在新的 `PBRAppearance` 里添加 `baseColorMap > ImageTexture`，`url` 填 `../textures/track_bw.png`。
7. 运行看地面是否出现黑白赛道纹理；若没显示，优先检查图片路径是否相对 `worlds/lane_test.wbt` 正确。

如果第 5 步找不到 `Replace`（部分版本入口不明显），直接走下面兜底法：

1. 保留当前 `Floor` 不动，只把它当“托底地面”。
2. 新增一个 `Solid` 叫 `track_floor`，放在 `Floor` 上方 `0.001m`（把 `translation.y` 略微抬高）。
3. 给 `track_floor` 添加 `Shape > PBRAppearance > ImageTexture`，并设 `Box size = 4 0.01 4`。
4. 这样你就能稳定贴自定义黑白赛道图，不受 `Parquetry` 限制。

### 6.1 可参考的 world 片段（手动改到你的 wbt）

```vrml
#VRML_SIM R2025a utf8

WorldInfo {
	basicTimeStep 32
}

Viewpoint {
	orientation -0.33 0.91 0.24 1.35
	position 2.2 2.0 2.2
}

TexturedBackground { }
TexturedBackgroundLight { }

Solid {
	name "track_floor"
	translation 0 -0.01 0
	children [
		Shape {
			appearance PBRAppearance {
				baseColorMap ImageTexture {
					url ["../textures/track_bw.png"]
				}
				roughness 1
				metalness 0
			}
			geometry Box {
				size 4 0.02 4
			}
		}
	]
	boundingObject Box {
		size 4 0.02 4
	}
}

E-puck {
	translation 0 0.02 0
	rotation 0 1 0 0
	controller "line_follow_test"
	camera_width 320
	camera_height 240
	camera_fieldOfView 1.0
}
```

---

## 7. 测试控制器代码（可直接运行）

将下面代码保存到：

`webots_project/controllers/line_follow_test/line_follow_test.py`

```python
from controller import Robot
import numpy as np


robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

base_speed = 4.0
kp = 1.8
kd = 0.15
threshold = 90
last_error = 0.0

print("camera width=", camera.getWidth(), "height=", camera.getHeight())

while robot.step(timestep) != -1:
		w = camera.getWidth()
		h = camera.getHeight()

		raw = camera.getImage()
		img = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))

		# Webots 是 BGRA 排列
		b = img[:, :, 0].astype(np.float32)
		g = img[:, :, 1].astype(np.float32)
		r = img[:, :, 2].astype(np.float32)
		gray = 0.114 * b + 0.587 * g + 0.299 * r

		# 只看下半区
		roi = gray[int(h * 0.60):, :]
		mask = roi < threshold  # 黑线为 True

		ys, xs = np.where(mask)
		if len(xs) < 40:
				error = last_error
		else:
				cx = float(xs.mean())
				error = (cx - (w * 0.5)) / (w * 0.5)

		derivative = (error - last_error) / max(timestep / 1000.0, 1e-3)
		last_error = error

		steer = kp * error + kd * derivative
		steer = max(-2.5, min(2.5, steer))

		left_speed = base_speed - steer
		right_speed = base_speed + steer

		left_motor.setVelocity(left_speed)
		right_motor.setVelocity(right_speed)

		print("err=%.3f steer=%.3f L=%.2f R=%.2f" % (error, steer, left_speed, right_speed))
```

---

## 8. 如何确认“摄像头分辨率已生效”

看两处：

1. world 里 e-puck 的 `camera_width/camera_height`。
2. 控制器启动日志是否打印同样的宽高。

如果日志显示 `camera width= 320 height= 240`，说明分辨率配置成功。

---

## 9. 运行与验收

1. 点击 Webots 运行按钮。
2. 控制台出现误差与轮速日志。
3. 小车能沿黑白赛道边缘线附近持续前进。

最小验收标准：

1. 连续运行 30 秒不完全丢线。
2. 直道误差基本可回到 0 附近。
3. 弯道能纠偏，不持续打满方向。

---

## 10. 你接下来最省时间的调参顺序

1. 先调 `threshold`（保证线能稳识别）。
2. 再调 ROI 高度（`int(h * 0.60)` 这行）。
3. 最后调 `kp`、`kd`。

先别上复杂动力学，先把视觉误差链路跑顺。

---

## 11. 备注

1. `el05.stp` 适合后续做高保真几何导入。
2. 你现在做视觉参数，不必一开始就走 CAD 精细建模。
3. 先用黑白纹理赛道跑通，效率最高。
