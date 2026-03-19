# OpenMV 黑色物体识别测试脚本（标题注释，说明脚本用途）。
# 用途：在 OpenMV IDE 中快速调试“黑色物体识别”阈值（功能说明）。
# 运行方式：在 IDE 打开本文件，点击 Run（使用方式说明）。

import sensor  # 导入摄像头传感器模块，用于采集图像。 # type: ignore
import image  # 导入图像处理模块，提供 find_blobs 等函数。 # type: ignore
import time  # 导入时间模块，用于计时和限频打印。
from pyb import LED  # 导入板载 LED 控制类，用于可视化状态指示。 # type: ignore

# ---------------------- 可调参数 ----------------------
# 灰度阈值：值越小越黑。常见可从 (0, 40) 或 (0, 50) 开始试。
BLACK_THRESHOLDS = [(0, 45)]  # 当前黑色阈值配置，后续根据现场光照调节。

# 过滤小噪点：像素数和面积低于该值的 blob 会被忽略。
MIN_PIXELS = 150  # 最小像素阈值，低于此值的候选目标不计入识别。
MIN_AREA = 150  # 最小面积阈值，低于此面积的候选目标不计入识别。

# 是否只在局部区域识别。None 表示整幅图。
# 示例 ROI = (x, y, w, h) -> ROI = (40, 20, 240, 200)
ROI = None  # 默认全图检测；如需抗干扰可改为固定 ROI。

# 终端打印间隔，避免刷屏太快（毫秒）。
PRINT_INTERVAL_MS = 200  # 日志打印节流时间，单位毫秒。


# ---------------------- 初始化 ----------------------
sensor.reset()  # 复位摄像头并初始化底层寄存器。
sensor.set_pixformat(sensor.GRAYSCALE)  # 设置灰度模式，简化黑色检测阈值调节。
sensor.set_framesize(sensor.QVGA)  # 设置分辨率为 QVGA(320x240)，兼顾速度与效果。
sensor.skip_frames(time=1500)  # 等待摄像头参数稳定，避免刚启动时画面波动。

clock = time.clock()  # 创建帧率时钟对象，用于统计 FPS。

red_led = LED(1)  # 获取红色 LED 控制句柄（未检测到时点亮）。
green_led = LED(2)  # 获取绿色 LED 控制句柄（检测到时点亮）。
blue_led = LED(3)  # 获取蓝色 LED 控制句柄（本脚本预留，默认关闭）。

last_print_ms = 0  # 记录上次打印时间戳，做日志限频。


def now_ms():  # 定义毫秒时间函数，统一获取当前 ticks。
    return time.ticks_ms()  # 返回当前毫秒计数值。


def pick_largest_blob(blobs):  # 定义函数：从多个 blob 中挑最大的一个。
    # 选最大目标，便于先把“能稳定识别一个主目标”调通。
    best = None  # 当前最优目标，初始为空。
    best_area = -1  # 当前最大面积，初始设为 -1。
    for b in blobs:  # 遍历每一个候选 blob。
        area = b.w() * b.h()  # 计算该 blob 的包围框面积。
        if area > best_area:  # 如果面积更大，就更新最优目标。
            best = b  # 保存当前更大的 blob。
            best_area = area  # 同步更新最大面积值。
    return best  # 返回面积最大的 blob（或 None）。


while True:  # 主循环：持续取图、检测、输出。
    clock.tick()  # 更新时钟，供 FPS 统计使用。
    img = sensor.snapshot()  # 抓取当前一帧图像。

    if ROI is not None:  # 如果设置了 ROI，就只在 ROI 内检测。
        img.draw_rectangle(ROI, color=128, thickness=1)  # 在画面上画出 ROI 边框便于观察。
        blobs = img.find_blobs(  # 在 ROI 内查找符合黑色阈值的 blob。
            BLACK_THRESHOLDS,  # 传入黑色阈值配置。
            roi=ROI,  # 指定检测区域为 ROI。
            pixels_threshold=MIN_PIXELS,  # 指定最小像素阈值。
            area_threshold=MIN_AREA,  # 指定最小面积阈值。
            merge=True,  # 合并相邻 blob，降低碎片化目标数量。
        )  # 结束 find_blobs 参数。
    else:  # 如果没设置 ROI，就全图检测。
        blobs = img.find_blobs(  # 在全图范围查找符合阈值的 blob。
            BLACK_THRESHOLDS,  # 传入黑色阈值配置。
            pixels_threshold=MIN_PIXELS,  # 指定最小像素阈值。
            area_threshold=MIN_AREA,  # 指定最小面积阈值。
            merge=True,  # 合并相邻 blob，提升稳定性。
        )  # 结束 find_blobs 参数。

    target = pick_largest_blob(blobs) if blobs else None  # 若有候选则选最大目标，否则为 None。

    if target:  # 如果识别到了目标。
        # 画面可视化：方框 + 十字 + 文本标签
        img.draw_rectangle(target.rect(), color=255, thickness=2)  # 绘制目标包围框。
        img.draw_cross(target.cx(), target.cy(), color=255)  # 在目标中心绘制十字。
        img.draw_string(target.x(), max(0, target.y() - 12), "BLACK", color=255)  # 在目标附近标注文字。

        # 识别到目标：绿灯亮
        red_led.off()  # 关闭红灯，表示当前不是“未识别”状态。
        green_led.on()  # 点亮绿灯，表示已识别到黑色目标。
        blue_led.off()  # 关闭蓝灯，避免干扰状态指示。

        if time.ticks_diff(now_ms(), last_print_ms) >= PRINT_INTERVAL_MS:  # 到达打印间隔才输出日志。
            print(  # 打印检测结果和目标参数，用于调试阈值。
                "DETECTED",  # 输出状态：已检测到目标。
                "cx=", target.cx(),  # 输出目标中心 x 坐标。
                "cy=", target.cy(),  # 输出目标中心 y 坐标。
                "w=", target.w(),  # 输出目标宽度。
                "h=", target.h(),  # 输出目标高度。
                "pixels=", target.pixels(),  # 输出目标像素数量。
                "fps=%.2f" % clock.fps(),  # 输出当前帧率。
            )  # 结束打印参数。
            last_print_ms = now_ms()  # 更新上次打印时间戳。
    else:  # 如果没有识别到目标。
        # 未识别到目标：红灯亮
        red_led.on()  # 点亮红灯，表示当前未识别到黑色目标。
        green_led.off()  # 关闭绿灯。
        blue_led.off()  # 关闭蓝灯。

        if time.ticks_diff(now_ms(), last_print_ms) >= PRINT_INTERVAL_MS:  # 到达打印间隔才输出日志。
            print("NO_BLACK", "fps=%.2f" % clock.fps())  # 打印“未识别到黑色”状态和帧率。
            last_print_ms = now_ms()  # 更新上次打印时间戳。