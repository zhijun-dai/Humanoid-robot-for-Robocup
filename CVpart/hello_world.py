# This work is licensed under the MIT license.
# Copyright (c) 2013-2025 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor  # 导入摄像头模块，用于采集图像。 # type: ignore
import time  # 导入时间模块，用于FPS计时。

sensor.reset()  # 复位并初始化摄像头。
sensor.set_pixformat(sensor.RGB565)  # 设置像素格式为RGB565（也可改为GRAYSCALE）。
sensor.set_framesize(sensor.VGA)  # 设置分辨率为VGA。
sensor.skip_frames(time=2000)  # 等待设置生效和曝光稳定。
clock = time.clock()  # 创建时钟对象，用来统计FPS。

while True:  # 主循环：持续采图并输出帧率。
    clock.tick()  # 更新时钟采样点。
    img = sensor.snapshot()  # 拍一帧图像并保存到img变量。
    print(clock.fps())  # 打印当前FPS；连接IDE时FPS会更低，断开后通常更高。
    # 这里保留img变量可供后续添加绘图或识别逻辑。
