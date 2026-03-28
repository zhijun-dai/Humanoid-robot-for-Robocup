# OpenMV 比赛主程序（单文件版）：
# 功能1：路边缘识别（参考前人代码的双ROI线回归思路）。
# 功能2：二维码识别（识别1~6，按比赛动作码输出）。
# 假设：摄像头安装在机器人头部，方向向前并略微向下。

import sensor  # 导入摄像头模块。 # type: ignore
import image  # 导入图像处理模块。 # type: ignore
import time  # 导入时间模块。 
from pyb import LED, UART  # 导入LED与UART。 # type: ignore

# ---------------------- 串口与协议 ----------------------
UART_ID = 1  # 使用UART1与主控通信。
UART_BAUD = 9600  # 串口波特率，联调时需与主控一致。
uart = UART(UART_ID, UART_BAUD)  # 初始化UART对象。

USE_LEGACY_CHAR_PROTOCOL = True  # 是否发送老协议字符码（兼容前人主控）。
USE_FRAME_PROTOCOL = True  # 是否发送3字节帧协议（更规范）。
FRAME_HEAD = 0xAA  # 帧协议帧头。

# 二维码动作映射：识别到字符串1~6后映射为命令字0x01~0x06。
QR_ACTION_MAP = {
    "1": 0x01,
    "2": 0x02,
    "3": 0x03,
    "4": 0x04,
    "5": 0x05,
    "6": 0x06,
}

# 巡线输出字符（兼容前人巡线串口风格）。
ROUTE_GO = "1"  # 直走。
ROUTE_LEFT = "2"  # 左转纠偏。
ROUTE_RIGHT = "3"  # 右转纠偏。
ROUTE_SLIGHT_LEFT = "4"  # 边走边左。

# ---------------------- 图像参数 ----------------------
SENSOR_SKIP_MS = 2000  # 相机启动稳定等待时间。
LENS_CORR_STRENGTH = 1.5  # 镜头畸变校正强度。

# 头部前向略向下视角：ROI放在画面下半部分更符合赛道边缘位置。
LEFT_ROI = (0, 90, 90, 120)  # 左侧边缘检测区。
RIGHT_ROI = (230, 90, 90, 120)  # 右侧边缘检测区。

LINE_THRESHOLDS = [(0, 79)]  # 灰度线回归阈值（参考前人代码）。
PIXEL_THRESHOLD = 300  # 线回归像素门限。

# 二维码识别防抖与节流参数。
QR_STABLE_FRAMES = 2  # 同一二维码连续识别到2帧后才发送。
QR_SEND_COOLDOWN_MS = 2500  # 两次二维码发送最小间隔。
ROUTE_SEND_INTERVAL_MS = 80  # 巡线命令发送周期。

# ---------------------- 状态变量 ----------------------
last_qr_sent_ms = 0  # 上次二维码发送时间。
last_route_sent_ms = 0  # 上次巡线发送时间。
qr_candidate = None  # 当前二维码候选值。
qr_candidate_count = 0  # 候选值连续出现次数。

# ---------------------- 初始化 ----------------------
LED(1).on()  # 红灯亮：程序启动。
LED(2).on()  # 绿灯亮：程序启动。
LED(3).on()  # 蓝灯亮：程序启动。

sensor.reset()  # 复位摄像头。
sensor.set_pixformat(sensor.GRAYSCALE)  # 灰度模式兼顾巡线与二维码稳定性。
sensor.set_framesize(sensor.QVGA)  # 320x240。
sensor.skip_frames(time=SENSOR_SKIP_MS)  # 等待稳定。
clock = time.clock()  # FPS时钟。


# ---------------------- 工具函数 ----------------------
def now_ms():  # 获取当前毫秒时间。
    return time.ticks_ms()  # 返回ticks毫秒值。


def send_frame(cmd):  # 发送3字节帧。
    checksum = (FRAME_HEAD + cmd) & 0xFF  # 计算校验和。
    packet = bytearray([FRAME_HEAD, cmd, checksum])  # 组包。
    uart.write(packet)  # 串口发送。


def send_legacy_char(ch):  # 发送老协议字符。
    uart.write(ch)  # 写入UART。


def send_qr_action(payload):  # 发送二维码动作结果。
    cmd = QR_ACTION_MAP.get(payload)  # 查询命令字。
    if cmd is None:  # 如果不是1~6。
        return False  # 不发送。
    if USE_LEGACY_CHAR_PROTOCOL:  # 若开启老协议。
        send_legacy_char(payload)  # 发送字符1~6。
    if USE_FRAME_PROTOCOL:  # 若开启帧协议。
        send_frame(cmd)  # 发送帧命令。
    print("qr:", payload, "cmd:", cmd)  # 打印调试信息。
    return True  # 返回发送成功。


def send_route_cmd(ch):  # 发送巡线方向。
    if USE_LEGACY_CHAR_PROTOCOL:  # 当前巡线走老协议。
        send_legacy_char(ch)  # 发送字符方向码。


# ---------------------- 感知函数 ----------------------
def detect_route(img):  # 路边缘检测（双ROI线回归）。
    line_l = img.get_regression(  # 左ROI线回归。
        LINE_THRESHOLDS,
        robust=True,
        roi=LEFT_ROI,
        pixel_threshold=PIXEL_THRESHOLD,
    )
    line_r = img.get_regression(  # 右ROI线回归。
        LINE_THRESHOLDS,
        robust=True,
        roi=RIGHT_ROI,
        pixel_threshold=PIXEL_THRESHOLD,
    )

    img.draw_rectangle(LEFT_ROI, thickness=2, fill=False)  # 画左ROI框。
    img.draw_rectangle(RIGHT_ROI, thickness=2, fill=False)  # 画右ROI框。

    if line_l:  # 若左线存在。
        img.draw_line(line_l.line(), color=127)  # 画左线。
    if line_r:  # 若右线存在。
        img.draw_line(line_r.line(), color=127)  # 画右线。

    if line_l and line_r:  # 左右都看到线。
        return ROUTE_GO  # 直走。
    if line_l and not line_r:  # 仅左线存在。
        return ROUTE_RIGHT  # 往右纠偏。
    if (not line_l) and line_r:  # 仅右线存在。
        theta_r = line_r.theta()  # 读取右线角度。
        if 140 < theta_r < 160:  # 特定角度区间。
            return ROUTE_SLIGHT_LEFT  # 边走边左。
        return ROUTE_LEFT  # 左转。
    return ROUTE_GO  # 都看不到线时保守直走。


def detect_qr_payload(img):  # 二维码检测函数。
    qrs = img.find_qrcodes()  # 查找二维码。
    if not qrs:  # 未检测到二维码。
        return None  # 返回空。
    for qr in qrs:  # 遍历本帧二维码。
        img.draw_rectangle(qr.rect(), color=127)  # 画二维码框。
        payload = qr.payload().strip()  # 读取并清理内容。
        if payload in QR_ACTION_MAP:  # 只接受1~6。
            return payload  # 返回首个有效码。
    return None  # 无有效码。


# ---------------------- 主循环 ----------------------
while True:  # 持续运行。
    clock.tick()  # 更新时钟。
    img = sensor.snapshot().lens_corr(LENS_CORR_STRENGTH)  # 拍照+畸变校正。
    now = now_ms()  # 当前时间戳。

    qr_payload = detect_qr_payload(img)  # 先识别二维码（比赛优先级更高）。
    if qr_payload is not None:  # 若识别到二维码。
        if qr_payload == qr_candidate:  # 与上一候选一致。
            qr_candidate_count += 1  # 连续计数+1。
        else:  # 与上一候选不同。
            qr_candidate = qr_payload  # 更新候选值。
            qr_candidate_count = 1  # 重置计数。
        if qr_candidate_count >= QR_STABLE_FRAMES:  # 达到防抖要求。
            if time.ticks_diff(now, last_qr_sent_ms) > QR_SEND_COOLDOWN_MS:  # 满足冷却时间。
                if send_qr_action(qr_payload):  # 发送二维码动作。
                    last_qr_sent_ms = now  # 更新时间戳。
                    qr_candidate_count = 0  # 清零计数防重复。
    else:  # 若没识别到二维码。
        qr_candidate = None  # 清空候选值。
        qr_candidate_count = 0  # 清空候选计数。
        if time.ticks_diff(now, last_route_sent_ms) > ROUTE_SEND_INTERVAL_MS:  # 到达巡线发送周期。
            route_cmd = detect_route(img)  # 获取巡线方向。
            send_route_cmd(route_cmd)  # 发送方向。
            print("route:", route_cmd)  # 打印巡线调试信息。
            last_route_sent_ms = now  # 更新时间戳。

    # print("fps:", clock.fps())  # 如需观察帧率，取消此行注释。
