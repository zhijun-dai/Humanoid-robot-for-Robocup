# OpenMV 比赛主程序（测试可读版）
# 说明：
# 1) 在 main.py 基础上保留原有核心逻辑（二维码优先 + 巡线兜底）。
# 2) 增加更易读的终端输出，便于现场调参和联调观察。

import sensor  # type: ignore
import image  # type: ignore
import time
from pyb import LED, UART  # type: ignore

# ---------------------- 串口与协议 ----------------------
UART_ID = 1  # 使用 UART1 和主控通信。
UART_BAUD = 9600  # 波特率需与主控严格一致。
uart = UART(UART_ID, UART_BAUD)  # 初始化串口对象。

USE_LEGACY_CHAR_PROTOCOL = True  # 兼容旧版单字符协议。
USE_FRAME_PROTOCOL = True  # 同时发送 3 字节帧协议，便于扩展。
FRAME_HEAD = 0xAA  # 帧头常量。

# 二维码动作映射（只接受 1~6）
QR_ACTION_MAP = {
    "1": 0x01,
    "2": 0x02,
    "3": 0x03,
    "4": 0x04,
    "5": 0x05,
    "6": 0x06,
}

# 动作中文说明（仅用于打印）
QR_ACTION_NAME = {
    "1": "举左手",
    "2": "举右手",
    "3": "抬左腿",
    "4": "抬右腿",
    "5": "举双手",
    "6": "左右摇头",
}

# 巡线输出字符
ROUTE_GO = "1"
ROUTE_LEFT = "2"
ROUTE_RIGHT = "3"
ROUTE_SLIGHT_LEFT = "4"

# 巡线中文说明（仅用于打印）
ROUTE_NAME = {
    ROUTE_GO: "直走",
    ROUTE_LEFT: "左转纠偏",
    ROUTE_RIGHT: "右转纠偏",
    ROUTE_SLIGHT_LEFT: "边走边左",
}

# ---------------------- 图像参数 ----------------------
SENSOR_SKIP_MS = 2000  # 相机上电后等待稳定时间。
LENS_CORR_STRENGTH = 1.5  # 镜头畸变矫正强度。

LEFT_ROI = (0, 90, 90, 120)  # 左边缘检测窗口。
RIGHT_ROI = (230, 90, 90, 120)  # 右边缘检测窗口。

LINE_THRESHOLDS = [(0, 79)]  # 灰度回归阈值。
PIXEL_THRESHOLD = 300  # 线回归有效像素下限。

# 二维码识别防抖与节流参数
QR_STABLE_FRAMES = 2  # 同一二维码连续识别到该帧数才认为稳定。
QR_SEND_COOLDOWN_MS = 2500  # 二维码动作发送冷却时间。
ROUTE_SEND_INTERVAL_MS = 80  # 巡线指令发送周期。

# 调试输出节流：避免终端刷屏太快
STATUS_PRINT_INTERVAL_MS = 1200  # 心跳状态打印间隔。
ROUTE_PRINT_INTERVAL_MS = 500  # 巡线重复信息打印间隔。
QR_COOLDOWN_PRINT_INTERVAL_MS = 800  # 冷却提示打印间隔。

# ---------------------- 状态变量 ----------------------
last_qr_sent_ms = 0  # 上次二维码发送时间戳。
last_route_sent_ms = 0  # 上次巡线发送时间戳。
last_status_print_ms = 0  # 上次状态日志时间戳。
last_route_print_ms = 0  # 上次巡线日志时间戳。
last_qr_cooldown_print_ms = 0  # 上次冷却日志时间戳。
qr_candidate = None  # 当前二维码候选值。
qr_candidate_count = 0  # 候选值连续计数。
frame_id = 0  # 帧计数器。
last_route_cmd_printed = None  # 上次打印过的巡线方向。

# ---------------------- 初始化 ----------------------
LED(1).on()  # 红灯亮，表示程序已启动。
LED(2).on()  # 绿灯亮。
LED(3).on()  # 蓝灯亮。

sensor.reset()  # 复位摄像头。
sensor.set_pixformat(sensor.GRAYSCALE)  # 灰度模式，兼顾巡线和二维码。
sensor.set_framesize(sensor.QVGA)  # 320x240。
sensor.skip_frames(time=SENSOR_SKIP_MS)  # 等待图像稳定。
clock = time.clock()  # FPS 计时器。

print("=== main_test.py 启动 ===")
print("串口: UART{} @ {}".format(UART_ID, UART_BAUD))
print("二维码白名单: 1~6")
print("规则: 先二维码, 后巡线")
print("------------------------")


def now_ms():
    # 获取当前毫秒时间。
    return time.ticks_ms()


def send_frame(cmd):
    # 按 [帧头, 命令字, 校验] 打包发送。
    checksum = (FRAME_HEAD + cmd) & 0xFF
    packet = bytearray([FRAME_HEAD, cmd, checksum])
    uart.write(packet)


def send_legacy_char(ch):
    # 发送旧版单字符协议。
    uart.write(ch)


def send_qr_action(payload):
    # 将二维码内容映射成动作码并发送。
    cmd = QR_ACTION_MAP.get(payload)
    if cmd is None:
        return False

    if USE_LEGACY_CHAR_PROTOCOL:
        send_legacy_char(payload)
    if USE_FRAME_PROTOCOL:
        send_frame(cmd)

    print("[二维码已发送] 内容={} 动作={} cmd=0x{:02X}".format(
        payload, QR_ACTION_NAME.get(payload, "未知"), cmd
    ))
    return True


def send_route_cmd(ch):
    # 巡线阶段仅发送旧版方向字符。
    if USE_LEGACY_CHAR_PROTOCOL:
        send_legacy_char(ch)


def detect_route(img):
    # 左右双 ROI 做线回归，得到边缘方向信息。
    line_l = img.get_regression(
        LINE_THRESHOLDS,
        robust=True,
        roi=LEFT_ROI,
        pixel_threshold=PIXEL_THRESHOLD,
    )
    line_r = img.get_regression(
        LINE_THRESHOLDS,
        robust=True,
        roi=RIGHT_ROI,
        pixel_threshold=PIXEL_THRESHOLD,
    )

    # 画出左右 ROI，方便确认当前边缘检测窗口。
    img.draw_rectangle(LEFT_ROI, color=90, thickness=2, fill=False)
    img.draw_rectangle(RIGHT_ROI, color=90, thickness=2, fill=False)

    if line_l:
        x1, y1, x2, y2 = line_l.line()
        img.draw_line((x1, y1, x2, y2), color=220, thickness=2)
        img.draw_cross(x1, y1, color=220, size=6)
        img.draw_cross(x2, y2, color=220, size=6)
        img.draw_string(LEFT_ROI[0] + 2, LEFT_ROI[1] + 2, "L", color=220, scale=1)
    if line_r:
        x1, y1, x2, y2 = line_r.line()
        img.draw_line((x1, y1, x2, y2), color=150, thickness=2)
        img.draw_cross(x1, y1, color=150, size=6)
        img.draw_cross(x2, y2, color=150, size=6)
        img.draw_string(RIGHT_ROI[0] + 2, RIGHT_ROI[1] + 2, "R", color=150, scale=1)

    if line_l and line_r:
        return ROUTE_GO
    if line_l and not line_r:
        return ROUTE_RIGHT
    if (not line_l) and line_r:
        theta_r = line_r.theta()
        if 140 < theta_r < 160:
            return ROUTE_SLIGHT_LEFT
        return ROUTE_LEFT
    return ROUTE_GO


def detect_qr_payload(img):
    # 查找二维码，返回首个白名单内的 payload（1~6）。
    qrs = img.find_qrcodes()
    if not qrs:
        return None

    for qr in qrs:
        rect = qr.rect()
        payload = qr.payload().strip()

        # 有效二维码亮框，无效二维码暗框。
        box_color = 255 if payload in QR_ACTION_MAP else 80
        img.draw_rectangle(rect, color=box_color, thickness=3)

        # 在中心打点并显示 payload，便于现场快速判断识别是否正确。
        cx = rect[0] + (rect[2] // 2)
        cy = rect[1] + (rect[3] // 2)
        img.draw_cross(cx, cy, color=box_color, size=8)
        img.draw_string(rect[0], rect[1] - 12, payload, color=box_color, scale=1)

        if payload in QR_ACTION_MAP:
            return payload
    return None


while True:
    clock.tick()  # 更新帧率计时。
    frame_id += 1  # 帧编号递增。
    img = sensor.snapshot().lens_corr(LENS_CORR_STRENGTH)  # 采集图像并矫正畸变。
    now = now_ms()  # 读取当前时间。

    qr_payload = detect_qr_payload(img)  # 二维码优先识别。

    if qr_payload is not None:
        if qr_payload == qr_candidate:  # 同一候选连续出现。
            qr_candidate_count += 1
        else:  # 出现新候选，重置计数。
            qr_candidate = qr_payload
            qr_candidate_count = 1
            print("[二维码候选] 发现内容={} 动作={} (开始计数)".format(
                qr_payload, QR_ACTION_NAME.get(qr_payload, "未知")
            ))

        if qr_candidate_count >= QR_STABLE_FRAMES:  # 达到防抖帧数后才考虑发送。
            elapsed = time.ticks_diff(now, last_qr_sent_ms)
            if elapsed > QR_SEND_COOLDOWN_MS:  # 冷却结束，可以发送。
                if send_qr_action(qr_payload):
                    last_qr_sent_ms = now
                    qr_candidate_count = 0
            else:  # 冷却未结束，仅提示不发送。
                remain = QR_SEND_COOLDOWN_MS - elapsed
                if remain < 0:
                    remain = 0
                if time.ticks_diff(now, last_qr_cooldown_print_ms) > QR_COOLDOWN_PRINT_INTERVAL_MS:
                    print("[二维码冷却] 内容={} 剩余={}ms".format(qr_payload, remain))
                    last_qr_cooldown_print_ms = now

    else:  # 当前帧没有识别到二维码，走巡线逻辑。
        qr_candidate = None
        qr_candidate_count = 0

        if time.ticks_diff(now, last_route_sent_ms) > ROUTE_SEND_INTERVAL_MS:
            route_cmd = detect_route(img)
            send_route_cmd(route_cmd)

            # 只在方向变化或到达打印周期时打印，减少重复日志。
            if (
                route_cmd != last_route_cmd_printed
                or time.ticks_diff(now, last_route_print_ms) > ROUTE_PRINT_INTERVAL_MS
            ):
                print("[巡线] 指令={} 含义={}".format(route_cmd, ROUTE_NAME.get(route_cmd, "未知")))
                last_route_cmd_printed = route_cmd
                last_route_print_ms = now

            last_route_sent_ms = now

    # 每隔一段时间打印运行状态，便于判断程序是否卡住
    if time.ticks_diff(now, last_status_print_ms) > STATUS_PRINT_INTERVAL_MS:
        print("[状态] 帧={} fps={:.1f} 候选={} 候选计数={}".format(
            frame_id,
            clock.fps(),
            qr_candidate,
            qr_candidate_count,
        ))
        last_status_print_ms = now
