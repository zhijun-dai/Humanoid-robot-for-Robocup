# OpenMV main script (baseline framework)
# Runs on OpenMV Cam (MicroPython), not on PC Python.

# 说明：
# 这是一个“先跑通再优化”的基础版本。
# 你可以先用默认参数联调，后面再按赛场光照和机位逐项标定。

import sensor
import image
import time
from pyb import LED, UART

# ---------------------- Runtime Config ----------------------
# NOTE:
# These are initial defaults for bring-up only, not competition-grade fixed targets.
# Tune them on real hardware and keep synced with Diary.md validated params.

# 串口参数：必须与主控端保持一致，否则主控收不到正确数据。

UART_ID = 1
UART_BAUD = 9600

# Keep compatibility with old controller char protocol.
# 兼容老协议：直接发送字符'1'~'6'给主控。
USE_LEGACY_CHAR_PROTOCOL = True
# Optional frame protocol draft: [HEAD][CMD][CHECKSUM]
# 新协议草案：发送三字节帧，抗干扰更好。
USE_FRAME_PROTOCOL = True

FRAME_HEAD = 0xAA

# Feature switches for staged bring-up.
# 联调建议：先只开一个功能，单点调通后再同时开启。
ENABLE_QR = True
ENABLE_ROUTE = True

# QR command mapping (payload string -> command id)
# 二维码内容到命令字的映射：识别到"1"就发送0x01，以此类推。
QR_ACTION_MAP = {
    "1": 0x01,
    "2": 0x02,
    "3": 0x03,
    "4": 0x04,
    "5": 0x05,
    "6": 0x06,
}

# Simple routing output chars (compatible with old code)
# '1': go/keep moving, '2': left turn, '3': right turn, '4': slight-left-while-going
# 巡线字符协议（老版本主控能直接识别）
ROUTE_GO = "1"
ROUTE_LEFT = "2"
ROUTE_RIGHT = "3"
ROUTE_SLIGHT_LEFT = "4"

# Debounce / timing defaults (tune after field tests)
# 防抖：同一个二维码连续识别到N帧后才真正发送，减少误触发。
QR_STABLE_FRAMES = 2
# 冷却：两次二维码动作发送之间的最短间隔（毫秒）。
QR_SEND_COOLDOWN_MS = 2500
# 巡线命令发送周期（毫秒）。
ROUTE_SEND_INTERVAL_MS = 80

# Camera / image defaults
# 上电后等待摄像头稳定。
SENSOR_SKIP_MS = 2000
# 镜头畸变校正强度。
LENS_CORR_STRENGTH = 1.5

# Line regression defaults (must be re-calibrated by lighting and camera pose)
# 两个ROI分别看左边界和右边界。
LEFT_ROI = (0, 70, 70, 100)
RIGHT_ROI = (190, 50, 130, 120)
# 线段回归阈值和像素门限，后续需要按实际赛道调。
LINE_THRESHOLDS = [(0, 79)]
PIXEL_THRESHOLD = 300

# ---------------------- Init ----------------------
LED(1).on()
LED(2).on()
LED(3).on()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=SENSOR_SKIP_MS)
clock = time.clock()

uart = UART(UART_ID, UART_BAUD)

last_qr_sent_ms = 0
last_route_sent_ms = 0
qr_candidate = None
qr_candidate_count = 0


# ---------------------- Utils ----------------------
def now_ms():
    # OpenMV里常用毫秒时间戳，用于做定时和防抖。
    return time.ticks_ms()


def send_frame(cmd):
    # 三字节帧：[帧头][命令字][校验和]
    checksum = (FRAME_HEAD + cmd) & 0xFF
    packet = bytearray([FRAME_HEAD, cmd, checksum])
    uart.write(packet)


def send_legacy_char(ch):
    # 老协议：直接发一个字符给主控。
    uart.write(ch)


def send_qr_action(payload):
    # payload是二维码字符串，例如"1"、"2"...
    cmd = QR_ACTION_MAP.get(payload)
    if cmd is None:
        # 识别到未知二维码内容则忽略。
        return False

    if USE_LEGACY_CHAR_PROTOCOL:
        send_legacy_char(payload)
    if USE_FRAME_PROTOCOL:
        send_frame(cmd)

    print("QR action sent:", payload)
    return True


def send_route_cmd(ch):
    # 当前巡线只走老字符协议，便于兼容前人主控代码。
    if USE_LEGACY_CHAR_PROTOCOL:
        send_legacy_char(ch)


# ---------------------- Perception ----------------------
def detect_route(img):
    # 在左右两个ROI里做线段回归，判断偏航方向。
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

    img.draw_rectangle(LEFT_ROI, thickness=2, fill=False)
    img.draw_rectangle(RIGHT_ROI, thickness=2, fill=False)

    if line_l:
        img.draw_line(line_l.line(), color=127)
    if line_r:
        img.draw_line(line_r.line(), color=127)

    if line_l and line_r:
        # 左右边界都看到了，默认直行。
        return ROUTE_GO

    if line_l and not line_r:
        # 只看到左边界，通常说明车体偏右，给右转指令纠偏。
        return ROUTE_RIGHT

    if (not line_l) and line_r:
        theta_r = line_r.theta()
        if 140 < theta_r < 160:
            # 右边界角度较大时，用更柔和的“边走边左”。
            return ROUTE_SLIGHT_LEFT
        return ROUTE_LEFT

    # 两边都没看到时，保守策略：先保持前进。
    return ROUTE_GO


def detect_qr_payload(img):
    if not ENABLE_QR:
        return None

    qrs = img.find_qrcodes()
    if not qrs:
        return None

    # Use the first valid payload in this frame.
    for qr in qrs:
        img.draw_rectangle(qr.rect(), color=127)
        payload = qr.payload().strip()
        if payload in QR_ACTION_MAP:
            # 找到第一个可识别的动作码就返回。
            return payload

    return None


# ---------------------- Main loop ----------------------
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(LENS_CORR_STRENGTH)

    # 1) QR recognition has higher priority than route control.
    # 规则里二维码动作得分高，优先级放在巡线前。
    qr_payload = detect_qr_payload(img)
    now = now_ms()

    if qr_payload is not None:
        if qr_payload == qr_candidate:
            qr_candidate_count += 1
        else:
            qr_candidate = qr_payload
            qr_candidate_count = 1

        if qr_candidate_count >= QR_STABLE_FRAMES:
            if time.ticks_diff(now, last_qr_sent_ms) > QR_SEND_COOLDOWN_MS:
                if send_qr_action(qr_payload):
                    last_qr_sent_ms = now
                    # 发送成功后清空计数，避免重复触发。
                    qr_candidate_count = 0
    else:
        qr_candidate = None
        qr_candidate_count = 0

        # 2) Route edge baseline control.
        if ENABLE_ROUTE and time.ticks_diff(now, last_route_sent_ms) > ROUTE_SEND_INTERVAL_MS:
            route_cmd = detect_route(img)
            send_route_cmd(route_cmd)
            print("route:", route_cmd)
            last_route_sent_ms = now

    # For FPS observation in OpenMV IDE terminal.
    # print("fps:", clock.fps())
