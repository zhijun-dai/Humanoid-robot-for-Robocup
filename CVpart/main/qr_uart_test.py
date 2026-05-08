# qr_uart_test.py — 仅测二维码识别 + UART1/115200 发动作码
# 逻辑来自 main_test0.py（find_qrcodes、防抖、帧协议）；串口与 uart_link_test.py 一致。
# 接线：P1(TX)→主控 RX，P0(RX)←主控 TX，GND 共地。
#
# 白名单 payload 为 "1"~"6"；发送：
#   - 旧协议：原始 ASCII 字符一个字节（如 b"3"）
#   - 帧协议：0xAA, cmd, checksum（与 main_test0 相同）

import sensor
import image
import time
from pyb import UART, LED

UART_ID = 1
BAUD = 115200
try:
	uart = UART(UART_ID, BAUD, timeout_char=100)
except TypeError:
	uart = UART(UART_ID, BAUD)

USE_LEGACY_CHAR_PROTOCOL = True
USE_FRAME_PROTOCOL = True
FRAME_HEAD = 0xAA

QR_ACTION_MAP = {
	"1": 0x01,
	"2": 0x02,
	"3": 0x03,
	"4": 0x04,
	"5": 0x05,
	"6": 0x06,
}

QR_ACTION_NAME = {
	"1": "举左手",
	"2": "举右手",
	"3": "抬左腿",
	"4": "抬右腿",
	"5": "举双手",
	"6": "左右摇头",
}

QR_STABLE_FRAMES = 2
QR_SEND_COOLDOWN_MS = 2500
LENS_CORR_STRENGTH = 1.5
SENSOR_SKIP_MS = 2000

last_qr_sent_ms = 0
qr_candidate = None
qr_candidate_count = 0


def now_ms():
	return time.ticks_ms()


def send_frame(cmd):
	checksum = (FRAME_HEAD + cmd) & 0xFF
	uart.write(bytearray([FRAME_HEAD, cmd, checksum]))


def send_qr_action(payload):
	cmd = QR_ACTION_MAP.get(payload)
	if cmd is None:
		return False
	if USE_LEGACY_CHAR_PROTOCOL:
		uart.write(payload)
	if USE_FRAME_PROTOCOL:
		send_frame(cmd)
	print("[TX QR] payload={} {} cmd=0x{:02X}".format(
		payload,
		QR_ACTION_NAME.get(payload, ""),
		cmd,
	))
	return True


def draw_qr_debug(img, qrs):
	"""绘制全部二维码框；返回首个白名单 payload 或 None。"""
	first_ok = None
	for qr in qrs:
		rect = qr.rect()
		payload = qr.payload().strip()
		box_color = 255 if payload in QR_ACTION_MAP else 80
		img.draw_rectangle(rect, color=box_color, thickness=3)
		cx = rect[0] + (rect[2] // 2)
		cy = rect[1] + (rect[3] // 2)
		img.draw_cross(cx, cy, color=box_color, size=8)
		img.draw_string(rect[0], max(0, rect[1] - 12), payload, color=box_color, scale=1)
		if payload in QR_ACTION_MAP and first_ok is None:
			first_ok = payload
		elif payload not in QR_ACTION_MAP:
			print("[QR 非白名单] payload={!r}".format(payload))
	return first_ok


LED(1).on()
LED(2).on()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=SENSOR_SKIP_MS)

clock = time.clock()
print("qr_uart_test: UART{} @ {}".format(UART_ID, BAUD))
print("白名单 1~6；稳定 {} 帧 + 冷却 {} ms 后发送".format(QR_STABLE_FRAMES, QR_SEND_COOLDOWN_MS))

while True:
	clock.tick()
	img = sensor.snapshot().lens_corr(LENS_CORR_STRENGTH)
	now = now_ms()

	qrs = img.find_qrcodes()
	qr_payload = draw_qr_debug(img, qrs) if qrs else None

	if qr_payload is not None:
		if qr_payload == qr_candidate:
			qr_candidate_count += 1
		else:
			qr_candidate = qr_payload
			qr_candidate_count = 1
			print("[候选] {} {}".format(qr_payload, QR_ACTION_NAME.get(qr_payload, "")))

		if qr_candidate_count >= QR_STABLE_FRAMES:
			if time.ticks_diff(now, last_qr_sent_ms) > QR_SEND_COOLDOWN_MS:
				if send_qr_action(qr_payload):
					last_qr_sent_ms = now
					qr_candidate_count = 0
					LED(2).toggle()
	else:
		qr_candidate = None
		qr_candidate_count = 0

	if uart.any():
		try:
			raw = uart.read()
			if raw:
				print("rx:", raw)
		except Exception as e:
			print("read err:", e)

	print("fps={:.1f}".format(clock.fps()))
