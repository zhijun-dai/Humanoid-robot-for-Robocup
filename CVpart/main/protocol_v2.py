try:
	import ustruct as struct  # type: ignore
except Exception:
	import struct  # type: ignore

import time


def _ticks_ms():
	try:
		return int(time.ticks_ms())  # type: ignore
	except Exception:
		return int(time.time() * 1000)


def quantize_to_step(v, step):
	if step <= 1:
		return int(v)
	return int(round(float(v) / float(step)) * float(step))


def _clamp_i16(v):
	if v < -32768:
		return -32768
	if v > 32767:
		return 32767
	return int(v)


def _crc16_ccitt_false(data):
	crc = 0xFFFF
	for b in data:
		crc ^= (b & 0xFF) << 8
		for _ in range(8):
			if crc & 0x8000:
				crc = ((crc << 1) ^ 0x1021) & 0xFFFF
			else:
				crc = (crc << 1) & 0xFFFF
	return crc


class VisionProtocolV2:
	SOF1 = 0x55
	SOF2 = 0xAA
	MSG_HEARTBEAT = 0x01
	MSG_LINE_CTRL = 0x02

	def __init__(self, version=2):
		self.version = int(version) & 0xFF
		self.seq = 0

	def _next_seq(self):
		s = self.seq
		self.seq = (self.seq + 1) & 0xFF
		return s

	def build_frame(self, msg_type, payload=b"", flags=0, seq=None, ts_ms=None):
		if ts_ms is None:
			ts_ms = _ticks_ms()
		if seq is None:
			seq = self._next_seq()
		if payload is None:
			payload = b""

		header = struct.pack(
			"<BBBBIB",
			self.version,
			int(msg_type) & 0xFF,
			int(flags) & 0xFF,
			int(seq) & 0xFF,
			int(ts_ms) & 0xFFFFFFFF,
			len(payload) & 0xFF,
		)
		body = header + payload
		crc = _crc16_ccitt_false(body)
		return bytes([self.SOF1, self.SOF2]) + body + struct.pack("<H", crc)

	def build_heartbeat(self, mode_u8):
		payload = struct.pack("<B", int(mode_u8) & 0xFF)
		return self.build_frame(self.MSG_HEARTBEAT, payload)

	def build_line_ctrl(
		self,
		mode_u8,
		conf_u8,
		lost_u8,
		route_u8,
		ex_mm_i16=0,
		ang_cdeg_i16=0,
		v_cmd_mmps_i16=0,
		w_cmd_mradps_i16=0,
	):
		# 按 docs/motor_protocol_v2_p1.md 的偏移定义：
		# mode, conf, lost, route, ex, ang, v_cmd, w_cmd
		payload = struct.pack(
			"<BBBBhhhh",
			int(mode_u8) & 0xFF,
			int(conf_u8) & 0xFF,
			int(lost_u8) & 0xFF,
			int(route_u8) & 0xFF,
			_clamp_i16(ex_mm_i16),
			_clamp_i16(ang_cdeg_i16),
			_clamp_i16(v_cmd_mmps_i16),
			_clamp_i16(w_cmd_mradps_i16),
		)
		return self.build_frame(self.MSG_LINE_CTRL, payload)

