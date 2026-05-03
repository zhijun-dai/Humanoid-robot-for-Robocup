# uart_link_test.py — OpenMV 与主控串口小测（UART1 / 115200）
# 用法：IDE 里 Run 本文件；或改名为 main.py 上电自跑。
# 现象：每 500ms 从 P1(TX) 发送一行 ASCII；若主控有回传，IDE 终端打印 RX 内容。
# 接线：P1->主控 RX，P0<-主控 TX，GND 共地。

import time
from pyb import UART, LED

UART_ID = 1
BAUD = 115200
# timeout_char 避免 read 卡死
uart = UART(UART_ID, BAUD, timeout_char=100)

LED(1).on()
LED(2).on()

n = 0
while True:
	n += 1
	line = "OMV ping %d\n" % n
	uart.write(line)

	if uart.any():
		try:
			raw = uart.read()
			if raw:
				print("RX bytes:", raw)
		except Exception as e:
			print("read err:", e)

	time.sleep_ms(500)
