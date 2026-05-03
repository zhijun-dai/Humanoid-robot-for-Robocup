OpenMV H7 Plus 一键拷盘包（Webots 对齐巡线 + 协议 V2）
====================================================

本目录三个文件请全部复制到 OpenMV U 盘根目录（与 boot.py 同级），覆盖同名文件即可。

  main.py              <- 由 main_webots_aligned.py 复制，上电自动运行
  protocol_v2.py       <- 协议打包/解析
  line_follow_params.json

接线：UART1，P1=TX -> 主控 RX，P0=RX <- 主控 TX，GND 共地；115200 8N1。

更新仓库里的参数后，请重新复制本目录中的 line_follow_params.json（或整包再拷一遍）。
