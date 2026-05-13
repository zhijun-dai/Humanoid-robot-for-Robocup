"""Jetson Nano 视觉主循环（Windows 可运行测试）
巡线 + QR + 红条 → UART 到 STM32（Protocol V2）
"""
import cv2
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from qr_detector import QRDetector


def main():
    # ── 相机 ──
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # ── QR 检测器 ──
    qr = QRDetector(
        stable_frames=1,
        cooldown_ms=2000,
        min_edge_px=20,
        max_edge_px=400,
        debug=True,
    )

    print("Jetson Vision — QR Test Mode (640x480)")
    print("  strategies: raw + clahe")
    print("  stable_frames=1  cooldown=2s  min_edge=20px  max_edge=400px")
    print("  Press ESC to quit\n")

    # ── 循环 ──
    fps_t0 = time.time()
    fps_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # QR 检测
        action, dbg = qr.update(frame)

        # 画框
        if dbg is not None and "w" in dbg:
            cv2.putText(frame, f"QR {dbg['action']} {dbg['w']}x{dbg['h']}px",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # FPS 计算
        fps_count += 1
        if fps_count % 30 == 0:
            now = time.time()
            dt = now - fps_t0
            fps = 30.0 / dt if dt > 0 else 0
            fps_t0 = now

        cv2.putText(frame, f"FPS: {30.0:.0f}" if fps_count % 30 == 0 else " ",
                    (540, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imshow("Jetson Vision", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
