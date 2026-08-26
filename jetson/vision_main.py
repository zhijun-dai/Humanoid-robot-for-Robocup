"""Jetson Nano 视觉 Demo — 巡线 + QR + 红条 实时可视化
双击 run_vision_demo.bat 运行。ESC 退出。
"""
import cv2
import time
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "v1_production"))
from qr_detector import QRDetector
from line_detector_v1_warp import LineDetector


# ── 可调参数 ──
CAM_IDX = 0          # 0=内置 1=USB（不确定就试）
CAM_W = 1280
CAM_H = 720
COOLDOWN_MS = 2000


def main():
    print(f"Opening camera [{CAM_IDX}] ({CAM_W}x{CAM_H})...")
    cap = cv2.VideoCapture(CAM_IDX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Actual: {actual_w}x{actual_h}")
    if not cap.isOpened():
        print("Failed to open camera! Try changing CAM_IDX.")
        return

    # 检测器
    qr = QRDetector(stable_frames=1, cooldown_ms=COOLDOWN_MS,
                    min_edge_px=20, max_edge_px=400, debug=False)
    ld = LineDetector(cam_w=actual_w, cam_h=actual_h,
                      cam_height_cm=40.0, cam_pitch_deg=45.0, cam_vfov_deg=56.2)

    print(f"Jetson Vision Demo  ({actual_w}x{actual_h})")
    print("  巡线: 几何原语拟合 (平行线 / 同心圆)")
    print("  QR:   raw + 2x upscale  红条: HSV mask")
    print("  Press ESC to quit\n")

    fps_t0 = time.time()
    fps_n = 0
    fps_val = 0.0
    last_qr_action = None
    last_qr_t = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        t_now = time.time()

        # FPS
        fps_n += 1
        if fps_n % 30 == 0:
            fps_val = 30 / max(t_now - fps_t0, 1e-3)
            fps_t0 = t_now

        # ── 巡线 ──
        dev_px, heading_deg, conf, vis_bird, dbg = ld.process(frame)
        vs = dbg.get("vision_speed_cm_s", 0.0)
        os = dbg.get("vision_omega_rad_s", 0.0)
        status_line = f"FPS={fps_val:.0f}  v={vs:.1f}cm/s"
        if dbg.get("red_bar_detected"):
            status_line += f"  RED! z={dbg.get('red_bar_z_cm',0):.0f}cm"
        nd = dbg.get("narrow_gate_dir", 0)
        if nd < 0:
            status_line += "  NARROW_IN"
        elif nd > 0:
            status_line += "  NARROW_OUT"

        if dev_px is not None and conf > 0.15:
            # 转弯方向判定
            if abs(heading_deg) < 4:
                turn_text, turn_color = "STRAIGHT", (0, 255, 0)
            elif heading_deg > 0:
                turn_text, turn_color = "RIGHT >>>", (0, 200, 255)
            else:
                turn_text, turn_color = "<<< LEFT", (0, 200, 255)

            status_line += f" | dev={dev_px:+.0f}px head={heading_deg:+.0f}deg [{turn_text}] c={conf:.2f}"

            # 转向指示（中央大箭头）
            cx, cy = actual_w // 2, actual_h // 2
            arrow_len = int(30 + abs(heading_deg) * 2.5)
            arrow_angle = np.radians(-heading_deg - 90)  # 上=0°，右转=右箭头
            dx = int(arrow_len * np.cos(arrow_angle))
            dy = int(arrow_len * np.sin(arrow_angle))
            cv2.arrowedLine(frame, (cx - dx, cy - dy), (cx + dx, cy + dy),
                            turn_color, 3, tipLength=0.4)

            # 偏离指示条（底部）
            bar_cx = actual_w // 2
            bar_y = actual_h - 25
            cv2.line(frame, (bar_cx - 80, bar_y), (bar_cx + 80, bar_y), (80, 80, 80), 2)
            cv2.circle(frame, (bar_cx, bar_y), 4, (255, 255, 255), -1)
            dev_indicator = int(bar_cx + dev_px * 0.5)
            dev_indicator = max(bar_cx - 80, min(bar_cx + 80, dev_indicator))
            cv2.circle(frame, (dev_indicator, bar_y), 7, turn_color, -1)
        else:
            status_line += " | NO LINE"

        # ── QR ──
        action, qr_dbg = qr.update(frame)
        if action is not None:
            last_qr_action = action
            last_qr_t = t_now
            cv2.putText(frame, f"QR={action}!", (actual_w - 150, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
        elif last_qr_action is not None and t_now - last_qr_t < 2.0:
            cv2.putText(frame, f"QR={last_qr_action}", (actual_w - 150, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 180, 180), 2)

        # ── 红条 (from V1 detector, overlay on original) ──
        if dbg.get("red_bar_detected"):
            rz = dbg.get("red_bar_z_cm", 0.0)
            rx = dbg.get("red_bar_x_cm", 0.0)
            status_line += f" | RED z={rz:.0f}cm x={rx:.0f}cm"
            rcx = dbg.get("red_bar_cx", 0.0)
            rcy = dbg.get("red_bar_cy", 0.0)
            if rcy > 0:
                cv2.line(frame, (0, int(rcy)), (actual_w - 1, int(rcy)), (0, 0, 255), 2)
                cv2.circle(frame, (int(rcx), int(rcy)), 10, (0, 0, 255), -1)
                cv2.putText(frame, f"z={rz:.0f}cm", (int(rcx) + 16, int(rcy) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # ── 状态栏 ──
        cv2.rectangle(frame, (0, 0), (actual_w, 28), (30, 30, 30), -1)
        cv2.putText(frame, status_line, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("1.Original", frame)

        # 中间结果窗口
        if "bird" in dbg and dbg["bird"] is not None:
            bird_bgr = cv2.cvtColor(dbg["bird"], cv2.COLOR_GRAY2BGR)
            cv2.imshow("2.Warp (birdseye)", cv2.resize(bird_bgr, (320, 400), interpolation=cv2.INTER_NEAREST))
        if "binary_raw" in dbg and dbg["binary_raw"] is not None:
            b_raw = cv2.cvtColor(dbg["binary_raw"], cv2.COLOR_GRAY2BGR)
            cv2.imshow("3.Adaptive (binary)", cv2.resize(b_raw, (320, 400), interpolation=cv2.INTER_NEAREST))
        if vis_bird is not None:
            cv2.imshow("4.Close+Fit", cv2.resize(vis_bird, (320, 400), interpolation=cv2.INTER_NEAREST))

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
