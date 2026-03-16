"""
PC-side QR smoke test for CVpart/QRcode/*.png
Run with:
    python pc_qr_smoketest.py

This script is diagnostic-only. It reports decode results to help calibration,
and does not enforce competition pass/fail criteria.

中文说明：
- 这个脚本用于在电脑上快速检查二维码素材是否能被OpenCV解码。
- 它不是比赛验收脚本，不做“通过/失败”裁决，只输出诊断信息。
"""

from pathlib import Path
import cv2
import numpy as np


def decode_qr_image(image_path: Path):
    # 创建OpenCV二维码解码器。
    detector = cv2.QRCodeDetector()
    # Use imdecode+fromfile to support non-ASCII paths on Windows.
    # 用fromfile+imdecode而不是imread，避免中文路径读取失败。
    data = np.fromfile(str(image_path), dtype=np.uint8)
    if data.size == 0:
        return None
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        return None

    text, points, _ = detector.detectAndDecode(img)
    if points is None:
        return None

    text = (text or "").strip()
    return text if text else None


def main():
    # 以脚本所在目录为基准定位QRcode文件夹，避免受当前工作目录影响。
    script_dir = Path(__file__).resolve().parent
    qr_dir = script_dir.parent / "QRcode"

    if not qr_dir.exists():
        print("QR directory not found:", qr_dir)
        return

    images = sorted(qr_dir.glob("*.png"))
    if not images:
        print("No PNG files found in:", qr_dir)
        return

    decoded = 0
    action_mapped = 0
    unknown_payloads = []

    # 逐张图片解码，并打印原始结果，便于排查某一张图的问题。
    for img_path in images:
        result = decode_qr_image(img_path)
        print(f"{img_path.name}: {result}")
        if result is not None:
            decoded += 1
            if result in {"1", "2", "3", "4", "5", "6"}:
                action_mapped += 1
            else:
                # 非1~6的内容先记下来，后续由你决定是否映射动作。
                unknown_payloads.append((img_path.name, result))

    # 输出汇总信息，作为调参和数据检查参考。
    print("--- Summary (diagnostic) ---")
    print(f"Decoded images: {decoded}/{len(images)}")
    print(f"Mapped action payloads (1-6): {action_mapped}")
    if unknown_payloads:
        print("Unknown payloads:")
        for name, payload in unknown_payloads:
            print(f"  - {name}: {payload}")


if __name__ == "__main__":
    main()
