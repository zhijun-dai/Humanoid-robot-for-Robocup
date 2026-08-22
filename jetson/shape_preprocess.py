"""图卡检测预处理 — YOLO输入前的传统CV处理（方案A：固定工作分辨率）。

目的: 真实场景帧（彩色/光照不均/复杂背景）→ 白底黑线的统一表示，
接近合成训练数据分布（训练数据是白底黑线渲染），弥补数据不足。

管线（模仿巡线 blackhat+自适应阈值，简化版，不做鸟瞰）:
    输入帧(任意分辨率) → resize到960×540 → 灰度
    → blackhat(31×31椭圆，抑制宽阴影/增强细黑线)
    → 自适应阈值(GAUSSIAN, blockSize=31, C=-12)
    → 反转成白底黑线（训练分布；blackhat输出是黑底白线）
    → 形态学: 闭运算(3×3) → 开运算(3×3)
    → BGR3通道 → YOLO

参数来源与实测:
    - blackhat核31×31椭圆、自适应块31: 照搬巡线(line_detector_v1_warp.py)
    - C=-12: 实测标定。巡线C=-8在鸟瞰(320×400)上，本模块960×540原图上
      更负的C保留图卡细线更完整。白底黑线下C∈{-8,-12,-16}检出率
      83%/88%/83%，取-12
    - 形态学简化为开闭各1次(3×3): 巡线是close5→open5→close5→open3，
      实测对图卡细线无增益；巡线的连通域过滤会删掉图卡细线小连通域，
      实测致命(88%→21%)，故不采用
    - 输出必须白底黑线: 训练数据是白底黑线，方向反了检出率62%→88%
"""
import cv2
import numpy as np

# ── 固定工作分辨率（方案A：参数与分辨率解耦）──
WORK_W = 960
WORK_H = 540

# ── 固定预处理参数（实测标定）──
BH_KERNEL = 31          # blackhat核（椭圆，巡线同款）
ADAPTIVE_BLOCK = 31     # 自适应阈值窗口（巡线同款）
ADAPTIVE_C = -12        # 阈值偏移（实测最优88%）
MORPH_KERNEL = 3        # 形态学核（简化：开闭各1次）


def preprocess_for_yolo(bgr, invert=False, save_debug=None,
                        method="adaptive"):
    """真实帧 → YOLO友好的白底黑线图（3通道BGR，960×540）。

    Args:
        bgr: 彩色帧 (任意分辨率, H, W, 3)
        invert: 额外反转（黑底白线），默认False
        save_debug: 若给路径，保存中间结果图（调试用）
        method: "adaptive"=blackhat+自适应阈值（默认，抗光照不均）
                "otsu"=CLAHE+Otsu全图阈值（备用，光照均匀场景）

    Returns:
        (540, 960, 3) BGR图，喂给YOLO
    """
    # 0. 统一到固定工作分辨率（方案A）
    if bgr.shape[1] != WORK_W or bgr.shape[0] != WORK_H:
        bgr = cv2.resize(bgr, (WORK_W, WORK_H))

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    if method == "otsu":
        # 备用：CLAHE + Otsu（光照均匀场景）
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        _, bw = cv2.threshold(enhanced, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        bw = 255 - bw  # 白底黑线
    else:
        # 主力：blackhat + 自适应阈值（模仿巡线，抗光照不均）
        # blackhat = closing(src) - src：提取比周围暗的细线条
        # 图卡黑色外框+图形线被提取；背景大尺度明暗变化被抑制
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (BH_KERNEL, BH_KERNEL))
        blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)
        # 自适应阈值：局部亮度归一化，克服反光/阴影
        bw = cv2.adaptiveThreshold(blackhat, 255,
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY,
                                   ADAPTIVE_BLOCK, ADAPTIVE_C)
        # 反转成白底黑线（训练分布；blackhat输出是黑底白线）
        bw = 255 - bw

    # 形态学：闭运算连断裂 → 开运算去噪（各1次）
    k = cv2.getStructuringElement(cv2.MORPH_RECT,
                                  (MORPH_KERNEL, MORPH_KERNEL))
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k)
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, k)

    if invert:
        bw = 255 - bw

    if save_debug is not None:
        cv2.imencode('.jpg', bw)[1].tofile(save_debug)

    # 转回3通道（YOLO输入要求）
    return cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)


if __name__ == "__main__":
    # 自测：跑视频一帧看预处理效果
    import os
    video = os.path.join(os.path.dirname(__file__), "..", "tests",
                         "6-shape-test", "video-1.mp4")
    cap = cv2.VideoCapture(video)
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(cap.get(7) * 0.5))
    ok, frame = cap.read()
    cap.release()
    out = preprocess_for_yolo(frame,
                              save_debug=os.path.join(
                                  os.path.dirname(__file__), "..",
                                  "tests", "6-shape-test",
                                  "preprocess_adaptive.jpg"))
    print(f"预处理完成: 输入{frame.shape} → 输出{out.shape}")
    print(f"调试图: tests/6-shape-test/preprocess_adaptive.jpg")
