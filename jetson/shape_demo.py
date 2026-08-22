"""几何图卡识别 Demo — 支持照片和视频。

用法:
    python jetson/shape_demo.py --image path/to/photo.jpg
    python jetson/shape_demo.py --video path/to/video.mp4
    python jetson/shape_demo.py --image 6_pictures/圆形.png   # 默认测试图

输出:
    - 识别结果: 图形名称 + 动作号 + 置信度
    - 可视化: 检测框画在图上（CV方案画外框，YOLO方案画bbox）

双方案:
    A. 传统CV (shape_detector.py) — 外框检测+单应性矫正+形状分类
    B. YOLO (shape_yolo_best.pt) — 直接检测
    C. 混合: 两者结果取高置信度
"""
import argparse
import os
import sys
import cv2
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from shape_detector import ShapeDetector
from shape_preprocess import preprocess_for_yolo

# 动作映射（与规则一致）
ACTION_NAMES = {
    1: "举左手", 2: "举右手", 3: "抬左腿",
    4: "抬右腿", 5: "举双手", 6: "摇头",
}
SHAPE_NAMES = {
    "circle": "圆形", "pentagon": "五角星", "square": "正方形",
    "diamond": "菱形", "cross": "十字", "triangle": "三角形",
}
# YOLO类别 → 动作号
YOLO_CLS = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5, 5: 6}
YOLO_NAMES = {0: "circle", 1: "pentagon", 2: "square",
              3: "diamond", 4: "cross", 5: "triangle"}


def load_image(path):
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def run_cv(img, detector):
    """传统CV方案。返回 (shape_name, action, conf, quad)"""
    action, dbg = detector.update(img)
    shape = dbg.get("shape")
    if shape is None:
        return None, None, 0.0, None
    return shape, action, 1.0, dbg.get("quad")


def run_yolo(img, model):
    """YOLO方案。返回 (shape_name, action, conf, box)"""
    results = model.predict(img, conf=0.3, verbose=False)
    best = None
    for r in results:
        for box in r.boxes:
            if box.cls is None:
                continue
            conf = float(box.conf[0])
            if best is None or conf > best[1]:
                best = (int(box.cls[0]), conf, box.xyxy[0].cpu().numpy())
    if best is None:
        return None, None, 0.0, None
    cls_id, conf, xyxy = best
    name = YOLO_NAMES[cls_id]
    return name, YOLO_CLS[cls_id], conf, xyxy


def main():
    parser = argparse.ArgumentParser(description="几何图卡识别 Demo")
    parser.add_argument("--image", type=str, default=None, help="照片路径")
    parser.add_argument("--video", type=str, default=None, help="视频路径")
    parser.add_argument("--yolo", action="store_true",
                        help="启用YOLO方案（需ultralytics+权重）")
    parser.add_argument("--preprocess", action="store_true",
                        help="YOLO前先做传统CV预处理（灰度/CLAHE/Otsu/形态学）")
    parser.add_argument("--save-pre", type=str, default=None,
                        help="保存预处理中间图到指定路径（调试用）")
    args = parser.parse_args()

    # CV检测器
    detector = ShapeDetector(stable_frames=1, cooldown_ms=0, debug=False,
                             roi_ratio=1.0)

    # YOLO模型（可选）
    model = None
    if args.yolo:
        try:
            from ultralytics import YOLO
            weights = os.path.join(_SCRIPT_DIR, "..", "6_pictures",
                                   "shape_yolo_best.pt")
            model = YOLO(os.path.abspath(weights))
            print(f"[yolo] 模型加载: {os.path.abspath(weights)}")
        except Exception as e:
            print(f"[yolo] 加载失败（跳过YOLO）: {e}")

    def process_frame(frame, frame_idx=0):
        # 方案A: CV（在原图上，不做预处理——CV有自己的二值化）
        cv_shape, cv_action, cv_conf, cv_quad = run_cv(frame, detector)
        # 方案B: YOLO（可选预处理：真实帧→白底黑线→喂模型）
        yo_shape, yo_action, yo_conf, yo_box = (None, None, 0.0, None)
        if model is not None:
            yolo_input = frame
            if args.preprocess:
                save_pre = None
                if args.save_pre and frame_idx == 0:
                    save_pre = args.save_pre
                yolo_input = preprocess_for_yolo(frame, save_debug=save_pre)
            yo_shape, yo_action, yo_conf, yo_box = run_yolo(yolo_input, model)

        # 混合决策：取置信度高者
        if cv_action is not None and yo_action is not None:
            if yo_conf > 0.6:
                final = ("YOLO", yo_shape, yo_action, yo_conf)
            else:
                final = ("CV", cv_shape, cv_action, cv_conf)
        elif cv_action is not None:
            final = ("CV", cv_shape, cv_action, cv_conf)
        elif yo_action is not None:
            final = ("YOLO", yo_shape, yo_action, yo_conf)
        else:
            final = None

        # 可视化
        disp = frame.copy()
        if cv_quad is not None:
            pts = cv_quad.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(disp, [pts], True, (0, 255, 0), 3)
        if yo_box is not None:
            x1, y1, x2, y2 = [int(v) for v in yo_box]
            cv2.rectangle(disp, (x1, y1), (x2, y2), (255, 0, 0), 3)

        if final is not None:
            src, shape, action, conf = final
            label = f"{SHAPE_NAMES.get(shape, shape)} 动作{action} {ACTION_NAMES[action]} ({src} conf={conf:.2f})"
        else:
            label = "未识别"
        cv2.putText(disp, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2)

        cv_info = f"CV: {SHAPE_NAMES.get(cv_shape, '-') if cv_shape else '-'} | YOLO: {SHAPE_NAMES.get(yo_shape, '-') if yo_shape else '-'}"
        cv2.putText(disp, cv_info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (200, 200, 0), 2)
        return disp, final

    # ── 照片模式 ──
    if args.image:
        img = load_image(args.image)
        if img is None:
            print(f"无法读取 {args.image}")
            return
        disp, final = process_frame(img)
        print(f"\n=== {os.path.basename(args.image)} ===")
        if final:
            src, shape, action, conf = final
            print(f"  识别: {SHAPE_NAMES.get(shape, shape)} (动作{action} {ACTION_NAMES[action]})")
            print(f"  方案: {src}  置信度: {conf:.2f}")
        else:
            print("  未识别")
        out = os.path.join(_SCRIPT_DIR, "..", "6_pictures", "demo_result.jpg")
        cv2.imencode(".jpg", disp)[1].tofile(os.path.abspath(out))
        print(f"  结果图: {os.path.abspath(out)}")
        cv2.imshow("Result", disp)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # ── 视频模式 ──
    elif args.video:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            print(f"无法打开视频 {args.video}")
            return
        print(f"=== {os.path.basename(args.video)} ===")
        frame_idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            disp, final = process_frame(frame, frame_idx)
            if final:
                src, shape, action, conf = final
                print(f"  帧{frame_idx:>4}: {SHAPE_NAMES.get(shape, shape)} 动作{action} ({src} conf={conf:.2f})")
            else:
                print(f"  帧{frame_idx:>4}: 未识别")
            cv2.imshow("Video", disp)
            frame_idx += 1
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
        print(f"  共处理 {frame_idx} 帧")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
