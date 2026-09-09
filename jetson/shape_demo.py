"""几何图卡识别 Demo — 支持照片和视频。

用法:
    python jetson/shape_demo.py --image path/to/photo.jpg --method both
    python jetson/shape_demo.py --video path/to/video.mp4 --method cv
    python jetson/shape_demo.py --image 6_pictures/圆形.png

分类路径（--method）:
    cv   纯 CV 规则法（多边形拟合+几何特征判定，资格审核用）
    cnn  ShapeCNN 神经网络分类
    both 两条路径都算，输出对比（默认）

找框 + 单应矫正对所有路径相同；YOLO 方案用 --yolo 单独启用。
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

# ── 中文绘制（cv2.putText 不支持中文，用 PIL）──
_FONT_CACHE = {}


def _load_font(size):
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    try:
        from PIL import ImageFont
    except Exception:
        _FONT_CACHE[size] = None
        return None
    for p in ("C:/Windows/Fonts/msyh.ttc",
              "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
        if os.path.exists(p):
            try:
                f = ImageFont.truetype(p, size)
                _FONT_CACHE[size] = f
                return f
            except Exception:
                pass
    _FONT_CACHE[size] = None
    return None


def put_text(img, text, org, color_bgr=(0, 0, 255), size=26):
    """绘制文本（支持中文；无字体时回退英文近似）。"""
    font = _load_font(size)
    if font is None:
        cv2.putText(img, text.encode("ascii", "replace").decode(), org,
                    cv2.FONT_HERSHEY_SIMPLEX, size / 36.0, color_bgr, 2)
        return img
    from PIL import Image, ImageDraw
    pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ImageDraw.Draw(pil).text(org, text, font=font,
                             fill=tuple(reversed(color_bgr)))
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
    return img


def load_image(path):
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def run_cv(img, detector):
    """找框+分类（路径由 detector.classify_mode 决定）。

    返回 (shape, action, conf, quad, dbg)；dbg 含 shape_cnn/shape_rules 两路结果。
    """
    action, dbg = detector.update(img)
    shape = dbg.get("shape")
    conf = dbg.get("cnn_prob")
    if conf is None:
        conf = 1.0 if shape is not None else 0.0
    return shape, action, conf, dbg.get("quad"), dbg


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
    parser.add_argument("--method", choices=("cv", "cnn", "both"), default=None,
                        help="分类路径: cv=纯CV规则 / cnn=神经网络(摄像头模式默认) / both=两路都算")
    parser.add_argument("--camera", action="store_true",
                        help="摄像头实时识别（默认 CNN 模式）")
    parser.add_argument("--cam", type=int, default=0, help="摄像头索引")
    parser.add_argument("--preprocess", action="store_true",
                        help="YOLO前先做传统CV预处理（灰度/CLAHE/Otsu/形态学）")
    parser.add_argument("--save-pre", type=str, default=None,
                        help="保存预处理中间图到指定路径（调试用）")
    args = parser.parse_args()

    # 摄像头模式默认 CNN，其余默认两路对比
    method = args.method or ("cnn" if args.camera else "both")
    # 分类路径: cv→rules, cnn→cnn, both→auto+两路都算
    mode_map = {"cv": ("rules", False), "cnn": ("cnn", False),
                "both": ("auto", True)}
    cm, cb = mode_map[method]
    detector = ShapeDetector(stable_frames=1, cooldown_ms=0, debug=False,
                             roi_ratio=0.5, classify_mode=cm, compare_both=cb)

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
        # 找框 + 分类（cv/cnn/both 由 detector 配置决定）
        cv_shape, cv_action, cv_conf, cv_quad, cv_dbg = run_cv(frame, detector)
        cnn_s = cv_dbg.get("shape_cnn")
        rules_s = cv_dbg.get("shape_rules")
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

        hu_d = cv_dbg.get("hu_dist")
        if final is not None:
            src, shape, action, conf = final
            if detector.classify_mode == "rules" and hu_d is not None:
                conf_s = f"Hu距离 {hu_d:.3f}"   # 纯CV无概率，用模板距离
            else:
                conf_s = f"置信度 {conf:.2f}"
            label = (f"{SHAPE_NAMES.get(shape, shape)} 动作{action} "
                     f"{ACTION_NAMES[action]} | {conf_s}")
        else:
            label = "未识别"
        put_text(disp, label, (10, 8), (0, 0, 255), 30)

        def _n(s):
            return SHAPE_NAMES.get(s, s) if s else "-"
        hu_b, hu_d = cv_dbg.get("hu_best"), cv_dbg.get("hu_dist")
        if detector.classify_mode == "rules":
            info = f"规则: {_n(rules_s)}"
            if hu_b:
                info += f" | Hu: {_n(hu_b)}({hu_d:.3f})"
        elif detector.classify_mode == "cnn":
            info = f"CNN: {_n(cnn_s)}"
        else:
            info = f"CNN: {_n(cnn_s)} | 规则: {_n(rules_s)}"
            if hu_b:
                info += f" | Hu: {_n(hu_b)}({hu_d:.3f})"
        if model is not None:
            info += f" | YOLO: {_n(yo_shape)}"
        put_text(disp, info, (10, 46), (200, 200, 0), 22)
        return disp, final, (cnn_s, rules_s, hu_b, hu_d)

    # ── 摄像头实时模式（默认纯 CV）──
    if args.camera:
        import sys as _sys
        api = cv2.CAP_DSHOW if _sys.platform == "win32" else cv2.CAP_V4L2
        cap = cv2.VideoCapture(args.cam, api)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not cap.isOpened():
            print(f"无法打开摄像头 {args.cam}（换个索引试 --cam 1）")
            return
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        mode_name = {"rules": "纯CV", "cnn": "CNN", "auto": "两路对比"}[cm]
        print(f"=== 摄像头 {args.cam} 实时识别（{mode_name}） {aw}x{ah} ===")
        if aw < 1000:
            print("  提示：分辨率偏低，图卡拿近些更容易找到框")
        print("Q/ESC 退出")
        frame_idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] 读帧失败")
                break
            disp, final, paths = process_frame(frame, frame_idx)
            cv2.imshow("Shape Detect", disp)
            frame_idx += 1
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
        cap.release()
        cv2.destroyAllWindows()
        print(f"  共处理 {frame_idx} 帧")

    # ── 照片模式 ──
    elif args.image:
        img = load_image(args.image)
        if img is None:
            print(f"无法读取 {args.image}")
            return
        disp, final, paths = process_frame(img)
        cnn_s, rules_s, hu_b, hu_d = paths
        print(f"\n=== {os.path.basename(args.image)} ===")
        print(f"  CNN: {SHAPE_NAMES.get(cnn_s, '-') if cnn_s else '-'}"
              f" | 规则(纯CV): {SHAPE_NAMES.get(rules_s, '-') if rules_s else '-'}")
        if hu_b:
            print(f"  Hu矩: {SHAPE_NAMES.get(hu_b, hu_b)} (距离 {hu_d:.4f}，辅助参考)")
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
            disp, final, paths = process_frame(frame, frame_idx)
            if final:
                src, shape, action, conf = final
                print(f"  帧{frame_idx:>4}: {SHAPE_NAMES.get(shape, shape)} 动作{action} "
                      f"({src} conf={conf:.2f})  [CNN:{paths[0] or '-'} "
                      f"规则:{paths[1] or '-'} Hu:{paths[2] or '-'}]")
            else:
                print(f"  帧{frame_idx:>4}: 未识别  [CNN:{paths[0] or '-'} "
                      f"规则:{paths[1] or '-'} Hu:{paths[2] or '-'}]")
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
