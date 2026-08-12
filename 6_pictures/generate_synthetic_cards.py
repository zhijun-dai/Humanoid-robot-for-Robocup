"""CV合成训练数据生成器 — 6种几何图卡。

模拟真实比赛场景：10cm图卡平贴地面，摄像头斜视（透视），光照不均，噪声模糊。
输出 YOLO 格式（class x_center y_center w h），可直接训练 yolov8n。

用法:
    python generate_synthetic_cards.py --count 500 --out synthetic_dataset
"""
import argparse
import os
import random
import cv2
import numpy as np

# 6种形状: (名字, 类别id, 动作号)
SHAPES = [
    ("circle", 0, 1),      # 举左手
    ("pentagon", 1, 2),    # 举右手
    ("square", 2, 3),      # 抬左腿
    ("diamond", 3, 4),     # 抬右腿
    ("cross", 4, 5),       # 举双手
    ("triangle", 5, 6),    # 摇头
]

# 类别名映射（YOLO yaml）
YAML = """names:
  0: circle
  1: pentagon
  2: square
  3: diamond
  4: cross
  5: triangle
"""


def render_card(shape_idx, size=256):
    """渲染一张图卡：白底 + 黑色图形 + 外框。返回灰度图。"""
    img = np.ones((size, size), dtype=np.uint8) * 255
    cx, cy = size // 2, size // 2
    c = 0  # 黑色

    # 外框（10cm卡边，线粗≈5%尺寸）
    cv2.rectangle(img, (int(size * 0.05), int(size * 0.05)),
                  (int(size * 0.95), int(size * 0.95)), c, max(2, size // 40))

    r = size * 0.28  # 图形半径

    if shape_idx == 0:  # circle
        cv2.circle(img, (cx, cy), int(r), c, max(2, size // 40))
    elif shape_idx == 1:  # pentagon 五角星
        pts = []
        for i in range(10):
            ang = -np.pi / 2 + np.pi * i / 5
            rr = r if i % 2 == 0 else r * 0.45
            pts.append([int(cx + rr * np.cos(ang)), int(cy + rr * np.sin(ang))])
        cv2.polylines(img, [np.array(pts, np.int32)], True, c, max(2, size // 40))
    elif shape_idx == 2:  # square
        s = r * 1.1
        cv2.rectangle(img, (int(cx - s), int(cy - s)),
                      (int(cx + s), int(cy + s)), c, max(2, size // 40))
    elif shape_idx == 3:  # diamond
        pts = np.array([[cx, cy - int(r * 1.1)], [cx - int(r * 1.1), cy],
                        [cx, cy + int(r * 1.1)], [cx + int(r * 1.1), cy]], np.int32)
        cv2.polylines(img, [pts], True, c, max(2, size // 40))
    elif shape_idx == 4:  # cross — 横线+竖线交叉（规则：一条横线一条竖线）
        arm = int(r * 1.2)
        t = max(2, size // 40)
        cv2.line(img, (cx - arm, cy), (cx + arm, cy), c, t)
        cv2.line(img, (cx, cy - arm), (cx, cy + arm), c, t)
    elif shape_idx == 5:  # triangle
        pts = np.array([[cx, cy - int(r * 1.1)], [cx - int(r * 1.1), cy + int(r * 0.9)],
                        [cx + int(r * 1.1), cy + int(r * 0.9)]], np.int32)
        cv2.polylines(img, [pts], True, c, max(2, size // 40))
    return img


def _crop_to_card(img):
    """裁剪到非白区域边界（外框完整包住），返回 (裁剪图, bbox)。"""
    ys, xs = np.where(img < 250)
    if len(ys) == 0:
        return img, (0, 0, img.shape[1], img.shape[0])
    y0, y1 = ys.min(), ys.max()
    x0, x1 = xs.min(), xs.max()
    return img[y0:y1 + 1, x0:x1 + 1], (x0, y0, x1 - x0 + 1, y1 - y0 + 1)


def compose_scene(card, img_w=640, img_h=480):
    """把图卡放到场景中：随机位置、旋转、透视、光照、噪声、模糊。

    关键：所有变换在大画布上进行，变换后裁剪到外框边界，
    保证外框四角完整——外框是"检测到图卡"的第一判断依据。

    返回 (scene_bgr, bbox_normalized)。
    """
    card_h, card_w = card.shape

    # 1. 随机缩放（40%~85% of 图宽）— 模拟距离远近
    scale = random.uniform(0.4, 0.85)
    new_w = int(card_w * scale)
    new_h = int(card_h * scale)
    card = cv2.resize(card, (new_w, new_h))

    # 2. 放大画布居中（padding 40%）→ 旋转不会裁掉外框角
    pad = int(max(new_w, new_h) * 0.4)
    canvas_w, canvas_h = new_w + 2 * pad, new_h + 2 * pad
    big = np.ones((canvas_h, canvas_w), dtype=np.uint8) * 255
    big[pad:pad + new_h, pad:pad + new_w] = card

    # 3. 旋转（±12°）— 模拟贴歪；正方形边∥外框、菱形45°是印刷决定
    ang = random.uniform(-12, 12)
    M = cv2.getRotationMatrix2D((canvas_w // 2, canvas_h // 2), ang, 1.0)
    big = cv2.warpAffine(big, M, (canvas_w, canvas_h),
                         flags=cv2.INTER_CUBIC,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    # 旋转后裁剪到外框（画布够大，外框完整）
    card, _ = _crop_to_card(big)
    new_h, new_w = card.shape

    # 4. 透视（50%概率）— 模拟斜视；变换后同样裁剪保外框完整
    if random.random() < 0.5:
        src = np.float32([[0, 0], [new_w, 0], [new_w, new_h], [0, new_h]])
        dx = random.uniform(0, new_w * 0.06)
        dy = random.uniform(0, new_h * 0.05)
        dst = np.float32([[dx, dy], [new_w - dx * 0.5, 0],
                          [new_w, new_h], [0, new_h - dy * 0.5]])
        M2 = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(card, M2, (new_w, new_h),
                                     flags=cv2.INTER_CUBIC,
                                     borderMode=cv2.BORDER_CONSTANT,
                                     borderValue=255)
        card, _ = _crop_to_card(warped)
        new_h, new_w = card.shape

    # 5. 随机位置（不出界）
    x0 = random.randint(0, max(1, img_w - new_w))
    y0 = random.randint(0, max(1, img_h - new_h))

    # 6. 场景：浅灰白底（模拟喷绘布，略偏灰 + 亮度不均）
    scene = np.ones((img_h, img_w), dtype=np.uint8) * random.randint(220, 250)
    grad = np.linspace(0, random.randint(5, 25), img_w, dtype=np.float32)
    scene = np.clip(scene.astype(np.float32) + grad[None, :], 0, 255).astype(np.uint8)

    scene[y0:y0 + new_h, x0:x0 + new_w] = card

    # 7. 全局光照/对比度
    brightness = random.uniform(0.85, 1.1)
    contrast = random.uniform(0.9, 1.1)
    scene = cv2.convertScaleAbs(scene, alpha=contrast, beta=(brightness - 1) * 128)

    # 8. 高斯噪声 + 模糊
    noise = np.random.normal(0, random.uniform(2, 5), scene.shape)
    scene = np.clip(scene.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    if random.random() < 0.25:
        scene = cv2.GaussianBlur(scene, (3, 3), 0)

    # 9. 转BGR（YOLO训练通常用RGB/BGR三通道）
    scene_bgr = cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR)

    # bbox（图卡区域，含外框）
    bbox = (x0 / img_w, y0 / img_h, new_w / img_w, new_h / img_h)
    cx, cy, w, h = bbox
    yolo_line = f"{cx + w / 2:.6f} {cy + h / 2:.6f} {w:.6f} {h:.6f}"
    return scene_bgr, yolo_line


def main():
    parser = argparse.ArgumentParser(description="CV合成几何图卡训练数据")
    parser.add_argument("--count", type=int, default=500,
                        help="每类生成数量（默认500，共3000张）")
    parser.add_argument("--out", type=str, default="synthetic_dataset",
                        help="输出目录")
    parser.add_argument("--size", type=int, default=640,
                        help="场景尺寸（宽）")
    args = parser.parse_args()

    img_w = args.size
    img_h = int(img_w * 0.75)
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.out)
    images_dir = os.path.join(out_dir, "images")
    labels_dir = os.path.join(out_dir, "labels")
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)

    # 写数据集yaml
    with open(os.path.join(out_dir, "dataset.yaml"), "w", encoding="utf-8") as f:
        f.write(f"path: {out_dir}\ntrain: images\nval: images\n{YAML}")

    total = args.count * len(SHAPES)
    idx = 0
    for shape_idx, (name, cls_id, action) in enumerate(SHAPES):
        for i in range(args.count):
            card = render_card(shape_idx, size=256)
            scene, yolo_line = compose_scene(card, img_w, img_h)
            fname = f"{name}_{i:05d}"
            # imencode+tofile 支持中文路径（cv2.imwrite 不支持）
            ext = ".jpg"
            ok, buf = cv2.imencode(ext, scene)
            if ok:
                buf.tofile(os.path.join(images_dir, fname + ext))
            with open(os.path.join(labels_dir, fname + ".txt"), "w") as f:
                f.write(f"{cls_id} {yolo_line}\n")
            idx += 1
            if idx % 500 == 0:
                print(f"  {idx}/{total}")

    print(f"Done: {total} images -> {out_dir}")
    print(f"  图片: {images_dir}")
    print(f"  标注: {labels_dir}")
    print("训练命令: yolo detect train data=dataset.yaml model=yolov8n.pt epochs=100")


if __name__ == "__main__":
    main()
