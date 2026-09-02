"""微型CNN训练 — 图卡矫正正视图分类（6类图形+背景）。

路线B分类器：找框→矫正96×96→本CNN。0.4M参数，MNIST级任务。
输入：96×96单通道二值图（黑底白线，来自闭环管线矫正输出）。

用法:
    python jetson/train_shape_cnn.py --data 6_pictures/frontal_dataset/batch_xxx

数据目录: images/*.png + labels/*.txt（每行一个类别id 0-5，6=背景）
"""
import argparse
import glob
import os
import sys
import random
import numpy as np
import cv2
import torch
import torch.nn.functional as F

from shape_cnn import N_CLASSES, ShapeCNN


def imread_p(path):
    return cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_GRAYSCALE)


class CardDataset(torch.utils.data.Dataset):
    def __init__(self, paths, labels, train=True):
        self.paths = paths
        self.labels = labels
        self.train = train

    def __len__(self):
        return len(self.paths)

    def __getitem__(self, i):
        img = imread_p(self.paths[i])
        if img is None:
            img = np.full((96, 96), 255, np.uint8)
        img = cv2.resize(img, (96, 96))
        x = torch.from_numpy(img).float().unsqueeze(0) / 255.0
        if self.train and random.random() < 0.5:
            # 轻增广：±2px平移（训练数据已是闭环矫正输出）
            dx = random.randint(-2, 2)
            dy = random.randint(-2, 2)
            x = torch.roll(x, shifts=(dy, dx), dims=(1, 2))
            if random.random() < 0.3:
                x = torch.flip(x, dims=[2])
        return x, self.labels[i]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=str, required=True, help="数据集批次目录")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--out", type=str, default=None,
                        help="权重输出路径（默认与数据同目录shape_cnn_best.pt）")
    args = parser.parse_args()

    torch.set_num_threads(args.workers)
    random.seed(0)
    torch.manual_seed(0)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"设备: {device}")

    # 支持多批次：--data 指向 frontal_dataset 根目录时遍历其下 batch_*
    if os.path.exists(os.path.join(args.data, "images")):
        data_dirs = [args.data]
    else:
        data_dirs = sorted(glob.glob(os.path.join(args.data, "batch_*")))
    images = []
    for dd in data_dirs:
        images += sorted(glob.glob(os.path.join(dd, "images", "*.png")))
    images = sorted(images)
    labels = []
    ok = []
    for p in images:
        base = os.path.splitext(os.path.basename(p))[0]
        lp = os.path.join(os.path.dirname(os.path.dirname(p)),
                          "labels", base + ".txt")
        if os.path.exists(lp):
            with open(lp) as f:
                labels.append(int(f.read().split()[0]))
            ok.append(p)
    print(f"数据: {len(ok)} 张, 类别分布: "
          f"{[labels.count(c) for c in range(N_CLASSES)]}")

    # 8:2 划分（同类比例）
    idx = list(range(len(ok)))
    random.shuffle(idx)
    val_idx = idx[len(idx) // 5 * 4:]
    tr_idx = idx[:len(idx) // 5 * 4]
    tr_ds = CardDataset([ok[i] for i in tr_idx], [labels[i] for i in tr_idx], True)
    va_ds = CardDataset([ok[i] for i in val_idx], [labels[i] for i in val_idx], False)
    tr_ld = torch.utils.data.DataLoader(tr_ds, batch_size=args.batch, shuffle=True,
                                        num_workers=0)
    va_ld = torch.utils.data.DataLoader(va_ds, batch_size=args.batch, shuffle=False,
                                        num_workers=0)

    model = ShapeCNN().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    crit = nn.CrossEntropyLoss()

    def evaluate():
        model.eval()
        correct = total = 0
        cm = np.zeros((N_CLASSES, N_CLASSES), int)
        with torch.no_grad():
            for x, y in va_ld:
                x = x.to(device)
                y = y.to(device)
                out = model(x)
                pred = out.argmax(1)
                correct += (pred == y).sum().item()
                total += y.size(0)
                for p, t in zip(pred.tolist(), y.tolist()):
                    cm[t, p] += 1
        return correct / total, cm

    best_acc = 0.0
    for ep in range(args.epochs):
        model.train()
        loss_sum = n = 0
        for x, y in tr_ld:
            x = x.to(device)
            y = y.to(device)
            opt.zero_grad()
            loss = crit(model(x), y)
            loss.backward()
            opt.step()
            loss_sum += loss.item() * len(y)
            n += len(y)
        sched.step()
        acc, cm = evaluate()
        if acc > best_acc:
            best_acc = acc
            out = args.out or os.path.join(args.data, "shape_cnn_best.pt")
            torch.save(model.state_dict(), out)
        if ep % 5 == 0 or ep == args.epochs - 1:
            print(f"epoch {ep:>3}  loss={loss_sum / n:.4f}  val_acc={acc:.3f}  "
                  f"best={best_acc:.3f}")
    print(f"\nBest val_acc: {best_acc:.3f}")
    print("混淆矩阵 (行=真值 0圆1星2方3菱4十5角6背景):")
    print(cm)
    out = args.out or os.path.join(args.data, "shape_cnn_best.pt")
    print(f"权重: {out}")


if __name__ == "__main__":
    main()
