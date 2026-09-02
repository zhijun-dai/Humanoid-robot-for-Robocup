"""ShapeCNN 模型定义 — 训练(train_shape_cnn.py)与推理(shape_detector.py)共用。

输入：96×96 单通道二值图（黑底白线），值域 /255。
输出 logits：索引 0-5 = SHAPES 序（circle/pentagon/square/diamond/cross/triangle），6 = 背景。
"""
import torch
import torch.nn as nn

N_CLASSES = 7  # 6图形 + 背景

CLASS_NAMES = ["circle", "pentagon", "square", "diamond", "cross", "triangle"]


class ShapeCNN(nn.Module):
    """96×96×1 → Conv32→64→128(stride2)+BN+ReLU → GAP → FC(7)，≈0.4M。"""

    def __init__(self, n_classes=N_CLASSES):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, 3, stride=2, padding=1), nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, 3, stride=2, padding=1), nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )
        self.head = nn.Linear(128, n_classes)

    def forward(self, x):
        x = self.features(x)
        x = x.mean(dim=(2, 3))  # GAP
        return self.head(x)
