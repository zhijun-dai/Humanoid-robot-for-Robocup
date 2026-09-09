"""生成 6 个标准图形打印页（TikZ/LaTeX）。

每个图形外沿最大跨度 10cm，线宽 0.5cm（比赛规则）。
中心线路径跨度 9.5cm，加 0.5cm 线宽后外沿 = 10cm。
图形形状按识别逻辑可正确判别的标准几何定义。
"""
import math
import os

OUT = r"C:\Users\Lenovo\AppData\Local\Temp\shapes_10cm.tex"
LINE_W = 0.5
S = 10.0 - LINE_W          # 中心线跨度 9.5cm
R = S / 2


def poly(pts):
    coords = " -- ".join(f"({x:.4f},{y:.4f})" for x, y in pts)
    return f"\\draw[shapeline] {coords} -- cycle;"


# ── 各图形中心线坐标（cm，中心为原点）──
# 圆形
d_circle = f"\\draw[shapeline] (0,0) circle ({R:.4f}cm);"

# 正方形：边长 9.5
h = S / 2
d_square = poly([(-h, -h), (h, -h), (h, h), (-h, h)])

# 等边三角形：边长 9.5，顶点朝上，重心居中
a = S
H = a * math.sqrt(3) / 2
d_tri = poly([(0, 2 * H / 3), (-a / 2, -H / 3), (a / 2, -H / 3)])

# 五角星：标准五角星，最大跨度 = 水平宽度 9.5cm（宽 = 2R·sin72°）
R_star = S / (2 * math.sin(math.radians(72)))
r_in = R_star * math.sin(math.radians(18)) / math.sin(math.radians(126))
star = []
for i in range(10):
    ang = math.pi / 2 + i * math.pi / 5
    rad = R_star if i % 2 == 0 else r_in
    star.append((rad * math.cos(ang), rad * math.sin(ang)))
d_star = poly(star)

# 菱形：正方形旋转 45°（对角线相等）→ 识别逻辑 minAreaRect 角度 45° 判菱形
d_rhom = poly([(R, 0), (0, R), (-R, 0), (0, -R)])

# 十字形：等臂，臂宽 1.5cm（中心线）→ 视觉臂宽 2.0cm，
# 填充率 ≈ 0.36 < 0.42（识别逻辑十字判据）
t = 0.75
L = R
d_cross = poly([(-t, L), (t, L), (t, t), (L, t), (L, -t), (t, -t),
                (t, -L), (-t, -L), (-t, -t), (-L, -t), (-L, t), (-t, t)])

SHAPES = [
    ("圆形", d_circle),
    ("五角星", d_star),
    ("正方形", d_square),
    ("菱形", d_rhom),
    ("十字形", d_cross),
    ("三角形", d_tri),
]

# ── 布局：A3 横向 42×29.7cm（文字区 40×27.7cm），3 列 × 2 行 ──
positions = []
for row in range(2):
    for col in range(3):
        positions.append((6.5 + col * 13.5, -6.5 - row * 13.0))

body = []
for (label, draw), (px, py) in zip(SHAPES, positions):
    body.append(
        f"    \\begin{{scope}}[shift={{({px},{py})}}]\n"
        f"      {draw}\n"
        f"      \\node[font=\\small] at (0,-6.0) {{{label}}};\n"
        f"    \\end{{scope}}"
    )

tex = r"""\documentclass[10pt]{ctexart}
\usepackage[a3paper,landscape,margin=1cm]{geometry}
\usepackage{tikz}
\pagestyle{empty}
\setlength{\parindent}{0pt}

% 比赛图形卡：每个图形外沿最大跨度 10cm，线宽 0.5cm
\tikzset{shapeline/.style={line width=0.5cm, line join=round, black}}

\begin{document}
\begin{tikzpicture}[x=1cm, y=1cm]
@BODY@
\end{tikzpicture}
\end{document}
""".replace("@BODY@", "\n".join(body))

with open(OUT, "w", encoding="utf-8") as f:
    f.write(tex)
print(f"已生成: {OUT}")
for label, _ in SHAPES:
    print(f"  - {label}")
