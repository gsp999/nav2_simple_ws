"""Generate red/blue PGM maps — explicit per-team obstacles.

注意：以下尺寸为"加宽后"的避障尺寸，非场地真实尺寸。
真实梅林区 X∈[2.8,7.6]，真实平台墙在 Y=±3.1 处为单线。
为防碰撞，将障碍物向外扩展约 0.2m。

Red obstacles (Y negative half, origin [-0.4, -4.6, 0]):
  梅林区(加宽):  X∈[2.6, 7.8], Y∈[-3.6, 0.4]   (真实: X∈[2.8,7.6], Y∈[-3.4,0.2])
  竞技区高台:   X=9.05, Y∈[-3.1, 1.4]

Blue obstacles (Y positive half, origin [-0.4, -1.4, 0]):
  梅林区(加宽):  X∈[2.6, 7.8], Y∈[-0.4, 3.6]   (真实: X∈[2.8,7.6], Y∈[-0.2,3.4])
  竞技区高台:   X=9.05, Y∈[-1.4, 3.1]

抬高平台边界墙 (raised platform edge wall, IN PGM, 加宽为矩形区):
  Red:  X∈[8.9, 10.4], Y∈[-3.3, -3.1]   (真实: Y=-3.1 单线)
  Blue: X∈[8.9, 10.4], Y∈[3.1, 3.3]     (真实: Y=3.1 单线)
斜坡区域 (ramp_zone only, NOT in PGM):
  Red:  X∈[8.9, 10.4], Y∈[-4.6, -3.1]
  Blue: X∈[8.9, 10.4], Y∈[3.1, 4.6]

地图外边界:
  PGM 图像四周像素强制刷成 WALL，确保 Nav2 把地图边界当作障碍物。
"""

import numpy as np, os

RES = 0.05; W, H = 240, 120
FREE, WALL = 254, 0
BORDER_THICKNESS_M = 0.05

def blank(): return np.full((H, W), FREE, dtype=np.uint8)
def m2px(v): return int(v / RES)
def border_px(): return max(1, int(round(BORDER_THICKNESS_M / RES)))

def map_border(img):
    """Mark the PGM image boundary as occupied."""
    t = border_px()
    img[:t, :] = WALL
    img[-t:, :] = WALL
    img[:, :t] = WALL
    img[:, -t:] = WALL

def rect(img, x1, x2, y1, y2, origin_y):
    """Fill [x1,x2]×[min(y1,y2), max(y1,y2)]."""
    c1 = max(0, m2px(x1)); c2 = min(W-1, m2px(x2))
    y_lo = min(y1, y2); y_hi = max(y1, y2)
    r_top = max(0, min(H-1, 119 - m2px(y_hi - origin_y)))
    r_bot = max(0, min(H-1, 119 - m2px(y_lo - origin_y)))
    if r_top > r_bot: r_top, r_bot = r_bot, r_top
    img[r_top:r_bot+1, c1:c2+1] = WALL

def vline(img, y_w, x1, x2, oy, t=30):
    t = max(1, int(t*0.001/RES))
    y_px = 119 - m2px(y_w - oy)
    for d in range(t):
        yy = max(0, min(H-1, y_px - d))
        img[yy, m2px(x1):m2px(x2)+1] = WALL

def hline(img, x_w, y1, y2, oy, t=30):
    t = max(1, int(t*0.001/RES))
    y_lo, y_hi = min(y1,y2), max(y1,y2)
    r1 = 119 - m2px(y_hi - oy); r2 = 119 - m2px(y_lo - oy)
    if r1 > r2: r1, r2 = r2, r1
    for d in range(t):
        img[r1:r2+1, max(0, min(W-1, m2px(x_w)+d))] = WALL

# ============================================================
def generate_blue():
    img = blank()
    oy = -1.4

    # Image/map boundary: hard obstacle at all four PGM edges.
    map_border(img)

    # field boundary hints in world coordinates
    hline(img, -0.4,  -1.4, 4.6,  oy, 30)   # left edge
    hline(img, 11.57, -1.4, 4.6,  oy, 30)   # right edge
    vline(img, 4.57,  -0.4, 11.57, oy, 30)  # top edge

    # center wall Y=0 — thin only, no thick wall in 武馆
    vline(img, 0.015, -0.4, 11.57, oy, 30)

    # 梅林区 (加宽: 真实 X∈[2.8,7.6], Y∈[-0.2,3.4])
    rect(img, 2.6, 7.8, -0.4, 3.6, oy)

    # 竞技区高台 wall
    rect(img, 9.05, 9.10, -1.4, 3.1, oy)

    # 抬高平台边界墙 (加宽为矩形: 真实 Y=3.1 单线)
    rect(img, 8.9, 10.4, 3.1, 3.3, oy)

    return img

def generate_red():
    img = blank()
    oy = -4.6

    # Image/map boundary: hard obstacle at all four PGM edges.
    map_border(img)

    # field boundary hints in world coordinates
    hline(img, -0.4,  -4.6, 1.4,  oy, 30)   # left edge
    hline(img, 11.57, -4.6, 1.4,  oy, 30)   # right edge
    vline(img, 1.37,  -0.4, 11.57, oy, 30)  # top edge (Y=1.4 is top of red map)

    # center wall Y=0 — thin only, no thick wall in 武馆
    vline(img, -0.015, -0.4, 11.57, oy, 30)

    # 梅林区 (加宽: 真实 X∈[2.8,7.6], Y∈[-3.4,0.2])
    rect(img, 2.6, 7.8, -3.6, 0.4, oy)

    # 竞技区高台 wall
    rect(img, 9.05, 9.10, -3.1, 1.4, oy)

    # 抬高平台边界墙 (加宽为矩形: 真实 Y=-3.1 单线)
    rect(img, 8.9, 10.4, -3.3, -3.1, oy)

    return img

# ============================================================
def save_pgm(img, path):
    with open(path, 'wb') as f:
        f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
        f.write(img.tobytes())

if __name__ == '__main__':
    out = os.path.join(os.path.dirname(__file__), 'src/nav2_pose_navigator/maps')
    os.makedirs(out, exist_ok=True)

    blue, red = generate_blue(), generate_red()
    save_pgm(blue, os.path.join(out, 'field_blue.pgm'))
    save_pgm(red,  os.path.join(out, 'field_red.pgm'))
    save_pgm(blue.copy(), os.path.join(out, 'field.pgm'))

    tpl = 'image: {pgm}\nresolution: 0.05\norigin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
    for n, pgm, ox, oy in [('field_blue','field_blue.pgm',-0.4,-1.4),
                            ('field_red','field_red.pgm',-0.4,-4.6),
                            ('field','field.pgm',-0.4,-1.4)]:
        with open(os.path.join(out, f'{n}.yaml'), 'w') as f:
            f.write(tpl.format(pgm=pgm, ox=ox, oy=oy))

    print(f'Blue: wall={np.sum(blue==0):5d}  free={np.sum(blue==254):5d}')
    print(f'Red:  wall={np.sum(red==0):5d}  free={np.sum(red==254):5d}')
    print(f'Done → {out}/')
