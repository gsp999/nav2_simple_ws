"""Generate red/blue PGM maps — explicit per-team obstacles.

Red obstacles (Y negative half, origin [0, -6, 0]):
  梅林区:      X∈[3.2, 8.0], Y∈[-4.8, -1.2]
  竞技区高台:   X=9.45, Y∈[-4.5, 0.0]

Blue obstacles (Y positive half, origin [0, 0, 0]):
  梅林区:      X∈[3.2, 8.0], Y∈[1.2, 4.8]
  竞技区高台:   X=9.45, Y∈[0.0, 4.5]

斜坡 coords (ramp_zone only, NOT in PGM):
  Red:  X∈[9.3, 10.8], Y∈[-6.0, -4.5]
  Blue: X∈[9.3, 10.8], Y∈[4.5, 6.0]
"""

import numpy as np, os

RES = 0.05; W, H = 240, 120
FREE, WALL = 254, 0

def blank(): return np.full((H, W), FREE, dtype=np.uint8)
def m2px(v): return int(v / RES)

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
    oy = 0.0

    # outer boundary
    hline(img, 0.0,   0.0, 6.0, oy, 30)    # top
    hline(img, 11.97, 0.0, 6.0, oy, 30)    # bottom
    vline(img, 5.97,  0.0, 11.97, oy, 30)  # right edge

    # center wall Y=0 (left edge) — thin only, no thick wall in 武馆
    vline(img, 0.015, 0.0, 11.97, oy, 30)

    # 梅林区
    rect(img, 3.2, 8.0, 1.2, 4.8, oy)

    # 竞技区高台 wall
    rect(img, 9.45, 9.50, 0.0, 4.5, oy)

    return img

def generate_red():
    img = blank()
    oy = -6.0

    # outer boundary
    hline(img, 0.0,   -6.0, 0.0,   oy, 30)  # top
    hline(img, 11.97, -6.0, 0.0,   oy, 30)  # bottom
    vline(img, -5.97, 0.0,  11.97, oy, 30)  # left edge

    # center wall Y=0 (right edge) — thin only, no thick wall in 武馆
    vline(img, -0.015, 0.0, 11.97, oy, 30)

    # 梅林区
    rect(img, 3.2, 8.0, -4.8, -1.2, oy)

    # 竞技区高台 wall
    rect(img, 9.45, 9.50, -4.5, 0.0, oy)

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
    for n, pgm, ox, oy in [('field_blue','field_blue.pgm',0,0),
                            ('field_red','field_red.pgm',0,-6),
                            ('field','field.pgm',0,0)]:
        with open(os.path.join(out, f'{n}.yaml'), 'w') as f:
            f.write(tpl.format(pgm=pgm, ox=ox, oy=oy))

    print(f'Blue: wall={np.sum(blue==0):5d}  free={np.sum(blue==254):5d}')
    print(f'Red:  wall={np.sum(red==0):5d}  free={np.sum(red==254):5d}')
    print(f'Done → {out}/')
