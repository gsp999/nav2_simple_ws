#!/usr/bin/env python3
"""Interactive map visualization + path planning tool for Nav2 GoToPose.

Features:
  - Display PGM costmap with world-coordinate grid overlay
  - Click on map or type coordinates to set start/goal
  - Offline A* path planning on the costmap
  - Switch between red/blue/custom maps
  - Configure origin, robot radius, and other parameters

Usage:
  python map_viz_tool.py --team red
  python map_viz_tool.py --team blue
  python map_viz_tool.py --map /path/to/custom.yaml
  python map_viz_tool.py --team red --online   # ROS 2 mode (if rclpy available)
"""

import argparse
import heapq
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation

import matplotlib
matplotlib.use("TkAgg")  # must be before pyplot import
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# Configure CJK-supporting font for Chinese labels
# Try common CJK font families; fall back gracefully if none available
_CJK_FAMILIES = [
    "Noto Sans CJK SC", "Noto Sans CJK", "WenQuanYi Micro Hei",
    "AR PL UMing CN", "AR PL UKai CN", "SimHei", "Microsoft YaHei",
]
_AVAILABLE_FONTS = {f.name for f in fm.fontManager.ttflist}
_CJK_FAMILY = None
for fam in _CJK_FAMILIES:
    if fam in _AVAILABLE_FONTS:
        _CJK_FAMILY = fam
        break

if _CJK_FAMILY:
    plt.rcParams["font.family"] = _CJK_FAMILY
    # Clear font cache so changes take effect
    fm._load_fontmanager(try_read_cache=False)
from matplotlib.backend_bases import MouseButton
from matplotlib.widgets import Button, RadioButtons, TextBox

# ── Constants ──────────────────────────────────────────────────────────────
RESOLUTION = 0.05          # m/pixel (hardcoded — our maps are always 0.05)
PGM_WIDTH, PGM_HEIGHT = 240, 120  # pixels per half-map
FIELD_WIDTH = PGM_WIDTH * RESOLUTION   # 12.0 m
FIELD_HEIGHT = PGM_HEIGHT * RESOLUTION # 6.0 m
DEFAULT_ROBOT_RADIUS = 0.20  # meters

# ── Package directory resolution ───────────────────────────────────────────
_PKG_DIR = Path(__file__).resolve().parent.parent  # nav2_pose_navigator package root
_MAPS_DIR = _PKG_DIR / "maps"


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  MAP DATA                                                               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class MapData:
    """Load a PGM+YAML pair and provide coordinate transforms."""

    def __init__(self, yaml_path: str):
        self.yaml_path = yaml_path
        self._parse_yaml(yaml_path)
        self._load_pgm()

    def _parse_yaml(self, yaml_path: str):
        """Read YAML — simple line-based parser (avoids pyyaml dependency)."""
        self.image_file = None
        self.resolution = RESOLUTION
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.occupied_thresh = 0.65
        self.free_thresh = 0.196

        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        with open(yaml_path) as f:
            for line in f:
                line = line.strip()
                if line.startswith("#") or ":" not in line:
                    continue
                key, _, val = line.partition(":")
                key, val = key.strip(), val.strip()
                if key == "image":
                    self.image_file = os.path.join(yaml_dir, val)
                elif key == "resolution":
                    self.resolution = float(val)
                elif key == "origin":
                    parts = [x.strip() for x in val.strip("[]").split(",")]
                    self.origin_x = float(parts[0])
                    self.origin_y = float(parts[1])
                elif key == "occupied_thresh":
                    self.occupied_thresh = float(val)
                elif key == "free_thresh":
                    self.free_thresh = float(val)

        if self.image_file is None:
            raise FileNotFoundError(f"No 'image' field in {yaml_path}")

    def _load_pgm(self):
        """Load PGM P5 binary. Returns uint8 numpy array (H×W)."""
        with open(self.image_file, "rb") as f:
            header = f.readline().strip()
            if header not in (b"P5", b"P2"):
                raise ValueError(f"Unsupported PGM format: {header}")

            # Skip comments
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()

            width, height = map(int, line.split())
            maxval = int(f.readline().strip())

            data = np.frombuffer(f.read(), dtype=np.uint8 if maxval < 256 else np.uint16)
            self.image = data.reshape((height, width)).astype(np.uint8)

        self.width = self.image.shape[1]
        self.height = self.image.shape[0]

    # ── computed properties ────────────────────────────────────────────
    @property
    def x_min(self): return self.origin_x

    @property
    def x_max(self): return self.origin_x + self.width * self.resolution

    @property
    def y_min(self): return self.origin_y

    @property
    def y_max(self): return self.origin_y + self.height * self.resolution

    @property
    def extent(self):
        """extent for imshow(..., origin='upper'): [xmin, xmax, ymin, ymax]."""
        return [self.x_min, self.x_max, self.y_min, self.y_max]

    # ── coordinate transforms ──────────────────────────────────────────
    def world_to_pixel(self, wx: float, wy: float):
        """World coords → pixel indices (col, row)."""
        col = int((wx - self.origin_x) / self.resolution)
        row = int((self.y_max - wy) / self.resolution)
        col = max(0, min(self.width - 1, col))
        row = max(0, min(self.height - 1, row))
        return col, row

    def pixel_to_world(self, px: int, py: int):
        """Pixel indices → world coords (pixel center)."""
        wx = self.origin_x + (px + 0.5) * self.resolution
        wy = self.y_max - (py + 0.5) * self.resolution
        return wx, wy

    def is_in_bounds(self, wx: float, wy: float) -> bool:
        return (self.x_min <= wx <= self.x_max and
                self.y_min <= wy <= self.y_max)

    # ── obstacle mask for planning ─────────────────────────────────────
    def obstacle_mask(self, robot_radius: float = DEFAULT_ROBOT_RADIUS):
        """Return bool array (True=obstacle) for the costmap, with inflation."""
        # Pixel values: 0=obstacle (black), 254=free (white)
        # Use occupied_thresh to decide
        occ_val = int(self.occupied_thresh * 100)  # 0.65 → 65
        mask = self.image <= occ_val

        if robot_radius > 0:
            # Inflate obstacles by robot radius
            inflate_px = max(1, int(robot_radius / self.resolution))
            struct = np.ones((2 * inflate_px + 1, 2 * inflate_px + 1))
            mask = binary_dilation(mask, structure=struct)

        return mask


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  A* PLANNER                                                             ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class AStarPlanner:
    """8-connected A* grid planner on the obstacle mask."""

    # 8-connected neighbor offsets (col, row, cost)
    _NEIGHBORS = [
        (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0,  1, 1.0),
        (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
    ]

    def __init__(self, map_data: MapData, robot_radius: float = DEFAULT_ROBOT_RADIUS):
        self.map_data = map_data
        self.mask = map_data.obstacle_mask(robot_radius)
        self._resolution = map_data.resolution

    def plan(self, start_wx: float, start_wy: float,
             goal_wx: float, goal_wy: float) -> list:
        """Run A* from start to goal. Returns list of (wx, wy) world coords, or empty."""
        sc, sr = self.map_data.world_to_pixel(start_wx, start_wy)
        gc, gr = self.map_data.world_to_pixel(goal_wx, goal_wy)

        if self.mask[sr, sc]:
            # Start inside obstacle — find nearest free cell
            sc, sr = self._nearest_free(sc, sr)
        if self.mask[gr, gc]:
            gc, gr = self._nearest_free(gc, gr)
        if self.mask[sr, sc] or self.mask[gr, gc]:
            return []  # truly stuck

        open_set = []
        heapq.heappush(open_set, (0.0, 0, sc, sr))  # (f, tiebreak, col, row)
        g_score = {(sc, sr): 0.0}
        came_from = {}
        closed = set()

        while open_set:
            _, _, cx, cy = heapq.heappop(open_set)
            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if (cx, cy) == (gc, gr):
                return self._reconstruct(came_from, (gc, gr))

            for dx, dy, cost in self._NEIGHBORS:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= self.map_data.width or ny < 0 or ny >= self.map_data.height:
                    continue
                if (nx, ny) in closed:
                    continue
                if self.mask[ny, nx]:
                    continue

                tentative_g = g_score[(cx, cy)] + cost
                if tentative_g < g_score.get((nx, ny), float("inf")):
                    g_score[(nx, ny)] = tentative_g
                    h = self._heuristic(nx, ny, gc, gr)
                    f = tentative_g + h * 1.001  # slight tie-break toward goal
                    came_from[(nx, ny)] = (cx, cy)
                    heapq.heappush(open_set, (f, len(closed), nx, ny))

        return []  # no path found

    def _heuristic(self, cx, cy, gx, gy):
        return np.hypot(cx - gx, cy - gy)

    def _reconstruct(self, came_from: dict, goal: tuple) -> list:
        path_px = []
        cur = goal
        while cur in came_from:
            path_px.append(cur)
            cur = came_from[cur]
        path_px.append(cur)
        path_px.reverse()
        return [self.map_data.pixel_to_world(c, r) for c, r in path_px]

    def _nearest_free(self, col: int, row: int) -> tuple:
        """BFS to find nearest non-obstacle cell."""
        q = deque([(col, row)])
        visited = {(col, row)}
        while q:
            c, r = q.popleft()
            if not self.mask[r, c]:
                return c, r
            for dx, dy, _ in self._NEIGHBORS[:4]:  # 4-connected for BFS
                nc, nr = c + dx, r + dy
                if 0 <= nc < self.map_data.width and 0 <= nr < self.map_data.height:
                    if (nc, nr) not in visited:
                        visited.add((nc, nr))
                        q.append((nc, nr))
        return col, row  # fallback


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  MAP VISUALIZER (matplotlib GUI)                                        ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class MapVisualizer:
    """Interactive matplotlib-based map viewer with path planning."""

    def __init__(self, yaml_path: str, team: str = "red", online: bool = False,
                 robot_radius: float = DEFAULT_ROBOT_RADIUS):
        self.map_data = MapData(yaml_path)
        self.team = team
        self.online = online
        self.robot_radius = robot_radius
        self.planner = AStarPlanner(self.map_data, robot_radius)

        # State
        self._mode = "goal"       # "start" | "goal" | "none"
        self.start_wx = 2.0       # default start
        self.start_wy = -3.0 if team == "red" else 3.0
        self.goal_wx = None
        self.goal_wy = None
        self.goal_yaw = 0.0
        self.path_xy = None       # list of (wx, wy)
        self.custom_map_path = ""

        # Build UI
        self._build_figure()
        self._connect_events()
        self._update_display()

    # ── Figure construction ────────────────────────────────────────────
    def _build_figure(self):
        """Build the figure with map area + bottom controls.

        Layout (figure-relative coordinates, 0=bottom, 1=top):
          0.205 : top of control area / bottom of map
          0.980 : top of map

          Control area (stacked from bottom):
            0.005-0.055  : row 3 — action buttons
            0.060-0.105  : row 2 — text input fields
            0.110-0.155  : row 1 — team radio + short inputs
            0.160-0.200  : row 0 — status bar
        """
        self.fig = plt.figure("Nav2 Map Viz — GoToPose Planner", figsize=(15, 8.5))
        self.fig.canvas.manager.set_window_title("Nav2 Map Viz Tool")

        # ── Main map axes (bulk of the figure) ──
        self.ax_map = self.fig.add_axes([0.04, 0.205, 0.92, 0.77])
        self._draw_map_base()

        # ── Status bar ──
        ax_status = self.fig.add_axes([0.04, 0.160, 0.92, 0.038])
        ax_status.set_facecolor("#f5f5f5")
        ax_status.set_xticks([])
        ax_status.set_yticks([])
        for spine in ax_status.spines.values():
            spine.set_visible(False)
        self._status_text = ax_status.text(
            0.01, 0.5, "就绪 — 点击地图设置目标，或使用下方控件",
            transform=ax_status.transAxes,
            fontsize=8.5, verticalalignment="center")

        # ── Row 1: team radio + origin + custom map ──
        row1_y, row1_h = 0.112, 0.042

        # Radio buttons (team selection)
        ax_radio = self.fig.add_axes([0.04, row1_y, 0.10, row1_h])
        radio_idx = 0 if self.team == "red" else (1 if self.team == "blue" else 2)
        self._radio_team = RadioButtons(ax_radio, ["红方", "蓝方", "自定义"],
                                         active=radio_idx, activecolor="#4a90d9")
        self._radio_team.on_clicked(self._on_radio_team)

        # Origin X, Y
        ox, oy = self.map_data.origin_x, self.map_data.origin_y
        ax_ox = self.fig.add_axes([0.155, row1_y, 0.065, row1_h])
        self._tb_ox = TextBox(ax_ox, "原点X:", textalignment="center",
                              initial=f"{ox:.1f}")
        self._tb_ox.on_submit(self._on_origin_change)

        ax_oy = self.fig.add_axes([0.225, row1_y, 0.065, row1_h])
        self._tb_oy = TextBox(ax_oy, "原点Y:", textalignment="center",
                              initial=f"{oy:.1f}")
        self._tb_oy.on_submit(self._on_origin_change)

        # Apply origin button
        ax_apply = self.fig.add_axes([0.295, row1_y, 0.055, row1_h])
        self._btn_apply_origin = Button(ax_apply, "应用原点", color="#ffeaa7",
                                         hovercolor="#fab1a0")
        self._btn_apply_origin.label.set_fontsize(7)
        self._btn_apply_origin.on_clicked(self._on_apply_origin)

        # Robot radius
        ax_rad = self.fig.add_axes([0.36, row1_y, 0.06, row1_h])
        self._tb_radius = TextBox(ax_rad, "半径(m):", textalignment="center",
                                  initial=f"{self.robot_radius:.2f}")
        self._tb_radius.on_submit(self._on_radius_change)

        # Custom map path
        ax_map_path = self.fig.add_axes([0.43, row1_y, 0.20, row1_h])
        self._tb_map = TextBox(ax_map_path, "自定义YAML:", textalignment="left",
                               initial="")
        self._tb_map.on_submit(self._on_custom_map)

        # ── Row 2: coordinate inputs (X, Y, Yaw) ──
        row2_y, row2_h = 0.063, 0.042

        ax_x = self.fig.add_axes([0.04, row2_y, 0.08, row2_h])
        self._tb_x = TextBox(ax_x, "目标X:", textalignment="center", initial="7.0")
        self._tb_x.on_submit(self._on_coord_input)

        ax_y = self.fig.add_axes([0.125, row2_y, 0.08, row2_h])
        self._tb_y = TextBox(ax_y, "目标Y:", textalignment="center", initial="0.0")
        self._tb_y.on_submit(self._on_coord_input)

        ax_yaw = self.fig.add_axes([0.21, row2_y, 0.08, row2_h])
        self._tb_yaw = TextBox(ax_yaw, "Yaw:", textalignment="center", initial="0.0")
        self._tb_yaw.on_submit(self._on_coord_input)

        # ── Row 3: action buttons ──
        row3_y, row3_h = 0.008, 0.048

        btn_specs = [
            (0.04, 0.085, "[S] 设起点", self._on_btn_start, "#a8e6cf"),
            (0.13, 0.085, "[G] 设目标", self._on_btn_goal, "#ffd3b6"),
            (0.22, 0.085, "[P] 规划路径", self._on_btn_plan, "#dcedc1"),
            (0.31, 0.085, "[X] 清除", self._on_btn_clear, "#eeeeee"),
            (0.40, 0.085, "[>>] 发送Nav2", self._on_send_nav2, "#74b9ff"),
            (0.49, 0.085, "[Save] 保存路径", self._on_save_path, "#dfe6e9"),
            (0.58, 0.085, "[Q] 退出", self._on_btn_quit, "#ff8b94"),
        ]

        self._action_buttons = []
        for left, width, label, cb, color in btn_specs:
            ax_btn = self.fig.add_axes([left, row3_y, width, row3_h])
            btn = Button(ax_btn, label, color=color, hovercolor="#b0b0b0")
            btn.label.set_fontsize(8)
            btn.on_clicked(cb)
            self._action_buttons.append(btn)

    def _draw_map_base(self):
        """Draw the base map layer — PGM image + grid."""
        self.ax_map.clear()
        extent = self.map_data.extent
        self.ax_map.imshow(self.map_data.image, extent=extent,
                           origin="upper", cmap="gray_r", vmin=0, vmax=255,
                           interpolation="nearest")

        # Coordinate grid
        x_ticks = np.arange(0, 12.01, 1.0)
        y_ticks = np.arange(-6.01, 6.01, 1.0)
        for x in x_ticks:
            self.ax_map.axvline(x, color="gray", alpha=0.25, linewidth=0.5)
        for y in y_ticks:
            self.ax_map.axhline(y, color="gray", alpha=0.25, linewidth=0.5)

        self.ax_map.set_xlabel("X (m) → 武馆→擂台", fontsize=10)
        self.ax_map.set_ylabel("Y (m)", fontsize=10)

        # Set limits with some padding
        x_pad, y_pad = 0.3, 0.3
        self.ax_map.set_xlim(self.map_data.x_min - x_pad, self.map_data.x_max + x_pad)
        self.ax_map.set_ylim(self.map_data.y_min - y_pad, self.map_data.y_max + y_pad)
        self.ax_map.set_aspect("equal")
        self.ax_map.grid(False)

        # Title
        if self.team == "red":
            title = f"[红方] — {os.path.basename(self.map_data.yaml_path)}"
        elif self.team == "blue":
            title = f"[蓝方] — {os.path.basename(self.map_data.yaml_path)}"
        else:
            title = f"[自定义] — {os.path.basename(self.map_data.yaml_path)}"
        self.ax_map.set_title(title, fontsize=12, fontweight="bold")

        # Zone labels
        self._add_zone_labels()

    def _add_zone_labels(self):
        """Add zone labels (武馆, 梅花林, 擂台, 斜坡)."""
        y_mid = (self.map_data.y_min + self.map_data.y_max) / 2
        self.ax_map.text(1.0, y_mid + 0.15, "武馆", ha="center", fontsize=9,
                         color="green", alpha=0.7, style="italic")
        self.ax_map.text(5.6, y_mid + 0.15, "梅花林区", ha="center", fontsize=9,
                         color="orange", alpha=0.7, style="italic")
        self.ax_map.text(10.65, y_mid + 0.15, "擂台", ha="center", fontsize=9,
                         color="red", alpha=0.7, style="italic")

        # Ramp zone marker
        ramp_y_mid = ((4.5 + 6.0) / 2) if self.team == "blue" else ((-6.0 + -4.5) / 2)
        if self.map_data.y_min <= ramp_y_mid <= self.map_data.y_max:
            self.ax_map.axvspan(9.3, 10.8, alpha=0.08, color="green")
            self.ax_map.text(10.05, ramp_y_mid, "斜坡", ha="center", fontsize=8,
                             color="green", alpha=0.6)

    # ── Event connections ──────────────────────────────────────────────
    def _connect_events(self):
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)

    # ── Event handlers ─────────────────────────────────────────────────
    def _on_click(self, event):
        if event.inaxes != self.ax_map:
            return
        if event.button != MouseButton.LEFT:
            return
        if event.xdata is None or event.ydata is None:
            return

        wx, wy = float(event.xdata), float(event.ydata)

        if self._mode == "start":
            self.start_wx, self.start_wy = wx, wy
            self._update_status(f"起点已设置: ({wx:.2f}, {wy:.2f})")
            self._mode = "goal"  # auto-switch to goal after setting start
        elif self._mode == "goal":
            self.goal_wx, self.goal_wy = wx, wy
            self._update_status(f"目标已设置: ({wx:.2f}, {wy:.2f})")
        else:
            # Just show coordinate
            self._update_status(f"点击坐标: ({wx:.2f}, {wy:.2f})")

        self._update_display()

    def _on_mouse_move(self, event):
        if event.inaxes != self.ax_map or event.xdata is None:
            return
        wx, wy = float(event.xdata), float(event.ydata)
        mode_label = {"start": "设起点", "goal": "设目标", "none": "浏览"}.get(self._mode, "浏览")
        info = f"模式: {mode_label} | 坐标: ({wx:.2f}, {wy:.2f})"
        if self.goal_wx is not None:
            dist = np.hypot(wx - self.goal_wx, wy - self.goal_wy)
            info += f" | 距目标: {dist:.2f}m"
        self._update_status(info)

    # ── Button callbacks ───────────────────────────────────────────────
    def _on_btn_start(self, event):
        self._mode = "start"
        self._update_status("模式: 设起点 — 点击地图设置起点位置")

    def _on_btn_goal(self, event):
        self._mode = "goal"
        self._update_status("模式: 设目标 — 点击地图或输入坐标设置目标")

    def _on_btn_plan(self, event):
        if self.goal_wx is None or self.goal_wy is None:
            self._update_status("⚠ 请先设置目标点!")
            return

        self._update_status("正在规划路径…")
        self.fig.canvas.draw_idle()

        t0 = time.perf_counter()
        path = self.planner.plan(self.start_wx, self.start_wy,
                                 self.goal_wx, self.goal_wy)
        elapsed = time.perf_counter() - t0

        if path:
            self.path_xy = path
            length = self._path_length(path)
            self._update_status(
                f"✓ 路径规划完成: {len(path)} 点, 长度 {length:.2f}m, 耗时 {elapsed*1000:.0f}ms")
        else:
            self.path_xy = None
            self._update_status(f"✗ 无可行路径! 耗时 {elapsed*1000:.0f}ms")

        self._update_display()

    def _on_btn_clear(self, event):
        self.goal_wx = None
        self.goal_wy = None
        self.path_xy = None
        self.goal_yaw = 0.0
        self._update_status("已清除目标和路径")
        self._update_display()

    def _on_btn_quit(self, event):
        plt.close(self.fig)

    def _on_radio_team(self, label):
        if label == "红方":
            self.team = "red"
            yaml_path = str(_MAPS_DIR / "field_red.yaml")
        elif label == "蓝方":
            self.team = "blue"
            yaml_path = str(_MAPS_DIR / "field_blue.yaml")
        else:
            self.team = "custom"
            if self.custom_map_path:
                yaml_path = self.custom_map_path
            else:
                self._update_status("⚠ 请在下方的「自定义地图」输入框输入 YAML 路径")
                return

        self._reload_map(yaml_path)

    def _on_coord_input(self, text):
        """User pressed Enter in X/Y/Yaw textbox."""
        try:
            x = float(self._tb_x.text)
            y = float(self._tb_y.text)
            yaw = float(self._tb_yaw.text)
        except ValueError:
            self._update_status("⚠ 坐标格式错误，请输入数字")
            return

        if not self.map_data.is_in_bounds(x, y):
            self._update_status(f"⚠ 坐标 ({x:.2f}, {y:.2f}) 超出地图范围!")
            return

        self.goal_wx = x
        self.goal_wy = y
        self.goal_yaw = yaw
        self._update_status(f"目标已设置: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)")
        self._update_display()

    def _on_origin_change(self, text):
        """Origin textbox changed — just show a note, apply on button click."""
        self._update_status("原点已修改，点击「应用原点」按钮生效")

    def _on_apply_origin(self, event):
        """Reload map with new origin."""
        try:
            new_ox = float(self._tb_ox.text)
            new_oy = float(self._tb_oy.text)
        except ValueError:
            self._update_status("⚠ 原点格式错误")
            return

        # Create a temporary YAML with new origin
        self._reload_with_origin(new_ox, new_oy)

    def _on_custom_map(self, text):
        """Custom map path entered."""
        path = text.strip()
        if not path:
            return
        if not os.path.exists(path):
            self._update_status(f"⚠ 文件不存在: {path}")
            return
        self.custom_map_path = path
        if self.team == "custom":
            self._reload_map(path)

    def _on_radius_change(self, text):
        try:
            self.robot_radius = float(text)
            self.planner = AStarPlanner(self.map_data, self.robot_radius)
            self._update_status(f"机器人半径已更新: {self.robot_radius:.2f}m")
            # Re-plan if there's a goal set
            if self.goal_wx is not None and self.goal_wy is not None:
                self.path_xy = self.planner.plan(
                    self.start_wx, self.start_wy, self.goal_wx, self.goal_wy)
            self._update_display()
        except ValueError:
            self._update_status("⚠ 半径格式错误")

    def _on_send_nav2(self, event):
        """Send goal to Nav2 via /go_to_pose Action (online mode)."""
        if self.goal_wx is None or self.goal_wy is None:
            self._update_status("⚠ 请先设置目标点!")
            return
        self._update_status("正在连接 Nav2… (需要 ROS 2 运行)")
        try:
            self._ros_send_goal()
        except Exception as e:
            self._update_status(f"✗ ROS 发送失败: {e}")

    def _on_save_path(self, event):
        """Save current path to a text file."""
        if not self.path_xy:
            self._update_status("⚠ 没有路径可保存，请先规划路径")
            return
        out_path = os.path.join(os.getcwd(),
                                f"path_{self.team}_{time.strftime('%Y%m%d_%H%M%S')}.txt")
        with open(out_path, "w") as f:
            f.write(f"# Path from ({self.start_wx:.3f},{self.start_wy:.3f}) "
                    f"to ({self.goal_wx:.3f},{self.goal_wy:.3f})\n")
            f.write(f"# Length: {self._path_length(self.path_xy):.3f}m, "
                    f"Points: {len(self.path_xy)}\n")
            for i, (wx, wy) in enumerate(self.path_xy):
                f.write(f"{i:4d}  {wx:.4f}  {wy:.4f}\n")
        self._update_status(f"✓ 路径已保存: {out_path}")

    # ── Display update ─────────────────────────────────────────────────
    def _update_display(self):
        """Redraw the map with all overlays."""
        self._draw_map_base()

        # Start marker (green circle)
        self.ax_map.plot(self.start_wx, self.start_wy, "o", color="green",
                         markersize=10, markeredgecolor="darkgreen",
                         markeredgewidth=2, zorder=10, label="起点")
        self.ax_map.annotate(f"起点\n({self.start_wx:.1f},{self.start_wy:.1f})",
                             (self.start_wx, self.start_wy),
                             textcoords="offset points", xytext=(8, 8),
                             fontsize=7, color="darkgreen", fontweight="bold")

        # Goal marker (red X)
        if self.goal_wx is not None and self.goal_wy is not None:
            self.ax_map.plot(self.goal_wx, self.goal_wy, "X", color="red",
                             markersize=12, markeredgecolor="darkred",
                             markeredgewidth=2, zorder=10, label="目标")
            yaw_arrow_len = 0.5
            dx = yaw_arrow_len * np.cos(self.goal_yaw)
            dy = yaw_arrow_len * np.sin(self.goal_yaw)
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                self.ax_map.arrow(self.goal_wx, self.goal_wy, dx, dy,
                                  head_width=0.15, head_length=0.2,
                                  fc="red", ec="darkred", zorder=10)
            self.ax_map.annotate(
                f"目标\n({self.goal_wx:.1f},{self.goal_wy:.1f})\nyaw={self.goal_yaw:.2f}",
                (self.goal_wx, self.goal_wy),
                textcoords="offset points", xytext=(8, -12),
                fontsize=7, color="darkred", fontweight="bold")

        # Path line
        if self.path_xy:
            px, py = zip(*self.path_xy)
            self.ax_map.plot(px, py, "-", color="blue", linewidth=2.0,
                             alpha=0.8, zorder=5, label="路径")
            # Waypoint dots
            if len(px) > 2:
                step = max(1, len(px) // 20)
                self.ax_map.scatter(px[::step], py[::step], s=8, color="blue",
                                    alpha=0.5, zorder=6)

        # Legend
        self.ax_map.legend(loc="upper right", fontsize=8, framealpha=0.8)

        self.fig.canvas.draw_idle()

    def _update_status(self, msg: str):
        self._status_text.set_text(msg)
        try:
            self.fig.canvas.draw_idle()
        except Exception:
            pass

    # ── Map reloading ──────────────────────────────────────────────────
    def _reload_map(self, yaml_path: str):
        try:
            self.map_data = MapData(yaml_path)
            self.planner = AStarPlanner(self.map_data, self.robot_radius)
            self.goal_wx = None
            self.goal_wy = None
            self.path_xy = None
            # Update origin textboxes
            self._tb_ox.set_val(f"{self.map_data.origin_x:.1f}")
            self._tb_oy.set_val(f"{self.map_data.origin_y:.1f}")
            self._update_status(f"✓ 地图已加载: {os.path.basename(yaml_path)}")
            self._update_display()
        except Exception as e:
            self._update_status(f"✗ 加载地图失败: {e}")

    def _reload_with_origin(self, ox: float, oy: float):
        """Reload map data with a different origin (modifies in-memory)."""
        # We create new map data by re-parsing the YAML and overriding origin
        self.map_data = MapData(self.map_data.yaml_path)
        self.map_data.origin_x = ox
        self.map_data.origin_y = oy
        self.planner = AStarPlanner(self.map_data, self.robot_radius)
        self.goal_wx = None
        self.goal_wy = None
        self.path_xy = None
        self._update_status(f"✓ 原点已更新: [{ox:.2f}, {oy:.2f}, 0.0]")
        self._update_display()

    # ── ROS integration (online mode) ──────────────────────────────────
    def _ros_send_goal(self):
        """Try to send goal via ROS 2 /go_to_pose Action."""
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from nav2_pose_navigator_interfaces.action import GoToPose
        from geometry_msgs.msg import PoseStamped
        from builtin_interfaces.msg import Duration

        if not rclpy.ok():
            rclpy.init(args=[])

        node = Node("map_viz_tool_client", namespace="")
        client = ActionClient(node, GoToPose, "go_to_pose")

        if not client.wait_for_server(timeout_sec=3.0):
            node.destroy_node()
            raise RuntimeError("go_to_pose Action server not available")

        goal = GoToPose.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = node.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = self.goal_wx
        goal.target_pose.pose.position.y = self.goal_wy
        goal.target_pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        import math
        half_yaw = self.goal_yaw / 2.0
        goal.target_pose.pose.orientation.z = math.sin(half_yaw)
        goal.target_pose.pose.orientation.w = math.cos(half_yaw)

        self._update_status("正在发送目标到 Nav2…")
        future = client.send_goal_async(goal)

        # Spin until result
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            node.destroy_node()
            raise RuntimeError("Nav2 rejected the goal")

        self._update_status("✓ 目标已发送到 Nav2，机器人正在导航…")
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)

        result = result_future.result()
        if result.result.success:
            self._update_status("✓ Nav2 导航成功! 已到达目标")
        else:
            self._update_status(f"✗ Nav2 导航失败: {result.result.message}")

        node.destroy_node()

    # ── Helpers ────────────────────────────────────────────────────────
    @staticmethod
    def _path_length(path: list) -> float:
        if len(path) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            total += np.hypot(dx, dy)
        return total

    def show(self):
        plt.show()


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  MAIN                                                                   ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def main(args=None):
    parser = argparse.ArgumentParser(
        description="Nav2 地图可视化 + 路径规划工具")
    parser.add_argument("--team", choices=["red", "blue"], default="red",
                        help="队伍颜色 (决定加载哪个默认地图)")
    parser.add_argument("--map", default="",
                        help="自定义地图 YAML 路径 (优先级高于 --team)")
    parser.add_argument("--online", action="store_true",
                        help="启用 ROS 2 在线模式 (可发送目标到 Nav2)")
    parser.add_argument("--radius", type=float, default=DEFAULT_ROBOT_RADIUS,
                        help=f"机器人半径 (米, 默认 {DEFAULT_ROBOT_RADIUS})")
    args = parser.parse_args(args)

    # Resolve map path
    if args.map:
        yaml_path = args.map
    else:
        yaml_path = str(_MAPS_DIR / f"field_{args.team}.yaml")

    if not os.path.exists(yaml_path):
        print(f"ERROR: Map YAML not found: {yaml_path}", file=sys.stderr)
        sys.exit(1)

    viz = MapVisualizer(yaml_path, team=args.team, online=args.online,
                        robot_radius=args.radius)
    viz.show()


if __name__ == "__main__":
    main()
