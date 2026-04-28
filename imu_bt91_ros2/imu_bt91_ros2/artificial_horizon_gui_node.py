import math
import tkinter as tk

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


class ArtificialHorizonGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("artificial_horizon_gui_node")

        self.declare_parameter("euler_topic", "imu/euler_deg")
        self.declare_parameter("window_title", "BT91 Horizonte Artificial")
        self.declare_parameter("window_width", 720)
        self.declare_parameter("window_height", 520)
        self.declare_parameter("pitch_pixels_per_deg", 3.0)

        euler_topic = str(self.get_parameter("euler_topic").value)
        self.window_title = str(self.get_parameter("window_title").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.pitch_pixels_per_deg = float(self.get_parameter("pitch_pixels_per_deg").value)

        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self._running = True

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry(f"{self.window_width}x{self.window_height}")
        self.root.configure(bg="#111111")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.canvas = tk.Canvas(self.root, bg="#1b1b1b", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.info_var = tk.StringVar(value="Roll: 0.00 deg   Pitch: 0.00 deg   Yaw: 0.00 deg")
        tk.Label(
            self.root,
            textvariable=self.info_var,
            font=("Consolas", 12),
            fg="#e6e6e6",
            bg="#111111",
            anchor="w",
            padx=10,
            pady=6,
        ).pack(fill=tk.X)

        self.create_subscription(Vector3Stamped, euler_topic, self._on_euler, 20)
        self.get_logger().info(f"Horizonte GUI listo. Escuchando {euler_topic}")

    def _on_close(self) -> None:
        self._running = False

    def is_running(self) -> bool:
        return self._running

    def _on_euler(self, msg: Vector3Stamped) -> None:
        self.roll_deg = msg.vector.x
        self.pitch_deg = msg.vector.y
        self.yaw_deg = msg.vector.z

    def _draw_pitch_ladder(
        self,
        horizon_x: float,
        horizon_y: float,
        dx: float,
        dy: float,
        nx: float,
        ny: float,
    ) -> None:
        for mark_deg in range(-30, 31, 5):
            if mark_deg == 0:
                continue

            offset = -mark_deg * self.pitch_pixels_per_deg
            line_cx = horizon_x + nx * offset
            line_cy = horizon_y + ny * offset
            half_len = 48.0 if mark_deg % 10 == 0 else 28.0
            line_w = 3 if mark_deg % 10 == 0 else 2

            x1 = line_cx - dx * half_len
            y1 = line_cy - dy * half_len
            x2 = line_cx + dx * half_len
            y2 = line_cy + dy * half_len
            self.canvas.create_line(x1, y1, x2, y2, fill="#f2f2f2", width=line_w)

            if mark_deg % 10 == 0:
                label = str(abs(mark_deg))
                label_offset = half_len + 18.0
                lx1 = line_cx - dx * label_offset
                ly1 = line_cy - dy * label_offset
                lx2 = line_cx + dx * label_offset
                ly2 = line_cy + dy * label_offset
                self.canvas.create_text(lx1, ly1, text=label, fill="#f2f2f2", font=("Arial", 10, "bold"))
                self.canvas.create_text(lx2, ly2, text=label, fill="#f2f2f2", font=("Arial", 10, "bold"))

    def _draw_roll_scale_and_bank(self, cx: float, cy: float, width: float, height: float) -> None:
        radius = min(width, height) * 0.38
        self.canvas.create_arc(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            start=30,
            extent=120,
            style=tk.ARC,
            outline="#f2f2f2",
            width=2,
        )

        for angle_deg in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            ang = math.radians(angle_deg - 90.0)
            outer_r = radius
            inner_r = radius - (16.0 if angle_deg % 30 == 0 else 10.0)
            ox = cx + outer_r * math.cos(ang)
            oy = cy + outer_r * math.sin(ang)
            ix = cx + inner_r * math.cos(ang)
            iy = cy + inner_r * math.sin(ang)
            self.canvas.create_line(ix, iy, ox, oy, fill="#f2f2f2", width=2)

        bank_ang = math.radians(self.roll_deg - 90.0)
        ind_r = radius - 20.0
        tip_x = cx + ind_r * math.cos(bank_ang)
        tip_y = cy + ind_r * math.sin(bank_ang)
        left_x = tip_x + 7.0 * math.cos(bank_ang + math.pi * 0.5)
        left_y = tip_y + 7.0 * math.sin(bank_ang + math.pi * 0.5)
        right_x = tip_x + 7.0 * math.cos(bank_ang - math.pi * 0.5)
        right_y = tip_y + 7.0 * math.sin(bank_ang - math.pi * 0.5)
        base_x = tip_x + 14.0 * math.cos(bank_ang + math.pi)
        base_y = tip_y + 14.0 * math.sin(bank_ang + math.pi)
        self.canvas.create_polygon(
            tip_x,
            tip_y,
            left_x,
            left_y,
            base_x,
            base_y,
            right_x,
            right_y,
            fill="#f4d35e",
            outline="",
        )

        top_x = cx
        top_y = cy - radius + 6.0
        self.canvas.create_polygon(
            top_x,
            top_y,
            top_x - 9.0,
            top_y - 15.0,
            top_x + 9.0,
            top_y - 15.0,
            fill="#f2f2f2",
            outline="",
        )

    def _draw_horizon(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        cx = width * 0.5
        cy = height * 0.5

        roll_rad = math.radians(self.roll_deg)
        pitch_px = self.pitch_deg * self.pitch_pixels_per_deg

        dx = math.cos(roll_rad)
        dy = math.sin(roll_rad)
        nx = -dy
        ny = dx

        horizon_x = cx
        horizon_y = cy + pitch_px
        ext = max(width, height) * 3.0

        def p(tx: float, ty: float) -> tuple[float, float]:
            return horizon_x + tx, horizon_y + ty

        sky = [
            p(dx * ext + nx * ext, dy * ext + ny * ext),
            p(-dx * ext + nx * ext, -dy * ext + ny * ext),
            p(-dx * ext, -dy * ext),
            p(dx * ext, dy * ext),
        ]
        ground = [
            p(dx * ext, dy * ext),
            p(-dx * ext, -dy * ext),
            p(-dx * ext - nx * ext, -dy * ext - ny * ext),
            p(dx * ext - nx * ext, dy * ext - ny * ext),
        ]

        self.canvas.create_polygon(sky, fill="#4b82d9", outline="")
        self.canvas.create_polygon(ground, fill="#8a5a2b", outline="")
        self.canvas.create_line(
            horizon_x - dx * ext,
            horizon_y - dy * ext,
            horizon_x + dx * ext,
            horizon_y + dy * ext,
            fill="#f3f3f3",
            width=3,
        )
        self._draw_pitch_ladder(horizon_x, horizon_y, dx, dy, nx, ny)

        wing = width * 0.16
        self.canvas.create_line(cx - wing, cy, cx + wing, cy, fill="#f4d35e", width=4)
        self.canvas.create_line(cx, cy - 18, cx, cy + 18, fill="#f4d35e", width=3)
        self.canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#f4d35e", outline="")
        self._draw_roll_scale_and_bank(cx, cy, width, height)

        self.info_var.set(
            f"Roll: {self.roll_deg: .2f} deg   Pitch: {self.pitch_deg: .2f} deg   Yaw: {self.yaw_deg: .2f} deg"
        )

    def update_ui(self) -> None:
        self._draw_horizon()
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self._running = False

    def destroy(self) -> None:
        if self.root.winfo_exists():
            self.root.destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArtificialHorizonGuiNode()
    try:
        while rclpy.ok() and node.is_running():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.update_ui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
