import asyncio
import math
import queue
import threading
import tkinter as tk
from bleak import BleakClient

from imu_read import ADDRESS, CHAR_UUID, parse_packet


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def rotate_point(point, roll_deg, pitch_deg, yaw_deg):
    x, y, z = point
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = cy * cp
    ry = cy * sp * sr - sy * cr
    rz = cy * sp * cr + sy * sr

    ux = sy * cp
    uy = sy * sp * sr + cy * cr
    uz = sy * sp * cr - cy * sr

    fx = -sp
    fy = cp * sr
    fz = cp * cr

    return (
        rx * x + ry * y + rz * z,
        ux * x + uy * y + uz * z,
        fx * x + fy * y + fz * z,
    )


class IMUGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("IMU Quadcopter 3D")
        self.root.geometry("980x640")
        self.root.configure(bg="#111111")

        self.data_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.status_var = tk.StringVar(value="Estado: desconectado")

        self.ax_var = tk.StringVar(value="Ax: 0.000 g")
        self.ay_var = tk.StringVar(value="Ay: 0.000 g")
        self.az_var = tk.StringVar(value="Az: 0.000 g")
        self.gx_var = tk.StringVar(value="Gx: 0.000 °/s")
        self.gy_var = tk.StringVar(value="Gy: 0.000 °/s")
        self.gz_var = tk.StringVar(value="Gz: 0.000 °/s")
        self.roll_var = tk.StringVar(value="Roll: 0.00 °")
        self.pitch_var = tk.StringVar(value="Pitch: 0.00 °")
        self.yaw_var = tk.StringVar(value="Yaw: 0.00 °")
        self.q_var = tk.StringVar(value="Quat: (0.000, 0.000, 0.000, 1.000)")

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self._build_ui()
        self._start_ble_thread()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(33, self._update_loop)

    def _build_ui(self):
        left = tk.Frame(self.root, bg="#111111")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        right = tk.Frame(self.root, bg="#111111")
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)

        self.canvas = tk.Canvas(left, width=680, height=620, bg="#1b1b1b", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        title = tk.Label(
            right,
            text="Datos IMU",
            font=("Arial", 16, "bold"),
            fg="#e8e8e8",
            bg="#111111",
        )
        title.pack(anchor="w", pady=(0, 10))

        for var in [
            self.status_var,
            self.ax_var,
            self.ay_var,
            self.az_var,
            self.gx_var,
            self.gy_var,
            self.gz_var,
            self.roll_var,
            self.pitch_var,
            self.yaw_var,
            self.q_var,
        ]:
            tk.Label(
                right,
                textvariable=var,
                font=("Consolas", 12),
                fg="#d6d6d6",
                bg="#111111",
                anchor="w",
                justify=tk.LEFT,
            ).pack(anchor="w", pady=2)

    def _start_ble_thread(self):
        thread = threading.Thread(target=self._ble_runner, daemon=True)
        thread.start()

    def _ble_runner(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._ble_task())
        finally:
            loop.close()

    async def _ble_task(self):
        self.status_queue.put("Estado: conectando...")
        async with BleakClient(ADDRESS) as client:
            if not client.is_connected:
                self.status_queue.put("Estado: no se pudo conectar")
                return

            self.status_queue.put("Estado: conectado")

            def callback(sender, data):
                parsed = parse_packet(data)
                if parsed is None:
                    return
                self.data_queue.put(parsed)

            await client.start_notify(CHAR_UUID, callback)
            try:
                while not self.stop_event.is_set():
                    await asyncio.sleep(0.05)
            finally:
                await client.stop_notify(CHAR_UUID)
                self.status_queue.put("Estado: desconectado")

    def _project(self, point):
        x, y, z = point
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        fov = 450.0
        dist = 450.0
        scale = fov / (dist + z + 300.0)
        sx = x * scale + width * 0.5
        sy = -y * scale + height * 0.5
        return sx, sy

    def _draw_model(self):
        self.canvas.delete("all")

        arm = 140.0
        rotor = 18.0
        body_points = {
            "center": (0.0, 0.0, 0.0),
            "front_left": (-arm, arm, 0.0),
            "front_right": (arm, arm, 0.0),
            "rear_left": (-arm, -arm, 0.0),
            "rear_right": (arm, -arm, 0.0),
            "up": (0.0, 0.0, 120.0),
        }

        world = {k: rotate_point(v, self.roll, self.pitch, self.yaw) for k, v in body_points.items()}

        def draw_line(a, b, color, width=3):
            x1, y1 = self._project(world[a])
            x2, y2 = self._project(world[b])
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width)

        def draw_rotor(p, color):
            cx, cy = self._project(world[p])
            self.canvas.create_oval(cx - rotor, cy - rotor, cx + rotor, cy + rotor, outline=color, width=2)

        draw_line("front_left", "rear_right", "#39a7ff", 4)
        draw_line("front_right", "rear_left", "#39a7ff", 4)
        draw_line("center", "up", "#f0f0f0", 3)

        draw_rotor("front_left", "#ff6b6b")
        draw_rotor("front_right", "#ffd166")
        draw_rotor("rear_left", "#06d6a0")
        draw_rotor("rear_right", "#c77dff")

        ux, uy = self._project(world["up"])
        self.canvas.create_text(ux, uy - 14, text="UP", fill="#f0f0f0", font=("Arial", 10, "bold"))

    def _update_labels(self, d):
        self.ax_var.set(f"Ax: {d['ax']: .3f} g")
        self.ay_var.set(f"Ay: {d['ay']: .3f} g")
        self.az_var.set(f"Az: {d['az']: .3f} g")
        self.gx_var.set(f"Gx: {d['gx']: .3f} °/s")
        self.gy_var.set(f"Gy: {d['gy']: .3f} °/s")
        self.gz_var.set(f"Gz: {d['gz']: .3f} °/s")
        self.roll_var.set(f"Roll: {d['roll']: .2f} °")
        self.pitch_var.set(f"Pitch: {d['pitch']: .2f} °")
        self.yaw_var.set(f"Yaw: {d['yaw']: .2f} °")

        qx, qy, qz, qw = euler_to_quaternion(d["roll"], d["pitch"], d["yaw"])
        self.q_var.set(f"Quat: ({qx: .3f}, {qy: .3f}, {qz: .3f}, {qw: .3f})")

    def _update_loop(self):
        while True:
            try:
                self.status_var.set(self.status_queue.get_nowait())
            except queue.Empty:
                break

        latest = None
        while True:
            try:
                latest = self.data_queue.get_nowait()
            except queue.Empty:
                break

        if latest is not None:
            self.roll = latest["roll"]
            self.pitch = latest["pitch"]
            self.yaw = latest["yaw"]
            self._update_labels(latest)

        self._draw_model()
        self.root.after(33, self._update_loop)

    def _on_close(self):
        self.stop_event.set()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    IMUGUI().run()
