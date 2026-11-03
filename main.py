import math
import tkinter as tk

from kinematics import (
    SwerveParams,
    SwerveResult,
    calculate_swerve,
    rotate_2d,
    world_to_screen,
)

BG_MAIN = "#18181b"
BG_PANEL = "#27272a"
BG_CANVAS = "#09090b"
BG_INPUT = "#18181b"
BORDER_COLOR = "#3f3f46"
TEXT_MAIN = "#f4f4f5"
TEXT_MUTED = "#a1a1aa"
COLOR_WHEEL_VEC = "#22c55e"
COLOR_YAW = "#f59e0b"
COLOR_LINEAR_VEC = "#38bdf8"
BTN_BG = "#2563eb"
BTN_ACTIVE = "#1d4ed8"


class SwerveDriveApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Swerve Drive Simulator")
        self.root.geometry("1000x650")
        self.root.minsize(880, 560)
        self.root.configure(bg=BG_MAIN)

        self.vars: dict[str, tk.DoubleVar] = {
            "velocity": tk.DoubleVar(value=0.0),
            "alpha_deg": tk.DoubleVar(value=0.0),
            "yaw_deg": tk.DoubleVar(value=0.0),
            "omega": tk.DoubleVar(value=2.0),
            "wheelbase": tk.DoubleVar(value=1.2),
            "track_width": tk.DoubleVar(value=1.2),
        }
        self.entry_vars: dict[str, tk.StringVar] = {}
        self.formats: dict[str, str] = {
            "velocity": "{:.2f}",
            "alpha_deg": "{:.1f}",
            "yaw_deg": "{:.1f}",
            "omega": "{:.2f}",
            "wheelbase": "{:.2f}",
            "track_width": "{:.2f}",
        }
        self.bounds: dict[str, tuple[float, float]] = {
            "velocity": (0.0, 10.0),
            "alpha_deg": (-180.0, 180.0),
            "yaw_deg": (-180.0, 180.0),
            "omega": (-10.0, 10.0),
            "wheelbase": (0.3, 2.5),
            "track_width": (0.3, 2.5),
        }
        self.auto_calc_var = tk.BooleanVar(value=False)
        self.card_labels: dict[str, tk.Label] = {}
        self.current_result: SwerveResult = calculate_swerve(self._read_params())

        self._build_layout()
        self.calculate_and_render(log_to_console=True)

    def _read_params(self) -> SwerveParams:
        return SwerveParams(
            velocity=float(self.vars["velocity"].get()),
            alpha_deg=float(self.vars["alpha_deg"].get()),
            yaw_deg=float(self.vars["yaw_deg"].get()),
            omega=float(self.vars["omega"].get()),
            wheelbase=float(self.vars["wheelbase"].get()),
            track_width=float(self.vars["track_width"].get()),
        )

    def _build_layout(self) -> None:
        container = tk.Frame(self.root, bg=BG_MAIN, padx=16, pady=16)
        container.pack(fill=tk.BOTH, expand=True)
        container.columnconfigure(0, weight=0, minsize=290)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(0, weight=1)

        # Left Panel: Input Sliders + Numeric Entries + Auto Calculate + Buttons
        left_panel = tk.Frame(
            container,
            bg=BG_PANEL,
            padx=16,
            pady=16,
            highlightbackground=BORDER_COLOR,
            highlightthickness=1,
        )
        left_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 16))

        self._add_section_header(left_panel, "MOTION PARAMETERS")
        self._add_slider(left_panel, "velocity", "Velocity (m/s)", 0.0, 10.0, 0.1)
        self._add_slider(left_panel, "alpha_deg", "Alpha (deg)", -180.0, 180.0, 1.0)
        self._add_slider(left_panel, "yaw_deg", "Yaw (deg)", -180.0, 180.0, 1.0)
        self._add_slider(left_panel, "omega", "Omega (rad/s)", -10.0, 10.0, 0.1)

        spacer = tk.Frame(left_panel, bg=BORDER_COLOR, height=1)
        spacer.pack(fill=tk.X, pady=(10, 10))

        self._add_section_header(left_panel, "CHASSIS DIMENSIONS")
        self._add_slider(left_panel, "wheelbase", "wheelbase (m)", 0.3, 2.5, 0.05)
        self._add_slider(left_panel, "track_width", "track_width (m)", 0.3, 2.5, 0.05)

        btn_frame = tk.Frame(left_panel, bg=BG_PANEL)
        btn_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(12, 0))

        auto_chk = tk.Checkbutton(
            btn_frame,
            text="Auto Calculate",
            variable=self.auto_calc_var,
            command=self._on_auto_toggle,
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            activebackground=BG_PANEL,
            activeforeground=TEXT_MAIN,
            selectcolor=BG_INPUT,
            highlightthickness=0,
            bd=0,
            font=("Sans", 10),
            anchor="w",
            cursor="hand2",
        )
        auto_chk.pack(fill=tk.X, pady=(0, 8))

        calc_btn = tk.Button(
            btn_frame,
            text="Calculate",
            command=lambda: self.calculate_and_render(log_to_console=True),
            bg=BTN_BG,
            activebackground=BTN_ACTIVE,
            fg=TEXT_MAIN,
            activeforeground=TEXT_MAIN,
            relief=tk.FLAT,
            font=("Sans", 11, "bold"),
            pady=8,
            cursor="hand2",
        )
        calc_btn.pack(fill=tk.X, pady=(0, 6))

        reset_btn = tk.Button(
            btn_frame,
            text="Reset Defaults",
            command=self.reset_defaults,
            bg=BORDER_COLOR,
            activebackground=BG_MAIN,
            fg=TEXT_MAIN,
            activeforeground=TEXT_MAIN,
            relief=tk.FLAT,
            font=("Sans", 9),
            pady=5,
            cursor="hand2",
        )
        reset_btn.pack(fill=tk.X)

        # Right Panel: 2D Vector Canvas + 4 Wheel Telemetry Cards
        right_panel = tk.Frame(container, bg=BG_MAIN)
        right_panel.grid(row=0, column=1, sticky="nsew")
        right_panel.rowconfigure(0, weight=1)
        right_panel.rowconfigure(1, weight=0)
        right_panel.columnconfigure(0, weight=1)

        canvas_wrapper = tk.Frame(
            right_panel,
            bg=BG_CANVAS,
            highlightbackground=BORDER_COLOR,
            highlightthickness=1,
        )
        canvas_wrapper.grid(row=0, column=0, sticky="nsew", pady=(0, 12))

        self.canvas = tk.Canvas(
            canvas_wrapper,
            bg=BG_CANVAS,
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind(
            "<Configure>", lambda _event: self._draw_canvas(self.current_result)
        )

        cards_frame = tk.Frame(right_panel, bg=BG_MAIN)
        cards_frame.grid(row=1, column=0, sticky="ew")
        for idx, (key, title) in enumerate(
            [
                ("FL", "Wheel A (FL)"),
                ("FR", "Wheel B (FR)"),
                ("BL", "Wheel C (BL)"),
                ("BR", "Wheel D (BR)"),
            ]
        ):
            cards_frame.columnconfigure(idx, weight=1)
            card = tk.Frame(
                cards_frame,
                bg=BG_PANEL,
                padx=12,
                pady=10,
                highlightbackground=BORDER_COLOR,
                highlightthickness=1,
            )
            card.grid(
                row=0,
                column=idx,
                sticky="ew",
                padx=(0 if idx == 0 else 6, 0 if idx == 3 else 6),
            )
            lbl_title = tk.Label(
                card,
                text=title,
                bg=BG_PANEL,
                fg=TEXT_MUTED,
                font=("Sans", 9),
                anchor="w",
            )
            lbl_title.pack(fill=tk.X)

            lbl_val = tk.Label(
                card,
                text="0.00 m/s | 0.0°",
                bg=BG_PANEL,
                fg=TEXT_MAIN,
                font=("Sans", 11, "bold"),
                anchor="w",
            )
            lbl_val.pack(fill=tk.X, pady=(4, 0))
            self.card_labels[key] = lbl_val

    def _add_section_header(self, parent: tk.Widget, title: str) -> None:
        lbl = tk.Label(
            parent,
            text=title,
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=("Sans", 10, "bold"),
            anchor="w",
        )
        lbl.pack(fill=tk.X, pady=(0, 6))

    def _add_slider(
        self,
        parent: tk.Widget,
        key: str,
        label_text: str,
        min_val: float,
        max_val: float,
        resolution: float,
    ) -> None:
        fmt = self.formats[key]
        row = tk.Frame(parent, bg=BG_PANEL)
        row.pack(fill=tk.X, pady=(2, 0))

        lbl_name = tk.Label(
            row,
            text=label_text,
            bg=BG_PANEL,
            fg=TEXT_MUTED,
            font=("Sans", 9),
        )
        lbl_name.pack(side=tk.LEFT)

        entry_var = tk.StringVar(value=fmt.format(self.vars[key].get()))
        self.entry_vars[key] = entry_var

        entry = tk.Entry(
            row,
            textvariable=entry_var,
            width=8,
            justify=tk.RIGHT,
            bg=BG_INPUT,
            fg=TEXT_MAIN,
            insertbackground=TEXT_MAIN,
            relief=tk.FLAT,
            highlightbackground=BORDER_COLOR,
            highlightcolor=BTN_BG,
            highlightthickness=1,
            font=("Sans", 9, "bold"),
        )
        entry.pack(side=tk.RIGHT)

        def commit_entry(_event: object = None) -> None:
            raw = entry_var.get().strip()
            try:
                val = float(raw)
            except ValueError:
                entry_var.set(fmt.format(self.vars[key].get()))
                return
            low, high = self.bounds[key]
            clamped = max(low, min(high, val))
            self.vars[key].set(clamped)
            entry_var.set(fmt.format(clamped))
            if self.auto_calc_var.get():
                self.calculate_and_render(log_to_console=False)

        entry.bind("<Return>", commit_entry)
        entry.bind("<KP_Enter>", commit_entry)
        entry.bind("<FocusOut>", commit_entry)

        def on_slide(val_str: str) -> None:
            entry_var.set(fmt.format(float(val_str)))
            if self.auto_calc_var.get():
                self.calculate_and_render(log_to_console=False)

        scale = tk.Scale(
            parent,
            variable=self.vars[key],
            from_=min_val,
            to=max_val,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            showvalue=False,
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            troughcolor=BG_MAIN,
            highlightthickness=0,
            bd=0,
            sliderrelief=tk.FLAT,
            command=on_slide,
        )
        scale.pack(fill=tk.X, pady=(0, 4))

    def _on_auto_toggle(self) -> None:
        if self.auto_calc_var.get():
            self.calculate_and_render(log_to_console=False)

    def _commit_all_entries(self) -> None:
        for key, entry_var in self.entry_vars.items():
            fmt = self.formats[key]
            raw = entry_var.get().strip()
            try:
                val = float(raw)
            except ValueError:
                entry_var.set(fmt.format(self.vars[key].get()))
                continue
            low, high = self.bounds[key]
            clamped = max(low, min(high, val))
            self.vars[key].set(clamped)
            entry_var.set(fmt.format(clamped))

    def reset_defaults(self) -> None:
        defaults = SwerveParams()
        self.vars["velocity"].set(defaults.velocity)
        self.vars["alpha_deg"].set(defaults.alpha_deg)
        self.vars["yaw_deg"].set(defaults.yaw_deg)
        self.vars["omega"].set(defaults.omega)
        self.vars["wheelbase"].set(defaults.wheelbase)
        self.vars["track_width"].set(defaults.track_width)

        for key, fmt in self.formats.items():
            self.entry_vars[key].set(fmt.format(self.vars[key].get()))

        self.calculate_and_render(log_to_console=True)

    def calculate_and_render(self, log_to_console: bool = True) -> None:
        self._commit_all_entries()
        params = self._read_params()
        self.current_result = calculate_swerve(params)

        for key, wheel in self.current_result.wheels.items():
            self.card_labels[key].config(
                text=f"{wheel.speed:.2f} m/s | {wheel.steering_deg:.1f}°"
            )

        if log_to_console:
            print("-" * 56)
            for key in ("FL", "FR", "BL", "BR"):
                w = self.current_result.wheels[key]
                print(
                    f"{w.label:12s} | Velocity: {w.speed:8.4f} m/s | Steering: {w.steering_deg:7.2f} deg"
                )

        self._draw_canvas(self.current_result)

    def _draw_canvas(self, result: SwerveResult) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 400)
        height = max(self.canvas.winfo_height(), 300)
        cx = width / 2.0
        cy = height / 2.0

        max_dim = max(result.params.wheelbase, result.params.track_width, 0.5)
        scale = min(width, height) * 0.30 / max_dim

        # Legend
        self.canvas.create_text(
            14,
            14,
            anchor="nw",
            text="Green = Wheel Velocity Vector   |   Orange = Robot Heading (Yaw, 0° = +Y)   |   Blue = Linear Velocity (Alpha)",
            fill=TEXT_MUTED,
            font=("Sans", 9),
        )

        # Coordinate axes (+X right, +Y up)
        self.canvas.create_line(
            24, cy, width - 24, cy, fill=BG_PANEL, width=1, dash=(4, 4)
        )
        self.canvas.create_line(
            cx, 36, cx, height - 24, fill=BG_PANEL, width=1, dash=(4, 4)
        )
        self.canvas.create_text(
            width - 30, cy - 10, text="+X", fill=TEXT_MUTED, font=("Sans", 9)
        )
        self.canvas.create_text(
            cx + 16, 42, text="+Y (0°)", fill=TEXT_MUTED, font=("Sans", 9)
        )

        # Rotated Chassis Polygon
        chassis_pts = []
        for wx, wy in result.chassis_corners_world:
            sx, sy = world_to_screen(wx, wy, cx, cy, scale)
            chassis_pts.extend([sx, sy])
        self.canvas.create_polygon(
            *chassis_pts,
            outline=COLOR_YAW,
            fill="",
            width=2,
        )

        # Robot Heading (Yaw) Arrow from Center (Yaw = 0 points along +Y)
        heading_rad = math.radians(90.0 + result.params.yaw_deg)
        front_len = (result.params.wheelbase / 2.0) * 0.75
        fx, fy = rotate_2d(front_len, 0.0, heading_rad)
        fsx, fsy = world_to_screen(fx, fy, cx, cy, scale)
        self.canvas.create_line(
            cx,
            cy,
            fsx,
            fsy,
            fill=COLOR_YAW,
            width=2,
            arrow=tk.LAST,
            arrowshape=(10, 12, 4),
        )
        lbl_fx, lbl_fy = rotate_2d(front_len * 0.55, 0.14 * max_dim, heading_rad)
        lsx, lsy = world_to_screen(lbl_fx, lbl_fy, cx, cy, scale)
        self.canvas.create_text(
            lsx,
            lsy,
            text="FRONT",
            fill=COLOR_YAW,
            font=("Sans", 8, "bold"),
        )

        # Center Linear Velocity Vector (Alpha = 0 points along +Y)
        if result.params.velocity > 1e-6:
            field_alpha_rad = math.radians(90.0 + result.params.alpha_deg)
            lin_len_px = min(40.0 + result.params.velocity * 8.0, 110.0)
            lx_end = cx + math.cos(field_alpha_rad) * lin_len_px
            ly_end = cy - math.sin(field_alpha_rad) * lin_len_px
            self.canvas.create_line(
                cx,
                cy,
                lx_end,
                ly_end,
                fill=COLOR_LINEAR_VEC,
                width=2,
                dash=(4, 2),
                arrow=tk.LAST,
                arrowshape=(10, 12, 4),
            )

        # Determine arrow scaling across all 4 wheels
        max_wheel_speed = max((w.speed for w in result.wheels.values()), default=0.0)

        # Draw each Wheel Module + Velocity Vector
        wheel_half_l = 26.0
        wheel_half_w = 12.0
        for wheel in result.wheels.values():
            wsx, wsy = world_to_screen(wheel.world_x, wheel.world_y, cx, cy, scale)
            angle_rad = math.radians(wheel.world_angle_deg)

            # Wheel rectangle corners in screen pixels
            local_box: tuple[tuple[float, float], ...] = (
                (wheel_half_l, wheel_half_w),
                (wheel_half_l, -wheel_half_w),
                (-wheel_half_l, -wheel_half_w),
                (-wheel_half_l, wheel_half_w),
            )
            box_pts = []
            for bx, by in local_box:
                rx, ry = rotate_2d(bx, by, angle_rad)
                box_pts.extend([wsx + rx, wsy - ry])

            self.canvas.create_polygon(
                *box_pts,
                fill=BORDER_COLOR,
                outline=TEXT_MUTED,
                width=2,
            )

            # Green Wheel Velocity Vector Arrow
            if wheel.speed > 1e-6 and max_wheel_speed > 1e-6:
                ratio = wheel.speed / max_wheel_speed
                vec_len_px = 34.0 + ratio * 52.0
                vx_end = wsx + math.cos(angle_rad) * vec_len_px
                vy_end = wsy - math.sin(angle_rad) * vec_len_px
                self.canvas.create_line(
                    wsx,
                    wsy,
                    vx_end,
                    vy_end,
                    fill=COLOR_WHEEL_VEC,
                    width=2.5,
                    arrow=tk.LAST,
                    arrowshape=(10, 12, 4),
                )

            # Wheel Label offset outward from center
            out_dist = math.hypot(wheel.world_x, wheel.world_y)
            if out_dist > 1e-6:
                ox = wsx + (wheel.world_x / out_dist) * 44.0
                oy = wsy - (wheel.world_y / out_dist) * 34.0
            else:
                ox, oy = wsx + 30.0, wsy - 30.0

            self.canvas.create_text(
                ox,
                oy,
                text=wheel.label,
                fill=TEXT_MAIN,
                font=("Sans", 9, "bold"),
            )


def main() -> None:
    root = tk.Tk()
    SwerveDriveApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
