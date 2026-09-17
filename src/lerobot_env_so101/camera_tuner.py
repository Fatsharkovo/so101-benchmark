"""Standalone front-camera editor: preview, seven sliders, and reusable local settings."""

from __future__ import annotations

import copy
from dataclasses import asdict, replace
from pathlib import Path

from .camera_profile import CAMERA_CONTROLS, FrontCameraPose, load_profile, profile_path, save_profile
from .config import RunConfig
from .env import SO101Env


class CameraTuningSession:
    """Keep editing and persistence independent of the widget event loop."""

    def __init__(self, config: RunConfig, path: Path | None = None):
        self.path = path or profile_path()
        config = copy.deepcopy(config)
        if len(config.tasks) != 1:
            raise ValueError("Camera tuning needs one task; use --task to select it")
        task = config.tasks[0] if config.tasks != ["all"] else "stack_blue_on_red"
        config.sim.randomization.setdefault("camera", {})["enabled"] = False
        front_override = config.sim.cameras.pop("front", None)
        self.env = SO101Env(task, config.sim, config.task_paths)
        try:
            self.env.reset(seed=config.seed)
            self.scene_default = FrontCameraPose.from_env(self.env)
            if front_override is not None:
                config.sim.cameras["front"] = front_override
                self.env.reset(seed=config.seed)
            self.initial = FrontCameraPose.from_env(self.env)
            self.pose = self.initial
            self.saved_pose = None
            self.load_error = None
            try:
                self.saved_pose = load_profile(self.path)
            except ValueError as exc:
                self.load_error = str(exc)
            if self.saved_pose is not None:
                self.set_pose(self.saved_pose)
        except BaseException:
            self.env.close()
            raise

    @property
    def dirty(self) -> bool:
        return self.pose != (self.saved_pose or self.initial)

    def set_pose(self, pose: FrontCameraPose) -> None:
        pose.apply(self.env)
        self.pose = pose

    def save(self) -> None:
        save_profile(self.pose, self.path)
        self.saved_pose = self.pose
        self.load_error = None

    def restore_saved(self) -> None:
        saved = load_profile(self.path)
        if saved is None:
            raise ValueError("尚未保存本机相机配置")
        self.set_pose(saved)
        self.saved_pose = saved
        self.load_error = None

    def restore_scene(self) -> None:
        self.set_pose(self.scene_default)

    def render(self):
        self.env.renderer.update_scene(self.env.data, camera="front")
        return self.env.renderer.render().copy()

    def close(self) -> None:
        self.env.close()


class CameraTunerWindow:
    """Tk UI, with rendering and coalesced edits on the same thread as MuJoCo."""

    def __init__(self, root, session: CameraTuningSession):
        import tkinter as tk
        from tkinter import font, ttk

        from PIL import Image, ImageTk

        self.tk, self.ttk, self.Image, self.ImageTk = tk, ttk, Image, ImageTk
        self.root, self.session = root, session
        families = set(font.families(root))
        family = next(
            (
                name
                for name in ("Noto Sans CJK SC", "Microsoft YaHei", "PingFang SC", "song ti")
                if name in families
            ),
            "sans-serif",
        )
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            font.nametofont(name, root=root).configure(family=family, size=12)
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("TButton", padding=(10, 6))
        style.map("TSpinbox", fieldbackground=[("invalid", "#ffe1df")])
        self.timer = None
        self.closed = False
        self.photo = self.frame = None
        self.pending_render = True
        self.invalid_fields = set()
        self.variables, self.slider_values, self.sliders, self.entries = {}, {}, {}, {}
        root.title("SO-101 Front 相机调参")
        root.geometry("1180x820")
        root.minsize(1000, 780)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        main = ttk.Frame(root, padding=16)
        main.grid(sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.rowconfigure(1, weight=1)
        ttk.Label(main, text="Front 实时仿真预览", font=(family, 12, "normal")).grid(
            row=0, column=0, sticky="w", pady=(0, 12)
        )
        self.preview = tk.Canvas(main, bg="#17212b", highlightthickness=0)
        self.preview.grid(row=1, column=0, sticky="nsew", padx=(0, 20))
        self.preview.bind("<Configure>", lambda event: self.present())
        self.image_item = self.preview.create_image(0, 0, anchor="center")
        panel = ttk.Frame(main, width=370)
        panel.grid(row=0, column=1, rowspan=2, sticky="ns")
        panel.columnconfigure(0, weight=1)
        ttk.Label(panel, text="相机参数", font=(family, 12, "normal")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 8)
        )
        for index, (key, label, low, high) in enumerate(CAMERA_CONTROLS):
            row = 1 + index * 2
            ttk.Label(panel, text=label).grid(row=row, column=0, sticky="w", pady=(6, 0))
            variable = tk.StringVar(root)
            value = tk.DoubleVar(root)
            self.variables[key], self.slider_values[key] = variable, value
            entry = ttk.Spinbox(
                panel,
                from_=low,
                to=high,
                increment=0.1,
                width=9,
                textvariable=variable,
                command=lambda name=key: self.entry_changed(name),
            )
            entry.grid(row=row, column=1, sticky="e", padx=(12, 0))
            entry.bind("<Return>", lambda event, name=key: self.entry_changed(name))
            entry.bind("<FocusOut>", lambda event, name=key: self.entry_changed(name))
            entry.bind("<KeyRelease>", lambda event, name=key: self.entry_changed(name))
            self.entries[key] = entry
            slider = ttk.Scale(
                panel,
                from_=low,
                to=high,
                variable=value,
                length=335,
                command=lambda number, name=key: self.slider_changed(name, number),
            )
            slider.grid(row=row + 1, column=0, columnspan=2, sticky="ew", pady=(4, 6))
            self.sliders[key] = slider
        ttk.Label(
            panel,
            text="+X：基座前方；+Y：基座左侧\n下俯向下为正；转向 0° 朝向 −X\nfovy 控制画面范围，与下俯角不同",
            wraplength=335,
        ).grid(row=15, column=0, columnspan=2, sticky="w", pady=(8, 12))
        self.save_button = ttk.Button(panel, text="保存并设为本机默认", command=self.save)
        self.save_button.grid(row=16, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Button(panel, text="恢复上次保存", command=self.restore_saved).grid(
            row=17, column=0, columnspan=2, sticky="ew", pady=4
        )
        ttk.Button(panel, text="恢复场景默认", command=self.restore_scene).grid(
            row=18, column=0, columnspan=2, sticky="ew", pady=4
        )
        self.status = tk.StringVar(root)
        ttk.Label(main, textvariable=self.status, wraplength=1050).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(12, 4)
        )
        ttk.Label(
            main,
            text=f"本机配置：{session.path}\n保存后，下一次遥操作自动使用；正在运行的采集保持原参数。",
            wraplength=1050,
        ).grid(row=3, column=0, columnspan=2, sticky="w")
        self.sync_controls()
        self.set_status(
            session.load_error or ("已加载上次保存的参数" if session.saved_pose else "已加载场景/运行配置")
        )
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.timer = root.after(0, self.tick)

    def set_status(self, message: str | None = None) -> None:
        self.status.set(message or ("参数已修改，尚未保存" if self.session.dirty else "当前参数已同步"))
        self.root.title("SO-101 Front 相机调参" + (" *" if self.session.dirty else ""))

    def sync_controls(self) -> None:
        for key, value in asdict(self.session.pose).items():
            self.variables[key].set(f"{value:.10g}")
            self.slider_values[key].set(value)
            self.entries[key].state(["!invalid"])
        self.invalid_fields.clear()
        self.pending_render = True

    def slider_changed(self, key: str, number: str) -> None:
        self.variables[key].set(f"{float(number):.1f}")
        self.entry_changed(key)

    def entry_changed(self, key: str) -> None:
        try:
            value = float(self.variables[key].get())
            pose = replace(self.session.pose, **{key: value})
            if pose != self.session.pose:
                self.session.set_pose(pose)
                self.pending_render = True
            self.slider_values[key].set(value)
            self.invalid_fields.discard(key)
            self.entries[key].state(["!invalid"])
            self.set_status()
        except (ValueError, OverflowError) as exc:
            self.invalid_fields.add(key)
            self.entries[key].state(["invalid"])
            self.set_status(f"输入未应用：{exc}")

    def tick(self) -> None:
        if self.closed:
            return
        try:
            if self.pending_render:
                self.frame = self.session.render()
                self.pending_render = False
                self.present()
        except Exception as exc:
            self.pending_render = False
            self.set_status(f"预览失败：{exc}")
        self.timer = self.root.after(34, self.tick)

    def present(self) -> None:
        if self.frame is None or self.closed:
            return
        width, height = self.preview.winfo_width(), self.preview.winfo_height()
        if width < 2 or height < 2:
            return
        image = self.Image.fromarray(self.frame)
        scale = min(width / image.width, height / image.height)
        image = image.resize(
            (max(1, round(image.width * scale)), max(1, round(image.height * scale))),
            self.Image.Resampling.BILINEAR,
        )
        self.photo = self.ImageTk.PhotoImage(image, master=self.root)
        self.preview.itemconfigure(self.image_item, image=self.photo)
        self.preview.coords(self.image_item, width / 2, height / 2)

    def save(self) -> bool:
        for key in self.variables:
            self.entry_changed(key)
        if self.invalid_fields:
            self.set_status("存在无效输入，请修正后再保存。")
            return False
        try:
            self.session.save()
            self.set_status("已保存并设为本机默认；下次遥操作启动时生效。")
            return True
        except OSError as exc:
            self.set_status(f"保存失败，旧配置保留：{exc}")
            return False

    def restore_saved(self) -> None:
        try:
            self.session.restore_saved()
            self.sync_controls()
            self.set_status("已重新读取上次保存的参数。")
        except ValueError as exc:
            self.set_status(str(exc))

    def restore_scene(self) -> None:
        self.session.restore_scene()
        self.sync_controls()
        self.set_status("已恢复场景默认；点击保存可将其设为本机默认。")

    def close(self) -> None:
        from tkinter import messagebox

        if self.session.dirty or self.invalid_fields:
            choice = messagebox.askyesnocancel(
                "未保存的相机参数", "关闭前保存为本机默认吗？", parent=self.root
            )
            if choice is None or (choice and not self.save()):
                return
        self.shutdown()

    def shutdown(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self.timer is not None:
            self.root.after_cancel(self.timer)
        self.session.close()
        self.root.destroy()


def run_camera_tuner(config: RunConfig) -> None:
    try:
        import tkinter as tk
    except ImportError as exc:
        raise RuntimeError(
            "Camera tuner requires a Python installation with Tk support (python3-tk)"
        ) from exc

    root = tk.Tk()
    root.withdraw()
    session = window = None
    try:
        session = CameraTuningSession(config)
        window = CameraTunerWindow(root, session)
        root.deiconify()
        root.mainloop()
    finally:
        if window:
            window.shutdown()
        else:
            if session:
                session.close()
            root.destroy()
