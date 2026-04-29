#!/usr/bin/env python3
"""
bag_to_navmap_gui.py
Tkinter GUI for running the bag-to-navmap Docker container.
"""

import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext, messagebox
import subprocess
import threading
import shlex
import os
import sys
import platform


# ─────────────────────────────────────────────────────────────────────────────
# Theme / colour constants
# ─────────────────────────────────────────────────────────────────────────────
BG          = "#1e1e2e"
BG2         = "#27273a"
BG3         = "#313148"
ACCENT      = "#4f98a3"
ACCENT_DARK = "#1a626b"
TEXT        = "#cdccca"
TEXT_MUTED  = "#797876"
TEXT_FAINT  = "#5a5957"
BORDER      = "#393836"
SUCCESS     = "#6daa45"
ERROR_COL   = "#dd6974"
WARN        = "#fdab43"

FONT_BODY   = ("Segoe UI", 10) if platform.system() == "Windows" else ("SF Pro Text", 10) if platform.system() == "Darwin" else ("DejaVu Sans", 10)
FONT_MONO   = ("Consolas", 9)  if platform.system() == "Windows" else ("Menlo", 9)         if platform.system() == "Darwin" else ("DejaVu Sans Mono", 9)
FONT_BOLD   = (FONT_BODY[0], 10, "bold")
FONT_LABEL  = (FONT_BODY[0], 9)
FONT_TITLE  = (FONT_BODY[0], 13, "bold")
FONT_SECTION= (FONT_BODY[0], 10, "bold")


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def entry(parent, textvariable, width=28, placeholder=""):
    e = ttk.Entry(parent, textvariable=textvariable, width=width, font=FONT_BODY)
    if placeholder and not textvariable.get():
        textvariable.set(placeholder)
        e.config(foreground=TEXT_FAINT)
        def on_focus_in(ev):
            if textvariable.get() == placeholder:
                textvariable.set("")
                e.config(foreground=TEXT)
        def on_focus_out(ev):
            if not textvariable.get():
                textvariable.set(placeholder)
                e.config(foreground=TEXT_FAINT)
        e.bind("<FocusIn>",  on_focus_in)
        e.bind("<FocusOut>", on_focus_out)
    return e


def section_label(parent, text):
    f = tk.Frame(parent, bg=BG2)
    tk.Label(f, text=text, font=FONT_SECTION, bg=BG2, fg=ACCENT).pack(side="left")
    tk.Frame(f, bg=BORDER, height=1).pack(side="left", fill="x", expand=True, padx=(8,0), pady=6)
    return f


def row(parent, label_text, widget, hint=""):
    f = tk.Frame(parent, bg=BG2)
    lbl = tk.Label(f, text=label_text, font=FONT_LABEL, bg=BG2, fg=TEXT_MUTED, width=26, anchor="w")
    lbl.pack(side="left", padx=(0, 6))
    widget_parent = f
    widget.pack(in_=widget_parent, side="left")
    if hint:
        tk.Label(f, text=hint, font=(FONT_BODY[0], 8), bg=BG2, fg=TEXT_FAINT).pack(side="left", padx=(6, 0))
    return f


# ─────────────────────────────────────────────────────────────────────────────
# Main Application
# ─────────────────────────────────────────────────────────────────────────────
class BagToNavmapGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("bag_to_navmap  ·  Docker Runner")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(820, 680)

        self._process   = None
        self._log_lines = 0

        self._setup_style()
        self._build_ui()
        self._refresh_command()

    # ── Style ─────────────────────────────────────────────────────────────
    def _setup_style(self):
        s = ttk.Style(self)
        s.theme_use("clam")

        s.configure(".",
            background=BG, foreground=TEXT,
            font=FONT_BODY, borderwidth=0, relief="flat",
            fieldbackground=BG3, selectbackground=ACCENT_DARK, selectforeground=TEXT)

        s.configure("TEntry",
            fieldbackground=BG3, foreground=TEXT,
            insertcolor=TEXT, borderwidth=1, relief="flat",
            padding=(6, 4))
        s.map("TEntry",
            fieldbackground=[("focus", BG3)],
            bordercolor=[("focus", ACCENT), ("!focus", BORDER)])

        s.configure("TCheckbutton",
            background=BG2, foreground=TEXT,
            focuscolor="none", indicatorcolor=BG3,
            indicatordiameter=14)
        s.map("TCheckbutton",
            background=[("active", BG2)],
            indicatorcolor=[("selected", ACCENT), ("!selected", BG3)])

        s.configure("TNotebook",
            background=BG, borderwidth=0, tabmargins=[0, 0, 0, 0])
        s.configure("TNotebook.Tab",
            background=BG3, foreground=TEXT_MUTED,
            padding=(14, 6), font=FONT_BODY, borderwidth=0)
        s.map("TNotebook.Tab",
            background=[("selected", ACCENT_DARK)],
            foreground=[("selected", TEXT)])

        s.configure("TScrollbar",
            background=BG3, troughcolor=BG, arrowcolor=TEXT_FAINT, borderwidth=0)

        s.configure("TCombobox",
            fieldbackground=BG3, foreground=TEXT, background=BG3,
            selectbackground=ACCENT_DARK, arrowcolor=TEXT_MUTED,
            borderwidth=1, relief="flat")

        s.configure("Run.TButton",
            background=ACCENT_DARK, foreground=TEXT,
            padding=(20, 8), font=FONT_BOLD, borderwidth=0, relief="flat")
        s.map("Run.TButton",
            background=[("active", ACCENT), ("disabled", BG3)],
            foreground=[("disabled", TEXT_FAINT)])

        s.configure("Stop.TButton",
            background="#7a1e2e", foreground=TEXT,
            padding=(20, 8), font=FONT_BOLD, borderwidth=0, relief="flat")
        s.map("Stop.TButton",
            background=[("active", ERROR_COL)])

        s.configure("Copy.TButton",
            background=BG3, foreground=TEXT_MUTED,
            padding=(10, 5), font=FONT_LABEL, borderwidth=0, relief="flat")
        s.map("Copy.TButton",
            background=[("active", BORDER)],
            foreground=[("active", TEXT)])

        s.configure("Browse.TButton",
            background=BG3, foreground=TEXT_MUTED,
            padding=(8, 4), font=FONT_LABEL, borderwidth=0, relief="flat")
        s.map("Browse.TButton",
            background=[("active", BORDER)])

    # ── UI Build ──────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Header ──
        hdr = tk.Frame(self, bg=BG, pady=12)
        hdr.pack(fill="x", padx=20)

        svg_frame = tk.Frame(hdr, bg=ACCENT_DARK, width=36, height=36)
        svg_frame.pack(side="left", padx=(0,12))
        svg_frame.pack_propagate(False)
        tk.Label(svg_frame, text="⬡", font=(FONT_BODY[0], 18), bg=ACCENT_DARK, fg=ACCENT).place(relx=.5, rely=.5, anchor="center")

        title_col = tk.Frame(hdr, bg=BG)
        title_col.pack(side="left")
        tk.Label(title_col, text="bag_to_navmap", font=FONT_TITLE, bg=BG, fg=TEXT).pack(anchor="w")
        tk.Label(title_col, text="ROS 2 bag → NavMap  ·  Docker Runner",
                 font=FONT_LABEL, bg=BG, fg=TEXT_MUTED).pack(anchor="w")

        # Docker image row (top-right)
        img_row = tk.Frame(hdr, bg=BG)
        img_row.pack(side="right")
        tk.Label(img_row, text="Docker image", font=FONT_LABEL, bg=BG, fg=TEXT_MUTED).pack(side="left", padx=(0,6))
        self.image_var = tk.StringVar(value="bag-to-navmap")
        ttk.Entry(img_row, textvariable=self.image_var, width=22, font=FONT_BODY).pack(side="left")

        tk.Frame(self, bg=BORDER, height=1).pack(fill="x")

        # ── Main paned area ──
        paned = tk.PanedWindow(self, orient="vertical", bg=BG,
                               sashwidth=5, sashrelief="flat", sashpad=0)
        paned.pack(fill="both", expand=True, padx=0, pady=0)

        # ── Top: Notebook tabs ──
        nb_frame = tk.Frame(paned, bg=BG)
        paned.add(nb_frame, stretch="always", minsize=360)

        nb = ttk.Notebook(nb_frame)
        nb.pack(fill="both", expand=True, padx=12, pady=(10,0))

        self.tab_required = self._build_tab_required(nb)
        self.tab_processing = self._build_tab_processing(nb)
        self.tab_loop = self._build_tab_loop(nb)
        self.tab_recon = self._build_tab_recon(nb)

        nb.add(self.tab_required,   text="  Required  ")
        nb.add(self.tab_processing, text="  Processing  ")
        nb.add(self.tab_loop,       text="  Loop Closure  ")
        nb.add(self.tab_recon,      text="  Reconstruction  ")

        # ── Command preview ──
        cmd_frame = tk.Frame(nb_frame, bg=BG2, pady=8)
        cmd_frame.pack(fill="x", padx=12, pady=(6,0))
        tk.Label(cmd_frame, text="Command preview", font=FONT_SECTION,
                 bg=BG2, fg=ACCENT, padx=10).pack(anchor="w")
        self.cmd_text = tk.Text(cmd_frame, height=4, font=FONT_MONO,
                                bg=BG3, fg=WARN, insertbackground=TEXT,
                                relief="flat", wrap="none", padx=8, pady=6,
                                state="disabled", cursor="arrow")
        self.cmd_text.pack(fill="x", padx=10, pady=(4,0))
        xscroll = ttk.Scrollbar(cmd_frame, orient="horizontal", command=self.cmd_text.xview)
        self.cmd_text.configure(xscrollcommand=xscroll.set)
        xscroll.pack(fill="x", padx=10)
        ttk.Button(cmd_frame, text="⎘  Copy", style="Copy.TButton",
                   command=self._copy_command).pack(anchor="e", padx=10, pady=(4,6))

        # ── Bottom: Log + run controls ──
        bot_frame = tk.Frame(paned, bg=BG)
        paned.add(bot_frame, stretch="always", minsize=160)

        log_hdr = tk.Frame(bot_frame, bg=BG, pady=6)
        log_hdr.pack(fill="x", padx=12)
        tk.Label(log_hdr, text="Output log", font=FONT_SECTION,
                 bg=BG, fg=TEXT_MUTED).pack(side="left")
        ttk.Button(log_hdr, text="Clear", style="Copy.TButton",
                   command=self._clear_log).pack(side="right")

        log_area = tk.Frame(bot_frame, bg=BG)
        log_area.pack(fill="both", expand=True, padx=12, pady=(0,8))
        self.log = scrolledtext.ScrolledText(log_area, font=FONT_MONO,
                                              bg=BG3, fg=TEXT, insertbackground=TEXT,
                                              relief="flat", state="disabled",
                                              padx=8, pady=6, wrap="word")
        self.log.pack(fill="both", expand=True)
        self.log.tag_config("err",  foreground=ERROR_COL)
        self.log.tag_config("ok",   foreground=SUCCESS)
        self.log.tag_config("warn", foreground=WARN)
        self.log.tag_config("info", foreground=ACCENT)
        self.log.tag_config("dim",  foreground=TEXT_FAINT)

        # ── Run row ──
        run_row = tk.Frame(bot_frame, bg=BG, pady=8)
        run_row.pack(fill="x", padx=12)
        self.status_lbl = tk.Label(run_row, text="Ready", font=FONT_LABEL,
                                   bg=BG, fg=TEXT_MUTED)
        self.status_lbl.pack(side="left")
        self.stop_btn = ttk.Button(run_row, text="⏹  Stop",
                                   style="Stop.TButton",
                                   command=self._stop, state="disabled")
        self.stop_btn.pack(side="right", padx=(8,0))
        self.run_btn = ttk.Button(run_row, text="▶  Run Docker",
                                  style="Run.TButton", command=self._run)
        self.run_btn.pack(side="right")

    # ── Tabs ──────────────────────────────────────────────────────────────
    def _tab_scroll_frame(self, nb):
        outer = tk.Frame(nb, bg=BG2)
        canvas = tk.Canvas(outer, bg=BG2, highlightthickness=0, bd=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner = tk.Frame(canvas, bg=BG2)
        win = canvas.create_window((0, 0), window=inner, anchor="nw")

        def on_configure(e):
            canvas.configure(scrollregion=canvas.bbox("all"))
        def on_canvas_resize(e):
            canvas.itemconfig(win, width=e.width)
        inner.bind("<Configure>", on_configure)
        canvas.bind("<Configure>", on_canvas_resize)

        def _on_mousewheel(event):
            if platform.system() == "Windows":
                canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            elif platform.system() == "Darwin":
                canvas.yview_scroll(int(-1*event.delta), "units")
            else:
                if event.num == 4:
                    canvas.yview_scroll(-1, "units")
                elif event.num == 5:
                    canvas.yview_scroll(1, "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        canvas.bind_all("<Button-4>", _on_mousewheel)
        canvas.bind_all("<Button-5>", _on_mousewheel)

        return outer, inner

    def _field_row(self, parent, label, var, hint="", width=32):
        f = tk.Frame(parent, bg=BG2, pady=3)
        f.pack(fill="x", padx=16)
        tk.Label(f, text=label, font=FONT_LABEL, bg=BG2, fg=TEXT_MUTED,
                 width=30, anchor="w").pack(side="left")
        e = ttk.Entry(f, textvariable=var, width=width, font=FONT_BODY)
        e.pack(side="left")
        e.bind("<KeyRelease>", lambda ev: self._refresh_command())
        if hint:
            tk.Label(f, text=hint, font=(FONT_BODY[0], 8),
                     bg=BG2, fg=TEXT_FAINT).pack(side="left", padx=(8,0))
        return f

    def _browse_row(self, parent, label, var, mode="file"):
        f = tk.Frame(parent, bg=BG2, pady=3)
        f.pack(fill="x", padx=16)
        tk.Label(f, text=label, font=FONT_LABEL, bg=BG2, fg=TEXT_MUTED,
                 width=30, anchor="w").pack(side="left")
        e = ttk.Entry(f, textvariable=var, width=32, font=FONT_BODY)
        e.pack(side="left")
        e.bind("<KeyRelease>", lambda ev: self._refresh_command())

        def browse():
            if mode == "file":
                path = filedialog.askopenfilename(title="Select ROS 2 bag directory / file")
            else:
                path = filedialog.askdirectory(title="Select output directory")
            if path:
                var.set(path)
                self._refresh_command()

        ttk.Button(f, text="Browse…", style="Browse.TButton",
                   command=browse).pack(side="left", padx=(6,0))
        return f

    def _check_row(self, parent, label, var, hint=""):
        f = tk.Frame(parent, bg=BG2, pady=3)
        f.pack(fill="x", padx=16)
        tk.Label(f, text="", width=30, bg=BG2).pack(side="left")
        cb = ttk.Checkbutton(f, text=label, variable=var,
                             command=self._refresh_command)
        cb.pack(side="left")
        if hint:
            tk.Label(f, text=hint, font=(FONT_BODY[0], 8),
                     bg=BG2, fg=TEXT_FAINT).pack(side="left", padx=(8,0))
        return f

    def _section_sep(self, parent, text):
        tk.Frame(parent, bg=BG2, height=10).pack()
        f = tk.Frame(parent, bg=BG2)
        f.pack(fill="x", padx=16)
        tk.Label(f, text=text, font=FONT_SECTION, bg=BG2, fg=ACCENT).pack(side="left")
        tk.Frame(f, bg=BORDER, height=1).pack(side="left", fill="x", expand=True, padx=(8,0))
        tk.Frame(parent, bg=BG2, height=2).pack()

    # ── Tab: Required ──
    def _build_tab_required(self, nb):
        outer, inner = self._tab_scroll_frame(nb)
        tk.Frame(inner, bg=BG2, height=8).pack()

        self._section_sep(inner, "Required Paths")

        self.bag_path_var  = tk.StringVar()
        self.output_dir_var = tk.StringVar()

        self._browse_row(inner, "bag_path  *", self.bag_path_var, mode="file")
        tk.Label(inner, text="    Path to the ROS 2 bag file (directory or .db3)",
                 font=(FONT_BODY[0], 8), bg=BG2, fg=TEXT_FAINT).pack(anchor="w", padx=16)

        tk.Frame(inner, bg=BG2, height=6).pack()

        self._browse_row(inner, "output_dir  *", self.output_dir_var, mode="dir")
        tk.Label(inner, text="    Directory to write output files (.ply, .obj, .navmap)",
                 font=(FONT_BODY[0], 8), bg=BG2, fg=TEXT_FAINT).pack(anchor="w", padx=16)

        tk.Frame(inner, bg=BG2, height=16).pack()
        self._section_sep(inner, "Point Cloud Topics")

        self.pc_topic_var   = tk.StringVar(value="/points")
        self.odom_topic_var = tk.StringVar(value="")

        self._field_row(inner, "--pc_topic", self.pc_topic_var,
                        hint="PointCloud2 topic name")
        self._field_row(inner, "--odom_topic", self.odom_topic_var,
                        hint="Odometry topic — leave blank to omit")
        self._field_row(inner, "--odom_max_latency", self._make("odom_max_latency_var", "0.5"),
                        hint="Max timestamp gap between odom and point cloud (s)")

        tk.Frame(inner, bg=BG2, height=16).pack()
        self._section_sep(inner, "Docker Options")

        self.rm_var = tk.BooleanVar(value=True)
        self._check_row(inner, "--rm  (auto-remove container on exit)", self.rm_var)

        tk.Frame(inner, bg=BG2, height=12).pack()
        return outer

    # ── Tab: Processing ──
    def _build_tab_processing(self, nb):
        outer, inner = self._tab_scroll_frame(nb)
        tk.Frame(inner, bg=BG2, height=8).pack()

        self._section_sep(inner, "ICP Registration")

        self._field_row(inner, "--voxel_size",
                        self._make("voxel_size_var", "0.05"),
                        hint="Downsampling resolution in metres")
        self._field_row(inner, "--icp_dist_thresh",
                        self._make("icp_dist_thresh_var", "0.2"),
                        hint="Max ICP point correspondence distance (m)")
        self._field_row(inner, "--icp_fitness_thresh",
                        self._make("icp_fitness_thresh_var", "0.6"),
                        hint="Min fraction of points aligned to accept a frame")

        tk.Frame(inner, bg=BG2, height=8).pack()

        # Speed/quality presets
        self._section_sep(inner, "Presets  (click to fill)")
        preset_row = tk.Frame(inner, bg=BG2, pady=6)
        preset_row.pack(fill="x", padx=16)

        presets = [
            ("🚀 Fast preview",    "0.1",  "0.5", "0.3"),
            ("⚖ Balanced",         "0.05", "0.2", "0.5"),
            ("🎨 Max quality",     "0.01", "0.1", "0.7"),
        ]
        for label, vs, idt, ift in presets:
            def _apply(v=vs, d=idt, f=ift):
                self.voxel_size_var.set(v)
                self.icp_dist_thresh_var.set(d)
                self.icp_fitness_thresh_var.set(f)
                self._refresh_command()
            btn = tk.Button(preset_row, text=label, font=FONT_LABEL,
                            bg=BG3, fg=TEXT_MUTED, relief="flat",
                            activebackground=BORDER, activeforeground=TEXT,
                            padx=10, pady=4, cursor="hand2",
                            command=_apply)
            btn.pack(side="left", padx=(0,8))

        tk.Frame(inner, bg=BG2, height=12).pack()
        return outer

    # ── Tab: Loop Closure ──
    def _build_tab_loop(self, nb):
        outer, inner = self._tab_scroll_frame(nb)
        tk.Frame(inner, bg=BG2, height=8).pack()
        self._section_sep(inner, "Loop Closure")

        self.enable_loop_closure_var = tk.BooleanVar(value=False)
        self._check_row(inner, "--enable_loop_closure  (flag — enables detection)",
                        self.enable_loop_closure_var,
                        hint="adds 3–8× processing time")

        tk.Frame(inner, bg=BG2, height=10).pack()
        self._section_sep(inner, "Parameters")

        self._field_row(inner, "--loop_closure_radius",
                        self._make("loop_closure_radius_var", "10.0"),
                        hint="Search radius for loop closures (m)")
        self._field_row(inner, "--loop_closure_fitness_thresh",
                        self._make("loop_closure_fitness_thresh_var", "0.3"),
                        hint="Min fitness score for a loop closure")
        self._field_row(inner, "--loop_closure_search_interval",
                        self._make("loop_closure_search_interval_var", "10"),
                        hint="Check for loop closures every N frames")

        tk.Frame(inner, bg=BG2, height=12).pack()

        note = ("Loop closure is best suited for warehouse / campus environments\n"
                "where the robot's path revisits previously-mapped areas.")
        tk.Label(inner, text=note, font=(FONT_BODY[0], 8), bg=BG2,
                 fg=TEXT_FAINT, justify="left").pack(anchor="w", padx=16)
        return outer

    # ── Tab: Reconstruction ──
    def _build_tab_recon(self, nb):
        outer, inner = self._tab_scroll_frame(nb)
        tk.Frame(inner, bg=BG2, height=8).pack()
        self._section_sep(inner, "Mesh Generation")

        self.level_floor_var = tk.BooleanVar(value=False)
        self._check_row(inner, "--level_floor  (rotate dominant floor plane to +Z)",
                        self.level_floor_var,
                        hint="do NOT use for multi-storey or terrain")

        self._field_row(inner, "--poisson_depth",
                        self._make("poisson_depth_var", "9"),
                        hint="Octree depth for Poisson reconstruction")
        self._field_row(inner, "--density_trim_percentile",
                        self._make("density_trim_percentile_var", "0.05"),
                        hint="Bottom density fraction of vertices to remove")

        tk.Frame(inner, bg=BG2, height=10).pack()
        self._section_sep(inner, "Decimation  (optional)")

        self._field_row(inner, "--decimate_target",
                        self._make("decimate_target_var", ""),
                        hint="≤1.0 = ratio, >1 = absolute triangle count — leave blank to skip")

        tk.Frame(inner, bg=BG2, height=10).pack()
        self._section_sep(inner, "Output")

        self._field_row(inner, "--surface_name",
                        self._make("surface_name_var", "map"),
                        hint="NavMap surface name embedded in output file")

        tk.Frame(inner, bg=BG2, height=12).pack()
        return outer

    # ── Variable factory ──────────────────────────────────────────────────
    def _make(self, attr, default):
        var = tk.StringVar(value=default)
        setattr(self, attr, var)
        return var

    # ── Command builder ───────────────────────────────────────────────────
    def _build_command(self):
        bag_host = self.bag_path_var.get().strip()
        out_host = self.output_dir_var.get().strip()
        image    = self.image_var.get().strip() or "bag-to-navmap"

        # Volume mounts: map host bag parent → /app/input (ro)
        #                map host output    → /app/output
        bag_parent   = os.path.dirname(bag_host) if bag_host else "/path/to/bag_parent"
        bag_name     = os.path.basename(bag_host) if bag_host else "your_bag"
        out_host_dir = out_host if out_host else "/path/to/output"

        container_bag = f"/app/input/{bag_name}"
        container_out = "/app/output"

        parts = ["docker", "run"]
        if self.rm_var.get():
            parts.append("--rm")

        # Mounts
        parts += ["-v", f"{bag_parent}:/app/input:ro"]
        parts += ["-v", f"{out_host_dir}:/app/output"]
        parts.append(image)

        # Positional args
        parts.append(container_bag)
        parts.append(container_out)

        # Optional flags
        def opt(flag, var, default=""):
            val = var.get().strip()
            if val and val != default:
                parts += [flag, val]

        def flag(flag_str, var):
            if var.get():
                parts.append(flag_str)

        pc = self.pc_topic_var.get().strip()
        if pc and pc != "/points":
            parts += ["--pc_topic", pc]

        odom = self.odom_topic_var.get().strip()
        if odom:
            parts += ["--odom_topic", odom]

        opt("--voxel_size",       self.voxel_size_var,       "0.05")
        opt("--icp_dist_thresh",  self.icp_dist_thresh_var,  "0.2")
        opt("--icp_fitness_thresh",self.icp_fitness_thresh_var,"0.6")

        flag("--enable_loop_closure", self.enable_loop_closure_var)
        opt("--loop_closure_radius",          self.loop_closure_radius_var,          "10.0")
        opt("--loop_closure_fitness_thresh",   self.loop_closure_fitness_thresh_var,  "0.3")
        opt("--loop_closure_search_interval",  self.loop_closure_search_interval_var, "10")

        flag("--level_floor", self.level_floor_var)

        dt = self.decimate_target_var.get().strip()
        if dt:
            parts += ["--decimate_target", dt]

        opt("--odom_max_latency",        self.odom_max_latency_var,        "0.5")
        opt("--poisson_depth",           self.poisson_depth_var,           "9")
        opt("--density_trim_percentile", self.density_trim_percentile_var, "0.05")
        opt("--surface_name",            self.surface_name_var,            "map")

        return parts

    def _command_str(self):
        parts = self._build_command()
        # Pretty-print with line continuations
        lines = [parts[0]]
        i = 1
        while i < len(parts):
            p = parts[i]
            if p.startswith("-"):
                if i + 1 < len(parts) and not parts[i+1].startswith("-"):
                    lines.append(f"  {p} {parts[i+1]}")
                    i += 2
                else:
                    lines.append(f"  {p}")
                    i += 1
            else:
                lines.append(f"  {p}")
                i += 1
        return " \\\n".join(lines)

    def _refresh_command(self, *_):
        cmd = self._command_str()
        self.cmd_text.config(state="normal")
        self.cmd_text.delete("1.0", "end")
        self.cmd_text.insert("1.0", cmd)
        self.cmd_text.config(state="disabled")

    def _copy_command(self):
        self.clipboard_clear()
        self.clipboard_append(self._command_str())
        self.status_lbl.config(text="Command copied to clipboard ✓", fg=SUCCESS)
        self.after(3000, lambda: self.status_lbl.config(text="Ready", fg=TEXT_MUTED))

    def _clear_log(self):
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")
        self._log_lines = 0

    # ── Run / Stop ────────────────────────────────────────────────────────
    def _validate(self):
        bag = self.bag_path_var.get().strip()
        out = self.output_dir_var.get().strip()
        if not bag:
            messagebox.showerror("Missing input", "bag_path is required.")
            return False
        if not out:
            messagebox.showerror("Missing input", "output_dir is required.")
            return False
        return True

    def _run(self):
        if not self._validate():
            return

        self._clear_log()
        self.run_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_lbl.config(text="Running…", fg=WARN)

        cmd = self._build_command()
        self._log("info", "$ " + " ".join(shlex.quote(p) for p in cmd) + "\n\n")

        def worker():
            try:
                self._process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                for line in self._process.stdout:
                    self._log_line(line)
                self._process.wait()
                rc = self._process.returncode
                if rc == 0:
                    self._log("ok", f"\n✓ Completed successfully (exit 0)\n")
                    self.after(0, lambda: self.status_lbl.config(
                        text="Completed ✓", fg=SUCCESS))
                else:
                    self._log("err", f"\n✗ Process exited with code {rc}\n")
                    self.after(0, lambda: self.status_lbl.config(
                        text=f"Failed (exit {rc})", fg=ERROR_COL))
            except FileNotFoundError:
                self._log("err", "\n✗ 'docker' not found — is Docker installed and on your PATH?\n")
                self.after(0, lambda: self.status_lbl.config(
                    text="Error: docker not found", fg=ERROR_COL))
            except Exception as ex:
                self._log("err", f"\n✗ Unexpected error: {ex}\n")
                self.after(0, lambda: self.status_lbl.config(
                    text="Error", fg=ERROR_COL))
            finally:
                self._process = None
                self.after(0, lambda: self.run_btn.config(state="normal"))
                self.after(0, lambda: self.stop_btn.config(state="disabled"))

        threading.Thread(target=worker, daemon=True).start()

    def _stop(self):
        if self._process:
            try:
                self._process.terminate()
                self._log("warn", "\n⏹ Process terminated by user.\n")
            except Exception:
                pass
        self.stop_btn.config(state="disabled")
        self.status_lbl.config(text="Stopped", fg=TEXT_MUTED)

    def _log_line(self, line):
        tag = "dim"
        l = line.lower()
        if any(k in l for k in ("error", "failed", "exception", "traceback")):
            tag = "err"
        elif any(k in l for k in ("warn", "warning")):
            tag = "warn"
        elif any(k in l for k in ("done", "success", "wrote", "saved", "complete")):
            tag = "ok"
        elif any(k in l for k in ("frame", "processing", "running", "reading")):
            tag = "info"
        self._log(tag, line)

    def _log(self, tag, text):
        def _insert():
            self.log.config(state="normal")
            self.log.insert("end", text, tag)
            self.log.see("end")
            self.log.config(state="disabled")
        self.after(0, _insert)


if __name__ == "__main__":
    app = BagToNavmapGUI()
    app.mainloop()
