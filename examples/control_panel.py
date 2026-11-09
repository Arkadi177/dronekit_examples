import collections.abc

collections.MutableMapping = collections.abc.MutableMapping
collections.Mapping = collections.abc.Mapping

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk

from dronekit import VehicleMode, connect
import tkintermapview

DEFAULT_CONNECTION = "127.0.0.1:14550"
MODES = ["GUIDED", "STABILIZE", "LOITER", "LAND", "RTL"]
DEFAULT_TAKEOFF_ALT = 10

PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000

LOOP_INTERVAL_MS = 20
RAMP_SPEED = 25


class ControlPanel:
    def __init__(self, root):
        self.root = root
        self.vehicle = None
        self.results = queue.Queue()

        self.rc_vals = {
            "1": float(PWM_MID),
            "2": float(PWM_MID),
            "3": float(PWM_MIN),
            "4": float(PWM_MID),
        }
        self.pressed_keys = set()

        # Map markers & yaw trajectory vector
        self.drone_marker = None
        self.yaw_path = None
        self.map_centered = False

        root.title("Drone Control Panel with Real-Time GPS & Yaw Map")
        root.resizable(True, True)

        # Main Layout: Left = Controls, Right = Map
        main_frame = ttk.Frame(root)
        main_frame.pack(fill="both", expand=True)

        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        right_frame = ttk.LabelFrame(main_frame, text="Real-Time Telemetry Map & Yaw", padding=5)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # ---------------- LEFT PANEL CONTROLS ----------------
        # 1. Connection Section
        top = ttk.Frame(left_frame, padding=5)
        top.grid(row=0, column=0, sticky="ew")
        ttk.Label(top, text="Connection:").grid(row=0, column=0, padx=(0, 5))
        self.conn_var = tk.StringVar(value=DEFAULT_CONNECTION)
        self.conn_entry = ttk.Entry(top, textvariable=self.conn_var, width=22)
        self.conn_entry.grid(row=0, column=1, padx=5)
        self.connect_btn = ttk.Button(top, text="Connect", command=self.on_connect)
        self.connect_btn.grid(row=0, column=2, padx=5)

        # 2. Flight Modes
        modes = ttk.LabelFrame(left_frame, text="Flight Mode", padding=5)
        modes.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.mode_buttons = []
        for i, mode in enumerate(MODES):
            btn = ttk.Button(
                modes,
                text=mode.capitalize(),
                command=lambda m=mode: self.set_mode(m),
            )
            btn.grid(row=0, column=i, padx=2)
            self.mode_buttons.append(btn)

        # 3. Actions & Takeoff
        actions = ttk.LabelFrame(left_frame, text="Actions & Takeoff", padding=5)
        actions.grid(row=2, column=0, padx=5, pady=5, sticky="ew")

        self.arm_btn = ttk.Button(actions, text="Arm", command=self.on_arm)
        self.arm_btn.grid(row=0, column=0, padx=5)

        ttk.Label(actions, text="Alt (m):").grid(row=0, column=1, padx=(10, 2))
        self.alt_var = tk.StringVar(value=str(DEFAULT_TAKEOFF_ALT))
        self.alt_entry = ttk.Entry(actions, textvariable=self.alt_var, width=5)
        self.alt_entry.grid(row=0, column=2)

        self.takeoff_btn = ttk.Button(actions, text="Takeoff", command=self.on_takeoff)
        self.takeoff_btn.grid(row=0, column=3, padx=5)

        self.close_btn = ttk.Button(actions, text="Close", command=self.on_close)
        self.close_btn.grid(row=0, column=4, padx=5)

        # 4. RC Signal Progress Bars
        rc_frame = ttk.LabelFrame(left_frame, text="Visual RC Channels (1000 - 2000 µs)", padding=5)
        rc_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")

        self.rc_bars = {}
        self.rc_labels = {}
        channels_info = [
            ("1", "Roll (Ch 1) - Left / Right"),
            ("2", "Pitch (Ch 2) - Up / Down"),
            ("3", "Throttle (Ch 3) - W / S"),
            ("4", "Yaw (Ch 4) - A / D"),
        ]

        for i, (ch_id, label_text) in enumerate(channels_info):
            ttk.Label(rc_frame, text=label_text, width=26, anchor="w").grid(row=i, column=0, padx=2, pady=2)
            bar = ttk.Progressbar(rc_frame, orient="horizontal", length=180, mode="determinate", maximum=1000)
            bar.grid(row=i, column=1, padx=2, pady=2)
            bar["value"] = int(self.rc_vals[ch_id]) - PWM_MIN
            lbl = ttk.Label(rc_frame, text=f"{int(self.rc_vals[ch_id])} µs", width=8)
            lbl.grid(row=i, column=2, padx=2, pady=2)
            self.rc_bars[ch_id] = bar
            self.rc_labels[ch_id] = lbl

        # ---------------- RIGHT PANEL MAP ----------------
        self.map_widget = tkintermapview.TkinterMapView(right_frame, width=450, height=350, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        # Default SITL initial coordinates (Canberra SITL location)
        self.map_widget.set_position(-35.363261, 149.165230)
        self.map_widget.set_zoom(19)

        # Telemetry Labels under map
        self.telemetry_var = tk.StringVar(value="Lat: N/A | Lon: N/A | Yaw: N/A°")
        ttk.Label(right_frame, textvariable=self.telemetry_var, font=("Consolas", 10, "bold"), padding=4).pack(
            anchor="w")

        # Status Bar
        self.status_var = tk.StringVar(value="Status: Not connected")
        ttk.Label(root, textvariable=self.status_var, relief="sunken", anchor="w", padding=6).pack(fill="x",
                                                                                                   side="bottom")

        # Key bindings
        root.bind("<KeyPress>", self.on_key_press)
        root.bind("<KeyRelease>", self.on_key_release)

        self.set_connected_ui(False)
        root.protocol("WM_DELETE_WINDOW", self.on_exit)

        self.refresh_status()
        self.poll_results()
        self.update_rc_loop()

    # --- UI Helper ---
    def set_connected_ui(self, connected):
        state = "normal" if connected else "disabled"
        for btn in self.mode_buttons + [self.arm_btn, self.close_btn, self.alt_entry, self.takeoff_btn]:
            btn.config(state=state)
        self.connect_btn.config(state="disabled" if connected else "normal")
        self.conn_entry.config(state="disabled" if connected else "normal")

    def refresh_status(self):
        """Update mode, telemetry, map marker position and Yaw direction vector."""
        if self.vehicle is not None:
            mode = self.vehicle.mode.name if self.vehicle.mode else "N/A"
            armed = "ARMED" if self.vehicle.armed else "DISARMED"

            # Extract Position & Heading
            try:
                lat = self.vehicle.location.global_relative_frame.lat
                lon = self.vehicle.location.global_relative_frame.lon
                alt = self.vehicle.location.global_relative_frame.alt
                heading = self.vehicle.heading  # 0 to 360 degrees

                alt_str = f"{alt:.1f} m" if alt is not None else "0.0 m"

                if lat is not None and lon is not None:
                    self.telemetry_var.set(f"Lat: {lat:.6f} | Lon: {lon:.6f} | Yaw/Heading: {heading}°")

                    # Update Map View Center once on first valid fix
                    if not self.map_centered:
                        self.map_widget.set_position(lat, lon)
                        self.map_centered = True

                    # Update Drone Marker
                    if self.drone_marker is None:
                        self.drone_marker = self.map_widget.set_marker(lat, lon, text=f"Drone ({heading}°)")
                    else:
                        self.drone_marker.set_position(lat, lon)
                        self.drone_marker.set_text(f"Drone ({heading}°)")

                    # Calculate Yaw direction line endpoint (approx 15 meters ahead in heading direction)
                    if heading is not None:
                        dist_meters = 15.0
                        heading_rad = math.radians(heading)
                        delta_lat = (dist_meters / 111320.0) * math.cos(heading_rad)
                        delta_lon = (dist_meters / (111320.0 * math.cos(math.radians(lat)))) * math.sin(heading_rad)

                        target_lat = lat + delta_lat
                        target_lon = lon + delta_lon

                        # Redraw Yaw vector arrow path on map
                        if self.yaw_path:
                            self.yaw_path.delete()
                        self.yaw_path = self.map_widget.set_path([(lat, lon), (target_lat, target_lon)], color="red",
                                                                 width=3)
            except Exception as e:
                alt_str = "0.0 m"

            self.status_var.set(f"Status: Connected | Mode: {mode} | State: {armed} | Alt: {alt_str}")
        else:
            self.status_var.set("Status: Not connected")

        self.root.after(200, self.refresh_status)

    # --- Connection Handlers ---
    def on_connect(self):
        self.connect_btn.config(state="disabled")
        self.status_var.set("Status: Connecting...")
        threading.Thread(target=self.connect_worker, args=(self.conn_var.get().strip(),), daemon=True).start()

    def connect_worker(self, conn_string):
        try:
            self.results.put(("connected", connect(conn_string, wait_ready=True, timeout=30)))
        except Exception as e:
            self.results.put(("failed", e))

    def poll_results(self):
        try:
            while True:
                kind, value = self.results.get_nowait()
                if kind == "connected":
                    self.vehicle = value
                    self.vehicle.channels.overrides = {}
                    self.set_connected_ui(True)
                else:
                    self.status_var.set(f"Connection failed: {value}")
                    self.connect_btn.config(state="normal")
        except queue.Empty:
            pass
        self.root.after(100, self.poll_results)

    # --- Drone Commands ---
    def set_mode(self, mode):
        if self.vehicle:
            self.vehicle.mode = VehicleMode(mode)

    def on_arm(self):
        if not self.vehicle:
            return
        if not self.vehicle.is_armable:
            self.status_var.set("Status: Vehicle not armable yet (waiting for GPS/EKF lock)")
            return

        self.vehicle.channels.overrides = {}
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

    def on_takeoff(self):
        if not self.vehicle:
            return
        try:
            target_alt = float(self.alt_var.get())
            if target_alt <= 0:
                raise ValueError
        except ValueError:
            self.status_var.set("Status: Enter a valid positive altitude")
            return

        if not self.vehicle.armed:
            self.status_var.set("Status: Arm the vehicle first")
            return

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.simple_takeoff(target_alt)

    def on_close(self):
        v, self.vehicle = self.vehicle, None
        self.set_connected_ui(False)
        self.status_var.set("Status: Not connected")
        if v:
            threading.Thread(target=v.close, daemon=True).start()

    def on_exit(self):
        if self.vehicle is not None:
            self.vehicle.channels.overrides = {}
            self.vehicle.close()
        self.root.destroy()

    # --- Key Events & Dynamic Ramping ---
    def on_key_press(self, event):
        self.pressed_keys.add(event.keysym.lower())

    def on_key_release(self, event):
        self.pressed_keys.discard(event.keysym.lower())

    def update_rc_loop(self):
        def ramp_channel(ch, key_pos, key_neg, target_default=PWM_MID):
            if key_pos in self.pressed_keys:
                target = float(PWM_MAX)
            elif key_neg in self.pressed_keys:
                target = float(PWM_MIN)
            else:
                target = float(target_default)

            curr = self.rc_vals[ch]
            if curr < target:
                self.rc_vals[ch] = min(target, curr + RAMP_SPEED)
            elif curr > target:
                self.rc_vals[ch] = max(target, curr - RAMP_SPEED)

        # Dynamic continuous controls
        ramp_channel("1", "right", "left", PWM_MID)  # Roll (Ch 1)
        ramp_channel("2", "down", "up", PWM_MID)  # Pitch (Ch 2)
        ramp_channel("3", "w", "s", PWM_MIN)  # Throttle (Ch 3)
        ramp_channel("4", "d", "a", PWM_MID)  # Yaw (Ch 4)

        for ch, val in self.rc_vals.items():
            int_val = int(round(val))
            self.rc_bars[ch]["value"] = int_val - PWM_MIN
            self.rc_labels[ch].config(text=f"{int_val} µs")

        if self.vehicle is not None:
            if self.vehicle.armed:
                self.vehicle.channels.overrides = {
                    "1": int(round(self.rc_vals["1"])),
                    "2": int(round(self.rc_vals["2"])),
                    "3": int(round(self.rc_vals["3"])),
                    "4": int(round(self.rc_vals["4"])),
                }
            else:
                self.vehicle.channels.overrides = {}

        self.root.after(LOOP_INTERVAL_MS, self.update_rc_loop)


if __name__ == "__main__":
    root = tk.Tk()
    ControlPanel(root)
    root.mainloop()