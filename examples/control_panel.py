import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
collections.Mapping = collections.abc.Mapping

import queue
import threading
import tkinter as tk
from tkinter import ttk

from dronekit import VehicleMode, connect

DEFAULT_CONNECTION = "127.0.0.1:14550"
MODES = ["GUIDED", "STABILIZE", "LOITER", "LAND", "RTL"]
DEFAULT_TAKEOFF_ALT = 10

PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000

# Loop rate in milliseconds (20 ms = 50 Hz, matching real RC transmitter refresh rate)
LOOP_INTERVAL_MS = 20
# Ramping speed: 25 µs per 20ms tick (ramps from neutral to full deflection in ~0.4s)
RAMP_SPEED = 25


class ControlPanel:
    def __init__(self, root):
        self.root = root
        self.vehicle = None
        self.results = queue.Queue()

        # Channel values stored as float for sub-integer smooth interpolation
        self.rc_vals = {
            "1": float(PWM_MID),
            "2": float(PWM_MID),
            "3": float(PWM_MIN),
            "4": float(PWM_MID),
        }
        self.pressed_keys = set()

        root.title("Drone Control Panel")
        root.resizable(False, False)

        # 1. Connection Section
        top = ttk.Frame(root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        ttk.Label(top, text="Connection string:").grid(row=0, column=0, padx=(0, 5))
        self.conn_var = tk.StringVar(value=DEFAULT_CONNECTION)
        self.conn_entry = ttk.Entry(top, textvariable=self.conn_var, width=28)
        self.conn_entry.grid(row=0, column=1, padx=5)
        self.connect_btn = ttk.Button(top, text="Connect", command=self.on_connect)
        self.connect_btn.grid(row=0, column=2, padx=5)

        # 2. Flight Modes
        modes = ttk.LabelFrame(root, text="Flight Mode", padding=10)
        modes.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        self.mode_buttons = []
        for i, mode in enumerate(MODES):
            btn = ttk.Button(
                modes,
                text=mode.capitalize(),
                command=lambda m=mode: self.set_mode(m),
            )
            btn.grid(row=0, column=i, padx=3)
            self.mode_buttons.append(btn)

        # 3. Arm, Takeoff & Close
        actions = ttk.LabelFrame(root, text="Actions & Takeoff", padding=10)
        actions.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

        self.arm_btn = ttk.Button(actions, text="Arm", command=self.on_arm)
        self.arm_btn.grid(row=0, column=0, padx=5)

        ttk.Label(actions, text="Altitude (m):").grid(row=0, column=1, padx=(15, 3))
        self.alt_var = tk.StringVar(value=str(DEFAULT_TAKEOFF_ALT))
        self.alt_entry = ttk.Entry(actions, textvariable=self.alt_var, width=6)
        self.alt_entry.grid(row=0, column=2)

        self.takeoff_btn = ttk.Button(actions, text="Takeoff", command=self.on_takeoff)
        self.takeoff_btn.grid(row=0, column=3, padx=5)

        self.close_btn = ttk.Button(actions, text="Close Connection", command=self.on_close)
        self.close_btn.grid(row=0, column=4, padx=(20, 5))

        # 4. Visual RC Channel Signals
        rc_frame = ttk.LabelFrame(root, text="Visual RC Channels (1000 - 2000 µs)", padding=10)
        rc_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")

        self.rc_bars = {}
        self.rc_labels = {}
        channels_info = [
            ("1", "Roll (Ch 1) - Left / Right"),
            ("2", "Pitch (Ch 2) - Up / Down"),
            ("3", "Throttle (Ch 3) - W / S"),
            ("4", "Yaw (Ch 4) - A / D"),
        ]

        for i, (ch_id, label_text) in enumerate(channels_info):
            ttk.Label(rc_frame, text=label_text, width=28, anchor="w").grid(row=i, column=0, padx=5, pady=2)
            bar = ttk.Progressbar(rc_frame, orient="horizontal", length=200, mode="determinate", maximum=1000)
            bar.grid(row=i, column=1, padx=5, pady=2)
            bar["value"] = int(self.rc_vals[ch_id]) - PWM_MIN
            lbl = ttk.Label(rc_frame, text=f"{int(self.rc_vals[ch_id])} µs", width=8)
            lbl.grid(row=i, column=2, padx=5, pady=2)
            self.rc_bars[ch_id] = bar
            self.rc_labels[ch_id] = lbl

        # 5. Status Bar
        self.status_var = tk.StringVar(value="Status: Not connected")
        ttk.Label(root, textvariable=self.status_var, relief="sunken", anchor="w", padding=6).grid(
            row=4, column=0, sticky="ew"
        )

        # Key bindings for RC control
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
        """Update mode, armed state, altitude, and connection status."""
        if self.vehicle is not None:
            mode = self.vehicle.mode.name if self.vehicle.mode else "N/A"
            armed = "ARMED" if self.vehicle.armed else "DISARMED"
            try:
                alt = self.vehicle.location.global_relative_frame.alt
                alt_str = f"{alt:.1f} m" if alt is not None else "0.0 m"
            except Exception:
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

    # --- Key Events & RC Overrides ---
    def on_key_press(self, event):
        self.pressed_keys.add(event.keysym.lower())

    def on_key_release(self, event):
        self.pressed_keys.discard(event.keysym.lower())

    def update_rc_loop(self):
        # Dynamic ramping toward target stick deflection
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

        # Smooth proportional continuous control for all 4 channels
        ramp_channel("1", "right", "left", PWM_MID)     # Roll (Ch 1) -> 1500 µs default
        ramp_channel("2", "down", "up", PWM_MID)        # Pitch (Ch 2) -> 1500 µs default
        ramp_channel("3", "w", "s", PWM_MIN)           # Throttle (Ch 3) -> 1000 µs default
        ramp_channel("4", "d", "a", PWM_MID)           # Yaw (Ch 4) -> 1500 µs default

        # Update visual progress bars and labels smoothly
        for ch, val in self.rc_vals.items():
            int_val = int(round(val))
            self.rc_bars[ch]["value"] = int_val - PWM_MIN
            self.rc_labels[ch].config(text=f"{int_val} µs")

        # Send channel overrides to vehicle if armed
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