"""
Drone control panel (Tkinter).

  [Connection string] -> [Connect] -> [Mode buttons / Arm / Takeoff] -> [Close Connection]

Run inside the container (needs a display):  python3 control_panel.py
"""
import queue
import threading
import tkinter as tk
from tkinter import ttk

from dronekit import VehicleMode, connect

DEFAULT_CONNECTION = "127.0.0.1:14550"
MODES = ["GUIDED", "STABILIZE", "LOITER", "LAND", "RTL"]
DEFAULT_TAKEOFF_ALT = 10      # meters
STATUS_REFRESH_MS = 500
QUEUE_POLL_MS = 100


class ControlPanel:
    def __init__(self, root):
        self.root = root
        self.vehicle = None
        self.results = queue.Queue()
        root.title("Drone Control Panel")
        root.resizable(False, False)

        # ---- BLOCK 1: Connection row ----
        top = ttk.Frame(root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        ttk.Label(top, text="Connection string:").grid(row=0, column=0, padx=(0, 5))
        self.conn_var = tk.StringVar(value=DEFAULT_CONNECTION)
        self.conn_entry = ttk.Entry(top, textvariable=self.conn_var, width=28)
        self.conn_entry.grid(row=0, column=1, padx=5)
        self.connect_btn = ttk.Button(top, text="Connect", command=self.on_connect)
        self.connect_btn.grid(row=0, column=2, padx=5)

        # ---- BLOCK 2: Flight mode buttons ----
        modes = ttk.LabelFrame(root, text="Flight mode", padding=10)
        modes.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        self.mode_buttons = []
        for i, mode in enumerate(MODES):
            btn = ttk.Button(modes, text=mode.capitalize(),
                             command=lambda m=mode: self.set_mode(m))
            btn.grid(row=0, column=i, padx=3)
            self.mode_buttons.append(btn)

        # ---- BLOCK 3: Arm and close buttons ----
        actions = ttk.Frame(root, padding=10)
        actions.grid(row=2, column=0, sticky="ew")
        self.arm_btn = ttk.Button(actions, text="Arm", command=self.on_arm)
        self.arm_btn.grid(row=0, column=0, padx=5)
        self.close_btn = ttk.Button(actions, text="Close Connection",
                                    command=self.on_close)
        self.close_btn.grid(row=0, column=1, padx=5)

        # takeoff: altitude input + button
        ttk.Label(actions, text="Altitude (m):").grid(row=0, column=2, padx=(15, 3))
        self.alt_var = tk.StringVar(value=str(DEFAULT_TAKEOFF_ALT))
        self.alt_entry = ttk.Entry(actions, textvariable=self.alt_var, width=6)
        self.alt_entry.grid(row=0, column=3)
        self.takeoff_btn = ttk.Button(actions, text="Takeoff", command=self.on_takeoff)
        self.takeoff_btn.grid(row=0, column=4, padx=5)

        # ---- BLOCK 4: Status bar ----
        self.status_var = tk.StringVar(value="Not connected")
        ttk.Label(root, textvariable=self.status_var, relief="sunken",
                  anchor="w", padding=4).grid(row=3, column=0, sticky="ew")

        self.set_connected_ui(False)
        root.protocol("WM_DELETE_WINDOW", self.on_exit)
        self.refresh_status()
        self.poll_results()

    # ---------- UI helpers ----------
    def set_connected_ui(self, connected):
        state = "normal" if connected else "disabled"
        for btn in self.mode_buttons + [self.arm_btn, self.close_btn,
                                         self.alt_entry, self.takeoff_btn]:
            btn.config(state=state)
        self.connect_btn.config(state="disabled" if connected else "normal")
        self.conn_entry.config(state="disabled" if connected else "normal")

    def refresh_status(self):
        """Show mode and armed state of the vehicle, twice per second."""
        if self.vehicle is not None:
            armed = "ARMED" if self.vehicle.armed else "disarmed"
            alt = self.vehicle.location.global_relative_frame.alt
            self.status_var.set("Connected | Mode: %s | %s | Alt: %.1f m"
                                % (self.vehicle.mode.name, armed, alt or 0))
        self.root.after(STATUS_REFRESH_MS, self.refresh_status)

    # ---------- Connect ----------
    def on_connect(self):
        self.connect_btn.config(state="disabled")
        self.status_var.set("Connecting...")
        # connect() blocks for several seconds, so run it off the UI thread
        threading.Thread(target=self.connect_worker,
                         args=(self.conn_var.get().strip(),), daemon=True).start()

    def connect_worker(self, conn_string):
        try:
            self.results.put(("connected", connect(conn_string, wait_ready=True, timeout=30)))
        except Exception as e:
            self.results.put(("failed", e))

    def poll_results(self):
        """Runs in the UI thread: handle results posted by worker threads."""
        try:
            while True:
                kind, value = self.results.get_nowait()
                if kind == "connected":
                    self.connect_done(value)
                else:
                    self.connect_failed(value)
        except queue.Empty:
            pass
        self.root.after(QUEUE_POLL_MS, self.poll_results)

    def connect_done(self, vehicle):
        self.vehicle = vehicle
        self.set_connected_ui(True)

    def connect_failed(self, error):
        self.status_var.set("Connection failed: %s" % error)
        self.connect_btn.config(state="normal")

    # ---------- Commands ----------
    def set_mode(self, mode):
        self.vehicle.mode = VehicleMode(mode)

    def on_arm(self):
        if not self.vehicle.is_armable:
            self.status_var.set("Cannot arm yet: vehicle not armable (wait for GPS/EKF)")
            return
        self.vehicle.mode = VehicleMode("GUIDED")   # arming from code needs GUIDED
        self.vehicle.armed = True

    def on_takeoff(self):
        try:
            altitude = float(self.alt_var.get())
            if altitude <= 0:
                raise ValueError
        except ValueError:
            self.status_var.set("Enter a takeoff altitude in meters (a number above 0)")
            return
        if not self.vehicle.armed:
            self.status_var.set("Arm the vehicle first")
            return
        self.vehicle.mode = VehicleMode("GUIDED")   # simple_takeoff needs GUIDED
        self.vehicle.simple_takeoff(altitude)

    def on_close(self):
        vehicle, self.vehicle = self.vehicle, None
        self.set_connected_ui(False)
        self.status_var.set("Not connected")
        threading.Thread(target=vehicle.close, daemon=True).start()

    def on_exit(self):
        if self.vehicle is not None:
            self.vehicle.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    ControlPanel(root)
    root.mainloop()
