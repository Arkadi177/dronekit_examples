import sys
import threading
import time
from pymavlink import mavutil


class TerminalGCS:

  def __init__(self, default_connection="127.0.0.1:14550"):
    self.connection_string = default_connection
    self.master = None
    self.is_connected = False
    self.heartbeat_thread = None

  def connect(self, conn_str=None):
    if conn_str:
      self.connection_string = conn_str

    print(f"\n[+] Connecting to drone on {self.connection_string}...")
    try:
      self.master = mavutil.mavlink_connection(self.connection_string)

      # 1. Send initial GCS heartbeat packet
      self.master.mav.heartbeat_send(
          mavutil.mavlink.MAV_TYPE_GCS,
          mavutil.mavlink.MAV_AUTOPILOT_INVALID,
          0,
          0,
          0,
      )

      # 2. Wait for heartbeat response from SITL
      hb = self.master.wait_heartbeat(timeout=10)
      if hb:
        self.master.target_system = hb.get_srcSystem()
        self.master.target_component = hb.get_srcComponent()
        print(
            f"[✓] Connected! System ID: {self.master.target_system}, Component"
            f" ID: {self.master.target_component}"
        )
      else:
        self.master.target_system = 1
        self.master.target_component = 1
        print(
            "[!] No heartbeat response in 10s. Defaulting System ID to 1 and"
            " Component ID to 1."
        )

      self.is_connected = True

      self.heartbeat_thread = threading.Thread(
          target=self._send_heartbeat_loop, daemon=True
      )
      self.heartbeat_thread.start()

    except Exception as e:
      print(f"[X] Connection error: {e}")
      self.is_connected = False

  def _send_heartbeat_loop(self):
    """Sends 1Hz GCS heartbeats so SITL doesn't drop the connection while waiting for terminal input."""
    while self.is_connected and self.master:
      try:
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        time.sleep(1)
      except Exception:
        break

  def set_mode(self, mode_name):
    if not self.is_connected or not self.master:
      print("[!] Drone is not connected. Connect first (Option 1).")
      return

    try:
      mode_mapping = self.master.mode_mapping()
      if not mode_mapping or mode_name.upper() not in mode_mapping:
        print(f"[X] Mode '{mode_name}' is not supported by autopilot.")
        return

      mode_id = mode_mapping[mode_name.upper()]

      self.master.mav.set_mode_send(
          self.master.target_system,
          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
          mode_id,
      )
      print(f"[✓] Flight mode change sent: {mode_name.upper()}")

    except Exception as e:
      print(f"[X] Could not set mode: {e}")

  def arm_drone(self):
    if not self.is_connected or not self.master:
      print("[!] Drone is not connected. Connect first (Option 1).")
      return

    try:
      sys_id = (
          self.master.target_system if self.master.target_system != 0 else 1
      )
      comp_id = (
          self.master.target_component
          if self.master.target_component != 0
          else 1
      )

      self.master.mav.command_long_send(
          sys_id,
          comp_id,
          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
          0,
          1,
          0,
          0,
          0,
          0,
          0,
          0,
      )
      print("[✓] Arm command sent!")

    except Exception as e:
      print(f"[X] Could not arm drone: {e}")

  def close_connection(self):
    if self.is_connected:
      self.is_connected = False
      if self.master:
        self.master.close()
        self.master = None
      print("[✓] Connection closed.")
    else:
      print("[!] Already disconnected.")


def print_menu():
  print("\n" + "=" * 45)
  print("         TERMINAL GROUND CONTROL STATION     ")
  print("=" * 45)
  print("  1. Connect to Drone")
  print("  2. Change Flight Mode (GUIDED, STABILIZE, RTL...)")
  print("  3. Arm Drone")
  print("  4. Close Connection")
  print("  5. Exit")
  print("=" * 45)


def main():
  default_conn = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:14550"
  gcs = TerminalGCS(default_connection=default_conn)

  while True:
    print_menu()
    choice = input("Enter option (1-5): ").strip()

    if choice == "1":
      conn_input = (
          input(
              f"Enter connection string [Press Enter for default:"
              f" {gcs.connection_string}]: "
          ).strip()
          or gcs.connection_string
      )
      gcs.connect(conn_input)

    elif choice == "2":
      mode = input("Enter one of this modes (GUIDED / STABILIZE / LOITER / LAND / RTL): ").strip()
      if mode:
        gcs.set_mode(mode)

    elif choice == "3":
      confirm = (
          input("Are you sure you want to ARM the drone? (Y/N): ")
          .strip()
          .lower()
      )
      if confirm == "y":
        gcs.arm_drone()
      else:
        print("[!] Arming cancelled.")

    elif choice == "4":
      gcs.close_connection()

    elif choice == "5":
      print("Exiting Ground Control Station...")
      gcs.close_connection()
      break

    else:
      print("[!] Invalid option. Please enter a number from 1 to 5.")


if __name__ == "__main__":
  main()