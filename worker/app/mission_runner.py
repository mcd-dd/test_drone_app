# worker/app/mission_runner.py
# Updated to match backend behavior (ARMING_CHECK disabled, force-reset, guided takeoff)
# Worker-compatible: no DB access, uses Railway REST endpoints.
import os
import time
import threading
import requests
from datetime import datetime
from math import radians, sin, cos, sqrt, atan2

# DroneKit compatibility fix for Python 3.10+
import collections
import collections.abc
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Backend URL (Railway) — must be set in worker env
API_BASE = os.getenv("RAILWAY_BACKEND_URL", "https://testdroneapp-production.up.railway.app/")
if not API_BASE:
    raise RuntimeError("RAILWAY_BACKEND_URL must be set in worker environment")

SAFE_DISTANCE_METERS = 2.0

class MissionController:
    """
    Worker-side MissionController that mirrors backend logic:
      - Fetch mission & waypoints from backend
      - Connect to local SITL (provided connection_string)
      - Force-reset vehicle, disable ARMING_CHECK, set home, arm & takeoff
      - Navigate waypoints, return home, land
      - Pause / resume / abort
    """

    def __init__(self, drone_id: int, connection_string: str):
        self.drone_id = int(drone_id)
        self.connection_string = connection_string
        self.vehicle = None

        self.flight_path = []
        self.flight_path_id = None
        self.current_wp_index = 0
        self.start_time = None
        self.avg_wp_time = 5.0

        self.running = False
        self.paused = False
        self.aborted = False
        self.reuse_sitl = False

        self.thread = None
        self.vehicle_lock = threading.Lock()
        self.state_lock = threading.Lock()

        print(f"[Worker][Drone {self.drone_id}] MissionController initialized.")

    # -------------------------
    # Helpers
    # -------------------------
    def _get_distance_meters(self, lat1, lon1, lat2, lon2):
        R = 6371000
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
        return R * 2 * atan2(sqrt(a), sqrt(1 - a))

    # -------------------------
    # REST fetches
    # -------------------------
    def fetch_active_mission(self):
        try:
            r = requests.get(f"{API_BASE}/missions/active/{self.drone_id}", timeout=5)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Failed to fetch active mission: {e}")
            return None

    def fetch_waypoints(self, flight_path_id):
        try:
            r = requests.get(f"{API_BASE}/flightpaths/{flight_path_id}/waypoints", timeout=5)
            r.raise_for_status()
            wps = r.json()
            formatted = [
                (float(w.get("latitude")), float(w.get("longitude")), float(w.get("altitude") or 50.0), bool(w.get("is_home")))
                for w in wps
            ]
            home_wp = next((wp for wp in formatted if wp[3]), None)
            if home_wp:
                ordered = [home_wp] + [wp for wp in formatted if wp != home_wp]
            else:
                ordered = formatted
            return [(lat, lon, alt) for lat, lon, alt, _ in ordered]
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Failed to fetch waypoints: {e}")
            return []

    # -------------------------
    # Connection & reset
    # -------------------------
    def connect_vehicle(self, wait_ready=True, timeout=60):
        print(f"[Worker][Drone {self.drone_id}] Connecting to {self.connection_string}")
        for attempt in range(1, 6):
            try:
                # connection_string is usually "udp:127.0.0.1:1455X"
                self.vehicle = connect(self.connection_string, wait_ready=wait_ready, timeout=timeout)
                print(f"[Worker][Drone {self.drone_id}] Connected to vehicle.")
                break
            except Exception as e:
                print(f"[Worker][Drone {self.drone_id}] Connection attempt {attempt} failed: {e}")
                time.sleep(3)
        else:
            raise RuntimeError(f"[Worker][Drone {self.drone_id}] Unable to connect to vehicle")

        # perform reset after connect
        self.force_reset_vehicle()

    def force_reset_vehicle(self):
        """
        Mirror backend safe reset:
          - disarm if armed
          - switch to STABILIZE then GUIDED
          - wait for is_armable up to a timeout (but continue if not ready)
        """
        try:
            print(f"[Worker][Drone {self.drone_id}] Performing force reset...")

            if not self.vehicle:
                return

            # Disarm if needed
            try:
                if self.vehicle.armed:
                    print(f"[Worker][Drone {self.drone_id}] Disarming vehicle...")
                    self.vehicle.armed = False
                    counter = 0
                    while self.vehicle.armed and counter < 10:
                        time.sleep(1)
                        counter += 1
            except Exception:
                pass

            # Switch to STABILIZE then GUIDED
            try:
                self.vehicle.mode = VehicleMode("STABILIZE")
                start = time.time()
                while self.vehicle.mode.name != "STABILIZE" and time.time() - start < 10:
                    time.sleep(0.5)
                self.vehicle.mode = VehicleMode("GUIDED")
                start = time.time()
                while self.vehicle.mode.name != "GUIDED" and time.time() - start < 10:
                    time.sleep(0.5)
            except Exception as e:
                print(f"[Worker][Drone {self.drone_id}] Mode switch warning: {e}")

            # Wait for armable or proceed after timeout
            print(f"[Worker][Drone {self.drone_id}] Waiting for is_armable (short timeout)...")
            timeout = time.time() + 30
            while not getattr(self.vehicle, "is_armable", True):
                if time.time() > timeout:
                    print(f"[Worker][Drone {self.drone_id}] is_armable timeout — continuing (ARMING_CHECK will be disabled).")
                    break
                time.sleep(1)

        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] force_reset_vehicle error: {e}")

    # -------------------------
    # Arm & takeoff (disable ARMING_CHECK)
    # -------------------------
    def arm_and_takeoff(self, target_altitude):
        print(f"[Worker][Drone {self.drone_id}] Arm and takeoff to {target_altitude}m")

        if not self.vehicle:
            raise RuntimeError("Vehicle not connected")

        # disable arming checks to allow arming without GPS/EKF in SITL
        try:
            print(f"[Worker][Drone {self.drone_id}] Disabling ARMING_CHECK (ARMING_CHECK=0)")
            self.vehicle.parameters["ARMING_CHECK"] = 0
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Warning disabling ARMING_CHECK: {e}")

        # ensure GUIDED mode
        try:
            self.vehicle.mode = VehicleMode("GUIDED")
            start = time.time()
            while self.vehicle.mode.name != "GUIDED" and time.time() - start < 15:
                time.sleep(0.5)
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Mode set to GUIDED warning: {e}")

        # reset home to current global frame
        try:
            lat = self.vehicle.location.global_frame.lat
            lon = self.vehicle.location.global_frame.lon
            alt = self.vehicle.location.global_frame.alt or target_altitude
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                1, 0, 0, 0, 0,
                lat, lon, alt
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            time.sleep(1)
            print(f"[Worker][Drone {self.drone_id}] Home set to {lat},{lon},{alt}")
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Set home failed: {e}")

        # arm
        try:
            print(f"[Worker][Drone {self.drone_id}] Arming...")
            self.vehicle.armed = True
            counter = 0
            while not self.vehicle.armed and counter < 30:
                time.sleep(1)
                counter += 1
            if not self.vehicle.armed:
                print(f"[Worker][Drone {self.drone_id}] Warning: vehicle did not arm after timeout")
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Arm error: {e}")

        # takeoff command (MAV_CMD_NAV_TAKEOFF)
        try:
            takeoff_msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0,
                float(target_altitude)
            )
            self.vehicle.send_mavlink(takeoff_msg)
            self.vehicle.flush()
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Takeoff send failed: {e}")

        # monitor climb
        for _ in range(60):
            try:
                alt = self.vehicle.location.global_relative_frame.alt or 0.0
                print(f"[Worker][Drone {self.drone_id}] Altitude: {alt:.1f} / Target: {target_altitude:.1f}")
                if alt >= target_altitude * 0.95:
                    print(f"[Worker][Drone {self.drone_id}] Reached target altitude.")
                    break
            except Exception:
                pass
            time.sleep(1)

    # -------------------------
    # Telemetry
    # -------------------------
    def send_telemetry(self, progress, eta):
        try:
            if not self.vehicle:
                return
            loc = self.vehicle.location.global_relative_frame
            data = {
                "drone_id": self.drone_id,
                "latitude": loc.lat,
                "longitude": loc.lon,
                "altitude": loc.alt,
                "battery": getattr(self.vehicle, "battery", {}).level if getattr(self.vehicle, "battery", None) else 100,
                "progress": int(progress),
                "eta": eta,
            }
            requests.post(f"{API_BASE}/telemetry", params=data, timeout=2)
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Telemetry failed: {e}")

    # -------------------------
    # Mission lifecycle
    # -------------------------
    def start_mission(self):
        if self.thread and self.thread.is_alive():
            print(f"[Worker][Drone {self.drone_id}] Mission already running.")
            return
        self.running = True   # <-- FIX
        self.aborted = False  # reset abort flag
        self.paused = False   # reset pause
        self.thread = threading.Thread(target=self._mission_loop, name=f"Mission-{self.drone_id}")
        self.thread.start()

    def _mission_loop(self):
        try:
            active = self.fetch_active_mission()
            if not active:
                print(f"[Worker][Drone {self.drone_id}] No active mission.")
                return

            self.flight_path_id = active.get("flight_path_id")
            wps = self.fetch_waypoints(self.flight_path_id)
            if not wps:
                print(f"[Worker][Drone {self.drone_id}] No waypoints for mission.")
                return

            # first waypoint treated as home
            home_alt = wps[0][2] if wps else 50.0
            mission_wps = wps[1:] if len(wps) > 1 else []

            # connect vehicle and takeoff
            if not self.vehicle:
                self.connect_vehicle()
                self.reuse_sitl = True

            # ensure vehicle ready and takeoff
            try:
                self.arm_and_takeoff(home_alt)
            except Exception as e:
                print(f"[Worker][Drone {self.drone_id}] arm_and_takeoff failed: {e}")

            self.start_time = time.time()
            total = len(mission_wps)
            if total == 0:
                print(f"[Worker][Drone {self.drone_id}] No mission waypoints to traverse.")
            for idx, (lat, lon, alt) in enumerate(mission_wps):
                if self.aborted:
                    print(f"[Worker][Drone {self.drone_id}] Mission aborted.")
                    break

                # handle pause
                while self.paused:
                    print(f"[Worker][Drone {self.drone_id}] Paused — loitering.")
                    time.sleep(1)

                print(f"[Worker][Drone {self.drone_id}] Going to WP {idx+1}/{total}: {lat},{lon},{alt}")
                try:
                    self.vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))
                except Exception as e:
                    print(f"[Worker][Drone {self.drone_id}] simple_goto error: {e}")

                # loop until close enough
                while True:
                    if self.aborted:
                        break
                    if self.paused:
                        time.sleep(1)
                        continue
                    try:
                        cur = self.vehicle.location.global_relative_frame
                        dist = self._get_distance_meters(cur.lat, cur.lon, lat, lon)
                        alt_err = abs((cur.alt or 0) - alt)
                        progress = int(((idx+1)/max(total,1)) * 100)
                        self.send_telemetry(progress, "ETA")
                        print(f"[Worker][Drone {self.drone_id}] Dist: {dist:.1f}m AltErr: {alt_err:.1f}m")
                        if dist <= 3.0 and alt_err <= 1.5:
                            print(f"[Worker][Drone {self.drone_id}] Reached WP {idx+1}")
                            break
                    except Exception:
                        pass
                    time.sleep(1)

            # return home
            if wps:
                home_lat, home_lon, home_alt = wps[0]
                print(f"[Worker][Drone {self.drone_id}] Returning home to {home_lat},{home_lon},{home_alt}")
                try:
                    self.vehicle.simple_goto(LocationGlobalRelative(home_lat, home_lon, home_alt))
                except Exception as e:
                    print(f"[Worker][Drone {self.drone_id}] return simple_goto error: {e}")

                # wait until near home
                while True:
                    if self.aborted:
                        break
                    cur = self.vehicle.location.global_relative_frame
                    dist = self._get_distance_meters(cur.lat, cur.lon, home_lat, home_lon)
                    alt_err = abs((cur.alt or 0) - home_alt)
                    self.send_telemetry(100, "Returning Home")
                    if dist <= 3.0 and alt_err <= 1.5:
                        print(f"[Worker][Drone {self.drone_id}] Home reached.")
                        break
                    time.sleep(1)

            # land
            try:
                print(f"[Worker][Drone {self.drone_id}] Landing...")
                self.vehicle.mode = VehicleMode("LAND")
                self.send_telemetry(100, "Landing")
            except Exception as e:
                print(f"[Worker][Drone {self.drone_id}] Land command failed: {e}")

            # notify backend on completion
            try:
                mission_id = active.get("mission_id")
                if mission_id:
                    requests.post(f"{API_BASE}/missions/{mission_id}/complete", timeout=3)
                    print(f"[Worker][Drone {self.drone_id}] Mission {mission_id} marked complete.")
            except Exception as e:
                print(f"[Worker][Drone {self.drone_id}] Failed to notify backend of completion: {e}")

        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Mission loop error: {e}")
        finally:
            self.running = False

    # -------------------------
    # Control functions
    # -------------------------
    def pause_mission(self):
        if not self.running:
            print(f"[Worker][Drone {self.drone_id}] No running mission to pause.")
            return
        self.paused = True
        try:
            if self.vehicle and self.vehicle.mode.name != "LOITER":
                self.vehicle.mode = VehicleMode("LOITER")
            print(f"[Worker][Drone {self.drone_id}] Paused (LOITER).")
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Pause failed: {e}")

    def resume_mission(self):
        if not self.paused:
            print(f"[Worker][Drone {self.drone_id}] Not paused.")
            return
        self.paused = False
        try:
            if self.vehicle and self.vehicle.mode.name != "GUIDED":
                self.vehicle.mode = VehicleMode("GUIDED")
            print(f"[Worker][Drone {self.drone_id}] Resumed (GUIDED).")
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Resume failed: {e}")

    def abort_mission(self):
        print(f"[Worker][Drone {self.drone_id}] Abort requested.")
        self.aborted = True
        # try to send vehicle home/land
        try:
            if self.vehicle:
                self.vehicle.mode = VehicleMode("GUIDED")
                # attempt immediate return to home (use last known wps[0] as fallback)
                # Backend will update mission state after abort via API calls from main worker endpoints.
                print(f"[Worker][Drone {self.drone_id}] Guiding to home for abort/land.")
                # let mission loop handle landing if it sees aborted flag
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Abort handling error: {e}")
