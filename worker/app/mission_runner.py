# ===============================================================
# mission_runner.py - Worker Version (no DB, REST-driven)
# ===============================================================

import time
import requests
import threading
from datetime import datetime
from math import radians, sin, cos, sqrt, atan2

# DroneKit
import collections
import collections.abc
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

import os

# Railway backend URL (must be set in worker environment)
API_BASE = os.getenv("RAILWAY_BACKEND_URL")
if not API_BASE:
    raise RuntimeError("RAILWAY_BACKEND_URL not set in worker environment.")

SAFE_DISTANCE_METERS = 2.0


# ===============================================================
# Mission Controller
# ===============================================================

class MissionController:
    """
    Worker-side mission controller.
    No database. All mission/waypoint data fetched via REST from Railway.
    """

    def __init__(self, drone_id: int, connection_string: str):
        self.drone_id = drone_id
        self.connection_string = connection_string
        self.vehicle = None

        self.flight_path = []
        self.flight_path_id = None
        self.current_wp_index = 0
        self.start_time = None
        self.avg_wp_time = 5
        self.running = False
        self.paused = False
        self.aborted = False
        self.completed = False

        self.thread = None
        self.vehicle_lock = threading.Lock()

        print(f"[Worker][Drone {self.drone_id}] MissionController initialized.")


    # ------------------------------------------
    # Helpers
    # ------------------------------------------

    def _get_distance_meters(self, lat1, lon1, lat2, lon2):
        R = 6371000
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
        return R * 2 * atan2(sqrt(a), sqrt(1 - a))


    # ------------------------------------------
    # REST FETCH — Active Mission
    # ------------------------------------------

    def fetch_active_mission(self):
        try:
            r = requests.get(f"{API_BASE}/missions/active/{self.drone_id}", timeout=5)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Failed to fetch active mission: {e}")
            return None


    # ------------------------------------------
    # REST FETCH — Waypoints
    # ------------------------------------------

    def fetch_waypoints(self, flight_path_id):
        try:
            r = requests.get(f"{API_BASE}/flightpaths/{flight_path_id}/waypoints", timeout=5)
            r.raise_for_status()
            wps = r.json()
            formatted = [
                (float(w["latitude"]), float(w["longitude"]), float(w["altitude"] or 50.0), bool(w["is_home"]))
                for w in wps
            ]
            # Home-first ordering
            home_wp = next((wp for wp in formatted if wp[3]), None)
            if home_wp:
                ordered = [home_wp] + [wp for wp in formatted if wp != home_wp]
            else:
                ordered = formatted
            return [(lat, lon, alt) for lat, lon, alt, _ in ordered]
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Failed to fetch waypoints: {e}")
            return []


    # ------------------------------------------
    # Vehicle Connect
    # ------------------------------------------

    def connect_vehicle(self):
        print(f"[Worker][Drone {self.drone_id}] Connecting to {self.connection_string}")

        for attempt in range(1, 6):
            try:
                self.vehicle = connect(self.connection_string, wait_ready=True)
                print(f"[Worker][Drone {self.drone_id}] Connected to vehicle.")
                break
            except Exception as e:
                print(f"[Worker][Drone {self.drone_id}] Connection attempt {attempt} failed: {e}")
                time.sleep(3)
        else:
            raise RuntimeError(f"Unable to connect vehicle for drone {self.drone_id}")

        self.reset_vehicle()


    def reset_vehicle(self):
        try:
            print(f"[Worker][Drone {self.drone_id}] Resetting vehicle...")

            # Disarm
            if self.vehicle.armed:
                self.vehicle.armed = False
                while self.vehicle.armed:
                    time.sleep(1)

            # STABILIZE then GUIDED
            self.vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(2)

            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(2)

        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Reset failed: {e}")


    # ------------------------------------------
    # Takeoff
    # ------------------------------------------

    def arm_and_takeoff(self, target_alt):
        print(f"[Worker][Drone {self.drone_id}] Taking off to {target_alt}m")

        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

        takeoff_msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            target_alt
        )
        self.vehicle.send_mavlink(takeoff_msg)
        self.vehicle.flush()

        # monitor
        for _ in range(60):
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_alt * 0.95:
                break
            time.sleep(1)


    # ------------------------------------------
    # Telemetry
    # ------------------------------------------

    def send_telemetry(self, progress, eta):
        try:
            loc = self.vehicle.location.global_relative_frame
            data = {
                "drone_id": self.drone_id,
                "latitude": loc.lat,
                "longitude": loc.lon,
                "altitude": loc.alt,
                "battery": self.vehicle.battery.level if self.vehicle.battery else 100,
                "progress": progress,
                "eta": eta,
            }
            requests.post(f"{API_BASE}/telemetry", params=data, timeout=2)
        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Telemetry failed: {e}")


    # ------------------------------------------
    # Mission Thread
    # ------------------------------------------

    def start_mission(self):
        if self.thread and self.thread.is_alive():
            print(f"[Worker][Drone {self.drone_id}] Mission already running.")
            return

        self.thread = threading.Thread(target=self._run_mission)
        self.thread.start()


    def _run_mission(self):
        try:
            active = self.fetch_active_mission()
            if not active:
                print(f"[Worker][Drone {self.drone_id}] No active mission.")
                return

            self.flight_path_id = active["flight_path_id"]

            wps = self.fetch_waypoints(self.flight_path_id)
            if not wps:
                print(f"[Worker][Drone {self.drone_id}] No waypoints.")
                return

            home_alt = wps[0][2]

            self.connect_vehicle()
            self.arm_and_takeoff(home_alt)

            # traverse waypoints
            mission_wps = wps[1:]
            total = len(mission_wps)

            for idx, (lat, lon, alt) in enumerate(mission_wps):
                if self.aborted:
                    return

                target = LocationGlobalRelative(lat, lon, alt)
                self.vehicle.simple_goto(target)

                while True:
                    if self.aborted:
                        return

                    loc = self.vehicle.location.global_relative_frame
                    dist = self._get_distance_meters(loc.lat, loc.lon, lat, lon)

                    progress = int(((idx+1)/total) * 100)
                    self.send_telemetry(progress, "ETA")

                    if dist < 2.5:
                        break
                    time.sleep(1)

            # Return home
            home_lat, home_lon, home_alt = wps[0]
            self.vehicle.simple_goto(LocationGlobalRelative(home_lat, home_lon, home_alt))

            while True:
                loc = self.vehicle.location.global_relative_frame
                dist = self._get_distance_meters(loc.lat, loc.lon, home_lat, home_lon)
                if dist < 3:
                    break
                time.sleep(1)

            # Land
            self.vehicle.mode = VehicleMode("LAND")
            self.send_telemetry(100, "Landing")

            # Notify backend of completion
            requests.post(f"{API_BASE}/missions/{active['mission_id']}/complete")

        except Exception as e:
            print(f"[Worker][Drone {self.drone_id}] Mission error: {e}")


    # ------------------------------------------
    # Pause / Resume / Abort
    # ------------------------------------------

    def pause_mission(self):
        print(f"[Worker][Drone {self.drone_id}] Pausing...")
        self.paused = True
        if self.vehicle:
            self.vehicle.mode = VehicleMode("LOITER")


    def resume_mission(self):
        print(f"[Worker][Drone {self.drone_id}] Resuming...")
        self.paused = False
        if self.vehicle:
            self.vehicle.mode = VehicleMode("GUIDED")


    def abort_mission(self):
        print(f"[Worker][Drone {self.drone_id}] Aborting...")
        self.aborted = True
        if self.vehicle:
            self.vehicle.mode = VehicleMode("LAND")
