# ===============================================================
# backend/app/mission_runner.py
# ===============================================================
"""
MissionController — manages autonomous drone missions under SITL.

Handles:
    • DroneKit vehicle connection
    • Safe waypoint retrieval (thread-isolated per drone)
    • Arm / takeoff / land control logic
    • Mission execution with telemetry and concurrency safety

This version:
    - Maintains full debug verbosity
    - Adds thread-safe waypoint and vehicle access
    - Improves per-drone isolation and mission loop stability
"""

import time
import requests
import threading
from datetime import datetime
from math import radians, sin, cos, sqrt, atan2
import collections
import collections.abc

# Fix DroneKit compatibility on Python 3.10+
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from sqlmodel import Session, select

from backend.app.models import FlightPath, Waypoint, Mission
from backend.app.database import engine

# ===============================================================
# Global constants and locks
# ===============================================================

# API_BASE = "http://localhost:8000"
API_BASE = "https://testdroneapp-production.up.railway.app/"
SAFE_DISTANCE_METERS = 2.0  # Min separation for collision avoidance
WAYPOINT_LOCKS = {}         # Per-drone DB access locks


# ===============================================================
# Mission Controller Class
# ===============================================================

class MissionController:
    """
    Manages a single drone's autonomous mission lifecycle.
    Handles connection, takeoff, navigation, pause/resume, and landing.
    Thread-safe per drone instance.
    """

    # -----------------------------------------------------------
    # Constructor
    # -----------------------------------------------------------
    def __init__(self, drone_id: int, connection_string: str):
        self.drone_id = drone_id
        self.connection_string = connection_string
        self.vehicle = None

        # Mission state
        self.flight_path = []
        self.flight_path_id = None
        self.current_wp_index = 0
        self.start_time = None
        self.avg_wp_time = 5
        self.running = False
        self.paused = False
        self.aborted = False
        self.reuse_sitl = False
        self.completed = False
        # Threading and safety
        self.thread = None
        self.vehicle_lock = threading.Lock()
        self.state_lock = threading.Lock()
        print(f"[Drone {self.drone_id}] 🧩 MissionController initialized.")

    # -----------------------------------------------------------
    # Utility Functions
    # -----------------------------------------------------------

    def _get_distance_meters(self, lat1, lon1, lat2, lon2):
        """Return distance in meters between two GPS coordinates."""
        R = 6371000  # Earth radius (m)
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c

    def _get_active_drones(self, max_age=10):
        """Fetch active drones (with valid telemetry and altitude > 5m)."""
        try:
            r = requests.get(f"{API_BASE}/telemetry", timeout=2)
            all_telem = r.json()
            now = datetime.now()
            active = []

            # Get current drone altitude (if connected)
            self_alt = 0.0
            try:
                if self.vehicle:
                    self_alt = self.vehicle.location.global_relative_frame.alt or 0.0
            except Exception:
                pass

            for t in all_telem:
                if (
                    t.get("drone_id") != self.drone_id
                    and t.get("latitude")
                    and t.get("longitude")
                    and t.get("last_updated")
                ):
                    age = (now - datetime.fromisoformat(t["last_updated"])).total_seconds()
                    alt = t.get("altitude") or 0.0
                    if alt < 5.0 or self_alt < 5.0 or age > max_age:
                        continue
                    active.append(t)

            return active

        except Exception as e:
            print(f"[Drone {self.drone_id}] ⚠️ Failed to fetch active drones: {e}")
            return []

    # -----------------------------------------------------------
    # Vehicle Connection and Reset
    # -----------------------------------------------------------

    def connect_vehicle(self):
        """Connect to SITL and ensure vehicle is ready for flight."""
        print(f"[Drone {self.drone_id}] Connecting to {self.connection_string}...")

        for attempt in range(1, 6):
            try:
                connection_string = f"udp:127.0.0.1:{14550 + ((self.drone_id - 1) * 10)}"
                self.vehicle = connect(connection_string, wait_ready=True)
                # self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
                print(f"[Drone {self.drone_id}] ✅ Connected (attempt {attempt})")
                break
            except Exception as e:
                print(f"[Drone {self.drone_id}] ⚠️ Connection attempt {attempt} failed: {e}")
                time.sleep(5)
        else:
            raise RuntimeError(f"[Drone {self.drone_id}] ❌ Unable to connect after retries")

        self.force_reset_vehicle()

        if self.reuse_sitl:
            print(f"[Drone {self.drone_id}] ♻️ Reusing SITL — skipping full reset next time.")
            return


    def force_reset_vehicle(self):
        """Perform a safe reset: disarm, stabilize, and ready GUIDED mode."""
        try:
            print(f"[Drone {self.drone_id}] 🔧 Performing safety reset...")
            sid = getattr(self.vehicle, "system_id", None)
            if sid and sid != self.drone_id:
                print(f"[Drone {self.drone_id}] 🚫 Skipping reset — vehicle sysid={sid}")
                return
            # 1. Ensure disarmed
            if self.vehicle.armed:
                print(f"[Drone {self.drone_id}] 🔧 Disarming vehicle...")
                self.vehicle.armed = False
                while self.vehicle.armed:
                    print(f"[Drone {self.drone_id}] Waiting for disarm...")
                    time.sleep(1)
            print(f"[Drone {self.drone_id}] ✅ Disarmed successfully.")

            # 2. Switch to STABILIZE
            print(f"[Drone {self.drone_id}] 🔄 Switching to STABILIZE mode...")
            self.vehicle.mode = VehicleMode("STABILIZE")
            while self.vehicle.mode.name != "STABILIZE":
                time.sleep(1)

            # 3. Wait for EKF/GPS readiness
            print(f"[Drone {self.drone_id}] 🧭 Waiting for EKF/GPS lock...")
            timeout = time.time() + 60
            while not self.vehicle.is_armable:
                time.sleep(2)
                if time.time() > timeout:
                    print(f"[Drone {self.drone_id}] ⚠️ EKF/GPS not ready — continuing.")
                    break

            # 4. Switch to GUIDED mode for next mission
            self.vehicle.mode = VehicleMode("GUIDED")
            t0 = time.time()
            while self.vehicle.mode.name != "GUIDED":
                time.sleep(1)
                if time.time() - t0 > 10:
                    self.vehicle.mode = VehicleMode("GUIDED")
                    t0 = time.time()
            print(f"[Drone {self.drone_id}] 🟢 GUIDED mode ready.")

        except Exception as e:
            print(f"[Drone {self.drone_id}] ⚠️ Vehicle reset failed: {e}")

    # -----------------------------------------------------------
    # Waypoint Handling (Thread-Safe)
    # -----------------------------------------------------------

    def get_waypoints_for_drone(self):
        """
        Safely fetch and isolate waypoints for this drone’s active mission.
        Locks DB access per-drone to prevent race conditions.
        """
        try:
            lock = WAYPOINT_LOCKS.setdefault(self.drone_id, threading.Lock())
            with lock:
                with Session(engine) as session:
                    mission = session.exec(
                        select(Mission)
                        .where(Mission.drone_id == self.drone_id)
                        .order_by(Mission.id.desc())
                    ).first()

                    if not mission:
                        print(f"[Drone {self.drone_id}] ❌ No mission found.")
                        return []

                    self.flight_path_id = mission.flight_path_id
                    if not self.flight_path_id:
                        print(f"[Drone {self.drone_id}] ⚠️ Mission has no flight path.")
                        return []

                    # Fetch and deep-copy waypoints
                    raw_wps = session.exec(
                        select(Waypoint)
                        .where(Waypoint.flight_path_id == self.flight_path_id)
                        .order_by(Waypoint.id.asc())
                    ).all()

                    if not raw_wps:
                        print(f"[Drone {self.drone_id}] ⚠️ No waypoints for flight path {self.flight_path_id}.")
                        return []

                    waypoints = [
                        (float(w.latitude), float(w.longitude), float(w.altitude or 50.0), bool(w.is_home))
                        for w in raw_wps
                    ]

                    home_wp = next((wp for wp in waypoints if wp[3]), None) or waypoints[0]
                    ordered = [home_wp] + [wp for wp in waypoints if wp != home_wp]

                    print(f"[Drone {self.drone_id}] ✅ Loaded {len(ordered)} waypoints (Home first).")
                    return [(lat, lon, alt) for lat, lon, alt, _ in ordered]

        except Exception as e:
            print(f"[Drone {self.drone_id}] ❌ Error fetching waypoints safely: {e}")
            return []

    # -----------------------------------------------------------
    # Arm and Takeoff
    # -----------------------------------------------------------

    def arm_and_takeoff(self, target_altitude):
        """Safely arm and take off to the specified altitude."""
        print(f"[Drone {self.drone_id}] 🚀 Starting arm and takeoff sequence...")

        self.force_reset_vehicle()
        self.vehicle.parameters["ARMING_CHECK"] = 0

        print(f"[Drone {self.drone_id}] Setting mode to GUIDED...")
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode.name != "GUIDED":
            print(f"[Drone {self.drone_id}] Waiting for GUIDED mode (current: {self.vehicle.mode.name})...")
            time.sleep(1)
        print(f"[Drone {self.drone_id}] ✅ GUIDED mode confirmed.")

        # Reset home position
        print(f"[Drone {self.drone_id}] 🏠 Resetting home position...")
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1, 0, 0, 0, 0,
            self.vehicle.location.global_frame.lat,
            self.vehicle.location.global_frame.lon,
            self.vehicle.location.global_frame.alt
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        time.sleep(2)

        # Arm and take off
        print(f"[Drone {self.drone_id}] Arming motors...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(f"[Drone {self.drone_id}] Waiting for arming...")
            time.sleep(1)

        print(f"[Drone {self.drone_id}] 🚀 Takeoff to {target_altitude}m")
        takeoff_msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            target_altitude
        )
        self.vehicle.send_mavlink(takeoff_msg)
        self.vehicle.flush()

        # Monitor climb
        print(f"[Drone {self.drone_id}] 📈 Climbing...")
        for _ in range(60):
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"[Drone {self.drone_id}] Altitude: {alt:.1f}m / Target: {target_altitude}m")
            if alt >= target_altitude * 0.95:
                print(f"[Drone {self.drone_id}] ✅ Target altitude reached ({alt:.1f}m)")
                break
            time.sleep(1)

        print(f"[Drone {self.drone_id}] 🛫 Takeoff complete — proceeding to waypoints.")


    # ===========================================================
    # Telemetry and Mission Execution
    # ===========================================================

    def send_telemetry(self, progress: int, eta: str):
        """
        Send live telemetry to backend.

        Includes: latitude, longitude, altitude, battery, progress, and ETA.
        """
        try:
            if not self.vehicle:
                return

            loc = self.vehicle.location.global_relative_frame
            lat, lon, alt = loc.lat, loc.lon, loc.alt

            if not lat or not lon or lat == 0.0 or lon == 0.0:
                print(f"[Drone {self.drone_id}] ⚠️ Skipping telemetry (invalid GPS).")
                return

            data = {
                "drone_id": self.drone_id,
                "latitude": lat,
                "longitude": lon,
                "altitude": alt,
                "battery": self.vehicle.battery.level if self.vehicle.battery.level else 100,
                "progress": progress,
                "eta": eta,
            }

            requests.post(f"{API_BASE}/telemetry", params=data, timeout=1)
            print(f"[Drone {self.drone_id}] 📡 Telemetry sent: {data}")

        except Exception as e:
            print(f"[Drone {self.drone_id}] ⚠️ Telemetry send failed: {e}")

    # ===========================================================
    # Mission Thread and Execution Loop
    # ===========================================================

    def start_mission(self):
        """
        Launch the mission in a background thread.
        Prevents multiple concurrent missions per drone.
        """
        if self.thread and self.thread.is_alive():
            print(f"[Drone {self.drone_id}] Mission already running.")
            return

        self.running = True
        self.paused = False
        self.aborted = False
        self.current_wp_index = 1

        print(f"[Drone {self.drone_id}] 🧭 Preparing new mission thread...")
        self.thread = threading.Thread(target=self._mission_loop, name=f"DroneMission-{self.drone_id}")
        self.thread.start()

    def _mission_loop(self):
        """Main mission loop: navigation through all waypoints."""
        try:
            if not self.vehicle:
                self.connect_vehicle()
                self.reuse_sitl = True
            retries = 5
            waypoints = None
            for attempt in range(1, retries + 1):
                waypoints = list(self.get_waypoints_for_drone())
                self.flight_path = list(self.get_waypoints_for_drone())
                if waypoints:
                    print(f"[Drone {self.drone_id}] 🗺️ Waypoints loaded on attempt {attempt}")
                    break
                print(f"[Drone {self.drone_id}] ⚠️ No waypoints (attempt {attempt}/{retries}) — retrying in {2}s...")
                time.sleep(2)

            
            # Load and freeze waypoint list for this mission
            # self.flight_path = list(self.get_waypoints_for_drone())
            # if not self.flight_path:
            #     print(f"[Drone {self.drone_id}] ❌ No waypoints — mission aborted.")
            #     return

            print(f"[Drone {self.drone_id}] 🧭 Loaded {len(waypoints)} frozen waypoints.")
            self.vehicle.wait_ready(True, timeout=60)

            # Takeoff to first waypoint altitude
            target_alt = waypoints[0][2]
            # Exclude the home waypoint for traversal
            mission_wps = waypoints[1:] if len(waypoints) > 1 else []


            self.arm_and_takeoff(target_alt)
            self.start_time = time.time()

            total_wps = len(mission_wps)
            arrival_tolerance = 2.0
            print(f"[Drone {self.drone_id}] Starting mission with {total_wps} waypoints.")

            # ------------------------------
            # Waypoint traversal loop
            # ------------------------------
            for idx, (lat, lon, alt) in enumerate(mission_wps):
                if self.aborted:
                    print(f"[Drone {self.drone_id}] 🛑 Mission aborted.")
                    break

                # LOITER if paused
                while self.paused:
                    print(f"[Drone {self.drone_id}] ⏸ Loitering mid-flight...")
                    time.sleep(2)

                print(f"[Drone {self.drone_id}] Navigating to WP {idx + 1}/{total_wps}: ({lat}, {lon}, {alt})")

                # Ensure single access to vehicle navigation
                # with self.vehicle_lock:
                target_loc = LocationGlobalRelative(lat, lon, alt)
                self.vehicle.simple_goto(target_loc)

                # Wait until close enough to target
                while True:
                    if self.aborted:
                        print(f"[Drone {self.drone_id}] 🛑 Mission aborted mid-flight.")
                        return

                    while self.paused:
                        print(f"[Drone {self.drone_id}] ⏸ Holding at current position...")
                        time.sleep(1)

                    current = self.vehicle.location.global_relative_frame
                    distance = self._get_distance_meters(current.lat, current.lon, lat, lon)
                    alt_error = abs(current.alt - alt)

                    progress = int(((idx + 1) / total_wps) * 100)
                    eta = f"{int((total_wps - idx - 1) * self.avg_wp_time // 60)}m"
                    self.send_telemetry(progress, eta)

                    print(f"[Drone {self.drone_id}] ➡️ WP{idx + 1} Dist: {distance:.1f}m AltDiff: {alt_error:.1f}m")

                    if distance <= arrival_tolerance and alt_error <= 1.0:
                        print(f"[Drone {self.drone_id}] ✅ Reached waypoint {idx + 1}.")
                        self.current_wp_index = idx + 1
                        break

                    time.sleep(1)

                wp_time = time.time() - self.start_time
                self.avg_wp_time = (self.avg_wp_time + wp_time) / 2

            # ===================================================
            # Return-to-Home Sequence
            # ===================================================
            with Session(engine) as session:
                mission = session.exec(
                    select(Mission)
                    .where(Mission.drone_id == self.drone_id)
                    .order_by(Mission.id.desc())
                ).first()

                if mission:
                    flight_path_id = mission.flight_path_id
                    wps = session.exec(
                        select(Waypoint)
                        .where(Waypoint.flight_path_id == flight_path_id)
                        .order_by(Waypoint.id.asc())
                    ).all()
                else:
                    print(f"[Drone {self.drone_id}] ⚠️ No mission record found.")
                    return

            # Ensure home waypoint is defined from frozen list
            if not waypoints or len(waypoints) == 0:
                print(f"[Drone {self.drone_id}] ⚠️ No waypoints available, skipping return.")
                return

            home_lat, home_lon, home_alt = waypoints[0]  # (lat, lon, alt)
            print(f"[Drone {self.drone_id}] 🧭 Returning home: ({home_lat}, {home_lon}, {home_alt})")


            with self.vehicle_lock:
                self.vehicle.simple_goto(LocationGlobalRelative(home_lat, home_lon, home_alt))

            while True:
                if self.aborted:
                    print(f"[Drone {self.drone_id}] 🛑 Mission aborted during return.")
                    return

                while self.paused:
                    print(f"[Drone {self.drone_id}] ⏸ Holding during return (LOITER)...")
                    if self.vehicle.mode.name != "LOITER":
                        self.vehicle.mode = VehicleMode("LOITER")
                    time.sleep(1)

                current = self.vehicle.location.global_relative_frame
                distance = self._get_distance_meters(current.lat, current.lon, home_lat, home_lon)
                alt_error = abs(current.alt - home_alt)

                self.send_telemetry(100, "Returning Home")
                print(f"[Drone {self.drone_id}] ↩️ Returning... Dist={distance:.1f}m AltDiff={alt_error:.1f}m")

                if distance <= 3.0 and alt_error <= 1.5:
                    print(f"[Drone {self.drone_id}] ✅ Home reached.")
                    break
                time.sleep(1)

            # Landing
            print(f"[Drone {self.drone_id}] 🚁 Landing sequence...")
            self.vehicle.mode = VehicleMode("LAND")
            self.send_telemetry(100, "0m 0s")
            self.running = False
            print(f"[Drone {self.drone_id}] ✅ Mission complete — landed safely.")

            # Update backend
            try:
                with Session(engine) as session:
                    mission = session.exec(
                        select(Mission)
                        .where(Mission.drone_id == self.drone_id)
                        .order_by(Mission.id.desc())
                    ).first()
                    if mission:
                        requests.post(f"{API_BASE}/missions/{mission.id}/complete")
                        print(f"[Drone {self.drone_id}] ✅ Mission {mission.id} marked complete.")
            except Exception as e:
                print(f"[Drone {self.drone_id}] ⚠️ Could not update mission completion: {e}")

        except Exception as e:
            print(f"[Drone {self.drone_id}] ❌ Mission loop error: {e}")
            self.running = False

    # ===========================================================
    # Mission Control Functions
    # ===========================================================

    def pause_mission(self):
        """Pause current mission — drone loiters in place."""
        if self.running and not self.paused:
            try:
                self.paused = True
                print(f"[Drone {self.drone_id}] ⏸ Pause requested — switching to LOITER...")
                if self.vehicle.mode.name != "LOITER":
                    self.vehicle.mode = VehicleMode("LOITER")
                    while self.vehicle.mode.name != "LOITER":
                        time.sleep(0.5)
                print(f"[Drone {self.drone_id}] 🌀 Drone holding position (LOITER mode).")
            except Exception as e:
                print(f"[Drone {self.drone_id}] ⚠️ Pause failed: {e}")

    def _find_next_waypoint(self, current_lat, current_lon):
        """Find next closest waypoint after current position."""
        if not self.flight_path:
            return None
        for (lat, lon, alt) in self.flight_path:
            dist = self._get_distance_meters(current_lat, current_lon, lat, lon)
            if dist > 5:  # arbitrary tolerance, adjust if needed
                return (lat, lon, alt)
        return self.flight_path[-1]

    def resume_mission(self):
        """Resume mission from pause — re-engage guided mode and continue."""
        if self.paused:
            try:
                self.paused = False
                print(f"[Drone {self.drone_id}] ▶️ Resume requested — switching to GUIDED mode.")

                if self.vehicle and self.vehicle.mode.name != "GUIDED":
                    self.vehicle.mode = VehicleMode("GUIDED")
                    while self.vehicle.mode.name != "GUIDED":
                        time.sleep(0.5)

                print(f"[Drone {self.drone_id}] 🧭 Back to GUIDED mode — resuming mission.")
                
                # Optional: send drone to next waypoint again (safety)
                if self.flight_path:
                    current = self.vehicle.location.global_relative_frame
                    if (self.current_wp_index + 1) >= len(self.flight_path):
                        next_wp = self.flight_path[0]
                    else:
                        next_wp = self.flight_path[self.current_wp_index + 1]
                    # next_wp = self._find_next_waypoint(current.lat, current.lon)
                    if next_wp:
                        lat, lon, alt = next_wp
                        self.vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))
                        print(f"[Drone {self.drone_id}] Continuing to next waypoint ({lat}, {lon}, {alt}).")

            except Exception as e:
                print(f"[Drone {self.drone_id}] ⚠️ Resume failed: {e}")

    def abort_mission(self):
        """
        Abort mission immediately and return to home waypoint.
        Also updates mission and drone status in backend.
        """
        if not self.running:
            print(f"[Drone {self.drone_id}] ⚠️ No active mission to abort.")
            return

        self.aborted = True
        self.running = False
        print(f"[Drone {self.drone_id}] 🛑 Abort requested — returning home.")

        try:
            with Session(engine) as session:
                mission = session.exec(
                    select(Mission)
                    .where(Mission.drone_id == self.drone_id)
                    .order_by(Mission.id.desc())
                ).first()
                if not mission:
                    self.vehicle.mode = VehicleMode("LAND")
                    print(f"[Drone {self.drone_id}] ⚠️ No mission record — landing immediately.")
                    return

                wps = session.exec(
                    select(Waypoint)
                    .where(Waypoint.flight_path_id == mission.flight_path_id)
                    .order_by(Waypoint.id.asc())
                ).all()
                home_wp = next((w for w in wps if w.is_home), wps[0] if wps else None)

            if not home_wp:
                print(f"[Drone {self.drone_id}] ⚠️ No home waypoint — landing in place.")
                self.vehicle.mode = VehicleMode("LAND")
                return

            home_lat, home_lon, home_alt = home_wp.latitude, home_wp.longitude, home_wp.altitude or 50.0
            print(f"[Drone {self.drone_id}] 🧭 Returning home for abort: ({home_lat}, {home_lon}, {home_alt})")

            self.vehicle.mode = VehicleMode("GUIDED")
            while self.vehicle.mode.name != "GUIDED":
                time.sleep(0.5)

            self.vehicle.simple_goto(LocationGlobalRelative(home_lat, home_lon, home_alt))

            # Wait until close to home
            while True:
                if self.paused:
                    print(f"[Drone {self.drone_id}] ⏸ Paused during abort.")
                    if self.vehicle.mode.name != "LOITER":
                        self.vehicle.mode = VehicleMode("LOITER")
                    while self.paused:
                        time.sleep(1)
                    self.vehicle.mode = VehicleMode("GUIDED")
                    self.vehicle.simple_goto(LocationGlobalRelative(home_lat, home_lon, home_alt))

                current = self.vehicle.location.global_relative_frame
                distance = self._get_distance_meters(current.lat, current.lon, home_lat, home_lon)
                alt_error = abs(current.alt - home_alt)
                if distance <= 3.0 and alt_error <= 2.0:
                    print(f"[Drone {self.drone_id}] ✅ Reached home position.")
                    break
                time.sleep(1)

            print(f"[Drone {self.drone_id}] 🛬 Landing after abort...")
            self.vehicle.mode = VehicleMode("LAND")
            self.send_telemetry(0, "Aborted - Returned Home")

            # Update backend states
            with Session(engine) as session:
                mission = session.exec(
                    select(Mission)
                    .where(Mission.drone_id == self.drone_id)
                    .order_by(Mission.id.desc())
                ).first()
                if mission:
                    mission.status = "planned"
                    session.add(mission)
                    print(f"[Drone {self.drone_id}] 🔁 Mission {mission.id} reset to 'planned'.")
                from app.models import Drone
                drone = session.get(Drone, self.drone_id)
                if drone:
                    drone.status = "available"
                    session.add(drone)
                    print(f"[Drone {self.drone_id}] ✅ Drone marked 'available'.")
                session.commit()

        except Exception as e:
            print(f"[Drone {self.drone_id}] ❌ Abort error: {e}")