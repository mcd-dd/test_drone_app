# ===============================================================
# backend/app/main.py
# ===============================================================
"""
Main FastAPI backend for Drone Mission Planner.

Features:
    - CRUD APIs for drones, surveys, flight paths, and missions.
    - Manages multiple concurrent SITL instances.
    - Controls drone missions via MissionController.
    - Supports pause, resume, and abort operations.
"""

from fastapi import FastAPI, Depends, HTTPException, BackgroundTasks
from sqlmodel import Session, select
from typing import List
import subprocess
import os
import time
from threading import Lock

from backend.app import crud, models
from backend.app.database import init_db, get_session
from backend.app.mission_runner import MissionController

# ===============================================================
# GLOBALS AND CONFIG
# ===============================================================

app = FastAPI(title="Mission Planner API")

# Thread locks (prevent concurrent DB/SITL setup conflicts)
controller_lock = Lock()
mission_lock = Lock()

# Track active mission controllers and SITL processes
mission_controllers = {}
active_missions = {}

# Base ports for SITL instances
SITL_BASE_UDP_PORT = 14550
SITL_BASE_TCP_PORT = 5760

# ===============================================================
# STARTUP EVENT
# ===============================================================

@app.on_event("startup")
def on_startup():
    """Initialize database on startup."""
    init_db()
    print("✅ Database initialized successfully.")

# ===============================================================
# 🛩️ DRONE ENDPOINTS
# ===============================================================

@app.post("/drones")
def create_drone(name: str, battery_level: int = 100, session: Session = Depends(get_session)):
    """Create a new drone."""
    return crud.create_drone(session, name, battery_level)

@app.get("/drones")
def list_drones(session: Session = Depends(get_session)):
    """List all drones."""
    return crud.list_drones(session)

@app.get("/drones/{drone_id}")
def get_drone(drone_id: int, session: Session = Depends(get_session)):
    """Get single drone by ID."""
    drone = crud.get_drone(session, drone_id)
    if not drone:
        raise HTTPException(status_code=404, detail="Drone not found")
    return drone

@app.delete("/drones/{drone_id}")
def delete_drone(drone_id: int, session: Session = Depends(get_session)):
    """Delete a drone by ID."""
    success = crud.delete_drone(session, drone_id)
    if not success:
        raise HTTPException(status_code=404, detail="Drone not found")
    return {"message": f"Drone {drone_id} deleted successfully"}

@app.get("/drones/available")
def available_drones(session: Session = Depends(get_session)):
    """List all available drones (not in mission)."""
    return crud.get_available_drones(session)

# ===============================================================
# 🗺️ SURVEY ENDPOINTS
# ===============================================================

@app.post("/surveys")
def create_survey(name: str, area: str = "", session: Session = Depends(get_session)):
    """Create a new survey."""
    return crud.create_survey(session, name, area)

@app.get("/surveys")
def list_surveys(session: Session = Depends(get_session)):
    """List all surveys."""
    return crud.list_surveys(session)

@app.get("/surveys/{survey_id}")
def get_survey(survey_id: int, session: Session = Depends(get_session)):
    """Retrieve survey details."""
    survey = crud.get_survey(session, survey_id)
    if not survey:
        raise HTTPException(status_code=404, detail="Survey not found")
    return survey

@app.delete("/surveys/{survey_id}")
def delete_survey(survey_id: int, session: Session = Depends(get_session)):
    """Delete a survey."""
    success = crud.delete_survey(session, survey_id)
    if not success:
        raise HTTPException(status_code=404, detail="Survey not found")
    return {"message": f"Survey {survey_id} deleted successfully"}

@app.post("/surveys/{survey_id}/assign")
def assign_flightpath_to_survey(survey_id: int, flightpath_id: int, session: Session = Depends(get_session)):
    """Assign a flight path to a survey."""
    return crud.assign_flightpath_to_survey(session, survey_id, flightpath_id)

@app.post("/surveys/{survey_id}/assign_drone")
def assign_drone_to_survey(survey_id: int, drone_id: int, session: Session = Depends(get_session)):
    """Assign a drone to a survey."""
    return crud.assign_drone_to_survey(session, survey_id, drone_id)

@app.delete("/surveys/{survey_id}/remove_fp/{fp_id}")
def remove_flightpath_from_survey(survey_id: int, fp_id: int, session: Session = Depends(get_session)):
    """Remove a flight path from a survey."""
    success = crud.remove_flightpath_from_survey(session, survey_id, fp_id)
    if not success:
        raise HTTPException(status_code=404, detail="Assignment not found")
    return {"message": f"Flight path {fp_id} removed from survey {survey_id}"}

@app.delete("/surveys/{survey_id}/remove_drone/{drone_id}")
def remove_drone_from_survey(survey_id: int, drone_id: int, session: Session = Depends(get_session)):
    """Remove a drone from a survey."""
    success = crud.remove_drone_from_survey(session, survey_id, drone_id)
    if not success:
        raise HTTPException(status_code=404, detail="Assignment not found")
    return {"message": f"Drone {drone_id} removed from survey {survey_id}"}

# ===============================================================
# ✈️ FLIGHT PATH ENDPOINTS
# ===============================================================

@app.post("/flightpaths")
def create_flightpath(name: str, survey_area: str = "", altitude: float = 100.0, session: Session = Depends(get_session)):
    """Create a new flight path."""
    return crud.create_flight_path(session, name, survey_area, altitude)

@app.get("/flightpaths")
def list_flightpaths(session: Session = Depends(get_session)):
    """List all flight paths."""
    return crud.list_flight_paths(session)

@app.get("/flightpaths/{fp_id}")
def get_flightpath(fp_id: int, session: Session = Depends(get_session)):
    """Get flight path details."""
    fp = crud.get_flight_path(session, fp_id)
    if not fp:
        raise HTTPException(status_code=404, detail="Flight path not found")
    return fp

@app.delete("/flightpaths/{fp_id}")
def delete_flightpath(fp_id: int, session: Session = Depends(get_session)):
    """Delete a flight path."""
    success = crud.delete_flight_path(session, fp_id)
    if not success:
        raise HTTPException(status_code=404, detail="Flight path not found")
    return {"message": f"Flight path {fp_id} deleted successfully"}

# ===============================================================
# 📍 WAYPOINT ENDPOINTS
# ===============================================================

@app.post("/flightpaths/{fp_id}/waypoints")
def add_waypoint(fp_id: int, latitude: float, longitude: float, altitude: float = 50.0, session: Session = Depends(get_session)):
    """Add a waypoint to a flight path."""
    return crud.add_waypoint(session, fp_id, latitude, longitude, altitude)

@app.get("/flightpaths/{fp_id}/waypoints")
def list_waypoints(fp_id: int, session: Session = Depends(get_session)):
    """List all waypoints in a flight path."""
    return crud.list_waypoints(session, fp_id)

@app.delete("/flightpaths/{fp_id}/waypoints/{wp_id}")
def delete_waypoint(fp_id: int, wp_id: int, session: Session = Depends(get_session)):
    """Delete a waypoint."""
    success = crud.delete_waypoint(session, fp_id, wp_id)
    if not success:
        raise HTTPException(status_code=404, detail="Waypoint not found")
    return {"message": f"Waypoint {wp_id} removed from flight path {fp_id}"}

# ===============================================================
# ⚙️ PARAMETERS
# ===============================================================

@app.post("/flightpaths/{fp_id}/parameters")
def set_parameters(fp_id: int, frequency_hz: int = 1, sensors: str = "camera", battery_threshold: int = 30, session: Session = Depends(get_session)):
    """Set data collection parameters for a flight path."""
    sensors_list = [s.strip() for s in sensors.split(",")]
    return crud.set_data_parameters(session, fp_id, frequency_hz, sensors_list, battery_threshold)

# ===============================================================
# 🎯 MISSION ENDPOINTS
# ===============================================================

@app.post("/missions/assign")
def assign_mission(flight_path_id: int, drone_id: int, session: Session = Depends(get_session)):
    """Assign a mission (flight path to a specific drone)."""
    try:
        return crud.assign_mission(session, flight_path_id, drone_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/missions/{mission_id}/complete")
def complete_mission(mission_id: int, session: Session = Depends(get_session)):
    """Mark a mission as complete."""
    mission = crud.complete_mission(session, mission_id)
    if not mission:
        raise HTTPException(status_code=404, detail="Mission not found")

    drone = session.get(models.Drone, mission.drone_id)
    if drone:
        drone.status = "available"
        session.add(drone)
        session.commit()
    return mission

# ===============================================================
# 🚀 MISSION CONTROL: START / PAUSE / RESUME / ABORT
# ===============================================================

@app.post("/missions/start")
def start_mission(
    drone_id: int,
    background_tasks: BackgroundTasks,
    session: Session = Depends(get_session),
):
    """
    Start or resume a mission for a specific drone.
    - Launches or reuses SITL and MAVProxy.
    - Creates/updates a MissionController for this drone.
    - Starts mission in background thread.
    """

    with mission_lock:
        # 1️⃣ Find latest planned/in_progress mission
        mission = session.exec(
            select(models.Mission)
            .where(
                models.Mission.drone_id == drone_id,
                models.Mission.status.in_(["planned", "in_progress"]),
            )
            .order_by(models.Mission.id.desc())
        ).first()

        if not mission:
            raise HTTPException(status_code=404, detail=f"No mission found for drone {drone_id}")

        # 2️⃣ Prevent flight-path conflicts across drones
        conflict = session.exec(
            select(models.Mission)
            .where(
                models.Mission.flight_path_id == mission.flight_path_id,
                models.Mission.drone_id != drone_id,
                models.Mission.status.in_(["planned", "in_progress"]),
            )
        ).first()

        if conflict:
            raise HTTPException(
                status_code=400,
                detail=f"Flight path {mission.flight_path_id} already used by Drone {conflict.drone_id}.",
            )

    print(f"🛰 Drone {drone_id} → FlightPath {mission.flight_path_id}")

    # 3️⃣ Prepare home position & SITL coordinates
    waypoints = session.exec(
        select(models.Waypoint).where(models.Waypoint.flight_path_id == mission.flight_path_id)
    ).all()
    if not waypoints:
        raise HTTPException(status_code=400, detail="No waypoints found for flight path")

    first_wp = waypoints[0]
    home_lat, home_lon, home_alt = first_wp.latitude, first_wp.longitude, first_wp.altitude or 50.0

    # Offset to avoid overlapping SITL spawn positions
    lat_offset = 0.0005 * (drone_id - 1)
    lon_offset = 0.0005 * (drone_id - 1)
    sitl_lat = home_lat + lat_offset
    sitl_lon = home_lon + lon_offset
    sitl_alt = home_alt + (20 * (drone_id - 1))

    sitl_udp = SITL_BASE_UDP_PORT + ((drone_id - 1) * 10)
    sitl_tcp = SITL_BASE_TCP_PORT + ((drone_id - 1) * 10)
    conn_string = f"udp:127.0.0.1:{sitl_udp}"

    # 4️⃣ Ensure a home waypoint exists
    existing_home = session.exec(
        select(models.Waypoint)
        .where(
            models.Waypoint.flight_path_id == mission.flight_path_id,
            models.Waypoint.is_home == True,
        )
    ).first()
    if not existing_home:
        home_wp = models.Waypoint(
            flight_path_id=mission.flight_path_id,
            latitude=sitl_lat,
            longitude=sitl_lon,
            altitude=sitl_alt,
            is_home=True,
        )
        session.add(home_wp)
        session.commit()
        print(f"🏠 Added home waypoint for flight path {mission.flight_path_id}")

    # 5️⃣ Launch or restart SITL
    sitl_active = drone_id in active_missions and active_missions[drone_id].poll() is None
    if sitl_active:
        print(f"♻️ Restarting existing SITL for Drone {drone_id}")
        try:
            active_missions[drone_id].terminate()
            active_missions[drone_id].wait(timeout=5)
        except Exception:
            pass

    sitl_cmd = [
        "sim_vehicle.py",
        "-v", "ArduCopter",
        f"-I{drone_id - 1}",
        "--sysid", str(drone_id),
        "--map",
        "--console",
        f"--mavproxy-args=--out=udp:127.0.0.1:{sitl_udp}",
        f"--custom-location={sitl_lat},{sitl_lon},{sitl_alt},0",
    ]
    try:
        proc = subprocess.Popen(
            sitl_cmd,
            cwd=os.path.expanduser("~/Desktop/ardupilot/ArduCopter"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        active_missions[drone_id] = proc
        print(f"✅ SITL started for Drone {drone_id} (UDP {sitl_udp})")
        time.sleep(8)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start SITL: {e}")

    # 6️⃣ Launch MAVProxy bridge
    mavproxy_cmd = [
        "mavproxy.py",
        f"--master=tcp:127.0.0.1:{sitl_tcp}",
        f"--out=udp:127.0.0.1:{sitl_udp}",
        "--console",
        "--map",
    ]
    try:
        mp_proc = subprocess.Popen(mavproxy_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        active_missions[f"mavproxy_{drone_id}"] = mp_proc
        print(f"✅ MAVProxy started for Drone {drone_id}")
    except Exception as e:
        print(f"⚠️ MAVProxy launch failed for Drone {drone_id}: {e}")

    # 7️⃣ Update DB states
    mission.status = "in_progress"
    drone = session.get(models.Drone, drone_id)
    if drone:
        drone.status = "in_mission"
        session.add(drone)
    session.add(mission)
    session.commit()

    # 8️⃣ Create or reuse MissionController
    with controller_lock:
        controller = mission_controllers.get(drone_id)
        if controller:
            print(f"♻️ Reusing existing MissionController for Drone {drone_id}")
        else:
            controller = MissionController(drone_id, conn_string)
            mission_controllers[drone_id] = controller
            print(f"🆕 Created new MissionController for Drone {drone_id}")

    # 9️⃣ Start mission thread
    background_tasks.add_task(controller.start_mission)

    return {
        "message": f"Mission started for Drone {drone_id} ({home_lat:.6f}, {home_lon:.6f})",
        "sitl_port": sitl_udp,
        "reused_sitl": sitl_active,
    }


# ---------------------------------------------------------------
# ⏸️ Pause Mission
# ---------------------------------------------------------------
@app.post("/missions/pause")
def pause_mission(drone_id: int):
    """Pause a running mission."""
    controller = mission_controllers.get(drone_id)
    if not controller:
        raise HTTPException(status_code=404, detail="No active mission for this drone")
    controller.pause_mission()
    return {"message": f"Mission paused for Drone {drone_id}"}


# ---------------------------------------------------------------
# ▶️ Resume Mission
# ---------------------------------------------------------------
@app.post("/missions/resume")
def resume_mission(drone_id: int):
    """Resume a paused mission."""
    controller = mission_controllers.get(drone_id)
    if not controller or not controller.paused:
        raise HTTPException(status_code=404, detail="No paused mission found")
    controller.resume_mission()
    return {"message": f"Mission resumed for Drone {drone_id}"}


# ---------------------------------------------------------------
# 🛑 Abort Mission
# ---------------------------------------------------------------
@app.post("/missions/abort")
def abort_mission(
    drone_id: int,
    background_tasks: BackgroundTasks,
    session: Session = Depends(get_session),
):
    """Abort mission and return drone home."""
    controller = mission_controllers.get(drone_id)
    if not controller:
        raise HTTPException(status_code=404, detail="No active mission for this drone")

    background_tasks.add_task(controller.abort_mission)

    mission = session.exec(
        select(models.Mission)
        .where(models.Mission.drone_id == drone_id)
        .order_by(models.Mission.id.desc())
    ).first()

    if mission:
        mission.status = "aborted"
        session.add(mission)

    drone = session.get(models.Drone, drone_id)
    if drone:
        drone.status = "available"
        session.add(drone)

    session.commit()
    return {"message": f"Abort signal sent to Drone {drone_id}"}


# ===============================================================
# 📡 TELEMETRY ENDPOINTS
# ===============================================================

@app.get("/telemetry")
def list_telemetry(session: Session = Depends(get_session)):
    """List telemetry for all drones."""
    return crud.get_all_telemetry(session)


@app.post("/telemetry")
def update_telemetry(
    drone_id: int,
    latitude: float,
    longitude: float,
    altitude: float,
    battery: int,
    progress: int,
    eta: str = "Unknown",
    session: Session = Depends(get_session),
):
    """Update telemetry record for a specific drone."""
    return crud.update_telemetry(
        session, drone_id, latitude, longitude, altitude, battery, progress, eta
    )
