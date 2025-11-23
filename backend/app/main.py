# ===============================================================
# backend/app/main.py  (Updated: Worker-Based Architecture)
# ===============================================================

from fastapi import FastAPI, Depends, HTTPException, BackgroundTasks, Request
from sqlmodel import Session, select
import os
import requests
import time
from threading import Lock
from typing import List

from backend.app import crud, models
from backend.app.database import init_db, get_session

# ===============================================================
# GLOBAL CONFIG
# ===============================================================

# 👉 Set this in Railway environment variables OR when running locally
WORKER_URL = os.getenv("WORKER_URL", "https://nehemiah-misleading-punctually.ngrok-free.dev")

app = FastAPI(title="Mission Planner API")

controller_lock = Lock()
mission_lock = Lock()

active_missions = {}

SITL_BASE_UDP_PORT = 14550
SITL_BASE_TCP_PORT = 5760

# ===============================================================
# STARTUP
# ===============================================================

@app.on_event("startup")
def on_startup():
    init_db()
    print("✅ Backend online. Using Worker:", WORKER_URL)


# ===============================================================
# DRONE CRUD
# ===============================================================

@app.post("/drones")
def create_drone(name: str, battery_level: int = 100, session: Session = Depends(get_session)):
    return crud.create_drone(session, name, battery_level)

@app.get("/drones")
def list_drones(session: Session = Depends(get_session)):
    return crud.list_drones(session)

@app.get("/drones/{drone_id}")
def get_drone(drone_id: int, session: Session = Depends(get_session)):
    drone = crud.get_drone(session, drone_id)
    if not drone:
        raise HTTPException(404, "Drone not found")
    return drone

@app.delete("/drones/{drone_id}")
def delete_drone(drone_id: int, session: Session = Depends(get_session)):
    success = crud.delete_drone(session, drone_id)
    if not success:
        raise HTTPException(404, "Drone not found")
    return {"message": "Drone deleted"}

@app.get("/drones/available")
def available_drones(session: Session = Depends(get_session)):
    return crud.get_available_drones(session)


# ===============================================================
# SURVEY CRUD
# ===============================================================

@app.post("/surveys")
def create_survey(name: str, area: str = "", session: Session = Depends(get_session)):
    return crud.create_survey(session, name, area)

@app.get("/surveys")
def list_surveys(session: Session = Depends(get_session)):
    return crud.list_surveys(session)

@app.get("/surveys/{survey_id}")
def get_survey(survey_id: int, session: Session = Depends(get_session)):
    survey = crud.get_survey(session, survey_id)
    if not survey:
        raise HTTPException(404, "Survey not found")
    return survey

@app.delete("/surveys/{survey_id}")
def delete_survey(survey_id: int, session: Session = Depends(get_session)):
    success = crud.delete_survey(session, survey_id)
    if not success:
        raise HTTPException(404, "Survey not found")
    return {"message": "Survey deleted"}

@app.post("/surveys/{survey_id}/assign")
def assign_flightpath_to_survey(survey_id: int, flightpath_id: int, session: Session = Depends(get_session)):
    return crud.assign_flightpath_to_survey(session, survey_id, flightpath_id)

@app.post("/surveys/{survey_id}/assign_drone")
def assign_drone_to_survey(survey_id: int, drone_id: int, session: Session = Depends(get_session)):
    return crud.assign_drone_to_survey(session, survey_id, drone_id)

@app.delete("/surveys/{survey_id}/remove_fp/{fp_id}")
def remove_fp_from_survey(survey_id: int, fp_id: int, session: Session = Depends(get_session)):
    success = crud.remove_flightpath_from_survey(session, survey_id, fp_id)
    if not success:
        raise HTTPException(404, "Assignment not found")
    return {"message": "Flightpath removed"}

@app.delete("/surveys/{survey_id}/remove_drone/{drone_id}")
def remove_drone_from_survey(survey_id: int, drone_id: int, session: Session = Depends(get_session)):
    success = crud.remove_drone_from_survey(session, survey_id, drone_id)
    if not success:
        raise HTTPException(404, "Assignment not found")
    return {"message": "Drone removed"}


# ===============================================================
# FLIGHT PATH CRUD
# ===============================================================

@app.post("/flightpaths")
def create_fp(name: str, survey_area: str = "", altitude: float = 100, session: Session = Depends(get_session)):
    return crud.create_flight_path(session, name, survey_area, altitude)

@app.get("/flightpaths")
def list_fps(session: Session = Depends(get_session)):
    return crud.list_flight_paths(session)

@app.get("/flightpaths/{fp_id}")
def get_fp(fp_id: int, session: Session = Depends(get_session)):
    fp = crud.get_flight_path(session, fp_id)
    if not fp:
        raise HTTPException(404, "Flight path not found")
    return fp

@app.delete("/flightpaths/{fp_id}")
def delete_fp(fp_id: int, session: Session = Depends(get_session)):
    success = crud.delete_flight_path(session, fp_id)
    if not success:
        raise HTTPException(404, "Flight path not found")
    return {"message": "Flightpath deleted"}


# ===============================================================
# WAYPOINTS
# ===============================================================

@app.post("/flightpaths/{fp_id}/waypoints")
def add_wp(fp_id: int, latitude: float, longitude: float, altitude: float = 50, session: Session = Depends(get_session)):
    return crud.add_waypoint(session, fp_id, latitude, longitude, altitude)

@app.get("/flightpaths/{fp_id}/waypoints")
def list_wps(fp_id: int, session: Session = Depends(get_session)):
    return crud.list_waypoints(session, fp_id)

@app.delete("/flightpaths/{fp_id}/waypoints/{wp_id}")
def delete_wp(fp_id: int, wp_id: int, session: Session = Depends(get_session)):
    success = crud.delete_waypoint(session, fp_id, wp_id)
    if not success:
        raise HTTPException(404, "Waypoint not found")
    return {"message": "Waypoint deleted"}


# ===============================================================
# PARAMETERS
# ===============================================================

@app.post("/flightpaths/{fp_id}/parameters")
def set_params(fp_id: int, frequency_hz: int, sensors: str, battery_threshold: int, session: Session = Depends(get_session)):
    sensor_list = [s.strip() for s in sensors.split(",")]
    return crud.set_data_parameters(session, fp_id, frequency_hz, sensor_list, battery_threshold)


# ===============================================================
# MISSION CRUD
# ===============================================================

@app.post("/missions/assign")
def assign_mission(flight_path_id: int, drone_id: int, session: Session = Depends(get_session)):
    try:
        return crud.assign_mission(session, flight_path_id, drone_id)
    except ValueError as e:
        raise HTTPException(400, str(e))

@app.post("/missions/{mission_id}/complete")
def complete_mission(mission_id: int, session: Session = Depends(get_session)):
    mission = crud.complete_mission(session, mission_id)
    if not mission:
        raise HTTPException(404, "Mission not found")
    drone = session.get(models.Drone, mission.drone_id)
    if drone:
        drone.status = "available"
        session.add(drone)
        session.commit()
    return mission


# ===============================================================
# NEW: ACTIVE MISSION ENDPOINT (required by worker)
# ===============================================================

@app.get("/missions/active/{drone_id}")
def get_active_mission(drone_id: int, session: Session = Depends(get_session)):
    mission = session.exec(
        select(models.Mission)
        .where(models.Mission.drone_id == drone_id,
               models.Mission.status.in_(["planned", "in_progress"]))
        .order_by(models.Mission.id.desc())
    ).first()

    if not mission:
        raise HTTPException(404, "No active mission found")

    return {
        "mission_id": mission.id,
        "flight_path_id": mission.flight_path_id,
        "drone_id": mission.drone_id,
        "status": mission.status
    }


# ===============================================================
# MISSION CONTROL — NOW REMOTE VIA WORKER
# ===============================================================

@app.post("/missions/start")
def start_mission(drone_id: int, session: Session = Depends(get_session)):

    mission = session.exec(
        select(models.Mission)
        .where(models.Mission.drone_id == drone_id,
               models.Mission.status.in_(["planned", "in_progress", "aborted"]))
        .order_by(models.Mission.id.desc())
    ).first()

    if not mission:
        raise HTTPException(404, "No mission found")

    # waypoint calculations for SITL spawn
    wps = session.exec(
        select(models.Waypoint).where(models.Waypoint.flight_path_id == mission.flight_path_id)
    ).all()

    if not wps:
        raise HTTPException(400, "No waypoints in flight path")

    first = wps[0]
    base_lat, base_lon, base_alt = first.latitude, first.longitude, first.altitude or 50

    lat_offset = 0.0005 * (drone_id - 1)
    lon_offset = 0.0005 * (drone_id - 1)

    sitl_lat = base_lat + lat_offset
    sitl_lon = base_lon + lon_offset
    sitl_alt = base_alt + (drone_id - 1) * 20

    sitl_udp = SITL_BASE_UDP_PORT + 10 * (drone_id - 1)
    sitl_tcp = SITL_BASE_TCP_PORT + 10 * (drone_id - 1)

    payload = {
        "drone_id": drone_id,
        "mission_id": mission.id,
        "sitl_udp": sitl_udp,
        "sitl_tcp": sitl_tcp,
        "lat": sitl_lat,
        "lon": sitl_lon,
        "alt": sitl_alt,
    }

    try:
        r = requests.post(f"{WORKER_URL}/sitl/start", json=payload, timeout=10)
        r.raise_for_status()
        resp = r.json()
    except Exception as e:
        raise HTTPException(500, f"Worker start failed: {e}")

    # Update DB state
    mission.status = "in_progress"
    drone = session.get(models.Drone, drone_id)
    if drone:
        drone.status = "in_mission"
        session.add(drone)
    session.add(mission)
    session.commit()

    active_missions[drone_id] = resp

    return {
        "message": "Worker started SITL mission",
        "response": resp
    }


@app.post("/missions/pause")
def pause(drone_id: int):
    try:
        r = requests.post(f"{WORKER_URL}/missions/pause",
                          json={"drone_id": drone_id}, timeout=5)
        r.raise_for_status()
        return {"message": "Pause sent to worker"}
    except Exception as e:
        raise HTTPException(500, f"Pause failed: {e}")


@app.post("/missions/resume")
def resume(drone_id: int):
    try:
        r = requests.post(f"{WORKER_URL}/missions/resume",
                          json={"drone_id": drone_id}, timeout=5)
        r.raise_for_status()
        return {"message": "Resume sent to worker"}
    except Exception as e:
        raise HTTPException(500, f"Resume failed: {e}")


@app.post("/missions/abort")
def abort(drone_id: int, session: Session = Depends(get_session)):
    try:
        r = requests.post(f"{WORKER_URL}/missions/abort",
                          json={"drone_id": drone_id}, timeout=5)
        r.raise_for_status()
    except Exception as e:
        raise HTTPException(500, f"Abort failed: {e}")

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
    return {"message": "Abort sent to worker"}


# ===============================================================
# WORKER LOGS
# ===============================================================

@app.post("/telemetry/logs")
async def worker_logs(request: Request):
    payload = await request.json()
    print(f"[WORKER LOG] {payload}")
    return {"status": "ok"}


# ===============================================================
# TELEMETRY
# ===============================================================

@app.get("/telemetry")
def telemetry(session: Session = Depends(get_session)):
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
    return crud.update_telemetry(
        session, drone_id, latitude, longitude, altitude, battery, progress, eta
    )
