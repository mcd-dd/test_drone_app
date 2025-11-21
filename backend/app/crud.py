# ===============================================================
# backend/app/crud.py
# ===============================================================
"""
CRUD (Create, Read, Update, Delete) operations for the Drone Mission Planner.
Handles database logic for:
 - Drones
 - Surveys
 - Flight Paths
 - Waypoints
 - Missions
 - Telemetry
 - Data parameters
"""

from sqlmodel import Session, select
from datetime import datetime
import math
import requests

from .models import (
    Drone, FlightPath, Waypoint, DataParameters,
    Mission, Telemetry, Survey
)
from .database import engine

# ===============================================================
# GLOBAL CONSTANTS
# ===============================================================

API_BASE = "https://testdroneapp-production.up.railway.app/"  # Adjust if backend runs remotely
SAFE_DISTANCE_METERS = 10.0          # Minimum drone separation
paused_drones = set()                # Track paused drones for safety logic

# ===============================================================
# 🔢 Utility: Haversine Distance
# ===============================================================

def get_distance_meters(lat1, lon1, lat2, lon2):
    """Return great-circle distance between two GPS points in meters."""
    R = 6371000  # Earth radius (m)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# ===============================================================
# 🛩️ DRONES
# ===============================================================

def create_drone(session: Session, name: str, battery_level: int = 100) -> Drone:
    """Create a new drone entry."""
    drone = Drone(name=name, battery_level=battery_level)
    session.add(drone)
    session.commit()
    session.refresh(drone)
    return drone

def list_drones(session: Session):
    """Return all drones."""
    return session.exec(select(Drone)).all()

def get_drone(session: Session, drone_id: int):
    """Get a specific drone."""
    return session.get(Drone, drone_id)

def delete_drone(session: Session, drone_id: int):
    """Delete a drone safely (cannot delete while in mission)."""
    drone = session.get(Drone, drone_id)
    if not drone:
        return False
    if drone.status == "in_mission":
        raise ValueError("Cannot delete a drone currently in mission.")
    session.delete(drone)
    session.commit()
    return True

def get_available_drones(session: Session):
    """Return all drones marked as available."""
    return session.exec(select(Drone).where(Drone.status == "available")).all()

# ===============================================================
# 🗺️ SURVEYS
# ===============================================================

def create_survey(session: Session, name: str, area: str = "") -> Survey:
    """Create a new survey."""
    survey = Survey(name=name, area=area)
    session.add(survey)
    session.commit()
    session.refresh(survey)
    return survey

def list_surveys(session: Session):
    """List all surveys."""
    return session.exec(select(Survey)).all()

def get_survey(session: Session, survey_id: int):
    """Retrieve one survey by ID."""
    return session.get(Survey, survey_id)

def delete_survey(session: Session, survey_id: int):
    """Delete a survey."""
    survey = session.get(Survey, survey_id)
    if not survey:
        return False
    session.delete(survey)
    session.commit()
    return True

# ===============================================================
# ✈️ FLIGHT PATHS
# ===============================================================

def create_flight_path(session: Session, name: str, survey_area: str, altitude: float):
    """Create a flight path."""
    fp = FlightPath(name=name, survey_area=survey_area, altitude=altitude)
    session.add(fp)
    session.commit()
    session.refresh(fp)
    return fp

def list_flight_paths(session: Session):
    """List all flight paths."""
    return session.exec(select(FlightPath)).all()

def get_flight_path(session: Session, fp_id: int):
    """Get a single flight path."""
    return session.get(FlightPath, fp_id)

def delete_flight_path(session: Session, fp_id: int):
    """Delete a flight path."""
    fp = session.get(FlightPath, fp_id)
    if not fp:
        return False
    session.delete(fp)
    session.commit()
    return True

# ===============================================================
# 📍 WAYPOINTS
# ===============================================================

def add_waypoint(session: Session, flight_path_id: int, lat: float, lon: float, alt: float = None):
    """Add a waypoint to a flight path."""
    wp = Waypoint(flight_path_id=flight_path_id, latitude=lat, longitude=lon, altitude=alt)
    session.add(wp)
    session.commit()
    session.refresh(wp)
    return wp

def list_waypoints(session: Session, flight_path_id: int):
    """List waypoints for a flight path."""
    return session.exec(select(Waypoint).where(Waypoint.flight_path_id == flight_path_id)).all()

def delete_waypoint(session: Session, flight_path_id: int, wp_id: int):
    """Delete a specific waypoint."""
    wp = session.get(Waypoint, wp_id)
    if not wp or wp.flight_path_id != flight_path_id:
        return False
    session.delete(wp)
    session.commit()
    return True

# ===============================================================
# ⚙️ DATA PARAMETERS
# ===============================================================

def set_data_parameters(session: Session, flight_path_id: int, frequency_hz: int, sensors: list, battery_threshold: int):
    """Store data collection parameters for a flight path."""
    sensors_str = ",".join(sensors)
    params = DataParameters(
        flight_path_id=flight_path_id,
        data_frequency_hz=frequency_hz,
        sensors=sensors_str,
        battery_threshold=battery_threshold,
    )
    session.add(params)
    session.commit()
    session.refresh(params)
    return params

# ===============================================================
# 🎯 MISSIONS
# ===============================================================

def assign_mission(session: Session, flight_path_id: int, drone_id: int):
    """Assign a mission to a drone (ensures no duplicate active missions)."""
    existing = session.exec(
        select(Mission)
        .where(Mission.drone_id == drone_id, Mission.status.in_(["planned", "in_progress"]))
        .order_by(Mission.id.desc())
    ).first()
    if existing:
        raise ValueError(f"Drone {drone_id} already has an active mission (ID {existing.id})")

    # Battery safety check
    dp = session.exec(select(DataParameters).where(DataParameters.flight_path_id == flight_path_id)).first()
    drone = session.get(Drone, drone_id)
    if dp and drone.battery_level < dp.battery_threshold:
        raise ValueError("Drone battery below required threshold")

    # Create new mission
    drone.status = "in_mission"
    mission = Mission(
        flight_path_id=flight_path_id,
        drone_id=drone_id,
        status="planned",
        started_at=datetime.now(),
    )
    session.add_all([drone, mission])
    session.commit()
    session.refresh(mission)
    return mission

def complete_mission(session: Session, mission_id: int):
    """Mark a mission as completed and release the drone."""
    mission = session.get(Mission, mission_id)
    if not mission:
        return None
    drone = session.get(Drone, mission.drone_id)
    drone.status = "available"
    mission.status = "planned"
    mission.completed_at = datetime.now()
    session.add_all([mission, drone])
    session.commit()
    session.refresh(mission)
    return mission

# ===============================================================
# 📡 TELEMETRY
# ===============================================================

def update_telemetry(session, drone_id, latitude, longitude, altitude, battery, progress, eta):
    """
    Create or update telemetry for a drone.
    Includes basic sanity checks and optional proximity safety logic.
    """
    now = datetime.now()
    existing = session.exec(select(Telemetry).where(Telemetry.drone_id == drone_id)).first()

    # Ignore invalid GPS
    if latitude == 0.0 and longitude == 0.0:
        print(f"[Drone {drone_id}] ⚠️ Ignoring invalid (0,0) telemetry.")
        return existing

    # Ignore sudden unrealistic altitude drops
    if existing and altitude < 1.0 and existing.altitude > 10.0:
        print(f"[Drone {drone_id}] ⚠️ Ignoring altitude drop {existing.altitude:.1f}→{altitude:.1f}")
        return existing

    # Update or create telemetry record
    if not existing:
        existing = Telemetry(
            drone_id=drone_id,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            battery=battery,
            progress=progress,
            eta=eta,
            timestamp=now,
        )
        session.add(existing)
    else:
        existing.latitude = latitude
        existing.longitude = longitude
        existing.altitude = altitude
        existing.battery = battery
        existing.progress = progress
        existing.eta = eta
        existing.timestamp = now

    session.commit()
    session.refresh(existing)

    # --- Optional proximity safety logic ---
    # (disabled for now, can re-enable if you want automatic pause/resume)
    #
    # others = session.exec(select(Telemetry).where(Telemetry.drone_id != drone_id)).all()
    # for other in others:
    #     if not other.latitude or not other.longitude:
    #         continue
    #     dist = get_distance_meters(latitude, longitude, other.latitude, other.longitude)
    #     alt_diff = abs((altitude or 0) - (other.altitude or 0))
    #     if alt_diff > 15:
    #         continue
    #     if dist < SAFE_DISTANCE_METERS:
    #         print(f"⚠️ {drone_id} too close to {other.drone_id} ({dist:.1f}m)")
    #         requests.post(f"{API_BASE}/missions/pause", params={"drone_id": drone_id})
    #     elif dist > SAFE_DISTANCE_METERS + 5 and drone_id in paused_drones:
    #         requests.post(f"{API_BASE}/missions/resume", params={"drone_id": drone_id})
    #         paused_drones.remove(drone_id)

    return existing

def get_all_telemetry(session: Session):
    """Return all drone telemetry entries."""
    return session.exec(select(Telemetry)).all()

# ===============================================================
# 🤝 SURVEY LINKING (DRONE ↔ FLIGHT PATH)
# ===============================================================

def assign_drone_to_survey(session: Session, survey_id: int, drone_id: int):
    """Assign a drone to a survey, creating a mission link."""
    drone = session.get(Drone, drone_id)
    if not drone:
        raise ValueError(f"Drone {drone_id} not found.")

    flight_path = session.exec(
        select(FlightPath).where(FlightPath.survey_area == str(survey_id))
    ).first()

    if not flight_path:
        flight_path = session.exec(select(FlightPath)).first()
        if not flight_path:
            raise ValueError("No FlightPath available.")
        flight_path.survey_area = str(survey_id)
        session.add(flight_path)
        session.commit()

    existing = session.exec(
        select(Mission).where(
            (Mission.flight_path_id == flight_path.id) & (Mission.drone_id == drone.id)
        )
    ).first()
    if existing:
        return {"message": "Drone already assigned to this survey."}

    mission = Mission(flight_path_id=flight_path.id, drone_id=drone.id, status="planned")
    drone.status = "in_mission"
    session.add_all([mission, drone])
    session.commit()
    session.refresh(mission)
    return {"message": f"Drone {drone.name} assigned to Survey {survey_id}", "mission_id": mission.id}

def remove_flightpath_from_survey(session, survey_id: int, flightpath_id: int) -> bool:
    """Unlink a flight path from a survey."""
    flight_path = session.get(FlightPath, flightpath_id)
    if not flight_path:
        raise ValueError(f"Flight Path {flightpath_id} not found.")
    if str(flight_path.survey_area) != str(survey_id):
        raise ValueError(f"Flight Path {flightpath_id} not linked to Survey {survey_id}.")
    flight_path.survey_area = None
    session.add(flight_path)
    session.commit()
    return True

def remove_drone_from_survey(session, survey_id: int, drone_id: int) -> bool:
    """Unlink a drone from a survey and mark it as available."""
    mission = session.exec(
        select(Mission)
        .where(Mission.drone_id == drone_id)
        .where(Mission.flight_path_id.in_(
            select(FlightPath.id).where(FlightPath.survey_area == str(survey_id))
        ))
    ).first()

    if mission:
        mission.status = "completed"
        mission.completed_at = datetime.now()
        session.add(mission)

    drone = session.get(Drone, drone_id)
    if not drone:
        raise ValueError(f"Drone {drone_id} not found.")

    drone.status = "available"
    session.add(drone)
    session.commit()
    return True
