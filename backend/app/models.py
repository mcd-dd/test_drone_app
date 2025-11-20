# ===============================================================
# backend/app/models.py
# ===============================================================
"""
Data models for the Drone Mission Planner backend.
Defines SQLModel ORM tables for:
 - Drones
 - Surveys
 - Flight Paths
 - Waypoints
 - Missions
 - Telemetry
 - Data Parameters
"""

from typing import Optional
from sqlmodel import SQLModel, Field
from datetime import datetime


# ===============================================================
# 🛩️ DRONES
# ===============================================================

class Drone(SQLModel, table=True):
    """
    Represents a physical drone entity with its operational state.
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(index=True, description="Unique name or identifier for the drone")
    battery_level: int = Field(default=100, description="Battery percentage (0–100)")
    status: str = Field(default="available", description="Drone state: available | in_mission | maintenance")


# ===============================================================
# 🗺️ SURVEYS
# ===============================================================

class Survey(SQLModel, table=True):
    """
    Represents a defined survey or mission area.
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(index=True, description="Survey name")
    area: Optional[str] = Field(default=None, description="Geographic area or boundary reference")


# ===============================================================
# ✈️ FLIGHT PATHS & WAYPOINTS
# ===============================================================

class FlightPath(SQLModel, table=True):
    """
    Represents a flight path consisting of ordered waypoints.
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(index=True, description="Flight path name")
    survey_area: Optional[str] = Field(default=None, description="Associated survey area identifier")
    altitude: Optional[float] = Field(default=None, description="Default cruising altitude in meters")


class Waypoint(SQLModel, table=True):
    """
    Represents a GPS coordinate (waypoint) along a flight path.
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    flight_path_id: int = Field(foreign_key="flightpath.id", description="Linked flight path ID")
    latitude: float = Field(description="Latitude of the waypoint")
    longitude: float = Field(description="Longitude of the waypoint")
    altitude: Optional[float] = Field(default=None, description="Waypoint altitude (m)")
    is_home: bool = Field(default=False, description="Whether this is the home/return waypoint")


# ===============================================================
# ⚙️ DATA COLLECTION PARAMETERS
# ===============================================================

class DataParameters(SQLModel, table=True):
    """
    Configuration parameters for data collection during flight.
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    flight_path_id: int = Field(foreign_key="flightpath.id", description="Linked flight path ID")
    data_frequency_hz: int = Field(default=1, description="Data capture frequency (Hz)")
    sensors: str = Field(default="camera", description="Comma-separated list of active sensors")
    battery_threshold: int = Field(default=30, description="Minimum required battery (%) to start mission")


# ===============================================================
# 🚀 MISSION METADATA
# ===============================================================

class Mission(SQLModel, table=True):
    """
    Represents an assigned mission between a drone and a flight path.
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    flight_path_id: int = Field(foreign_key="flightpath.id", description="Associated flight path ID")
    drone_id: int = Field(foreign_key="drone.id", description="Assigned drone ID")
    status: str = Field(default="planned", description="Mission status: planned | in_progress | completed | aborted")
    started_at: Optional[datetime] = Field(default=None, description="Start timestamp of mission")
    completed_at: Optional[datetime] = Field(default=None, description="Completion timestamp of mission")


# ===============================================================
# 📡 REAL-TIME TELEMETRY
# ===============================================================

class Telemetry(SQLModel, table=True):
    """
    Real-time telemetry data for drone position and status.
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    drone_id: int = Field(foreign_key="drone.id", description="Drone ID reference")
    latitude: float = Field(default=0.0, description="Current GPS latitude")
    longitude: float = Field(default=0.0, description="Current GPS longitude")
    altitude: float = Field(default=0.0, description="Current altitude (m)")
    battery: int = Field(default=100, description="Current battery percentage")
    progress: int = Field(default=0, description="Mission progress (0–100%)")
    eta: str = Field(default="Unknown", description="Estimated time of arrival or completion")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Telemetry timestamp (UTC)")
