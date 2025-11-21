# ===============================================================
# backend/app/database.py
# ===============================================================
"""
Database configuration and session management for the Drone Mission Planner.
Handles database engine creation, initialization, and session lifecycle.
"""

from sqlmodel import SQLModel, create_engine, Session
from typing import Generator
import os


# ===============================================================
# ⚙️ CONFIGURATION
# ===============================================================

# Path to SQLite database (can be overridden via environment variable)
DB_FILE = os.environ.get("MP_DB", "backend/mission_planner.db")

# Full database connection string
DATABASE_URL = f"sqlite:///{DB_FILE}"

# Create SQLModel engine (SQLite by default)
# "check_same_thread" is disabled to allow multi-threaded access in FastAPI
engine = create_engine(
    DATABASE_URL,
    echo=False,  # set to True for SQL query logging
    connect_args={"check_same_thread": False},
)


# ===============================================================
# 🧱 DATABASE INITIALIZATION
# ===============================================================

def init_db() -> None:
    """
    Initialize the database by creating all tables defined in models.py.

    Imports are done inside the function to avoid circular dependencies
    during module import time.
    """
    from .models import Drone, FlightPath, Waypoint, DataParameters, Mission, Telemetry, Survey
    SQLModel.metadata.create_all(engine)


# ===============================================================
# 🔁 DATABASE SESSION HANDLING
# ===============================================================

def get_session() -> Generator[Session, None, None]:
    """
    Dependency function for FastAPI routes.

    Yields a SQLModel session connected to the main database engine.
    Ensures the session is properly closed after each request.

    Usage Example:
        >>> from .database import get_session
        >>> with next(get_session()) as session:
        ...     drones = session.exec(select(Drone)).all()
    """
    with Session(engine) as session:
        yield session
