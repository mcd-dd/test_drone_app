# 🛰️ Drone Survey & Mission Management System

An integrated **mission planning, simulation, and control platform** for multi-drone survey operations — featuring **mission scheduling, real-time telemetry visualization**, and **SITL-based flight simulation**.

---

## 🚀 Features

### 🧭 Mission Planning
- Create and manage **drones, surveys, flight paths, and waypoints** via an interactive Streamlit dashboard.
- Use **Folium maps** for intuitive waypoint placement.
- Assign flight paths to specific drones for **mission scheduling**.

### 📡 Mission Execution
- Launch **SITL (Software-In-The-Loop)** drone simulations automatically per mission.
- Execute missions with full lifecycle control:
  - **Start**, **Pause**, **Resume**, **Abort**, **Complete**
- Supports **multiple concurrent drones** via isolated MAVLink and TCP/UDP ports.

### 🌍 Real-Time Monitoring
- Live telemetry streaming: **position**, **altitude**, **battery**, **progress**, **ETA**.
- **3D visualization** using PyDeck (Mapbox) with dynamic path rendering.
- Integrated **Google Maps Street View** for real-world ground-level context.

### 🧠 Safety & Adaptability
- **Autonomous Return-to-Home (RTH)** and safe landing on mission completion or abort.
- Automatic synchronization between Mission and Drone states:  
  `available → in_mission → completed / aborted`
- Modular backend design — compatible with **SITL or real drones** via MAVLink.

---

## 🏗️ System Architecture
```
├── backend/
│ ├── app/
│ │ ├── main.py # FastAPI app and API endpoints
│ │ ├── mission_runner.py # MissionController (drone control logic)
│ │ ├── models.py # SQLModel ORM definitions
│ │ ├── crud.py # CRUD operations for all entities
│ │ ├── database.py # DB engine and session configuration
│ │ └── init.py
│ └── mission_planner.db # SQLite database
│
├── frontend/
│ ├── streamlit_app.py # Streamlit dashboard (Planner, Monitoring, Analytics)
│
├── README.md
└── requirements.txt
```

---

## ⚙️ Core Components**

### 1️⃣ Backend (FastAPI)
- Provides RESTful endpoints for:
  - `/drones`, `/surveys`, `/flightpaths`, `/waypoints`, `/missions`, `/telemetry`
- Manages mission lifecycle via:
  
  POST /missions/start → Launch mission
  POST /missions/pause → Pause mission
  POST /missions/resume → Resume mission
  POST /missions/abort → Abort mission safely
  POST /missions/complete_by_drone → Mark mission complete

- Integrates with **DroneKit** for MAVLink-based UAV control.
- Launches **SITL + MAVProxy** subprocesses per mission with unique port assignments.

### 2️⃣ Mission Runner
- Threaded controller per drone:
  - Connection retry and timeout logic.
  - Arming, takeoff, waypoint traversal.
  - Real-time telemetry streaming.
  - Return-to-home and auto-landing behavior.
- Maintains mission state integrity:  
  `planned → in_progress → completed / aborted`

### 3️⃣ Frontend (Streamlit)
- Unified UI with four main dashboards:
- 🛰 **Mission Planner** — Create and assign surveys, paths, and drones  
- 🚁 **Fleet Visualization** — Monitor drone availability and battery  
- 📡 **Mission Monitoring** — Real-time telemetry + 3D visualization  
- 📊 **Survey Analytics Portal** — Summarized reports and charts
- Uses **PyDeck (Mapbox)** for 3D visualization and **Folium** for waypoint editing.

---

## 🧩 Installation

### Prerequisites
- Python ≥ 3.10  
- ArduPilot SITL (with `sim_vehicle.py` in PATH)  
- MAVProxy (with `mavproxy.py` in PATH)  
- DroneKit-Python (`pip install dronekit`)

### Steps
  ```bash
  # 1. Clone repository
  git clone https://github.com/your-username/drone-mission-system.git
  cd drone-mission-system
  
  # 2. Install dependencies
  pip install -r requirements.txt
  
  # 3. Initialize database
  cd backend
  python -m app.database
  
  # 4. Run backend server
  uvicorn app.main:app --reload --port 8000
  
  # 5. Run frontend dashboard
  cd frontend
  streamlit run streamlit_app.py
    
  Then open http://localhost:8501 in your browser.

  For hosted application:
  Backend is on Railway with link: flytbase-assignment.railway.internal
  Frontend is on Streamli with link: https://flytbase-assignment-cpcawzj2phhpbfnm5o7k59.streamlit.app/
  ```
  ---

## 🕹 Usage Guide

### 🧭 Mission Planning

1. Create and manage **drones**, **surveys**, **flight paths**, and **waypoints** using the interactive Streamlit dashboard.  
2. Use **Folium maps** for waypoint definition and spatial visualization.  
3. Assign **Flight Paths** to specific Drones for mission scheduling and management.  

---

### 📡 Mission Execution

1. Open the **📡 Mission Monitoring** tab from the dashboard.  
2. Start missions for selected drones and monitor **position**, **progress**, and **battery** in real time.  
3. Pause, resume, or abort missions as needed during flight.  
4. Upon completion, drones **automatically return home** for safe landing.  

---

## 🔒 Safety & Fault Tolerance

### 1️⃣ Collision Avoidance System

Real-time proximity detection prevents drone collisions.  
If two drones approach within a **10 m safety radius**, automatic **pause** is triggered for involved drones.  
Alerts are logged, and optional dashboard notifications are generated.  

**In-flight safety layers:**
- 🛫 **Pre-Takeoff Check** — Ensures clear airspace before arming.  
- ✈️ **Dynamic Altitude Offsets** — Auto-adjusts (+5 m) when another drone is nearby.  
- 🛬 **Safe Landing Queue** — Sequential descent to prevent simultaneous landings.  

---

### 2️⃣ State Safety

Automatic drone-state reset (`available`) occurs after mission completion or abort.  
The backend ensures no concurrent missions are assigned to the same drone.  

---

### 3️⃣ Auto Recovery

On backend startup, any drone stuck in the `in_mission` state is **automatically reset**.  
This guarantees reliability and mission continuity after server restarts or system failures.  

---

### 4️⃣ Connection Handling

`MissionController` retries **SITL/MAVLink** connections up to **5 times**.  
Connection failures are handled gracefully without disrupting other missions.  

---

### 5️⃣ Telemetry Reliability

Continuous telemetry streams provide real-time updates for **GPS**, **altitude**, **battery**, **progress**, and **ETA**.  
All errors are logged safely, ensuring ongoing mission stability.  

---

## 🔄 Adaptability

### 🔁 SITL → Real Drone Transition
- SITL → Real Drone Transition:
  Replace SITL connection (tcp:127.0.0.1:5760) with your drone’s MAVLink UDP endpoint (udp:192.168.x.x:14550).
- Extensible Control Logic:
  Extend MissionController for swarm coordination, AI route re-planning, or safety analytics.
- API-First Design:
  Compatible with external ground stations or dashboards.

### 📋 Example Workflow

| 🧩 Step | 🪶 Action | 🔗 API / Module |
|:-------:|-----------|----------------|
| 1️⃣ | Add drone to system | `/drones` |
| 2️⃣ | Define survey + flight path | `/surveys`, `/flightpaths` |
| 3️⃣ | Add waypoints | `/flightpaths/{id}/waypoints` |
| 4️⃣ | Assign mission | `/missions/assign` |
| 5️⃣ | Start mission | `/missions/start` |
| 6️⃣ | Track progress | `/telemetry` |
| 7️⃣ | Complete mission | `/missions/complete_by_drone` |

### 🧱 Technologies Used
- Python 3.10+
- FastAPI (Backend REST API)
- Streamlit (Frontend Dashboard)
- DroneKit-Python (MAVLink control)
- ArduPilot SITL (Simulation)
    • 
