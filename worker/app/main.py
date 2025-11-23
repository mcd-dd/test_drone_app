# worker/app/main.py
import os
import subprocess
import threading
import time
import signal
from fastapi import FastAPI, HTTPException, BackgroundTasks, Request
import requests
from typing import Dict, Any

# Import MissionController from worker copy of mission_runner.py
from worker.app.mission_runner_old import MissionController

app = FastAPI(title="SITL Worker")

# Config
RAILWAY_BACKEND = os.getenv("RAILWAY_BACKEND_URL", "http://your-railway-app-url")  # set this on worker env
WORKER_HOST = os.getenv("WORKER_HOST", "0.0.0.0")
WORKER_PORT = int(os.getenv("WORKER_PORT", "8001"))

SITL_BASE_UDP_PORT = 14550
SITL_BASE_TCP_PORT = 5760

# Track processes and controllers
sitl_processes: Dict[int, subprocess.Popen] = {}
mavproxy_processes: Dict[int, subprocess.Popen] = {}
controllers: Dict[int, MissionController] = {}
controller_threads = {}

def stream_proc_output(proc: subprocess.Popen, drone_id: int):
    """Stream process stdout lines to Railway backend."""
    try:
        for line in proc.stdout:
            if not line:
                break
            text = line.decode(errors="ignore") if isinstance(line, (bytes, bytearray)) else str(line)
            # POST log to Railway
            try:
                requests.post(f"{RAILWAY_BACKEND}/telemetry/logs", json={"drone_id": drone_id, "log": text.strip()}, timeout=2)
            except Exception:
                pass
    except Exception as e:
        print(f"[Worker] Stream error for drone {drone_id}: {e}")

@app.post("/sitl/start")
def sitl_start(payload: Dict[str, Any], background_tasks: BackgroundTasks):
    """
    Payload expects: { drone_id, mission_id, sitl_udp, sitl_tcp, lat, lon, alt }
    Starts SITL, MAVProxy, and MissionController.
    """
    drone_id = int(payload["drone_id"])
    sitl_udp = int(payload["sitl_udp"])
    sitl_tcp = int(payload["sitl_tcp"])
    lat = float(payload["lat"])
    lon = float(payload["lon"])
    alt = float(payload["alt"])

    if drone_id in sitl_processes:
        return {"status": "already_running"}

    # Build the headless sim_vehicle.py command - avoid --console and --map
    sitl_cmd = [
        "sim_vehicle.py",
        "-v", "ArduCopter",
        f"-I{drone_id - 1}",
        "--sysid", str(drone_id),
        # no --map/no --console to avoid GUI
        f"--mavproxy-args=--out=udp:127.0.0.1:{sitl_udp}",
        f"--custom-location={lat},{lon},{alt},0",
    ]

    try:
        p = subprocess.Popen(
            sitl_cmd,
            cwd=os.path.expanduser("~/ardupilot/ArduCopter"),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT
        )
        sitl_processes[drone_id] = p
        # Start streaming stdout to Railway
        background_tasks.add_task(stream_proc_output, p, drone_id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"SITL start failed: {e}")

    # Start MAVProxy headless bridging tcp->udp (no console/map)
    mavproxy_cmd = [
        "mavproxy.py",
        f"--master=tcp:127.0.0.1:{SITL_BASE_TCP_PORT + ((drone_id - 1) * 10)}",
        f"--out=udp:127.0.0.1:{sitl_udp}"
    ]
    try:
        mp = subprocess.Popen(mavproxy_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        mavproxy_processes[drone_id] = mp
        background_tasks.add_task(stream_proc_output, mp, drone_id)
    except Exception as e:
        print(f"[Worker] MAVProxy start warning: {e}")

    # Create a MissionController and start mission thread locally
    conn_string = f"udp:127.0.0.1:{sitl_udp}"
    controller = MissionController(drone_id, conn_string)
    controllers[drone_id] = controller
    # Let MissionController run in background
    controller.start_mission()

    # Return minimal info to Railway
    return {"status": "started", "worker_pid": p.pid, "sitl_port": sitl_udp}


@app.post("/sitl/stop")
def sitl_stop(payload: Dict[str, int]):
    drone_id = int(payload["drone_id"])
    if drone_id in controllers:
        try:
            controllers[drone_id].abort_mission()
        except Exception:
            pass

    if drone_id in sitl_processes:
        try:
            sitl_processes[drone_id].terminate()
            sitl_processes[drone_id].wait(timeout=5)
        except Exception:
            pass
        del sitl_processes[drone_id]

    if drone_id in mavproxy_processes:
        try:
            mavproxy_processes[drone_id].terminate()
            mavproxy_processes[drone_id].wait(timeout=5)
        except Exception:
            pass
        del mavproxy_processes[drone_id]

    return {"status": "stopped"}


@app.post("/missions/pause")
def worker_pause(payload: Dict[str, int]):
    drone_id = int(payload["drone_id"])
    controller = controllers.get(drone_id)
    if not controller:
        raise HTTPException(status_code=404, detail="Controller not found")
    controller.pause_mission()
    return {"status": "paused"}


@app.post("/missions/resume")
def worker_resume(payload: Dict[str, int]):
    drone_id = int(payload["drone_id"])
    controller = controllers.get(drone_id)
    if not controller:
        raise HTTPException(status_code=404, detail="Controller not found")
    controller.resume_mission()
    return {"status": "resumed"}


@app.post("/missions/abort")
def worker_abort(payload: Dict[str, int]):
    drone_id = int(payload["drone_id"])
    controller = controllers.get(drone_id)
    if not controller:
        return {"status": "no_controller"}
    controller.abort_mission()
    return {"status": "aborted"}