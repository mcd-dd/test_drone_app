from streamlit_folium import st_folium
import folium
# frontend/streamlit_app.py
import streamlit as st
import requests
import time
import pandas as pd
import pydeck as pdk
import os
from streamlit_autorefresh import st_autorefresh 
from datetime import datetime
# Set Mapbox API key
MAPBOX_TOKEN = os.getenv("MAPBOX_API_KEY", "pk.AIzaSyBwQCtCyGbGaIwzqHegUyaPO6oWqDAdN-E")
pdk.settings.mapbox_api_key = MAPBOX_TOKEN

API_KEY = "AIzaSyA6pjubO9eU_jyEju3An16XGVNEQL-cjwk"

st.set_page_config(page_title="Drone Survey Management System", layout="wide")

# -------------------------------
# Sidebar Navigation
# -------------------------------
st.sidebar.title("📊 Menu")
page = st.sidebar.radio(
    "Choose a Dashboard:",
    [
        "🛰 Mission Planner",
        "🚁 Fleet Visualization",
        "📡 Mission Monitoring",
        "📊 Survey Reporting & Analytics Portal"
    ]
)

# API = st.sidebar.text_input("API base URL", value="http://localhost:8000")
API = st.sidebar.text_input("API base URL", value="https://flytbase-assignment-production.up.railway.app/")

# -------------------------------
# 1️⃣ MISSION PLANNER DASHBOARD
# -------------------------------
if page == "🛰 Mission Planner":
    st.title("🛰 Drone Survey Management System")

    # ---------- Helper Functions ----------
    def get_data(endpoint):
        try:
            r = requests.get(f"{API}/{endpoint}")
            return r.json() if r.ok else []
        except Exception as e:
            st.error(f"Error fetching {endpoint}: {e}")
            return []

    def delete_item(endpoint, item_id):
        try:
            r = requests.delete(f"{API}/{endpoint}/{item_id}")
            if r.ok:
                st.success("Deleted successfully")
                st.rerun()
            else:
                st.error(r.text)
        except Exception as e:
            st.error(f"Error deleting: {e}")

    # ---------- DRONES ----------
    st.header("🛩️ Manage Drones")
    drones = get_data("drones")
    if drones:
        for d in drones:
            cols = st.columns([3, 1])
            cols[0].write(f"**{d['id']} — {d['name']}** | Battery: {d['battery_level']}% | Status: {d['status']}")
            if cols[1].button("🗑 Remove", key=f"del_drone_{d['id']}"):
                delete_item("drones", d["id"])
    else:
        st.info("No drones added yet.")

    with st.form("add_drone"):
        name = st.text_input("Drone Name")
        battery = st.number_input("Battery %", 0, 100, 100)
        if st.form_submit_button("Add Drone"):
            r = requests.post(f"{API}/drones", params={"name": name, "battery_level": battery})
            st.success("Drone added") if r.ok else st.error(r.text)
            st.rerun()

    st.divider()

    # ---------- SURVEYS ----------
    st.header("🗺️ Manage Surveys")
    surveys = get_data("surveys")
    if surveys:
        for s in surveys:
            cols = st.columns([3, 1])
            cols[0].write(f"**Survey {s['id']} — {s['name']}** in {s['area']}")
            if cols[1].button("🗑 Remove", key=f"del_survey_{s['id']}"):
                delete_item("surveys", s["id"])
    else:
        st.info("No surveys created yet.")

    with st.form("add_survey"):
        name = st.text_input("Survey Name")
        area = st.text_input("Survey Area (e.g., city or coordinates)")
        if st.form_submit_button("Add Survey"):
            r = requests.post(f"{API}/surveys", params={"name": name, "area": area})
            st.success("Survey added") if r.ok else st.error(r.text)
            st.rerun()

    st.divider()

    # ---------- FLIGHT PATHS ----------
    st.header("🛰️ Manage Flight Paths")
    fps = get_data("flightpaths")
    if fps:
        for fp in fps:
            cols = st.columns([3, 1])
            cols[0].write(
                f"**Flight Path {fp['id']} — {fp['name']}** | Altitude: {fp['altitude']}m | Area: {fp['survey_area']}"
            )
            if cols[1].button("🗑 Remove", key=f"del_fp_{fp['id']}"):
                delete_item("flightpaths", fp["id"])
    else:
        st.info("No flight paths yet.")

    with st.form("add_fp"):
        name = st.text_input("Flight Path Name")
        # Fetch surveys for dropdown
        surveys = get_data("surveys")
        survey_options = {f"{s['name']} (ID: {s['id']})": s["id"] for s in surveys} if surveys else {}
        selected_survey = st.selectbox("Select Survey", options=list(survey_options.keys()) if survey_options else ["No surveys available"])
        altitude = st.number_input("Altitude (m)", 0.0, 1000.0, 100.0)

        if st.form_submit_button("Add Flight Path"):
            if not survey_options:
                st.warning("Please create a survey first.")
            else:
                survey_id = survey_options[selected_survey]
                r = requests.post(
                    f"{API}/flightpaths",
                    params={"name": name, "survey_area": str(survey_id), "altitude": altitude}
                )
                st.success("Flight path added") if r.ok else st.error(r.text)
                st.rerun()


    st.divider()

    # ---------- WAYPOINTS ----------
    st.header("📍 Manage Waypoints")
    # --- Flight Path dropdown instead of ID input ---
    flightpaths = get_data("flightpaths")
    if flightpaths:
        fp_map = {f"{fp['name']} (ID: {fp['id']})": fp["id"] for fp in flightpaths}
        selected_fp = st.selectbox("Select Flight Path", list(fp_map.keys()))
        fp_id = fp_map[selected_fp]
    else:
        fp_id = None
        st.warning("No flight paths available. Please create one first.")
    waypoints = get_data(f"flightpaths/{int(fp_id)}/waypoints")

    # Display existing waypoints on map
    if waypoints:
        df_wp = pd.DataFrame(waypoints)
        st.map(df_wp.rename(columns={"latitude": "lat", "longitude": "lon"}))
        for wp in waypoints:
            cols = st.columns([4, 1])
            cols[0].write(f"Waypoint {wp['id']} | lat={wp['latitude']:.6f} | lon={wp['longitude']:.6f} | alt={wp['altitude']}m")
            if cols[1].button("🗑 Remove", key=f"del_wp_{wp['id']}"):
                delete_item(f"flightpaths/{fp_id}/waypoints", wp["id"])
    else:
        st.info("No waypoints yet.")
        st.map(pd.DataFrame({"lat": [0.0], "lon": [0.0]}), zoom=1)

    st.divider()

    # Interactive map waypoint adder
    st.subheader("🗺️ Add Waypoints via Interactive Map")
    st.caption("Click on the map to set latitude/longitude, then enter altitude below.")

    # ✅ Initialize session state
    if "clicked_latlon" not in st.session_state:
        st.session_state.clicked_latlon = None

    # Default map center
    if waypoints:
        center_lat = waypoints[0]["latitude"]
        center_lon = waypoints[0]["longitude"]
    else:
        center_lat, center_lon = 20.5937, 78.9629  # Default: India center

    # Create map
    m = folium.Map(location=[center_lat, center_lon], zoom_start=5)

    # Show existing waypoints
    for wp in waypoints:
        folium.Marker(
            [wp["latitude"], wp["longitude"]],
            popup=f"Alt: {wp['altitude']}m",
            icon=folium.Icon(color="blue", icon="info-sign"),
        ).add_to(m)

    # Capture map click
    clicked = st_folium(m, height=400, width=700)

    clicked_latlon = st.session_state.get("clicked_latlon", None)

    if clicked and clicked.get("last_clicked"):
        lat_clicked = clicked["last_clicked"]["lat"]
        lon_clicked = clicked["last_clicked"]["lng"]
        st.session_state.clicked_latlon = (lat_clicked, lon_clicked)
        # ✅ Update the input fields' session state directly
        st.session_state["lat_input"] = lat_clicked
        st.session_state["lon_input"] = lon_clicked
        st.success(f"Selected point: lat={lat_clicked:.6f}, lon={lon_clicked:.6f}")

    # --- Use updated session state for display ---
    lat_val = st.session_state.get("lat_input", 0.0)
    lon_val = st.session_state.get("lon_input", 0.0)

    # Altitude + editable coordinate inputs
    col1, col2, col3 = st.columns(3)
    lat = col1.number_input("Latitude", value=lat_val, format="%.6f", key="lat_input")
    lon = col2.number_input("Longitude", value=lon_val, format="%.6f", key="lon_input")
    alt = col3.number_input("Altitude (m)", value=50.0, key="alt_input")

    # Add waypoint
    if st.button("➕ Add Waypoint"):
        if lat == 0.0 and lon == 0.0:
            st.warning("Please click on the map to select coordinates.")
        else:
            r = requests.post(
                f"{API}/flightpaths/{int(fp_id)}/waypoints",
                params={"latitude": lat, "longitude": lon, "altitude": alt},
            )
            if r.ok:
                st.success(f"Waypoint added at ({lat:.6f}, {lon:.6f})")
                st.session_state.clicked_latlon = None
                st.rerun()
            else:
                st.error(r.text)

     # ---------- ASSIGNMENTS ----------
    st.header("🔗 Assignments & Management")

    # Fetch all existing entities
    surveys = get_data("surveys")
    drones = get_data("drones")
    flightpaths = get_data("flightpaths")

    # Create readable dropdown mappings
    survey_options = {f"{s['name']} (ID: {s['id']})": s["id"] for s in surveys} if surveys else {}
    drone_options = {f"{d['name']} (ID: {d['id']})": d["id"] for d in drones} if drones else {}
    flightpath_options = {f"{f['name']} (ID: {f['id']})": f["id"] for f in flightpaths} if flightpaths else {}

   # -------------------------
    # 🛰 ASSIGN DRONE TO FLIGHT PATH
    # -------------------------
    st.subheader("🗺 Assign Flight Path to Drone")

    if not drones or not flightpaths:
        st.warning("Please make sure both drones and flight paths exist before assigning.")
    else:
        sel_drone_label = st.selectbox("Select Drone", list(drone_options.keys()), key="assign_drone_for_fp")
        sel_fp_label = st.selectbox("Select Flight Path", list(flightpath_options.keys()), key="assign_fp_for_drone")

        sel_drone = drone_options[sel_drone_label]
        sel_fp = flightpath_options[sel_fp_label]

        if st.button("✅ Assign Flight Path"):
            r = requests.post(f"{API}/missions/assign", params={"flight_path_id": sel_fp, "drone_id": sel_drone})
            if r.ok:
                st.success(f"Assigned **{sel_fp_label}** to **{sel_drone_label}** successfully!")
            else:
                st.error(f"Failed to assign flight path: {r.text}")

    # --------------------------------
    # ❌ REMOVE DRONE OR FLIGHT PATH
    # --------------------------------
    st.subheader("🧹 Remove Assignments")

    rem_tab1, rem_tab2 = st.tabs(["🛩 Remove Drone from Survey", "🗺 Remove Flight Path from Survey"])

    # --- Remove Drone from Survey ---
    with rem_tab1:
        if not surveys or not drones:
            st.info("No drones or surveys available.")
        else:
            rem_survey_label = st.selectbox("Select Survey", list(survey_options.keys()), key="rem_survey_dropdown")
            rem_drone_label = st.selectbox("Select Drone", list(drone_options.keys()), key="rem_drone_dropdown")

            rem_survey = survey_options[rem_survey_label]
            rem_drone = drone_options[rem_drone_label]

            if st.button("🗑 Remove Drone"):
                r = requests.delete(f"{API}/surveys/{rem_survey}/remove_drone/{rem_drone}")
                if r.ok:
                    st.success(f"Removed Drone **{rem_drone_label}** from **{rem_survey_label}**.")
                else:
                    st.error(r.text)

    # --- Remove Flight Path from Survey ---
    with rem_tab2:
        if not surveys or not flightpaths:
            st.info("No surveys or flight paths available.")
        else:
            rem_survey_label_fp = st.selectbox("Select Survey", list(survey_options.keys()), key="rem_survey_fp_dropdown")
            rem_fp_label = st.selectbox("Select Flight Path", list(flightpath_options.keys()), key="rem_fp_dropdown")

            rem_survey_fp = survey_options[rem_survey_label_fp]
            rem_fp = flightpath_options[rem_fp_label]

            if st.button("🗑 Remove Flight Path"):
                r = requests.delete(f"{API}/surveys/{rem_survey_fp}/remove_fp/{rem_fp}")
                if r.ok:
                    st.success(f"Removed Flight Path **{rem_fp_label}** from **{rem_survey_label_fp}**.")
                else:
                    st.error(r.text)


    rem_tab3 = st.tabs(["🛩 Remove Drone from Survey", "🗺 Remove Flight Path from Survey", "🧭 Remove Flight Path from Drone"])[2]

    with rem_tab3:
        missions = get_data("missions")  # you may add a /missions list endpoint if not exists
        if not missions:
            st.info("No active missions found.")
        else:
            mission_map = {f"Mission {m['id']} — Drone {m['drone_id']} → FP {m['flight_path_id']}": m["id"] for m in missions}
            sel_mission = st.selectbox("Select Mission to Remove", list(mission_map.keys()), key="rem_mission_dropdown")
            mission_id = mission_map[sel_mission]

            if st.button("🗑 Remove Flight Path from Drone"):
                r = requests.post(f"{API}/missions/{mission_id}/complete")
                if r.ok:
                    st.success(f"Mission {mission_id} marked as completed and removed.")
                else:
                    st.error(r.text)

                
# -------------------------------
# 2️⃣ FLEET VISUALIZATION DASHBOARD
# -------------------------------
elif page == "🚁 Fleet Visualization":
    st.title("🚁 Fleet Visualization & Management Dashboard")
    refresh_interval = st.sidebar.slider("Auto-refresh (seconds)", 2, 30, 5)
    placeholder = st.empty()

    while True:
        try:
            # Fetch drone data
            resp = requests.get(f"{API}/drones")
            if not resp.ok:
                st.error("Failed to fetch drones")
                time.sleep(refresh_interval)
                continue

            drones = resp.json()
            if not drones:
                st.warning("No drones available in inventory.")
                time.sleep(refresh_interval)
                continue

            df = pd.DataFrame(drones)
            df = df.rename(columns={
                "id": "Drone ID",
                "name": "Drone Name",
                "status": "Status",
                "battery_level": "Battery (%)"
            })

            # Battery + status color formatting
            def color_battery(val):
                if val >= 70:
                    color = 'lightgreen'
                elif val >= 40:
                    color = 'khaki'
                else:
                    color = 'salmon'
                return f'background-color: {color}'

            def color_status(val):
                color = 'lightgreen' if val == 'available' else 'lightcoral'
                return f'background-color: {color}'

            styled_df = df.style.applymap(color_battery, subset=["Battery (%)"]).applymap(color_status, subset=["Status"])

            with placeholder.container():
                st.subheader("📦 Drone Inventory")
                st.dataframe(styled_df, use_container_width=True)

                avg_battery = df["Battery (%)"].mean()
                available_count = len(df[df["Status"] == "available"])
                in_mission_count = len(df[df["Status"] == "in_mission"])

                col1, col2, col3 = st.columns(3)
                col1.metric("Total Drones", len(df))
                col2.metric("Available Drones", available_count)
                col3.metric("In Mission", in_mission_count)
                st.progress(int(avg_battery))

                st.caption(f"🔄 Auto-refresh every {refresh_interval} sec")

                # Add Mission Start Controls
                st.subheader("🚀 Start Missions")
                for _, row in df.iterrows():
                    d_id = int(row["Drone ID"])
                    cols = st.columns([3, 2, 2, 2])
                    cols[0].write(f"**{row['Drone Name']}**")
                    cols[1].write(f"Status: {row['Status']}")
                    cols[2].write(f"🔋 {row['Battery (%)']}%")
                    if row["Status"] == "available":
                        if cols[3].button("🚀 Start Mission", key=f"fleet_start_{d_id}"):
                            r = requests.post(f"{API}/missions/start", params={"drone_id": d_id})
                            if r.ok:
                                st.success(f"Mission started for Drone {d_id}")
                            else:
                                st.error(r.text)
                    else:
                        cols[3].write("🟡 In Mission")

        except Exception as e:
            st.error(f"Error fetching data: {e}")

        time.sleep(refresh_interval)
        st.rerun()

# -----------------------------------------------------------
# 3️⃣  REAL-TIME MISSION MONITORING DASHBOARD (Enhanced UI)
# -----------------------------------------------------------
elif page == "📡 Mission Monitoring":
    import pydeck as pdk

    st.title("🛰 Multi-Drone Mission Control Center")
    refresh_interval = st.sidebar.slider("Telemetry refresh (seconds)", 1, 20, 3)
    placeholder = st.empty()

    colors = [
        [0, 100, 255],   # blue
        [0, 200, 100],   # green
        [255, 50, 50],   # red
        [255, 150, 0],   # orange
        [150, 0, 255],   # purple
    ]

    if "drone_paths" not in st.session_state:
        st.session_state["drone_paths"] = {}

    count = st_autorefresh(interval=refresh_interval * 1000, key="mission_refresh")
    placeholder = st.empty()

    def send_command(endpoint, params, success_msg, warn=False):
        try:
            r = requests.post(f"{API}/{endpoint}", params=params)
            if r.ok:
                if warn:
                    st.warning(r.json().get("message", success_msg))
                else:
                    st.success(r.json().get("message", success_msg))
            else:
                st.error(r.text)
        except Exception as e:
            st.error(f"Error calling {endpoint}: {e}")

    with placeholder.container():
        st.subheader("🚁 Active Drones")

        try:
            drones = requests.get(f"{API}/drones").json()
            telemetry = requests.get(f"{API}/telemetry").json()
        except Exception as e:
            st.error(f"Error fetching data: {e}")
            drones, telemetry = [], []

        if not drones:
            st.info("No drones available.")
        else:
            telem_map = {t["drone_id"]: t for t in telemetry}

            # ✅ 3 cards per row (adjust dynamically)
            drones_per_row = 3
            for i in range(0, len(drones), drones_per_row):
                row_drones = drones[i:i + drones_per_row]
                cols = st.columns(len(row_drones))

                for idx, d in enumerate(row_drones):
                    with cols[idx]:
                        t = telem_map.get(d["id"])
                        prog = int(t["progress"]) if t else 0
                        bat = t["battery"] if t else d["battery_level"]
                        eta = t["eta"] if t else "Calculating..."

                        # 🎯 Drone Header
                        st.markdown(f"### 🛩 Drone {d['id']} — {d['name']}")
                        st.metric("Battery", f"{bat}%")
                        st.progress(prog)
                        st.caption(f"Status: {d['status'].capitalize()} | ETA: {eta}")

                        # 🎮 Control Buttons (side-by-side)
                        btns = st.columns(4)

                        if btns[0].button("🚀 Start", key=f"start_{d['id']}"):
                            send_command("missions/start", {"drone_id": d["id"]}, "Mission started")

                        if btns[1].button("⏸ Pause", key=f"pause_{d['id']}"):
                            send_command("missions/pause", {"drone_id": d["id"]}, "Mission paused")

                        if btns[2].button("▶️ Resume", key=f"resume_{d['id']}"):
                            send_command("missions/resume", {"drone_id": d["id"]}, "Mission resumed")

                        if btns[3].button("🛑 Abort", key=f"abort_{d['id']}"):
                            send_command("missions/abort", {"drone_id": d["id"]}, "Mission aborted", warn=True)
                    
            st.divider()

            # --- Enhanced 3D Mission Visualization ---
            st.subheader("🌍 Real-Time Drone Flight Map (3D View)")

            if telemetry:
                # Convert telemetry to DataFrame
                df_map = pd.DataFrame([
                    {
                        "drone_id": t["drone_id"],
                        "lat": t["latitude"],
                        "lon": t["longitude"],
                        "alt": t["altitude"],
                        "progress": t["progress"],
                    }
                    for t in telemetry
                ])

                # Update path history
                for t in telemetry:
                    d_id = t["drone_id"]
                    if d_id not in st.session_state["drone_paths"]:
                        st.session_state["drone_paths"][d_id] = []
                    st.session_state["drone_paths"][d_id].append(
                        (t["longitude"], t["latitude"], t["altitude"])
                    )
                    # Keep last 200 points max
                    if len(st.session_state["drone_paths"][d_id]) > 200:
                        st.session_state["drone_paths"][d_id] = st.session_state["drone_paths"][d_id][-200:]

                # Build deck.gl layers
                layers = []

                for idx, (d_id, path_points) in enumerate(st.session_state["drone_paths"].items()):
                    color = colors[idx % len(colors)]

                    # Flight trail (3D)
                    path_df = pd.DataFrame([{"path": path_points}])
                    layers.append(pdk.Layer(
                        "PathLayer",
                        data=path_df,
                        get_path="path",
                        get_color=color,
                        width_scale=10,
                        width_min_pixels=3,
                        get_width=5,
                        opacity=0.7,
                    ))

                    # Current drone marker (3D sphere)
                    lon, lat, alt = path_points[-1]
                    layers.append(pdk.Layer(
                        "ColumnLayer",
                        data=pd.DataFrame([{"lon": lon, "lat": lat, "alt": alt}]),
                        get_position='[lon, lat]',
                        get_elevation="alt",
                        elevation_scale=1,
                        radius=10,
                        get_fill_color=color,
                        pickable=True,
                    ))

                # Dynamic 3D camera
                midpoint = [df_map["lat"].mean(), df_map["lon"].mean()]
                view_state = pdk.ViewState(
                    latitude=midpoint[0],
                    longitude=midpoint[1],
                    zoom=17,
                    pitch=60,
                    bearing=0,
                )

                # Deck rendering
                deck = pdk.Deck(
                    map_style="mapbox://styles/mapbox/satellite-v9",
                    layers=layers,
                    initial_view_state=view_state,
                    tooltip={"text": "Drone {drone_id}\nAltitude: {alt} m"},
                )

                col1, col2 = st.columns([3, 2])

                with col1:
                    st.pydeck_chart(deck, use_container_width=True)

                with col2:
                    st.subheader("🛣️ Street View (Google)")
                    if telemetry:
                        # Get last drone position
                        t = telemetry[-1]
                        lat, lon = t["latitude"], t["longitude"]
                        # ✅ Correct Street View URL (uses embedded Google Maps with lat/lon)

                        markers = ""
                        for t in telemetry:
                            d_id = t["drone_id"]
                            lat, lon = t["latitude"], t["longitude"]
                            # Label limited to 1 character (A–Z, 0–9)
                            label = str(d_id)[:1].upper()
                            markers += f"&markers=color:red%7Clabel:{label}%7C{lat},{lon}"


                        gmaps_aerial = (
                            f"https://www.google.com/maps/embed/v1/place"
                            f"?key={API_KEY}"
                            f"&q={lat},{lon}"
                            f"&zoom=18"
                            f"&maptype=satellite"
                        )

                        st.markdown(
                            f"""
                            <iframe width="100%" height="400"
                            src="{gmaps_aerial}"
                            style="border:0;" allowfullscreen loading="lazy"></iframe>
                            """,
                            unsafe_allow_html=True,
                        )
                    else:
                        st.info("Waiting for drone position to load...")
            else:
                st.info("Waiting for live telemetry data...")


# -----------------------------------------------------------
# 4️⃣ SURVEY REPORTING & ANALYTICS PORTAL (Improved)
# -----------------------------------------------------------
elif page == "📊 Survey Reporting & Analytics Portal":
    st.title("📊 Survey Reporting and Analytics Portal")

    # --- Helper function for API calls ---
    def get_data(endpoint):
        try:
            r = requests.get(f"{API}/{endpoint}")
            if not r.ok:
                st.warning(f"⚠️ Failed to fetch {endpoint}: {r.status_code}")
                return []
            return r.json()
        except Exception as e:
            st.error(f"Error fetching {endpoint}: {e}")
            return []

    # --- Fetch data from backend ---
    surveys = get_data("surveys")
    flightpaths = get_data("flightpaths")
    missions = get_data("missions")

    # --- Sanity check and fix data consistency ---
    survey_map = {s.get("id"): s.get("area", f"Survey {s.get('id')}") for s in surveys} if surveys else {}

    for fp in flightpaths:
        # Map survey ID → readable name
        sa = fp.get("survey_area")
        fp["survey_area"] = survey_map.get(sa, f"Survey {sa}" if sa else "Unknown")

        # Fill missing stats with defaults
        fp.setdefault("distance", 0)
        fp.setdefault("duration", 0)
        fp.setdefault("coverage", 0)
        fp.setdefault("altitude", 0)

    # --- Organization-level Statistics ---
    st.header("🏢 Organization-wide Survey Statistics")

    total_surveys = len(surveys)
    total_flights = len(flightpaths)
    total_missions = len(missions)

    avg_distance = (
        sum(fp.get("distance", 0) for fp in flightpaths) / total_flights if total_flights else 0
    )
    avg_duration = (
        sum(fp.get("duration", 0) for fp in flightpaths) / total_flights if total_flights else 0
    )

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Total Surveys", total_surveys)
    col2.metric("Total Flight Paths", total_flights)
    col3.metric("Active Missions", total_missions)
    col4.metric("Avg Flight Distance (km)", f"{avg_distance:.2f}")
    st.caption(f"🕒 Avg Flight Duration: {avg_duration:.2f} mins")

    st.divider()

    # --- Survey Summaries Table ---
    st.header("🗺️ Survey Summaries")

    if not surveys:
        st.info("No survey data available yet.")
    else:
        survey_df = pd.DataFrame(surveys)
        survey_df = survey_df.rename(columns={"name": "Survey Name", "area": "Area"})
        st.dataframe(survey_df, use_container_width=True)

    st.divider()

    # --- Flight Statistics Section ---
    st.header("✈️ Flight Statistics by Mission/Drone")

    if not flightpaths:
        st.info("No flight path data available.")
    else:
        fp_df = pd.DataFrame(flightpaths)
        fp_df = fp_df.rename(columns={
            "name": "Flight Path",
            "survey_area": "Survey",
            "altitude": "Altitude (m)",
            "distance": "Distance (km)",
            "duration": "Duration (min)",
            "coverage": "Coverage (%)"
        })

        # Show chart for numeric metrics
        numeric_cols = [c for c in ["Distance (km)", "Duration (min)", "Coverage (%)"] if c in fp_df.columns]
        if numeric_cols:
            st.bar_chart(fp_df.set_index("Flight Path")[numeric_cols])
        st.dataframe(fp_df, use_container_width=True)

    st.divider()

    # --- Survey Performance Overview ---
    st.header("📈 Survey Performance Overview")

    if not flightpaths:
        st.info("No performance data to display.")
    else:
        perf_df = pd.DataFrame([
            {
                "Survey": fp.get("survey_area", "Unknown"),
                "Distance (km)": fp.get("distance", 0),
                "Duration (min)": fp.get("duration", 0),
                "Coverage (%)": fp.get("coverage", 0)
            }
            for fp in flightpaths
        ])

        if not perf_df.empty:
            st.bar_chart(perf_df.set_index("Survey"))
            st.dataframe(perf_df, use_container_width=True)
        else:
            st.info("No data available for chart visualization yet.")

    st.divider()

    # --- Missions Summary (if available) ---
    st.header("🚀 Missions Overview")

    if not missions:
        st.info("No missions found.")
    else:
        missions_df = pd.DataFrame(missions)
        missions_df = missions_df.rename(columns={
            "id": "Mission ID",
            "drone_id": "Drone ID",
            "flight_path_id": "Flight Path ID",
            "status": "Status",
        })
        st.dataframe(missions_df, use_container_width=True)





