import numpy as np
import pygame
import math
import os
import argparse
import signal
import socket
import threading
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple
from scipy.ndimage import distance_transform_edt

# CONNTING TO NETWORKKKKKKK
DEFAULT_IP = os.environ.get("MBOT2_IP", "172.20.10.3")
DEFAULT_PORT = int(os.environ.get("MBOT2_PORT", "5000"))

# MAP SETUPPPP
CELL_SIZE = 5.0
GRID_WIDTH = 160
GRID_HEIGHT = 160
START_X = GRID_WIDTH / 2.0
START_Y = GRID_HEIGHT / 2.0

UNKNOWN = 0
FREE = 1
OBSTACLE_FIRST = 2
OBSTACLE_CONFIRMED = 3
CONFIRM_HITS = 3  

# GEROMETRYYYYY
MINIMUM_SENSOR_RANGE = 4.0
MAXIMUM_SENSOR_RANGE = 200.0
MAXIMUM_HIT_RANGE = 195.0
NO_ECHO_FREE_LIMIT = 110.0
NO_ECHO_FREE_WEIGHT = 0.32
STRAIGHT_FREE_WEIGHT = 1.00
ARC_FREE_WEIGHT = 0.55
STOP_FREE_WEIGHT = 1.00
ARC_OCCUPIED_WEIGHT = 0.60
TURN_OCCUPIED_WEIGHT = 0.28
ARC_CONFIRM_HITS = 5
TENTATIVE_FREE_PROTECTION = 0.35
NO_ECHO_DISTANCE = 290.0
CONTACT_DISTANCE = 10.8
BLOCKED_DISTANCE = 18.0
USABLE_DISTANCE = 24.0
COMFORT_DISTANCE = 36.0
FAR_OPEN_DISTANCE = 52.0
ROBOT_RADIUS = 10.0
ROBOT_RADIUS_CELLS = max(1, int(round(ROBOT_RADIUS / CELL_SIZE))) # robot radius converted into map cells

FORWARD_SPEED = 18
SLOW_SPEED = 13
INNER_ARC_SPEED = 10
OUTER_ARC_SPEED = 18
TURN_SPEED = 12
SCAN_SPEED = 8
REVERSE_SPEED = 14
COMMAND_REFRESH = 0.14
TURN_DEADBAND = 30.0
SCAN_TURN_STEP = 18.0 # amount to rotate for each small scan step
SCAN_TURN_SETTLE = 0.50
SCAN_SETTLE_MAX_WAIT_S = 1.05
SCAN_SETTLED_MIN_PACKETS = 1
SCAN_SETTLED_HEADING_TOL_DEG = 10.0
SCAN_TURN_TARGET_TOL_DEG = 17.0  
SCAN_TURN_TOTAL_TIMEOUT_S = 7.5  
SCAN_TURN_MAX_DEG = 144.0  
SCAN_TURN_OPEN_CM = 27.0 
SCAN_TURN_PREFER_UNKNOWN = True  
TURN_TIMEOUT = 3.2
POST_SCAN_DRIVE_S = 10.0
POST_SCAN_DRIVE_CM = 28.0

FULL_SCAN_STEP = 24.0
FULL_SCAN_SETTLE = 0.70
FULL_SCAN_TOTAL = 360.0
FULL_SCAN_TIMEOUT = 34.0

FULL_SCAN_PREFLIGHT_CM = CONTACT_DISTANCE + 0.8 # minimum space needed before starting a full scan
FULL_SCAN_PREFLIGHT_REVERSE_S = 0.52
FULL_SCAN_PREFLIGHT_MAX_RETRIES = 2
FULL_SCAN_HARD_ABORT_CM = 7.8
FULL_SCAN_HARD_ABORT_HITS = 2
FULL_SCAN_TARGET_TOL_DEG = 13.0
FULL_SCAN_OVERSHOOT_ACCEPT_DEG = 24.0
FULL_SCAN_TARGET_TIMEOUT_S = 3.2

PERIODIC_FULL_SCAN_ENABLED = True

PERIODIC_FULL_SCAN_DISTANCE_CM = 150.0
PERIODIC_FULL_SCAN_COOLDOWN_S = 55.0
PERIODIC_FULL_SCAN_POST_DRIVE_S = 10.0
PERIODIC_FULL_SCAN_POST_DRIVE_CM = 35.0
PERIODIC_FULL_SCAN_MAX_PER_RUN = 6

MAPPING_SNAPSHOT_ENABLED = True
MAPPING_SNAPSHOT_DISTANCE_CM = 25.0
MAPPING_SNAPSHOT_RETRY_CM = 8.0
MAPPING_SNAPSHOT_SETTLE_S = 0.38
MAPPING_SNAPSHOT_MAX_WAIT_S = 1.05
MAPPING_SNAPSHOT_HEADING_TOL_DEG = 8.0
MAPPING_SNAPSHOT_MIN_PACKETS = 1
MAPPING_SNAPSHOT_MAX_WALL_RANGE_CM = 155.0
MAPPING_SNAPSHOT_MIN_CLEARANCE_CM = BLOCKED_DISTANCE + 2.0 # dont pause for mapping if the wall is too close
MAPPING_SNAPSHOT_AFTER_TURN = True
MAPPING_SNAPSHOT_MIN_INTERVAL_S = 1.8

FRONTIER_INFORMATION_GAIN_ENABLED = True
FRONTIER_INFO_RADIUS_CELLS = 5
FRONTIER_TENTATIVE_RADIUS_CELLS = 4
FRONTIER_UNKNOWN_WEIGHT = 0.22 # reward frontiers that reveal more unknown cells
FRONTIER_TENTATIVE_WEIGHT = 0.85
FRONTIER_DISTANCE_PENALTY = 0.10
FRONTIER_VISIT_PENALTY = 0.15
FRONTIER_GOAL_POOL = 48

CONTINUITY_SUPPORT_ENABLED = True
CONTINUITY_MIN_GAP_CELLS = 1.7
CONTINUITY_MAX_GAP_CELLS = 5.2
CONTINUITY_HEADING_TOL_DEG = 18.0
CONTINUITY_NORMAL_DEVIATION_CELLS = 1.25
CONTINUITY_SUPPORT_WEIGHT = 0.46
CONTINUITY_FREE_VETO_HITS = 3
CONTINUITY_FREE_VETO_LOG_ODDS = -0.75

ADAPTIVE_MAPPING_SCAN_ENABLED = True # allow extra scans when the map is uncertain
ADAPTIVE_SCAN_LOCAL_RADIUS_CELLS = 6
ADAPTIVE_SCAN_MIN_TENTATIVE_CELLS = 8
ADAPTIVE_SCAN_MIN_TENTATIVE_RATIO = 0.50
ADAPTIVE_SCAN_LOW_POSE_CONF_PCT = 46.0
ADAPTIVE_SCAN_MIN_KNOWN_CELLS = 18
ADAPTIVE_SCAN_MIN_CLEARANCE_CM = COMFORT_DISTANCE
ADAPTIVE_SCAN_MIN_TRAVEL_SINCE_FULL_CM = 70.0
ADAPTIVE_SCAN_MIN_TIME_SINCE_FULL_S = 35.0
ADAPTIVE_SCAN_MAX_PER_RUN = 2

VICINITY_STALL_WINDOW_S = 30.0 # how long to watch for the robot staying in the same area
VICINITY_STALL_RADIUS_CELLS = 6.0     
VICINITY_STALL_NET_DISPLACEMENT_CELLS = 5.0
VICINITY_STALL_SAMPLE_DT_S = 0.50
VICINITY_STALL_MIN_SAMPLES = 18
VICINITY_STALL_REPEAT_STUCK_EVENTS = 2

MOVE_COMMIT_S = 3.2
POST_SCANTURN_COMMIT_S = 8.0
POST_SCANTURN_MIN_CM = 20.0  
SCANTURN_COOLDOWN_S = 8.0   

WALLHUG_WINDOW_S = 6.0
WALLHUG_MAX_ARCS = 10   
WALLHUG_MIN_PROGRESS_CM = 14.0
WALLHUG_ESCAPE_COOLDOWN_S = 7.0  

OPEN_CAPTURE_CM = 34.0   # distance that counts as finding an open direction   
STRONG_OPEN_CAPTURE_CM = 52.0
OPEN_CAPTURE_MIN_HITS = 2    
OPEN_CAPTURE_COMMIT_S = 8.0
OPEN_CAPTURE_MIN_DRIVE_CM = 22.0
TURN_OBSERVE_MAX_CM = 190.0
STUCK_WINDOW_S = 5.5
STUCK_MIN_DIST_CM = 3.5
FALLBACK_CM_PER_SPEED_S = 0.33

# PYGAME DISPLAYYYYYYYYY
CELL_PX = 10
WIN_W = 940
WIN_H = 720
FPS = 30
COL_UNKNOWN = (0, 0, 0)
COL_ROBOT = (40, 110, 255)
COL_TRAIL = (255, 255, 255)
COL_PLAN = (255, 0, 220)
COL_SENSOR = (255, 95, 60)
COL_START = (180, 120, 255)
COL_EXIT = (255, 255, 255)
# LOCKS FOR SHARED DATAGGGG
pose_lock = threading.Lock()
map_lock = threading.Lock()
send_lock = threading.Lock()
settled_scan_lock = threading.Lock()
settled_scan_samples = deque(maxlen=240) # store recent sonar readings while the robot is settled

robot_x = START_X # starting robot position in the middle of the map
robot_y = START_Y
robot_heading = 0.0
raw_robot_x = START_X
raw_robot_y = START_Y
raw_robot_heading = 0.0
online_heading_bias_deg = 0.0
start_cell = (int(round(START_X)), int(round(START_Y))) # remember where the robot originally started
exit_cell: Optional[Tuple[int, int]] = None
trail: Deque[Tuple[float, float]] = deque(maxlen=30000)
trail.append((robot_x, robot_y))
raw_trail: Deque[Tuple[float, float]] = deque(maxlen=30000)
raw_trail.append((raw_robot_x, raw_robot_y))
grid_shape = (GRID_HEIGHT, GRID_WIDTH)
occupancy_grid = np.zeros(grid_shape)

obstacle_hits = np.zeros(grid_shape)
arc_obstacle_hits = np.zeros(grid_shape)
turn_obstacle_hits = np.zeros(grid_shape)
support_obstacle_hits = np.zeros(grid_shape)

# remember the heading of stable obstacle readings
stable_endpoint_heading = np.full(grid_shape, np.nan)
free_hits = np.zeros(grid_shape)
visit_count = np.zeros(grid_shape)

LOG_ODDS_FREE = -0.42 # log odds values used to update occupied and free cells
LOG_ODDS_OCC = 0.85
LOG_ODDS_ARC_OCC = 0.52
LOG_ODDS_TURN_OCC = 0.24
LOG_ODDS_SUPPORT_OCC = 0.20
LOG_ODDS_MIN = -4.0
LOG_ODDS_MAX = 4.0
LOG_ODDS_FREE_THRESH = -0.35
LOG_ODDS_FIRST_THRESH = 0.65
LOG_ODDS_CONFIRMED_THRESH = 1.45

SONAR_HALF_CONE_DEG = 4.0
ENDPOINT_RADIAL_SNAP_CELLS = 1
log_odds_grid = np.zeros(grid_shape, dtype=np.float32) # stores the probability evidence for each map cell

# LIKELIHOOD FIELDDDDDDDDDD

OCCUPIED_PROB_HARD = 0.76
FREE_PROB_DISPLAY = 0.38

NAV_LIKELIHOOD_SEED_PROB = 0.68
NAV_LIKELIHOOD_SIGMA_CELLS = 1.55  
NAV_LIKELIHOOD_SOFT_RADIUS_CELLS = 4.5  

SCAN_LIKELIHOOD_SEED_PROB = 0.66
SCAN_LIKELIHOOD_SIGMA_CELLS = 1.10  
SCAN_LIKELIHOOD_RADIUS_CELLS = 3.0   

LIKELIHOOD_COST_WEIGHT = 3.6
LIKELIHOOD_LOCAL_HEADING_WEIGHT = 8.5
LIKELIHOOD_CACHE_MIN_INTERVAL_S = 0.45

# COMMUNICATION STUFFFF
sock: Optional[socket.socket] = None
stop_event = threading.Event()
connected_event = threading.Event()
first_packet_event = threading.Event()
# keeping the main runtime information here
state = {"connected": False,
    "dist": MAXIMUM_SENSOR_RANGE,
    "raw_dist": MAXIMUM_SENSOR_RANGE,
    "yaw_zero": None,
    "exit_found": False,
    "mode": "BOOT",
    "behaviour": "starting",
    "active_cmd": "S",
    "distance_cm_total": 0.0,
    "start_t": time.time(),
    "paused": False,
    "result_ready": False,}

last_left_encoder: Optional[int] = None
last_right_encoder: Optional[int] = None
last_update_t: Optional[float] = None
previous_yaw: Optional[float] = None
previous_yaw_time: Optional[float] = None
packet_times: Deque[float] = deque(maxlen=80) # recent packet times are used to work out telemetry speed
progress_history: Deque[Tuple[float, float]] = deque(maxlen=200)

vicinity_history: Deque[Tuple[float, float, float, float, int]] = deque(maxlen=180)
range_history: Deque[float] = deque(maxlen=7)

active_command = "S"
last_command_send = 0.0

controller = {"mode": "WAIT_FIRST", "hand": "right",}
# creates a fresh cache for the likelihood field
def make_likelihood_cache() -> Dict[str, object]:
    return {"field": np.zeros(grid_shape, dtype=np.float32),
            "distance": np.full(grid_shape, 999.0, dtype=np.float32),
            "map_updates": -1, "t": 0.0}

likelihood_cache = {"navigation": make_likelihood_cache(),
                    "scan": make_likelihood_cache()}

def invalidate_likelihood_caches() -> None: # reset the cached map when the map changes
    for cache in likelihood_cache.values():
        cache["map_updates"] = -1
        cache["t"] = 0.0
encoder_diag = {}

BASE_CM_PER_TICK = math.pi * 6.5 / 360.0 # converting encoder ticks into travelled distance
TRACK_WIDTH_CM = 11.0
PIVOT_TRANSLATION_YAW_MAX_DEG = 2.0
PIVOT_TRANSLATION_MAX_CM = 1.2
WHEEL_SCALE_MIN = 0.82
WHEEL_SCALE_MAX = 1.18
WHEEL_CALIBRATION_MIN_SAMPLES = 12
WHEEL_CALIBRATION_WINDOW = 180
WHEEL_CALIBRATION_PRIOR_WEIGHT = 8.0
wheel_calibration = {
    "left_sign_score": 4.0,
    "right_sign_score": -4.0,}
# keep recent wheel readings for calibration
wheel_calibration_samples: Deque[Tuple[float, float, float]] = deque(maxlen=WHEEL_CALIBRATION_WINDOW)

def clamp(v: float, lo: float, hi: float) -> float: # keep a value between its minimum and maximum
    return max(lo, min(hi, v))

def normalise_angle(a: float) -> float: # keep headings between 0 and 360 degrees
    return a % 360.0
def signed_angle_error(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0

def normalise_distance(raw: object) -> float: # clean up the raw sonar reading before using it
    try:
        d = float(raw)
    except Exception:
        return MAXIMUM_SENSOR_RANGE
    if d <= 0 or d >= NO_ECHO_DISTANCE:
        return MAXIMUM_SENSOR_RANGE
    return clamp(d, MINIMUM_SENSOR_RANGE, MAXIMUM_SENSOR_RANGE)
# GETTING THE CURRENT POSEEEE
def current_pose() -> Tuple[float, float, float]:
    with pose_lock:
        return robot_x, robot_y, robot_heading

def update_map_counts() -> None: # count what the robot currently knows about the map
    with map_lock:
        free = int(np.count_nonzero(occupancy_grid == FREE))
        first = int(np.count_nonzero(occupancy_grid == OBSTACLE_FIRST))
        confirmed = int(np.count_nonzero(occupancy_grid == OBSTACLE_CONFIRMED))
    known = free + first + confirmed
    state["free"] = free
    state["first"] = first
    state["confirmed"] = confirmed
    state["known"] = known
    state["unknown"] = int(GRID_HEIGHT * GRID_WIDTH - known)

def logodds_to_probability(values): # turn log odds back into normal probability
    arr = np.asarray(values, dtype=np.float32)
    arr = np.clip(arr, LOG_ODDS_MIN, LOG_ODDS_MAX)
    return 1.0 / (1.0 + np.exp(-arr))

def probability_to_rgb(prob: float) -> Tuple[int, int, int]: # choose a display colour from the occupancy probability
    p = float(clamp(prob, 0.0, 1.0))
    if abs(p - 0.5) < 0.035:
        return (12, 12, 15)
    if p < 0.5:
        confidence = clamp((0.5 - p) / 0.5, 0.0, 1.0)
        return (int(18 + 18 * (1.0 - confidence)),
                int(35 + 90 * confidence),
                int(55 + 170 * confidence))
    confidence = clamp((p - 0.5) / 0.5, 0.0, 1.0)
    return (int(70 + 185 * confidence),
            int(52 + 125 * (1.0 - confidence)),
            int(20 + 30 * (1.0 - confidence)))
# BUILDING THE LIKELIHOOD FIELDDDD
def compute_likelihood_field( 
    log_grid: Optional[np.ndarray] = None,
    force: bool = False, profile: str = "navigation",):
    profile = "scan" if str(profile).lower().startswith("scan") else "navigation"
    if profile == "scan":
        seed_prob = SCAN_LIKELIHOOD_SEED_PROB
        sigma_cells = SCAN_LIKELIHOOD_SIGMA_CELLS
        radius_cells = SCAN_LIKELIHOOD_RADIUS_CELLS
    else:
        seed_prob = NAV_LIKELIHOOD_SEED_PROB
        sigma_cells = NAV_LIKELIHOOD_SIGMA_CELLS
        radius_cells = NAV_LIKELIHOOD_SOFT_RADIUS_CELLS

    cache = likelihood_cache[profile]
    if log_grid is None:
        updates = int(state.get("map_updates", 0))
        now = time.time()
        # reuse the previous field if the map has not changed much
        if (not force and int(cache.get("map_updates", -1)) == updates and
                now - float(cache.get("t", 0.0)) < LIKELIHOOD_CACHE_MIN_INTERVAL_S):
            return cache["field"], cache["distance"]
        with map_lock:
            lg = log_odds_grid.copy()
    else:
        updates = -999
        now = time.time()
        lg = np.asarray(log_grid, dtype=np.float32)

    probs = logodds_to_probability(lg)
    obstacles = probs >= seed_prob # cells above this probability are treated as obstacles
    if not np.any(obstacles):
        distance = np.full(lg.shape, 999.0, dtype=np.float32)
        risk = np.zeros(lg.shape, dtype=np.float32)
    elif SCIPY_AVAILABLE and distance_transform_edt is not None:

        distance, nearest = distance_transform_edt(~obstacles, return_indices=True)
        distance = distance.astype(np.float32)
        seed_strength = np.clip((probs - seed_prob) / max(1e-6, 1.0 - seed_prob), 0.0, 1.0)
        nearest_strength = seed_strength[nearest[0], nearest[1]].astype(np.float32)
        gaussian = np.exp(-0.5 * (distance / sigma_cells) ** 2).astype(np.float32)
        risk = gaussian * (0.55 + 0.45 * nearest_strength)
        risk[obstacles] = np.maximum(risk[obstacles], 0.82 + 0.18 * seed_strength[obstacles])
        risk[distance > radius_cells] = 0.0
    else:

        ys, xs = np.nonzero(obstacles)
        distance = np.full(lg.shape, 999.0, dtype=np.float32)
        risk = np.zeros(lg.shape, dtype=np.float32)
        radius_i = int(math.ceil(radius_cells))
        for oy, ox in zip(ys, xs):
            y0 = max(0, oy - radius_i)
            y1 = min(GRID_HEIGHT, oy + radius_i + 1)
            x0 = max(0, ox - radius_i)
            x1 = min(GRID_WIDTH, ox + radius_i + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            d = np.sqrt((xx - ox) ** 2 + (yy - oy) ** 2)
            distance[y0:y1, x0:x1] = np.minimum(distance[y0:y1, x0:x1], d)
            strength = float(clamp((probs[oy, ox] - seed_prob) / max(1e-6, 1.0 - seed_prob), 0.0, 1.0))
            kernel = np.exp(-0.5 * (d / sigma_cells) ** 2) * (0.55 + 0.45 * strength)
            kernel[d > radius_cells] = 0.0
            risk[y0:y1, x0:x1] = np.maximum(risk[y0:y1, x0:x1], kernel.astype(np.float32))
        risk[obstacles] = np.maximum(risk[obstacles], 0.82)

    if log_grid is None:
        cache.update(field=risk, distance=distance, map_updates=updates, t=now)
    return risk.astype(np.float32), distance.astype(np.float32)

def raw_send(cmd: str) -> None: # sending the command through the socket
    if sock is None:
        return
    with send_lock:
        try:
            sock.send((cmd.strip() + "\n").encode()) # send the actual text command to the robot
        except Exception as exc:
            state["error"] = "send: " + str(exc)

def set_command(cmd: str, behaviour: Optional[str] = None) -> None:
    global active_command
    active_command = cmd.strip()
    state["active_cmd"] = active_command
    if behaviour is not None:
        state["behaviour"] = behaviour

def stop_robot(behaviour: str = "stop") -> None:
    set_command("S", behaviour)
    raw_send("S")

def pump_command() -> None:
    global last_command_send
    if not state.get("connected"):
        return
    if bool(state.get("paused")):
        raw_send("S")
        return
    now = time.time()
    if now - last_command_send >= COMMAND_REFRESH:
        raw_send(active_command)
        last_command_send = now

def parse_average_command_speed() -> float:
    cmd = active_command.upper().strip()
    try:
        if cmd.startswith("W:"):
            unused_value, l, r = cmd.split(":")[:3]
            return (float(l) + float(r)) * 0.5
        if cmd.startswith("F"):
            return float(cmd.split(":", 1)[1]) if ":" in cmd else FORWARD_SPEED
        if cmd.startswith("B"):
            return -(float(cmd.split(":", 1)[1]) if ":" in cmd else REVERSE_SPEED)
    except Exception:
        pass
    return 0.0

# heading PIDDDDDDD
HEADING_PID_KP = 0.23
HEADING_PID_KI = 0.004
HEADING_PID_KD = 0.035
HEADING_PID_I_CLAMP = 60.0
MOTOR_LEFT_TRIM = float(os.environ.get("MBOT2_LEFT_TRIM", "1.00"))
MOTOR_RIGHT_TRIM = float(os.environ.get("MBOT2_RIGHT_TRIM", "1.00"))
heading_pid = {"target": None, "i": 0.0, "last_e": 0.0, "last_t": None}
# PID keeps the robot pointing towards the target heading
def drive_heading_pid(target_h: float, base_speed: float, behaviour: str) -> None:
    h = current_pose()[2]
    target_h = normalise_angle(target_h)

    if heading_pid["target"] is None or abs(signed_angle_error(target_h, float(heading_pid["target"]))) > 8.0:
        heading_pid.update(target=target_h, i=0.0, last_e=0.0, last_t=None)

    now = time.time()
    dt = 0.05 if heading_pid["last_t"] is None else clamp(now - float(heading_pid["last_t"]), 0.02, 0.35)
    error = signed_angle_error(target_h, h)
    heading_pid["i"] = clamp(float(heading_pid["i"]) + error * dt, -HEADING_PID_I_CLAMP, HEADING_PID_I_CLAMP)
    change = (error - float(heading_pid["last_e"])) / dt
    # combine the P I and D parts into one steering correction
    correction = HEADING_PID_KP * error + HEADING_PID_KI * float(heading_pid["i"]) + HEADING_PID_KD * change
    heading_pid["last_e"] = error
    heading_pid["last_t"] = now

    left = int(round(clamp((base_speed - correction) * MOTOR_LEFT_TRIM, -28, 28)))
    right = int(round(clamp((base_speed + correction) * MOTOR_RIGHT_TRIM, -28, 28)))
    set_command(f"W:{left}:{right}", behaviour + " pid_err={:.1f}".format(error))
    controller["intent_h"] = target_h
    state["pid_corrections"] = state.get("pid_corrections", 0) + 1