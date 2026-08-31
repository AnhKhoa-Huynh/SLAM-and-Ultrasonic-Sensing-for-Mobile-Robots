# EXPLORATIONNNNNN
def set_mode(mode: str, behaviour: str = "") -> None:
    controller["mode"] = mode
    state["mode"] = mode
    if behaviour:
        state["behaviour"] = behaviour


def turn_to(target_h: float, after: str = "MOVE", reason: str = "turn") -> None:
    controller["target_h"] = normalise_angle(target_h)
    controller["after_turn"] = after
    controller["turn_t0"] = time.time()
    controller["turn_reason"] = reason
    controller["intent_h"] = normalise_angle(target_h)
    set_mode("TURN", reason)


def begin_mapping_snapshot(reason: str, after: str = "MOVE") -> None:
    if not MAPPING_SNAPSHOT_ENABLED:
        set_mode(after, "mapping snapshot not active")
        return
    unused_value, unused_value, h = current_pose()
    now = time.time()
    controller["mapping_snapshot_heading"] = float(h)
    controller["mapping_snapshot_started_t"] = now
    controller["mapping_snapshot_settle_t"] = now + MAPPING_SNAPSHOT_SETTLE_S
    controller["mapping_snapshot_deadline"] = now + MAPPING_SNAPSHOT_MAX_WAIT_S
    controller["mapping_snapshot_min_packet"] = int(state.get("packets", 0))
    controller["mapping_snapshot_reason"] = str(reason)
    controller["mapping_snapshot_after"] = str(after)
    clear_settled_scan_samples()
    state["mapping_snapshot_attempts"] = int(state.get("mapping_snapshot_attempts", 0)) + 1
    stop_robot("active-mapping snapshot")
    set_mode("MAP_SNAPSHOT", reason)


def should_trigger_mapping_snapshot(now: float) -> Tuple[bool, str]:
    if not MAPPING_SNAPSHOT_ENABLED or str(controller.get("mode", "")) != "MOVE":
        return False, ""
    if bool(controller.get("full_scan_active", False)) or bool(controller.get("post_scan_drive_pending", False)):
        return False, ""
    travel = float(state.get("distance_cm_total", 0.0))
    if travel < float(controller.get("next_mapping_snapshot_travel", MAPPING_SNAPSHOT_DISTANCE_CM)):
        return False, ""
    if now - float(controller.get("last_mapping_snapshot_t", -999.0)) < MAPPING_SNAPSHOT_MIN_INTERVAL_S:
        return False, ""
    distance = float(state.get("dist", MAXIMUM_SENSOR_RANGE))

    # only want a wall reading that is useful but not too close
    useful_wall_return = MINIMUM_SENSOR_RANGE <= distance <= MAPPING_SNAPSHOT_MAX_WALL_RANGE_CM
    safe_to_pause = distance >= MAPPING_SNAPSHOT_MIN_CLEARANCE_CM
    if not (useful_wall_return and safe_to_pause):
        controller["next_mapping_snapshot_travel"] = travel + MAPPING_SNAPSHOT_RETRY_CM
        state["mapping_snapshot_skips"] = int(state.get("mapping_snapshot_skips", 0)) + 1
        return False, ""
    return True, "25 cm active-mapping snapshot; sonar {:.1f} cm".format(distance)

# asking for a 360 scan but only if there is enough space
def request_full_scan(
    reason: str,
    *,
    retry_after_preflight: bool = False,
    trigger_kind: str = "startup",) -> None:
    distance = float(state.get("dist", MAXIMUM_SENSOR_RANGE))
    if not retry_after_preflight:
        controller["full_scan_preflight_retries"] = 0

    retries = int(controller.get("full_scan_preflight_retries", 0))
    if distance <= FULL_SCAN_PREFLIGHT_CM: # too close to the wall to safely spin around
        if retries < FULL_SCAN_PREFLIGHT_MAX_RETRIES:
            controller["full_scan_preflight_retries"] = retries + 1
            controller["pending_full_scan_reason"] = reason
            controller["pending_full_scan_kind"] = trigger_kind
            state["full_scan_deferrals"] = int(state.get("full_scan_deferrals", 0)) + 1
            begin_reverse(
                FULL_SCAN_PREFLIGHT_REVERSE_S,
                after="FULL_SCAN_PREP",
                reason="one-time clearance before 360 scan",
            )
            return

        state["full_scan_aborts"] = int(state.get("full_scan_aborts", 0)) + 1
        arm_full_scan_reentry_guard(completed=False)
        set_mode("MOVE", "360 scan skipped: no safe initial rotation clearance")
        return

    begin_full_scan(reason, trigger_kind=trigger_kind)

# STARTING THE FULL SCANNNN
def begin_full_scan(reason: str, trigger_kind: str = "startup") -> None:
    unused_value, unused_value, h = current_pose()
    controller["full_scan_done"] = 0.0
    controller["full_scan_target"] = normalise_angle(h - FULL_SCAN_STEP) # setting the next angle for the robot to scan
    controller["full_scan_settle_t"] = 0.0
    controller["full_scan_readings"] = []
    controller["full_scan_t0"] = time.time()
    controller["full_scan_target_started_t"] = float(controller["full_scan_t0"])
    controller["full_scan_prev_err"] = None
    controller["full_scan_active"] = True
    controller["full_scan_reason"] = reason
    controller["full_scan_trigger_kind"] = str(trigger_kind)
    controller["full_scan_hard_hits"] = 0
    controller["full_scan_hard_last_packet"] = -1
    controller["full_scan_preflight_retries"] = 0
    state["scan_count"] = int(state.get("scan_count", 0)) + 1
    state["full_scan_attempts"] = int(state.get("full_scan_attempts", 0)) + 1


    controller["open_commit_until"] = 0.0
    controller["move_commit_until"] = 0.0
    controller["drive_commit_dist"] = float(state.get("distance_cm_total", 0.0))
    controller["live_open_hits"] = 0
    stop_robot("prepare full scan")
    set_mode("FULL_SCAN", reason)
def begin_start_scan(reason: str = "startup 360-degree scan") -> None:
    request_full_scan(reason, trigger_kind="startup")


def arm_full_scan_reentry_guard(completed: bool) -> None:
    now = time.time()
    travel = float(state.get("distance_cm_total", 0.0))
    controller["last_full_scan_t"] = now
    controller["last_full_scan_travel"] = travel
    # decide how far to travel before doing another full scan
    controller["next_periodic_full_scan_travel"] = travel + PERIODIC_FULL_SCAN_DISTANCE_CM
    controller["next_mapping_snapshot_travel"] = max(
        float(controller.get("next_mapping_snapshot_travel", 0.0)),
        travel + MAPPING_SNAPSHOT_DISTANCE_CM,
    )
    controller["periodic_full_scan_inhibit_until"] = now + PERIODIC_FULL_SCAN_COOLDOWN_S
    controller["full_scan_post_drive_until"] = now + PERIODIC_FULL_SCAN_POST_DRIVE_S
    controller["full_scan_post_drive_dist"] = travel + PERIODIC_FULL_SCAN_POST_DRIVE_CM
    vicinity_history.clear()
    if completed and str(controller.get("full_scan_trigger_kind", "startup")) in ("periodic", "vicinity", "uncertainty"):
        state["periodic_full_scan_completions"] = int(state.get("periodic_full_scan_completions", 0)) + 1
    if completed and str(controller.get("full_scan_trigger_kind", "startup")) == "uncertainty":
        state["adaptive_full_scan_completions"] = int(state.get("adaptive_full_scan_completions", 0)) + 1


def update_vicinity_history(now: float, mode: str) -> None:


    if mode in ("START_SCAN", "FULL_SCAN", "MAP_SNAPSHOT"):
        return
    if vicinity_history and now - vicinity_history[-1][0] < VICINITY_STALL_SAMPLE_DT_S:
        return
    x, y, unused_value = current_pose() # get the robots current position for the history
    vicinity_history.append((
        now,
        float(x),
        float(y),
        float(state.get("distance_cm_total", 0.0)),
        int(state.get("stuck_events", 0)),
    ))
    keep_s = VICINITY_STALL_WINDOW_S + 5.0
    while vicinity_history and now - vicinity_history[0][0] > keep_s:
        vicinity_history.popleft()

# checks if the robot has been hanging around the same area
def vicinity_stalled(now: float) -> Tuple[bool, str]:
    if len(vicinity_history) < VICINITY_STALL_MIN_SAMPLES:
        return False, ""
    points = [p for p in vicinity_history if now - p[0] <= VICINITY_STALL_WINDOW_S + 0.75]
    if len(points) < VICINITY_STALL_MIN_SAMPLES:
        return False, ""
    span = points[-1][0] - points[0][0]
    if span < VICINITY_STALL_WINDOW_S:
        return False, ""

    xs = [p[1] for p in points]
    ys = [p[2] for p in points]
    cx = float(np.median(xs))
    cy = float(np.median(ys))
    # how far the robot wandered from the centre of this area
    max_radius = max(math.hypot(px - cx, py - cy) for px, py in zip(xs, ys))
    net_disp = math.hypot(points[-1][1] - points[0][1], points[-1][2] - points[0][2])
    stuck_delta = int(points[-1][4]) - int(points[0][4])

    confined = (
        max_radius <= VICINITY_STALL_RADIUS_CELLS
        and net_disp <= VICINITY_STALL_NET_DISPLACEMENT_CELLS
    )
    repeated_recovery = (
        stuck_delta >= VICINITY_STALL_REPEAT_STUCK_EVENTS
        and net_disp <= VICINITY_STALL_RADIUS_CELLS + 1.0
    )
    if not (confined or repeated_recovery):
        return False, ""
    return True, (
        "confined {:.0f}s near same area; radius {:.1f} cells, net {:.1f} cells, stuck +{}"
        .format(span, max_radius, net_disp, stuck_delta)
    )


def local_mapping_uncertainty(x: float, y: float, radius: int = ADAPTIVE_SCAN_LOCAL_RADIUS_CELLS) -> Dict[str, float]:
    cx, cy = int(round(x)), int(round(y))
    tentative = confirmed = known = unknown = 0
    with map_lock:
        for grid_y in range(max(0, cy - radius), min(GRID_HEIGHT, cy + radius + 1)):
            for grid_x in range(max(0, cx - radius), min(GRID_WIDTH, cx + radius + 1)):
                v = int(occupancy_grid[grid_y, grid_x])
                if v == UNKNOWN:
                    unknown += 1
                else:
                    known += 1
                if v == OBSTACLE_FIRST:
                    tentative += 1
                elif v == OBSTACLE_CONFIRMED:
                    confirmed += 1
    ratio = tentative / max(1, tentative + confirmed) # proportion of obstacle cells that are still uncertain
    return {
        "tentative": float(tentative), "confirmed": float(confirmed),
        "known": float(known), "unknown": float(unknown),
        "tentative_ratio": float(ratio),}
# decide whether another full scan is actually needed
def should_trigger_periodic_full_scan(now: float):
    if not PERIODIC_FULL_SCAN_ENABLED and not ADAPTIVE_MAPPING_SCAN_ENABLED:
        return False, "", ""
    if str(controller.get("mode", "")) != "MOVE":
        return False, "", ""
    if bool(controller.get("full_scan_active", False)):
        return False, "", ""
    if now < float(controller.get("periodic_full_scan_inhibit_until", 0.0)):
        return False, "", ""

    travel = float(state.get("distance_cm_total", 0.0))


    if (now < float(controller.get("full_scan_post_drive_until", 0.0)) or
            travel < float(controller.get("full_scan_post_drive_dist", 0.0))):
        return False, "", ""

    nonstartup_requests = (int(state.get("periodic_full_scan_triggers", 0)) +
                           int(state.get("vicinity_full_scan_triggers", 0)) +
                           int(state.get("adaptive_full_scan_triggers", 0)))
    if nonstartup_requests >= PERIODIC_FULL_SCAN_MAX_PER_RUN:
        return False, "", ""

    next_travel = float(controller.get("next_periodic_full_scan_travel", PERIODIC_FULL_SCAN_DISTANCE_CM))
    if PERIODIC_FULL_SCAN_ENABLED and travel >= next_travel:
        return True, (
            "periodic 360 scan after {:.0f} cm since last full scan"
            .format(max(0.0, travel - float(controller.get("last_full_scan_travel", 0.0))))
        ), "periodic"


    
    if (ADAPTIVE_MAPPING_SCAN_ENABLED
            and int(state.get("adaptive_full_scan_triggers", 0)) < ADAPTIVE_SCAN_MAX_PER_RUN
            and float(state.get("dist", MAXIMUM_SENSOR_RANGE)) >= ADAPTIVE_SCAN_MIN_CLEARANCE_CM
            and travel - float(controller.get("last_full_scan_travel", 0.0)) >= ADAPTIVE_SCAN_MIN_TRAVEL_SINCE_FULL_CM
            and now - float(controller.get("last_full_scan_t", -999.0)) >= ADAPTIVE_SCAN_MIN_TIME_SINCE_FULL_S
            and now >= float(controller.get("move_commit_until", 0.0))):
        ux, uy, unused_value = current_pose()
        local = local_mapping_uncertainty(ux, uy) # check how uncertain the map is around the robot
        tentative = int(local["tentative"]); known = int(local["known"]); ratio = float(local["tentative_ratio"])
        pose_conf = float(state.get("pose_conf_pct", 100.0))
        weak_wall_cluster = (tentative >= ADAPTIVE_SCAN_MIN_TENTATIVE_CELLS
                             and ratio >= ADAPTIVE_SCAN_MIN_TENTATIVE_RATIO
                             and known >= ADAPTIVE_SCAN_MIN_KNOWN_CELLS)
        low_pose_with_wall_evidence = (pose_conf <= ADAPTIVE_SCAN_LOW_POSE_CONF_PCT
                                       and tentative >= max(4, ADAPTIVE_SCAN_MIN_TENTATIVE_CELLS // 2)
                                       and known >= ADAPTIVE_SCAN_MIN_KNOWN_CELLS)
        if weak_wall_cluster or low_pose_with_wall_evidence:
            return True, (
                "uncertainty 360 scan: local tentative={} confirmed={} ratio={:.2f}, pose confidence {:.0f}%"
                .format(tentative, int(local["confirmed"]), ratio, pose_conf)
            ), "uncertainty"

    stalled, detail = vicinity_stalled(now) if PERIODIC_FULL_SCAN_ENABLED else (False, "")
    if stalled:
        return True, "360 scan requested: " + detail, "vicinity"
    return False, "", ""


def close_full_scan_accounting(completed: bool) -> None:
    if not bool(controller.get("full_scan_active", False)):
        return
    controller["full_scan_active"] = False
    if completed:
        state["full_scans"] = int(state.get("full_scans", 0)) + 1


def abort_full_scan(reason: str, reverse_after: bool = True) -> None:
    close_full_scan_accounting(completed=False)
    arm_full_scan_reentry_guard(completed=False)
    state["full_scan_aborts"] = int(state.get("full_scan_aborts", 0)) + 1
    controller["full_scan_settle_t"] = 0.0
    controller["full_scan_readings"] = []
    controller["full_scan_hard_hits"] = 0
    if reverse_after:
        begin_reverse(0.52, after="MOVE", reason="abort 360 scan and create clearance")
    else:
        stop_robot(reason) # stop moving once the full scan is done
        set_mode("MOVE", reason)

# FULL SCAN FINISHED, now choose where to go
def finish_full_scan(reason: str = "full scan complete") -> None:
    readings = list(controller.get("full_scan_readings", []))
    x, y, h = current_pose()
    controller["full_scan_hard_hits"] = 0
    close_full_scan_accounting(completed=True)
    arm_full_scan_reentry_guard(completed=True)
    stop_robot(reason)
    note_rotation(FULL_SCAN_TOTAL)

    if not readings:
        set_mode("MOVE", reason + ": no valid samples")
        return

    def score_reading(item):
        hh, dd, unknown = item
        clearance = min(dd, 85.0) * 0.18
        unknown_bonus = 10.0 if unknown else 0.0
        return heading_exploration_score(x, y, hh, max_cells=20) + clearance + unknown_bonus

    usable = [r for r in readings if r[1] >= USABLE_DISTANCE]
    chosen = max(usable or readings, key=score_reading)
    detail = "{}; chose {:.0f}deg, sonar {:.1f}cm, samples {}".format(
        reason, chosen[0], chosen[1], len(readings))
    full_kind = str(controller.get("full_scan_trigger_kind", "startup"))
    if full_kind in ("periodic", "vicinity"):
        drive_cm = PERIODIC_FULL_SCAN_POST_DRIVE_CM
        drive_s = PERIODIC_FULL_SCAN_POST_DRIVE_S
        now = time.time()

        controller["move_commit_until"] = now + drive_s
        controller["drive_commit_dist"] = state.get("distance_cm_total", 0.0) + drive_cm
        controller["scanturn_cooldown_until"] = now + drive_s + 3
    else:
        drive_cm = 20.0
        drive_s = 6.0

    if abs(signed_angle_error(chosen[0], h)) <= TURN_DEADBAND:
        commit_to_open_heading(chosen[0], detail, min_drive_cm=drive_cm, commit_s=drive_s)
    else:
        turn_to(chosen[0], after="MOVE", reason=detail)


def arm_post_scan_drive(base_heading: float, reason: str) -> None:
    now = time.time()
    heading = normalise_angle(base_heading)
    travel = state.get("distance_cm_total", 0.0)
    controller["open_commit_h"] = heading
    controller["open_commit_until"] = now + POST_SCAN_DRIVE_S
    controller["move_commit_until"] = now + POST_SCAN_DRIVE_S
    controller["drive_commit_dist"] = travel + POST_SCAN_DRIVE_CM
    controller["scanturn_cooldown_until"] = now + POST_SCAN_DRIVE_S + 3
    controller["intent_h"] = heading


def begin_reverse(duration_s: float, after: str, reason: str) -> None:
    controller["hold_until"] = time.time() + duration_s
    controller["after_hold"] = after
    set_mode("REVERSE", reason)
def is_stuck() -> bool:
    moving_cmd = active_command.upper().startswith(("F", "W"))
    clear_front = float(state.get("dist", MAXIMUM_SENSOR_RANGE)) > CONTACT_DISTANCE + 2
    if moving_cmd and clear_front:
        now = time.time()
        while progress_history and now - progress_history[0][0] > STUCK_WINDOW_S:
            progress_history.popleft()
        if len(progress_history) >= 4:
            travelled = progress_history[-1][1] - progress_history[0][1]
            if now - progress_history[0][0] >= STUCK_WINDOW_S * 0.8 and travelled < STUCK_MIN_DIST_CM:
                return True


        
        if (active_command.upper().startswith("F") and
                time.time() - float(encoder_diag.get("t", 0.0)) < 1.0 and
                float(encoder_diag.get("imbalance", 0.0)) > 0.72 and
                abs(float(encoder_diag.get("dl", 0.0))) + abs(float(encoder_diag.get("dr", 0.0))) > 8.0):
            state["wheel_imbalance_events"] = int(state.get("wheel_imbalance_events", 0)) + 1
            return True
    return False



def ray_cells_for_heading(x: float, y: float, heading: float, max_cells: int = 22) -> List[Tuple[int, int]]:
    ar = math.radians(heading)
    dx = math.cos(ar)
    dy = -math.sin(ar)
    cells: List[Tuple[int, int]] = []
    last = None
    for s in np.linspace(1.0, float(max_cells), max_cells * 2):
        grid_x = int(round(x + dx * s))
        grid_y = int(round(y + dy * s))
        if not in_grid(grid_x, grid_y):
            break
        if (grid_x, grid_y) != last:
            cells.append((grid_x, grid_y))
            last = (grid_x, grid_y)
    return cells

# gives each possible direction a scoreeeeee
def heading_exploration_score(x: float, y: float, heading: float, max_cells: int = 18) -> float:
    score = 0.0
    cells = ray_cells_for_heading(x, y, heading, max_cells=max_cells) # get the map cells lying ahead in this direction
    risk_field, unused_value = compute_likelihood_field()
    with map_lock:
        for i, (grid_x, grid_y) in enumerate(cells):
            v = int(occupancy_grid[grid_y, grid_x]) # checking what type of cell this is on the map
            visits = int(visit_count[grid_y, grid_x])
            dist_weight = 1.0 + min(i, 10) * 0.05
            if v == OBSTACLE_CONFIRMED:
                score -= 60.0 / (1.0 + i)
                break
            if v == OBSTACLE_FIRST:
                score -= 25.0 / (1.0 + i)
                break
            if v == UNKNOWN: # unexplored cells make this direction more attractive
                score += 3.0 * dist_weight
            elif v == FREE:
                score += max(0.2, 1.4 - 0.08 * visits) * dist_weight
            if visits > 3:
                score -= min(2.0, 0.25 * visits)
    for i, (grid_x, grid_y) in enumerate(cells[:12]):
        score -= LIKELIHOOD_LOCAL_HEADING_WEIGHT * float(risk_field[grid_y, grid_x]) / (1.0 + 0.15 * i)
    return score


def best_exploration_heading(x: float, y: float, h: float, hand: str) -> Tuple[float, float, List[Tuple[float, float]]]:
    if hand == "right":
        offsets = [0, -20, 20, -40, 40, -65, 65]
        hand_bias = { -20: 0.5, -40: 0.4, -65: 0.2 }
    else:
        offsets = [0, 20, -20, 40, -40, 65, -65]
        hand_bias = { 20: 0.5, 40: 0.4, 65: 0.2 }
    scored: List[Tuple[float, float]] = []
    for off in offsets:
        hh = normalise_angle(h + off)
        sc = heading_exploration_score(x, y, hh, max_cells=18) + hand_bias.get(off, 0.0)
        
        sc -= abs(off) * 0.015
        scored.append((hh, sc))
    best_h, best_s = max(scored, key=lambda t: t[1]) # take whichever heading ended up with the highest score
    return best_h, best_s, scored


def register_wallhug_arc_and_maybe_escape(x0: float, y0: float) -> bool:
    now = time.time()
    times = controller.get("wallhug_arc_times")
    if times is None:
        times = deque(maxlen=64)
        controller["wallhug_arc_times"] = times
    times.append(now)

    if now - float(controller.get("wallhug_ref_t", 0.0)) > WALLHUG_WINDOW_S:
        controller["wallhug_ref_t"] = now
        controller["wallhug_ref_travel"] = float(state.get("distance_cm_total", 0.0))
        while times and now - times[0] > WALLHUG_WINDOW_S:
            times.popleft()
        return False

    while times and now - times[0] > WALLHUG_WINDOW_S:
        times.popleft()

    net_travel = float(state.get("distance_cm_total", 0.0)) - float(controller.get("wallhug_ref_travel", 0.0))
# lots of wall following without much progress probably means looping
    hugging = (len(times) >= WALLHUG_MAX_ARCS and net_travel < WALLHUG_MIN_PROGRESS_CM)
    if not hugging:
        return False
    if now - float(controller.get("last_wallhug_escape_t", 0.0)) < WALLHUG_ESCAPE_COOLDOWN_S:
        return False

    
    controller["last_wallhug_escape_t"] = now
    controller["wallhug_ref_t"] = now
    controller["wallhug_ref_travel"] = float(state.get("distance_cm_total", 0.0))
    times.clear()
    state["wallhug_escapes"] = int(state.get("wallhug_escapes", 0)) + 1

    frontier = find_nearest_frontier(x0, y0) # try to find some unexplored space nearby
    if frontier is not None:
        state["frontier_seeks"] = int(state.get("frontier_seeks", 0)) + 1
        frontier_heading = heading_to_cell(x0, y0, frontier)

        controller["frontier_path"] = []
        controller["last_frontier_plan_t"] = 0.0
        begin_reverse(0.40, after="SIDE_ESCAPE", reason="wall-hug escape: reverse before frontier seek")
    else:
        controller["sturn_after"] = "MOVE"

        
        begin_scan_turn(-1.0 if hand_global() == "right" else 1.0, "wall-hug escape: scan away from wall")
    return True

def hand_global() -> str:
    return str(controller.get("hand", "right"))
def find_nearest_frontier(x: float, y: float, max_radius: int = 40):
    cx, cy = int(round(x)), int(round(y))

    with map_lock:
        for r in range(2, max_radius):
            frontiers = []

            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue

                    grid_x, grid_y = cx + dx, cy + dy
                    if not in_grid(grid_x, grid_y):
                        continue
                    if occupancy_grid[grid_y, grid_x] != FREE:
                        continue

                    for ax, ay in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        if in_grid(grid_x + ax, grid_y + ay) and occupancy_grid[grid_y + ay, grid_x + ax] == UNKNOWN:
                            frontiers.append((grid_x, grid_y))
                            break

            if frontiers:
                return min(frontiers, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
    return None
def blocked_inflated(x: int, y: int, inflate_cells: int = ROBOT_RADIUS_CELLS) -> bool:
    if not (0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT):
        return True
    hard_log = math.log(OCCUPIED_PROB_HARD / (1.0 - OCCUPIED_PROB_HARD))
    with map_lock:
        for oy in range(-inflate_cells, inflate_cells + 1):
            for ox in range(-inflate_cells, inflate_cells + 1):
                if ox * ox + oy * oy > inflate_cells * inflate_cells:
                    continue
                nx, ny = x + ox, y + oy
# dont plan through cells that are probably occupied
                if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT and float(log_odds_grid[ny, nx]) >= hard_log:
                    return True
    return False


def passable_for_frontier_astar(x: int, y: int) -> bool:
    if not (0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT):
        return False
    if blocked_inflated(x, y, inflate_cells=max(1, ROBOT_RADIUS_CELLS - 1)):
        return False
    free_log = math.log(FREE_PROB_DISPLAY / (1.0 - FREE_PROB_DISPLAY))
    with map_lock:
        return int(occupancy_grid[y, x]) == FREE or float(log_odds_grid[y, x]) <= free_log


def frontier_goal_cells(x: float, y: float, max_radius: int = 45) -> List[Tuple[int, int]]:
    cx, cy = int(round(x)), int(round(y))
    scored_goals: List[Tuple[float, float, Tuple[int, int], int, int]] = []
    with map_lock:
        for grid_y in range(max(1, cy - max_radius), min(GRID_HEIGHT - 1, cy + max_radius + 1)):
            for grid_x in range(max(1, cx - max_radius), min(GRID_WIDTH - 1, cx + max_radius + 1)):
                if int(occupancy_grid[grid_y, grid_x]) != FREE:
                    continue
                if visit_count[grid_y, grid_x] > 18:
                    continue
                is_frontier = any(
                    int(occupancy_grid[grid_y + ay, grid_x + ax]) == UNKNOWN
                    for ax, ay in ((1,0), (-1,0), (0,1), (0,-1)))
                if not is_frontier:
                    continue
                distance_cells = math.hypot(grid_x - cx, grid_y - cy)
                unknown_gain = 0
                tentative_gain = 0
                if FRONTIER_INFORMATION_GAIN_ENABLED:
                    r = FRONTIER_INFO_RADIUS_CELLS
                    for sy in range(max(0, grid_y - r), min(GRID_HEIGHT, grid_y + r + 1)):
                        for sx in range(max(0, grid_x - r), min(GRID_WIDTH, grid_x + r + 1)):
                            if int(occupancy_grid[sy, sx]) == UNKNOWN:
                                unknown_gain += 1
                            if (abs(sx - grid_x) <= FRONTIER_TENTATIVE_RADIUS_CELLS
                                    and abs(sy - grid_y) <= FRONTIER_TENTATIVE_RADIUS_CELLS
                                    and int(occupancy_grid[sy, sx]) == OBSTACLE_FIRST):
                                tentative_gain += 1
                    utility = (FRONTIER_UNKNOWN_WEIGHT * unknown_gain # reward unexplored areas but penalise distance and revisiting
                               + FRONTIER_TENTATIVE_WEIGHT * tentative_gain
                               - FRONTIER_DISTANCE_PENALTY * distance_cells
                               - FRONTIER_VISIT_PENALTY * float(visit_count[grid_y, grid_x]))
                else:
                    utility = -distance_cells
                scored_goals.append((float(utility), float(distance_cells), (grid_x, grid_y),
                                     int(unknown_gain), int(tentative_gain)))
    nearest_order = sorted(scored_goals, key=lambda t: t[1])
    controller["frontier_nearest_fallback_goals"] = [item[2] for item in nearest_order[:90]]
    controller["frontier_goal_scores"] = {
        item[2]: {"utility": item[0], "distance_cells": item[1],
                  "unknown_gain": item[3], "tentative_gain": item[4]}
        for item in scored_goals}
    if FRONTIER_INFORMATION_GAIN_ENABLED:
        scored_goals.sort(key=lambda t: (-t[0], t[1]))
    else:
        scored_goals.sort(key=lambda t: t[1])
    chosen = scored_goals[:FRONTIER_GOAL_POOL if FRONTIER_INFORMATION_GAIN_ENABLED else 90]
    return [item[2] for item in chosen]


# PATH FINDINGGGGG
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# A STAR PATH FINDINGGGGGGGGGGG
def astar_to_any_frontier(
    start: Tuple[int, int],
    goals: List[Tuple[int, int]],
    max_expand: int = 4500,
) -> Tuple[Optional[List[Tuple[int, int]]], int, Optional[Tuple[int, int]]]:
    import heapq
    if not goals:
        return None, 0, None
    goal_set = set(goals)
    risk_field, unused_value = compute_likelihood_field()
    moves = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (1, 1, 1.4142),]

    heuristic_goals = goals[:25]
    def heuristic(x, y):
        return min(abs(x - grid_x) + abs(y - grid_y) for grid_x, grid_y in heuristic_goals)

    open_queue = [(heuristic(*start), 0.0, start)] # first position waiting to be checked by A star
    came_from = {}
    best_cost = {start: 0.0}
    expanded = 0
    while open_queue and expanded < max_expand:
        unused_value, current_cost, current = heapq.heappop(open_queue)
        if current_cost > best_cost.get(current, float("inf")):
            continue
        expanded += 1
        if current in goal_set: # reached one of the frontier goals, rebuild the route
            return reconstruct_path(came_from, current), expanded, current
        cx, cy = current
        for dx, dy, base_cost in moves:
            nx, ny = cx + dx, cy + dy
            if not passable_for_frontier_astar(nx, ny):
                continue
            if dx and dy and (not passable_for_frontier_astar(cx, ny) or not passable_for_frontier_astar(nx, cy)):
                continue
            soft_risk = float(risk_field[ny, nx])
            step_cost = base_cost * (1.0 + LIKELIHOOD_COST_WEIGHT * soft_risk)

            
            revisit = min(1.2, float(visit_count[ny, nx]) * 0.025)
            new_cost = current_cost + step_cost + revisit
            node = (nx, ny)
            if new_cost >= best_cost.get(node, float("inf")):
                continue
            best_cost[node] = new_cost
            came_from[node] = current
            heapq.heappush(open_queue, (new_cost + heuristic(nx, ny), new_cost, node))
    return None, expanded, None


def update_frontier_plan(force: bool = False) -> Optional[List[Tuple[int, int]]]:
    now = time.time()
    if not force and now - float(controller.get("last_frontier_plan_t", 0.0)) < 2.2:
        return list(controller.get("frontier_path", [])) or None
    if now < float(controller.get("frontier_plan_fail_until", 0.0)) and not force:
        return None
    x, y, unused_value = current_pose()
    start = (int(round(x)), int(round(y)))
    goals = frontier_goal_cells(x, y)
    path, expanded, goal = astar_to_any_frontier(start, goals) # try to find a path from the robot to one of the frontiers
    if path is None and FRONTIER_INFORMATION_GAIN_ENABLED:
        fallback_goals = list(controller.get("frontier_nearest_fallback_goals", []))
        if fallback_goals and fallback_goals != goals:
            fallback_path, fallback_expanded, fallback_goal = astar_to_any_frontier(start, fallback_goals)
            expanded += fallback_expanded
            if fallback_path is not None:
                path, goal = fallback_path, fallback_goal
                state["frontier_info_fallbacks"] = int(state.get("frontier_info_fallbacks", 0)) + 1
    controller["last_frontier_plan_t"] = now


    if path is not None and goal is None:
        goal = path[-1] if path else None
    if path and len(path) >= 2:
        controller["frontier_path"] = path
        controller["frontier_goal"] = goal
        state["frontier_plans"] = int(state.get("frontier_plans", 0)) + 1
        score_info = dict(controller.get("frontier_goal_scores", {}).get(goal, {})) if goal is not None else {}
        if FRONTIER_INFORMATION_GAIN_ENABLED and score_info:
            state["frontier_info_gain_plans"] = int(state.get("frontier_info_gain_plans", 0)) + 1
            state["frontier_last_utility"] = float(score_info.get("utility", 0.0))
            state["frontier_last_unknown_gain"] = int(score_info.get("unknown_gain", 0))
            state["frontier_last_tentative_gain"] = int(score_info.get("tentative_gain", 0))
        return path
    controller["frontier_path"] = []
    controller["frontier_goal"] = None
    controller["frontier_plan_fail_until"] = now + 3.5
    state["frontier_plan_failures"] = int(state.get("frontier_plan_failures", 0)) + 1
    return None


def frontier_heading_hint() -> Optional[float]:
    path = update_frontier_plan(False)
    if not path or len(path) < 2:
        return None
    x, y, unused_value = current_pose()

    idx = min(len(path) - 1, 5) # look a few cells ahead instead of aiming at every single path cell
    while len(path) > 2 and math.hypot(path[1][0] - x, path[1][1] - y) < 2.0:
        path = path[1:]
        controller["frontier_path"] = path
        idx = min(len(path) - 1, 5)
    return heading_to_cell(x, y, path[idx])


def heading_to_cell(x: float, y: float, cell: Tuple[int, int]) -> float:
    return normalise_angle(math.degrees(math.atan2(y - cell[1], cell[0] - x)))


def update_planned_path(heading: Optional[float] = None, length_cells: int = 24) -> None:
    x, y, h = current_pose()
    hh = h if heading is None else heading
    controller["planned_path"] = ray_cells_for_heading(x, y, hh, max_cells=length_cells)

def commit_to_open_heading(heading: float, reason: str,
                           min_drive_cm: float = OPEN_CAPTURE_MIN_DRIVE_CM,
                           commit_s: float = OPEN_CAPTURE_COMMIT_S) -> None:
    heading = normalise_angle(heading)
    now = time.time()

    controller["open_commit_h"] = heading
    controller["open_commit_until"] = now + commit_s
    controller["move_commit_until"] = now + commit_s
    controller["drive_commit_dist"] = state.get("distance_cm_total", 0.0) + min_drive_cm
    controller["scanturn_cooldown_until"] = now + commit_s + 2
    controller["intent_h"] = heading
    controller["live_open_hits"] = 0
    controller["post_scan_drive_pending"] = False

    state["open_commits"] = state.get("open_commits", 0) + 1
    state["opening_captures"] = state.get("opening_captures", 0) + 1

    stop_robot("opening captured: stop scan") # found an opening so stop scanning and start moving
    set_mode("MOVE", reason)
    update_planned_path(heading, 30)

def heading_faces_unknown(x: float, y: float, heading: float, look_cells: int = 6) -> bool:
    ar = math.radians(heading)
    dx, dy = math.cos(ar), -math.sin(ar)
    unknown = 0
    with map_lock:
        for s in range(2, look_cells + 2):
            grid_x = int(round(x + dx * s))
            grid_y = int(round(y + dy * s))
            if not in_grid(grid_x, grid_y):
                break
            v = int(occupancy_grid[grid_y, grid_x])
            if v == OBSTACLE_CONFIRMED:
                return False
            if v == UNKNOWN:
                unknown += 1
    return unknown >= 2


def choose_scan_turn_result() -> None:
    readings = list(controller.get("sturn_readings", []))
    if not readings:
        commit_to_open_heading(current_pose()[2] + (30.0 if controller.get("hand", "right") == "left" else -30.0),
                               "scan-turn empty: drive out with shallow escape")
        return
    x, y, cur_h = current_pose()
    # keep the scan directions with enough room to move into
    open_dirs = [r for r in readings if r[1] >= SCAN_TURN_OPEN_CM]
    unknown_open = [r for r in open_dirs if r[2]]
    if SCAN_TURN_PREFER_UNKNOWN and unknown_open:
        chosen = max(unknown_open, key=lambda r: r[1])
        reason = "scan-turn -> opening into UNKNOWN {:.0f}cm @ {:.0f}deg".format(chosen[1], chosen[0])
    elif open_dirs:
        chosen = max(open_dirs, key=lambda r: r[1])
        reason = "scan-turn -> most open {:.0f}cm @ {:.0f}deg".format(chosen[1], chosen[0])
    else:
        chosen = max(readings, key=lambda r: r[1])
        state["deadends"] = int(state.get("deadends", 0)) + 1
        reason = "scan-turn -> best available {:.0f}cm @ {:.0f}deg (dead-end)".format(chosen[1], chosen[0])

    err = signed_angle_error(chosen[0], cur_h)
    controller["move_commit_until"] = time.time() + POST_SCANTURN_COMMIT_S
    controller["drive_commit_dist"] = float(state.get("distance_cm_total", 0.0)) + POST_SCANTURN_MIN_CM
    controller["scanturn_cooldown_until"] = time.time() + SCANTURN_COOLDOWN_S
    if abs(err) <= 35.0 or chosen[1] >= STRONG_OPEN_CAPTURE_CM:
        commit_to_open_heading(chosen[0], reason, POST_SCANTURN_MIN_CM, POST_SCANTURN_COMMIT_S)
    else:
        turn_to(chosen[0], after=str(controller.get("sturn_after", "MOVE")), reason=reason + " -> short align")

# MAIN EXPLORATION LOOPPPPP
def controller_tick(start_scan_default: bool) -> None:
    if not state.get("connected"):
        return
    if bool(state.get("paused")):
        stop_robot("paused")
        return
    if bool(state.get("exit_found")):
        stop_robot("exit found")
        return

    mode = str(controller.get("mode", "WAIT_FIRST"))
    distance = float(state.get("dist", MAXIMUM_SENSOR_RANGE))
    unused_value, unused_value, h = current_pose()
    hand = str(controller.get("hand", "right"))
    now = time.time()
    update_vicinity_history(now, mode)
    if mode == "MOVE" and now - float(controller.get("last_kf_check_t", 0.0)) >= 0.55:
        controller["last_kf_check_t"] = now

    if mode == "WAIT_FIRST":
        if not first_packet_event.is_set():
            stop_robot("waiting for first telemetry")
            return
        if start_scan_default:
            begin_start_scan("startup stepped 360 observation scan")
        else:
            controller["move_commit_until"] = time.time() + 4.0
            set_mode("MOVE", "drive-first start: map while moving")
        return

    if mode == "MAP_SNAPSHOT":
        stop_robot("active-mapping snapshot: hold still")
        target_h = float(controller.get("mapping_snapshot_heading", h))
        if now < float(controller.get("mapping_snapshot_settle_t", 0.0)):
            return
        scan_dist, sample_count = settled_scan_median(
            int(controller.get("mapping_snapshot_min_packet", -1)), target_h,
            MAPPING_SNAPSHOT_HEADING_TOL_DEG,
        )
        deadline = float(controller.get("mapping_snapshot_deadline", now))
        if sample_count < MAPPING_SNAPSHOT_MIN_PACKETS and now < deadline:
            return
        if scan_dist is None:
            scan_dist = distance
            state["mapping_snapshot_timeouts"] = int(state.get("mapping_snapshot_timeouts", 0)) + 1
        commit_settled_scan_observation(float(scan_dist), h, "MAP_SNAPSHOT")
        travel_now = float(state.get("distance_cm_total", 0.0))
        controller["last_mapping_snapshot_t"] = now
        controller["last_mapping_snapshot_travel"] = travel_now
        controller["next_mapping_snapshot_travel"] = travel_now + MAPPING_SNAPSHOT_DISTANCE_CM
        state["mapping_snapshots"] = int(state.get("mapping_snapshots", 0)) + 1
        set_mode(str(controller.get("mapping_snapshot_after", "MOVE")), "active-mapping snapshot complete")
        return

    if mode in ("START_SCAN", "FULL_SCAN"):
        packet_id = int(state.get("packets", 0))
        if packet_id != int(controller.get("full_scan_hard_last_packet", -1)):
            controller["full_scan_hard_last_packet"] = packet_id
            if distance <= FULL_SCAN_HARD_ABORT_CM:
                controller["full_scan_hard_hits"] = int(controller.get("full_scan_hard_hits", 0)) + 1
            else:
                controller["full_scan_hard_hits"] = 0
        if int(controller.get("full_scan_hard_hits", 0)) >= FULL_SCAN_HARD_ABORT_HITS:
            abort_full_scan(
                "repeated hard near-contact during scan; sonar {:.1f} cm".format(distance),
                reverse_after=True,)
            return

        done = float(controller.get("full_scan_done", 0.0))
        target = float(controller.get("full_scan_target", h))
        err = signed_angle_error(target, h)
        prev_err = controller.get("full_scan_prev_err")
        crossed_target = (
            prev_err is not None
            and ((float(prev_err) < 0.0 <= err) or (float(prev_err) > 0.0 >= err))
            and abs(err) <= FULL_SCAN_OVERSHOOT_ACCEPT_DEG
        )
        age = now - float(controller.get("full_scan_t0", now))
        target_age = now - float(controller.get("full_scan_target_started_t", now))
        if age >= FULL_SCAN_TIMEOUT:
            finish_full_scan("full scan timeout")
            return

        if (float(controller.get("full_scan_settle_t", 0.0)) == 0.0 and
                target_age >= FULL_SCAN_TARGET_TIMEOUT_S):
            state["full_scan_skipped_targets"] = int(state.get("full_scan_skipped_targets", 0)) + 1
            stop_robot("360 scan: skip timed-out heading")
            done += FULL_SCAN_STEP
            controller["full_scan_done"] = done
            controller["full_scan_prev_err"] = None
            controller["full_scan_settle_t"] = 0.0
            if done >= FULL_SCAN_TOTAL - FULL_SCAN_STEP * 0.25:
                finish_full_scan("360 scan complete with skipped heading")
                return
            controller["full_scan_target"] = normalise_angle(h - FULL_SCAN_STEP)
            controller["full_scan_target_started_t"] = now
            return

        reached_target = abs(err) <= FULL_SCAN_TARGET_TOL_DEG or crossed_target
        if reached_target:
            if float(controller.get("full_scan_settle_t", 0.0)) == 0.0:
                stop_robot("360 scan settle")
                clear_settled_scan_samples()
                controller["full_scan_settle_packet"] = int(state.get("packets", 0))
                controller["full_scan_settle_t"] = now + FULL_SCAN_SETTLE
                controller["full_scan_settle_deadline"] = now + SCAN_SETTLE_MAX_WAIT_S
                controller["full_scan_prev_err"] = 0.0
                return
            if now < float(controller.get("full_scan_settle_t", 0.0)):
                stop_robot("360 scan sampling")
                return
            scan_dist, sample_count = settled_scan_median(int(controller.get("full_scan_settle_packet", -1)), target)
            if sample_count < SCAN_SETTLED_MIN_PACKETS and now < float(controller.get("full_scan_settle_deadline", now)):
                stop_robot("360 scan waiting for fresh packets")
                return
            if scan_dist is None:
                scan_dist = distance
            commit_settled_scan_observation(scan_dist, h, mode)
            faces_unknown = heading_faces_unknown(*current_pose()[:2], h, look_cells=8)
            controller.setdefault("full_scan_readings", []).append((normalise_angle(h), scan_dist, faces_unknown))
            controller["full_scan_settle_t"] = 0.0
            controller["full_scan_prev_err"] = None
            done += FULL_SCAN_STEP
            controller["full_scan_done"] = done
            if done >= FULL_SCAN_TOTAL - FULL_SCAN_STEP * 0.25:
                finish_full_scan("360 scan complete")
                return
            controller["full_scan_target"] = normalise_angle(h - FULL_SCAN_STEP)
            controller["full_scan_target_started_t"] = now
            return

        controller["full_scan_settle_t"] = 0.0
        controller["full_scan_prev_err"] = float(err)
        set_command(f"R:{SCAN_SPEED}", "stepped 360 observation scan") # keep rotating towards the next scan angle
        controller["intent_h"] = target
        return

    if mode == "REVERSE":
        if now < float(controller.get("hold_until", 0.0)):
            set_command(f"B:{REVERSE_SPEED}", str(state.get("behaviour", "reverse")))
            controller["intent_h"] = h
            return
        stop_robot("reverse complete")
        after = str(controller.get("after_hold", "MOVE"))
        if after == "TURN_AWAY":
            controller["sturn_after"] = "MOVE"
            begin_scan_turn(1.0 if hand == "right" else -1.0, "contact release: scan for opening")
        elif after == "FULL_SCAN_PREP":
            reason = str(controller.get("pending_full_scan_reason", "360 scan after preflight"))
            kind = str(controller.get("pending_full_scan_kind", "startup"))
            request_full_scan(
                reason,
                retry_after_preflight=True,
                trigger_kind=kind,
            )
        elif after == "SIDE_ESCAPE":
            sign = float(controller.get("escape_sign", 1.0))
            controller["escape_sign"] = -sign
            controller["sturn_after"] = "MOVE"
            begin_scan_turn(sign, "wheel/side snag: alternate escape direction")
        else:
            set_mode("MOVE", "resume after reverse")
        return

    if mode == "TURN":
        target = float(controller.get("target_h", h))
        err = signed_angle_error(target, h)
        elapsed = now - float(controller.get("turn_t0", now))



        if elapsed > 0.35 and distance >= OPEN_CAPTURE_CM: # if a good opening appears while turning, just take it
            note_rotation(abs(signed_angle_error(target, h)))
            commit_to_open_heading(h, "turn saw open corridor {:.1f}cm; cancel remaining spin".format(distance))
            return

        if abs(err) <= TURN_DEADBAND or elapsed >= TURN_TIMEOUT: # either reached the target angle or spent too long turning
            if elapsed >= TURN_TIMEOUT and abs(err) > TURN_DEADBAND:
                state["turn_timeouts"] = int(state.get("turn_timeouts", 0)) + 1
            stop_robot("turn accepted")
            note_rotation(abs(signed_angle_error(target, h)))
            controller["move_commit_until"] = max(
                float(controller.get("move_commit_until", 0.0)),
                time.time() + MOVE_COMMIT_S,
            )
            if (MAPPING_SNAPSHOT_AFTER_TURN and MAPPING_SNAPSHOT_ENABLED
                    and distance >= MAPPING_SNAPSHOT_MIN_CLEARANCE_CM
                    and time.time() - float(controller.get("last_mapping_snapshot_t", -999.0)) >= MAPPING_SNAPSHOT_MIN_INTERVAL_S):
                begin_mapping_snapshot("post-turn active-mapping snapshot", after="MOVE")
            else:
                set_mode("MOVE", "turn accepted: drive")
            return
# choose which way to rotate from the heading error
        if err > 0:
            set_command(f"L:{TURN_SPEED}", "turn left err {:.0f}".format(err))
        else:
            set_command(f"R:{TURN_SPEED}", "turn right err {:.0f}".format(err))
        controller["intent_h"] = target
        return

    if mode == "SCAN_TURN":

        done = float(controller.get("sturn_done", 0.0))
        target = float(controller.get("sturn_target", h))
        err = signed_angle_error(target, h)
        scan_age = now - float(controller.get("sturn_t0", now))
        prev_err_obj = controller.get("sturn_prev_err")
        prev_err = None if prev_err_obj is None else float(prev_err_obj)
        crossed_target = (
            prev_err is not None and prev_err * err < 0.0 and
            min(abs(prev_err), abs(err)) <= SCAN_TURN_TARGET_TOL_DEG * 2.0)
        if distance >= OPEN_CAPTURE_CM: # keeping track of repeated open readings during the scan
            controller["live_open_hits"] = int(controller.get("live_open_hits", 0)) + 1
            controller["live_open_h"] = h
        else:
            controller["live_open_hits"] = 0
        if distance >= STRONG_OPEN_CAPTURE_CM or int(controller.get("live_open_hits", 0)) >= OPEN_CAPTURE_MIN_HITS:
            faces_unknown_live = heading_faces_unknown(*current_pose()[:2], h)
            if faces_unknown_live or distance >= STRONG_OPEN_CAPTURE_CM or scan_age > 1.2:
                commit_to_open_heading(h, "live opening captured during scan-turn {:.1f}cm".format(distance))
                return

# dont let the scan turn keep spinning forever
        if scan_age >= SCAN_TURN_TOTAL_TIMEOUT_S:
            state["spin_escapes"] = int(state.get("spin_escapes", 0)) + 1
            state["scan_turn_timeouts"] = int(state.get("scan_turn_timeouts", 0)) + 1
            if controller.get("sturn_readings"):
                choose_scan_turn_result()
            elif distance >= USABLE_DISTANCE:
                commit_to_open_heading(
                    h,
                    "scan-turn timeout with no settled sample; current sonar {:.1f}cm".format(distance),
                    min_drive_cm=24.0,
                    commit_s=9.0,)
            else:
                sign = float(controller.get("sturn_sign", 1.0))
                escape_h = normalise_angle(h + sign * 35.0)
                arm_post_scan_drive(escape_h, "scan-turn timeout: reverse then translate")
                begin_reverse(0.38, after="MOVE", reason="scan-turn timeout: create clearance")
            return
# check if this small scan step has reached its target
        reached_step = abs(err) <= SCAN_TURN_TARGET_TOL_DEG or crossed_target
        if reached_step:
            if float(controller.get("sturn_settle_t", 0.0)) == 0.0:
                stop_robot("scan-turn: settle for fresh sonar")
                clear_settled_scan_samples()
                controller["sturn_settle_packet"] = int(state.get("packets", 0))
                controller["sturn_settle_t"] = now + SCAN_TURN_SETTLE
                controller["sturn_settle_deadline"] = now + SCAN_SETTLE_MAX_WAIT_S
                controller["sturn_prev_err"] = 0.0
                return
            if now < float(controller["sturn_settle_t"]):
                stop_robot("scan-turn: settling")
                return
            # use the newest settled sonar samples for this direction
            scan_dist, sample_count = settled_scan_median(int(controller.get("sturn_settle_packet", -1)), target)
            if sample_count < SCAN_SETTLED_MIN_PACKETS and now < float(controller.get("sturn_settle_deadline", now)):
                stop_robot("scan-turn: waiting for fresh packets")
                return
            if scan_dist is None:
                scan_dist = distance
            commit_settled_scan_observation(scan_dist, h, mode)
            faces_unknown = heading_faces_unknown(*current_pose()[:2], h)
 # save this direction and distance so it can be compared later
            controller.setdefault("sturn_readings", []).append((normalise_angle(h), scan_dist, faces_unknown))
            controller["sturn_settle_t"] = 0.0
            done += SCAN_TURN_STEP

            if scan_dist >= SCAN_TURN_OPEN_CM and (
                    faces_unknown or scan_dist >= COMFORT_DISTANCE or
                    done >= SCAN_TURN_STEP * 2):
                commit_to_open_heading(
                    h,
                    "settled scan opening {:.1f}cm unknown={}".format(scan_dist, faces_unknown),)
                return
            if done >= SCAN_TURN_MAX_DEG:
                controller["sturn_done"] = done
                choose_scan_turn_result()
                return
            controller["sturn_done"] = done
            sign = float(controller.get("sturn_sign", 1.0))
            controller["sturn_target"] = normalise_angle(h + sign * SCAN_TURN_STEP)
            controller["sturn_target_started_t"] = now
            controller["sturn_prev_err"] = None
            return

        controller["sturn_prev_err"] = float(err)
        sign = float(controller.get("sturn_sign", 1.0))
        if sign >= 0:
            set_command(f"L:{TURN_SPEED}", "scan-turn / observe walls")
        else:
            set_command(f"R:{TURN_SPEED}", "scan-turn / observe walls")
        return
    
    if mode in ("DECIDE_TURN", "DECIDE_SAMPLE"):
        set_mode("MOVE", "cancel scan-decision: drive-first fallback")
        return

    if mode == "MOVE": # check first if the robot should stop and do another full scan
        full_trigger, full_reason, full_kind = should_trigger_periodic_full_scan(now)
        if full_trigger:
            if full_kind == "periodic":
                state["periodic_full_scan_triggers"] = int(state.get("periodic_full_scan_triggers", 0)) + 1
            elif full_kind == "uncertainty":
                state["adaptive_full_scan_triggers"] = int(state.get("adaptive_full_scan_triggers", 0)) + 1
            else:
                state["vicinity_full_scan_triggers"] = int(state.get("vicinity_full_scan_triggers", 0)) + 1
            request_full_scan(full_reason, trigger_kind=full_kind)
            return

        snapshot_trigger, snapshot_reason = should_trigger_mapping_snapshot(now)
        if snapshot_trigger:
            begin_mapping_snapshot(snapshot_reason, after="MOVE")
            return

        x0, y0, h0 = current_pose()
        best_h, best_score, scored = best_exploration_heading(x0, y0, h0, hand) # compare possible headings from the robots current position
        frontier_h = frontier_heading_hint()

        follow_frontier = frontier_h is not None and distance > CONTACT_DISTANCE + 2.0
        if follow_frontier:
            best_h = frontier_h
            best_score += 10.0
        update_planned_path(best_h, 24)

        if distance <= CONTACT_DISTANCE: # too close, reverse first instead of trying to squeeze through
            state["contacts"] = int(state.get("contacts", 0)) + 1
            begin_reverse(0.42, after="TURN_AWAY", reason="contact/bumper release")
            return

        if distance >= COMFORT_DISTANCE:
            if now > float(controller.get("open_commit_until", 0.0)):
                controller["open_commit_h"] = best_h if best_score > 15.0 else h0
                controller["open_commit_until"] = now + (9.0 if distance >= FAR_OPEN_DISTANCE else 6.0)
                state["open_commits"] = int(state.get("open_commits", 0)) + 1
            commit_h = float(controller.get("open_commit_h", h0))
            err = signed_angle_error(commit_h, h0)
            update_planned_path(commit_h, 28)
            if abs(err) > 18:
                if err > 0:
                    set_command(f"W:{INNER_ARC_SPEED}:{OUTER_ARC_SPEED}", "open: curve toward new cells")
                else:
                    set_command(f"W:{OUTER_ARC_SPEED}:{INNER_ARC_SPEED}", "open: curve toward new cells")
                controller["intent_h"] = commit_h
            else:
                drive_heading_pid(commit_h, FORWARD_SPEED, "open/clear: PID drive into new cells")
            return
        if is_stuck() and now - float(controller.get("last_unstuck_t", 0.0)) > 8.0:
            controller["last_unstuck_t"] = now
            state["stuck_events"] = int(state.get("stuck_events", 0)) + 1
            
            
            frontier = find_nearest_frontier(x0, y0)
            if frontier is not None:
                state["frontier_seeks"] = int(state.get("frontier_seeks", 0)) + 1
                frontier_heading = heading_to_cell(x0, y0, frontier)
                state["side_snag_escapes"] = int(state.get("side_snag_escapes", 0)) + 1
                begin_reverse(0.42, after="SIDE_ESCAPE", reason="side/wheel snag release before frontier seek")
                controller["frontier_path"] = []
                controller["last_frontier_plan_t"] = 0.0
            else:
                controller["sturn_after"] = "MOVE"
                begin_scan_turn(1.0 if hand == "right" else -1.0, "stuck: scan for opening")
            return

        drive_committed = (now < float(controller.get("move_commit_until", 0.0)) and
                           float(state.get("distance_cm_total", 0.0)) < float(controller.get("drive_commit_dist", 0.0)))
        scanturn_ready = now >= float(controller.get("scanturn_cooldown_until", 0.0))
        if distance <= BLOCKED_DISTANCE and scanturn_ready and not drive_committed:
            state["front_blocks"] = int(state.get("front_blocks", 0)) + 1
            controller["sturn_after"] = "MOVE"
            begin_scan_turn(1.0 if hand == "right" else -1.0, "front blocked: scan for opening")
            return
        if distance <= BLOCKED_DISTANCE and (drive_committed or not scanturn_ready):

            if register_wallhug_arc_and_maybe_escape(x0, y0):
                return
            arc_left = f"W:{INNER_ARC_SPEED}:{OUTER_ARC_SPEED}"   
            arc_right = f"W:{OUTER_ARC_SPEED}:{INNER_ARC_SPEED}"  
            if follow_frontier:
                frontier_error = signed_angle_error(best_h, h0)  
                if frontier_error > 8.0:
                    set_command(arc_left, "committed: curve past wall toward frontier (left)")
                elif frontier_error < -8.0:
                    set_command(arc_right, "committed: curve past wall toward frontier (right)")
                else:

                
                    set_command(arc_left if hand == "left" else arc_right,
                                "committed: curve past wall (frontier ahead, hand bias)")
            else:
                set_command(arc_left if hand == "left" else arc_right,
                            "committed: curve past wall")
            return

        if hand == "right":
            arc_toward = f"W:{OUTER_ARC_SPEED}:{INNER_ARC_SPEED}"
            arc_away = f"W:{INNER_ARC_SPEED}:{OUTER_ARC_SPEED}"
            intent_away = h + 22.0
        else:
            arc_toward = f"W:{INNER_ARC_SPEED}:{OUTER_ARC_SPEED}"
            arc_away = f"W:{OUTER_ARC_SPEED}:{INNER_ARC_SPEED}"
            intent_away = h - 22.0

        committed = now < float(controller.get("move_commit_until", 0.0))
        if distance >= USABLE_DISTANCE or committed:
            err = signed_angle_error(best_h, h)
            if abs(err) > 22 and best_score > heading_exploration_score(x0, y0, h0, max_cells=18) + 8.0:
                if err > 0:
                    set_command(f"W:{INNER_ARC_SPEED}:{OUTER_ARC_SPEED}", "usable: curve toward unknown/free cells")
                else:
                    set_command(f"W:{OUTER_ARC_SPEED}:{INNER_ARC_SPEED}", "usable: curve toward unknown/free cells")
                controller["intent_h"] = best_h
                update_planned_path(best_h, 22)
            else:
                drive_heading_pid(h, SLOW_SPEED, "usable: PID keep moving")
                update_planned_path(h, 20)
        else:
            set_command(arc_away, "narrow: curve away without stopping")
            controller["intent_h"] = normalise_angle(intent_away)
            update_planned_path(float(controller["intent_h"]), 18)
        return

    set_mode("MOVE", "fallback to drive-first move")
