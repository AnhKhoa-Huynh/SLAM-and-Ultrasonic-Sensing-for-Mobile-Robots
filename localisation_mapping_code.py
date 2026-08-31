# MAPPING + LOCALISATIONNNNN
# CHECKING MAP BOUNDARIESSS
def in_grid(grid_x, grid_y):
    return 0 <= grid_x < GRID_WIDTH and 0 <= grid_y < GRID_HEIGHT


def tentative_hit_count(grid_x, grid_y): # count the weaker obstacle readings around this cell
    if not in_grid(grid_x, grid_y):
        return 0
    return int(arc_obstacle_hits[grid_y, grid_x] + turn_obstacle_hits[grid_y, grid_x] + support_obstacle_hits[grid_y, grid_x])


def cell_from_logodds(grid_x, grid_y): # decide what type of map cell this should be
    log_value = float(log_odds_grid[grid_y, grid_x])
    stable_hits = int(obstacle_hits[grid_y, grid_x])
    arc_hits = int(arc_obstacle_hits[grid_y, grid_x])
# enough repeated evidence to treat this as a confirmed obstacle
    if (stable_hits >= CONFIRM_HITS or arc_hits >= ARC_CONFIRM_HITS) and log_value >= LOG_ODDS_FIRST_THRESH:
        return OBSTACLE_CONFIRMED
    if log_value >= LOG_ODDS_FIRST_THRESH or stable_hits > 0 or tentative_hit_count(grid_x, grid_y) > 0:
        return OBSTACLE_FIRST
    if log_value <= LOG_ODDS_FREE_THRESH or free_hits[grid_y, grid_x] > 0:
        return FREE
    return UNKNOWN


def apply_logodds(grid_x, grid_y, delta): # update the probability of this map cell
    if not in_grid(grid_x, grid_y):
        return
# keep the probability evidence inside its allowed range
    log_odds_grid[grid_y, grid_x] = clamp(log_odds_grid[grid_y, grid_x] + delta, LOG_ODDS_MIN, LOG_ODDS_MAX)
    occupancy_grid[grid_y, grid_x] = cell_from_logodds(grid_x, grid_y)


def range_occupied_weight(distance): def range_occupied_weight(
    if distance <= 70:
        return 1.0
    if distance <= 110:
        return 0.72
    if distance <= 150:
        return 0.48
    if distance <= MAXIMUM_HIT_RANGE:
        return 0.28
    return 0.0

# marking cells that the sonar ray passed through as free
def mark_free(grid_x, grid_y, amount=1, weight=1.0, protect_tentative=True):
    if not in_grid(grid_x, grid_y):
        return

    free_hits[grid_y, grid_x] = min(65535, free_hits[grid_y, grid_x] + max(1, amount))
    delta = LOG_ODDS_FREE * max(1, amount) * weight
# dont erase a possible wall too quickly
    if protect_tentative and obstacle_hits[grid_y, grid_x] == 0 and tentative_hit_count(grid_x, grid_y) > 0:
        delta *= TENTATIVE_FREE_PROTECTION

    apply_logodds(grid_x, grid_y, delta)

# keep the space underneath the robot clear on the map
def mark_body_free(x, y):
    center_x, center_y = int(round(x)), int(round(y))
    radius = ROBOT_RADIUS_CELLS

    for offset_y in range(-radius, radius + 1):
        for offset_x in range(-radius, radius + 1):
            if offset_x * offset_x + offset_y * offset_y <= radius * radius:
                mark_free(center_x + offset_x, center_y + offset_y, protect_tentative=False)

# ADDING OBSTACLE EVIDENCEEEE
def add_obstacle_evidence(grid_x, grid_y, source, delta):
    if not in_grid(grid_x, grid_y):
        return

    x, y = current_pose()[:2]
    # ignore obstacle hits that appear inside the robot itself
    if (grid_x - x) ** 2 + (grid_y - y) ** 2 <= (ROBOT_RADIUS_CELLS + 0.5) ** 2:
        return

    if source == "stable": # different movement types give different kinds of obstacle evidence
        obstacle_hits[grid_y, grid_x] = min(65535, obstacle_hits[grid_y, grid_x] + 1)
    elif source == "arc":
        arc_obstacle_hits[grid_y, grid_x] = min(65535, arc_obstacle_hits[grid_y, grid_x] + 1)
    elif source == "turn":
        turn_obstacle_hits[grid_y, grid_x] = min(65535, turn_obstacle_hits[grid_y, grid_x] + 1)
    elif source == "support":
        support_obstacle_hits[grid_y, grid_x] = min(65535, support_obstacle_hits[grid_y, grid_x] + 1)

    apply_logodds(grid_x, grid_y, delta)

    if obstacle_hits[grid_y, grid_x] < CONFIRM_HITS and arc_obstacle_hits[grid_y, grid_x] < ARC_CONFIRM_HITS:
        if log_odds_grid[grid_y, grid_x] >= LOG_ODDS_CONFIRMED_THRESH:
            log_odds_grid[grid_y, grid_x] = LOG_ODDS_CONFIRMED_THRESH - 0.05
            occupancy_grid[grid_y, grid_x] = OBSTACLE_FIRST
    else:
        occupancy_grid[grid_y, grid_x] = OBSTACLE_CONFIRMED


def mark_obstacle(grid_x, grid_y, confirm_allowed=True):
    source = "stable" if confirm_allowed else "turn"
    delta = LOG_ODDS_OCC if confirm_allowed else LOG_ODDS_TURN_OCC
    add_obstacle_evidence(grid_x, grid_y, source, delta)


def total_endpoint_hits(grid_x, grid_y):
    if not in_grid(grid_x, grid_y):
        return 0
    return int(obstacle_hits[grid_y, grid_x]) + tentative_hit_count(grid_x, grid_y)


def concentrate_radial_endpoint(hit_x, hit_y, dx, dy): # move a sonar endpoint onto an existing nearby wall if possible
    best = None

    for offset_y in range(-ENDPOINT_RADIAL_SNAP_CELLS, ENDPOINT_RADIAL_SNAP_CELLS + 1):
        for offset_x in range(-ENDPOINT_RADIAL_SNAP_CELLS, ENDPOINT_RADIAL_SNAP_CELLS + 1):
            if offset_x == 0 and offset_y == 0:
                continue

            grid_x, grid_y = hit_x + offset_x, hit_y + offset_y
            if not in_grid(grid_x, grid_y):
                continue

            hits = total_endpoint_hits(grid_x, grid_y)
            if hits <= 0:
                continue

            radial = abs(offset_x * dx + offset_y * dy) # check how closely this old hit lines up with the new sonar direction
            lateral = abs(-offset_x * dy + offset_y * dx)

            if radial <= ENDPOINT_RADIAL_SNAP_CELLS + 0.25 and lateral <= 0.55:
                rank = (occupancy_grid[grid_y, grid_x] == OBSTACLE_CONFIRMED, hits, -radial)
                if best is None or rank > best[0]:
                    best = (rank, grid_x, grid_y)

    return (best[1], best[2]) if best else (hit_x, hit_y)

# adds some wall support beside a strong sonar hit
def add_lateral_wall_support(hit_x, hit_y, dx, dy, range_weight):
    if range_weight <= 0:
        return
    perpendicular_x, perpendicular_y = -dy, dx # direction running sideways from the sonar beam
    for sign in (-1, 1):
        grid_x = int(round(hit_x + sign * perpendicular_x))
        grid_y = int(round(hit_y + sign * perpendicular_y))

        if not in_grid(grid_x, grid_y):
            continue
        if free_hits[grid_y, grid_x] >= 4 and log_odds_grid[grid_y, grid_x] < -0.8:
            continue

        add_obstacle_evidence(grid_x, grid_y, "support", LOG_ODDS_SUPPORT_OCC * range_weight)


def interpolated_grid_line(start, end): # fill the cells between two nearby wall points
    start_x, start_y = start
    end_x, end_y = end
    span = max(abs(end_x - start_x), abs(end_y - start_y))

    if span <= 1:
        return []

    cells = []
    last = None

    for i in range(1, span):
        amount = i / span
        cell = (int(round(start_x + (end_x - start_x) * amount)), int(round(start_y + (end_y - start_y) * amount)))

        if cell != last and cell not in (start, end):
            cells.append(cell)
            last = cell
    return cells

# look for nearby wall points that line up with this one
def continuity_candidate_cells(hit_x, hit_y, heading, dx, dy, heading_grid):
    if not CONTINUITY_SUPPORT_ENABLED:
        return []
# only search a small area around the new wall point
    radius = int(math.ceil(CONTINUITY_MAX_GAP_CELLS))
    best = {-1: None, 1: None}

    for y in range(max(0, hit_y - radius), min(GRID_HEIGHT, hit_y + radius + 1)):
        for x in range(max(0, hit_x - radius), min(GRID_WIDTH, hit_x + radius + 1)):
            if (x, y) == (hit_x, hit_y):
                continue

            old_heading = heading_grid[y, x] # compare against the heading that created this older wall point
            gap = math.hypot(x - hit_x, y - hit_y)

            if not math.isfinite(old_heading) or not CONTINUITY_MIN_GAP_CELLS <= gap <= CONTINUITY_MAX_GAP_CELLS:
                continue
            if abs(signed_angle_error(old_heading, heading)) > CONTINUITY_HEADING_TOL_DEG:
                continue

            offset_x, offset_y = x - hit_x, y - hit_y
            normal = abs(offset_x * dx + offset_y * dy)
            tangent = -offset_x * dy + offset_y * dx

            if normal > CONTINUITY_NORMAL_DEVIATION_CELLS or abs(tangent) < 1.25:
                continue

            side = 1 if tangent >= 0 else -1 # keep one candidate from each side of the wall
            candidate = (normal, gap, x, y)

            if best[side] is None or candidate[:2] < best[side][:2]:
                best[side] = candidate

    return [(item[2], item[3]) for item in best.values() if item]
# CONNECTING WALL PIECESSSS
def add_aligned_wall_continuity(hit_x, hit_y, heading, dx, dy, range_weight):
    if not CONTINUITY_SUPPORT_ENABLED or range_weight <= 0:
        if in_grid(hit_x, hit_y):
            stable_endpoint_heading[hit_y, hit_x] = heading
        return

    candidates = continuity_candidate_cells(hit_x, hit_y, heading, dx, dy, stable_endpoint_heading)

    for grid_x, grid_y in candidates:
        cells = interpolated_grid_line((hit_x, hit_y), (grid_x, grid_y))
        if not cells:
            continue
            # dont join two walls through cells already strongly marked as free
        veto = any(in_grid(x, y) and free_hits[y, x] >= CONTINUITY_FREE_VETO_HITS and log_odds_grid[y, x] <= CONTINUITY_FREE_VETO_LOG_ODDS for x, y in cells)
        if veto:
            continue

        added = 0
        for x, y in cells:
            if in_grid(x, y):
                add_obstacle_evidence(x, y, "support", LOG_ODDS_SUPPORT_OCC * CONTINUITY_SUPPORT_WEIGHT * range_weight)
                added += 1

        if added:
            state["continuity_support_links"] = state.get("continuity_support_links", 0) + 1
            state["continuity_support_cells"] = state.get("continuity_support_cells", 0) + added

    if in_grid(hit_x, hit_y):
        stable_endpoint_heading[hit_y, hit_x] = heading

# cast one sonar reading into the occupancy map
def cast_single_sonar_ray(x, y, heading, distance, motion, weight=1.0, allow_obstacle=True):
    distance = normalise_distance(distance)
    hit_real = MINIMUM_SENSOR_RANGE <= distance <= MAXIMUM_HIT_RANGE # decide whether the sonar actually saw something

    angle = math.radians(heading) # convert heading into a direction for the sonar ray
    dx, dy = math.cos(angle), -math.sin(angle)

    scan_context = state.get("mode", "") in ("SCAN_TURN", "START_SCAN", "FULL_SCAN", "TURN")
    clear_front = distance > CONTACT_DISTANCE + 2
    straight = motion == "F" and clear_front
    arc = motion == "A" and clear_front
    stopped = motion == "S" and clear_front
    rotating = motion == "T" and scan_context

    if straight or arc or stopped:
        free_weight = STRAIGHT_FREE_WEIGHT if straight else ARC_FREE_WEIGHT if arc else STOP_FREE_WEIGHT
        ray_distance = distance

        if not hit_real:
            ray_distance = min(ray_distance, NO_ECHO_FREE_LIMIT)
            free_weight *= NO_ECHO_FREE_WEIGHT
            state["capped_no_echo_rays"] = state.get("capped_no_echo_rays", 0) + 1

        free_until = max(0, ray_distance / CELL_SIZE - (0.9 if hit_real else 0))
        step = 0.75

        while step < free_until: # walk along the beam and clear each cell before the obstacle
            grid_x = int(round(x + dx * step))
            grid_y = int(round(y + dy * step))
            mark_free(grid_x, grid_y, 1, free_weight * weight)
            step += 0.65

    if not allow_obstacle or not hit_real or distance < CONTACT_DISTANCE + 0.5:
        return

    range_weight = range_occupied_weight(distance)
    if range_weight <= 0:
        return

    if straight or stopped:
        source = "stable"
        delta = LOG_ODDS_OCC * range_weight * weight
    elif arc:
        source = "arc"
        delta = LOG_ODDS_ARC_OCC * ARC_OCCUPIED_WEIGHT * range_weight * weight
    elif rotating and distance <= TURN_OBSERVE_MAX_CM:
        source = "turn"
        delta = LOG_ODDS_TURN_OCC * TURN_OCCUPIED_WEIGHT * range_weight * weight
    else:
        return

    distance_cells = distance / CELL_SIZE
    hit_x = int(round(x + dx * distance_cells)) # work out where the sonar hit lands on the grid
    hit_y = int(round(y + dy * distance_cells))
    hit_x, hit_y = concentrate_radial_endpoint(hit_x, hit_y, dx, dy)

    add_obstacle_evidence(hit_x, hit_y, source, delta)

    if source == "stable" and distance <= 150:
        add_lateral_wall_support(hit_x, hit_y, dx, dy, range_weight)
        add_aligned_wall_continuity(hit_x, hit_y, heading, dx, dy, range_weight)
    elif source == "turn":
        state["turn_observations"] = state.get("turn_observations", 0) + 1
    elif source == "arc":
        state["arc_map_observations"] = state.get("arc_map_observations", 0) + 1


def sensor_update_v8(x, y, heading, distance, motion): # update the map using the centre and edges of the sonar cone
    mark_body_free(x, y)
    cast_single_sonar_ray(x, y, heading, distance, motion)
    # weaker side rays help represent the width of the sonar cone
    cast_single_sonar_ray(x, y, heading - SONAR_HALF_CONE_DEG, distance, motion, 0.18, False)
    cast_single_sonar_ray(x, y, heading + SONAR_HALF_CONE_DEG, distance, motion, 0.18, False)
    state["map_updates"] = state.get("map_updates", 0) + 1


def clear_settled_scan_samples():
    with settled_scan_lock:
        settled_scan_samples.clear()


def record_settled_scan_sample(packet_id, heading, distance): # save a clean sonar reading while the robot is stopped
    with settled_scan_lock:
        settled_scan_samples.append((packet_id, heading, distance))

# use several settled readings instead of trusting only one
def settled_scan_median(min_packet, target_heading, heading_tol_deg=SCAN_SETTLED_HEADING_TOL_DEG):
    with settled_scan_lock:
        values = [distance for packet, heading, distance in settled_scan_samples if packet > min_packet and abs(signed_angle_error(target_heading, heading)) <= heading_tol_deg]
    if not values:
        return None, 0
    return float(np.median(values)), len(values) # median is less affected by one strange sonar reading

# apply the settled sonar reading to the map
def commit_settled_scan_observation(distance, heading, mode):
    x, y = current_pose()[:2]
    with map_lock:
        sensor_update_v8(x, y, heading, distance, "S")

    state["settled_scan_commits"] = state.get("settled_scan_commits", 0) + 1
def unwrap_encoder_delta(value): # make sure encoder jumps wrap around properly
    candidates = [value, value + 65536, value - 65536, value + 4294967296, value - 4294967296]
    return min(candidates, key=abs)


def update_encoder_sign(raw_delta, expected_direction, side):
    if abs(raw_delta) < 1 or not expected_direction:
        return

    key = side + "_sign_score"
    direction = 1 if raw_delta > 0 else -1
    score = clamp(wheel_calibration.get(key, 0) + expected_direction * direction, -12, 12)

    wheel_calibration[key] = score
    wheel_calibration[side + "_sign"] = 1 if score >= 0 else -1


def solve_wheel_scale_calibration():
    if not wheel_calibration.get("auto_enabled", True) or len(wheel_calibration_samples) < WHEEL_CALIBRATION_MIN_SAMPLES:
        return
    rows = []
    targets = []

    for left_ticks, right_ticks, yaw_delta in wheel_calibration_samples:
        rows.append([-BASE_CM_PER_TICK * left_ticks, BASE_CM_PER_TICK * right_ticks])
        targets.append(wheel_calibration.get("track_width_cm", TRACK_WIDTH_CM) * yaw_delta)

    prior = WHEEL_CALIBRATION_PRIOR_WEIGHT
    left_prior = wheel_calibration.get("left_scale", 1.0)
    right_prior = wheel_calibration.get("right_scale", 1.0)
    rows.extend([[prior, 0], [0, prior]])
    targets.extend([prior * left_prior, prior * right_prior])

    matrix = np.asarray(rows)
    target_values = np.asarray(targets)

    try:
        solution = np.linalg.lstsq(matrix, target_values, rcond=None)[0]
        residuals = np.abs(matrix[:-2] @ solution - target_values[:-2])

        if len(residuals) >= WHEEL_CALIBRATION_MIN_SAMPLES:
            median = float(np.median(residuals))
            keep = residuals <= max(0.35, median * 3.5)

            if np.count_nonzero(keep) >= WHEEL_CALIBRATION_MIN_SAMPLES // 2:
                filtered_matrix = np.vstack([matrix[:-2][keep], matrix[-2:]])
                filtered_targets = np.concatenate([target_values[:-2][keep], target_values[-2:]])
                solution = np.linalg.lstsq(filtered_matrix, filtered_targets, rcond=None)[0]
                residuals = np.abs(filtered_matrix[:-2] @ solution - filtered_targets[:-2])

        target_left = clamp(solution[0], WHEEL_SCALE_MIN, WHEEL_SCALE_MAX)
        target_right = clamp(solution[1], WHEEL_SCALE_MIN, WHEEL_SCALE_MAX)

        wheel_calibration["left_scale"] = 0.92 * left_prior + 0.08 * target_left
        wheel_calibration["right_scale"] = 0.92 * right_prior + 0.08 * target_right
        wheel_calibration["last_solution_residual"] = float(np.mean(residuals)) if len(residuals) else 0.0
        state["wheel_calibration_updates"] = state.get("wheel_calibration_updates", 0) + 1
    except Exception:
        return

# turn encoder changes into left and right wheel travel
def encoder_wheel_deltas_cm(left_encoder, right_encoder, motion, dt, yaw_delta_deg):
    global last_left_encoder, last_right_encoder

    if left_encoder is None or right_encoder is None:
        return None

    if last_left_encoder is None or last_right_encoder is None:
        last_left_encoder, last_right_encoder = left_encoder, right_encoder
        return None
# get how much each encoder changed since the last packet
    left_raw = unwrap_encoder_delta(left_encoder - last_left_encoder)
    right_raw = unwrap_encoder_delta(right_encoder - last_right_encoder)
    last_left_encoder, last_right_encoder = left_encoder, right_encoder

    expected = 1 if motion in ("F", "A") else -1 if motion == "B" else 0
    update_encoder_sign(left_raw, expected, "left")
    update_encoder_sign(right_raw, expected, "right")

    left_ticks = left_raw * wheel_calibration.get("left_sign", 1.0)
    right_ticks = right_raw * wheel_calibration.get("right_sign", -1.0)

    distance_left = left_ticks * BASE_CM_PER_TICK * wheel_calibration.get("left_scale", 1.0)
    distance_right = right_ticks * BASE_CM_PER_TICK * wheel_calibration.get("right_scale", 1.0)

    left_amount, right_amount = abs(left_raw), abs(right_raw)
    imbalance = abs(left_amount - right_amount) / max(1, left_amount + right_amount)

    encoder_diag.update(dl=left_raw, dr=right_raw, d_left_cm=distance_left, d_right_cm=distance_right, imbalance=imbalance, yaw_delta_deg=yaw_delta_deg, t=time.time())

    if expected != 0 and abs(left_raw) + abs(right_raw) >= 3 and abs(yaw_delta_deg) <= 45 and dt > 0 and state.get("dist", MAXIMUM_SENSOR_RANGE) > CONTACT_DISTANCE + 2:
        wheel_calibration_samples.append((left_ticks, right_ticks, math.radians(yaw_delta_deg)))

        if len(wheel_calibration_samples) % 8 == 0:
            solve_wheel_scale_calibration()

    return distance_left, distance_right

# MAIN LOCALISATION + MAP UPDATEEEE
def update_pose_and_map(raw_distance, raw_yaw, left_encoder, right_encoder, motion):
    global robot_x, robot_y, robot_heading, raw_robot_x, raw_robot_y, raw_robot_heading, online_heading_bias_deg
    global last_update_t, previous_yaw, previous_yaw_time, exit_cell, last_left_encoder, last_right_encoder

    now = time.time()
    distance = normalise_distance(raw_distance)
    range_history.append(distance)

    state["raw_dist"] = raw_distance
    state["dist"] = distance
    state["yaw_raw"] = raw_yaw
    state["motion"] = motion

    if state.get("result_ready") or state.get("result_computing"):
        return

    if state["yaw_zero"] is None:
        state["yaw_zero"] = raw_yaw

    raw_heading = normalise_angle(raw_yaw - state["yaw_zero"])

    if last_update_t is None:
        last_update_t = now
        previous_yaw = raw_heading
        previous_yaw_time = now

        if left_encoder is not None and right_encoder is not None:
            last_left_encoder, last_right_encoder = left_encoder, right_encoder

        with pose_lock:
            raw_robot_heading = raw_heading
            robot_heading = normalise_angle(raw_heading + online_heading_bias_deg)

        with map_lock:
            sensor_update_v8(robot_x, robot_y, robot_heading, distance, motion)

        return

    dt = clamp(now - last_update_t, 0, 0.75)
    last_update_t = now

    previous_heading = raw_heading if previous_yaw is None else previous_yaw
    yaw_delta = signed_angle_error(raw_heading, previous_heading)
    previous_yaw = raw_heading
    previous_yaw_time = now

    wheel_delta = encoder_wheel_deltas_cm(left_encoder, right_encoder, motion, dt, yaw_delta)
    average_speed = parse_average_command_speed()
    step = 0.0
    distance_left = distance_right = 0.0

    if wheel_delta is not None:
        distance_left, distance_right = wheel_delta

    max_step = clamp(38 * dt, 2, 24)

    if motion in ("F", "A", "B"):
        if motion in ("F", "A") and distance <= CONTACT_DISTANCE:
            state["odom_mode"] = "front contact freeze"

        elif wheel_delta is not None:
            step = clamp((distance_left + distance_right) * 0.5, -max_step, max_step)
            expected = 1 if motion in ("F", "A") else -1

            if step * expected < -0.08:
                step = 0.0
                state["odom_mode"] = "encoder sign transient rejected"
            else:
                state["encoder_distance_cm"] = state.get("encoder_distance_cm", 0.0) + abs(step)
                state["odom_mode"] = "signed differential encoders"

        else:
            step = clamp(average_speed * FALLBACK_CM_PER_SPEED_S * dt, -max_step, max_step)
            state["fallback_distance_cm"] = state.get("fallback_distance_cm", 0.0) + abs(step)
            state["odom_mode"] = "speed fallback"

    elif motion == "T":
        same_direction = wheel_delta is not None and distance_left * distance_right > 0
        translation_like = abs(yaw_delta) <= PIVOT_TRANSLATION_YAW_MAX_DEG
        candidate = (distance_left + distance_right) * 0.5 if wheel_delta is not None else 0.0

        if same_direction and translation_like and abs(candidate) <= PIVOT_TRANSLATION_MAX_CM:
            step = candidate
            state["pivot_translation_accepts"] = state.get("pivot_translation_accepts", 0) + 1
            state["odom_mode"] = "pivot packet: encoder+IMU translation"
        else:
            if wheel_delta is not None and abs(distance_left) + abs(distance_right) > 0.10:
                state["pivot_translation_rejections"] = state.get("pivot_translation_rejections", 0) + 1
            state["odom_mode"] = "pivot translation suppressed"

    else:
        state["odom_mode"] = "stop"

    raw_mid_heading = normalise_angle(previous_heading + 0.5 * yaw_delta)
    corrected_mid_heading = normalise_angle(raw_mid_heading + online_heading_bias_deg)
    corrected_heading = normalise_angle(raw_heading + online_heading_bias_deg)

    with pose_lock:
        raw_robot_heading = raw_heading
        raw_robot_x += math.cos(math.radians(raw_mid_heading)) * (step / CELL_SIZE)
        raw_robot_y -= math.sin(math.radians(raw_mid_heading)) * (step / CELL_SIZE)
        raw_robot_x = clamp(raw_robot_x, 3, GRID_WIDTH - 4)
        raw_robot_y = clamp(raw_robot_y, 3, GRID_HEIGHT - 4)

        if not raw_trail or abs(raw_robot_x - raw_trail[-1][0]) + abs(raw_robot_y - raw_trail[-1][1]) > 0.08:
            raw_trail.append((raw_robot_x, raw_robot_y))

        robot_heading = corrected_heading
        robot_x += math.cos(math.radians(corrected_mid_heading)) * (step / CELL_SIZE)
        robot_y -= math.sin(math.radians(corrected_mid_heading)) * (step / CELL_SIZE)
        robot_x = clamp(robot_x, 3, GRID_WIDTH - 4)
        robot_y = clamp(robot_y, 3, GRID_HEIGHT - 4)

        state["distance_cm_total"] = state.get("distance_cm_total", 0.0) + abs(step)

        if not trail or abs(robot_x - trail[-1][0]) + abs(robot_y - trail[-1][1]) > 0.08:
            trail.append((robot_x, robot_y))

        grid_x, grid_y = int(round(robot_x)), int(round(robot_y))
        if in_grid(grid_x, grid_y):
            visit_count[grid_y, grid_x] = min(65535, visit_count[grid_y, grid_x] + 2)

    mode = state.get("mode", "")
    settled_scan = motion == "S" and mode in ("SCAN_TURN", "START_SCAN", "FULL_SCAN", "TURN", "MAP_SNAPSHOT")

    if settled_scan:
        record_settled_scan_sample(state.get("packets", 0), robot_heading, distance)
    else:
        with map_lock:
            sensor_update_v8(robot_x, robot_y, robot_heading, distance, motion)

    if step:
        note_translation(abs(step))

    state["pose_conf_pct"] = pose_confidence_pct()
    state["left_wheel_scale"] = wheel_calibration.get("left_scale", 1.0)
    state["right_wheel_scale"] = wheel_calibration.get("right_scale", 1.0)
    state["left_encoder_sign"] = wheel_calibration.get("left_sign", 1.0)
    state["right_encoder_sign"] = wheel_calibration.get("right_sign", -1.0)

    progress_history.append((now, state.get("distance_cm_total", 0.0)))

    if state.get("exit_found") and exit_cell is None:
        with pose_lock:
            exit_cell = (int(round(robot_x)), int(round(robot_y)))


def telemetry_loop(): # receiving telemetry continuously from the robot
    global sock, exit_cell

    buffer = ""
    while not stop_event.is_set():
        try:
            if sock is None:
                time.sleep(0.05)
                continue

            data = sock.recv(512)
            if not data:
                state["error"] = "socket closed"
                break
            buffer += data.decode(errors="ignore")
        except socket.timeout:
            continue
        except Exception as exc:
            if not stop_event.is_set():
                state["error"] = "recv: " + str(exc)
            break

        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()

            if not line:
                continue

            if line.upper().startswith("EXIT"):
                state["exit_found"] = True

                if exit_cell is None:
                    x, y = current_pose()[:2]
                    exit_cell = (int(round(x)), int(round(y)))
                continue

            parts = line.split(",")
            if len(parts) < 7:
                state["bad_lines"] = state.get("bad_lines", 0) + 1
                continue

            try:
                distance = float(parts[0])
                yaw = float(parts[1])
                left_encoder = int(float(parts[2]))
                right_encoder = int(float(parts[3]))
                motion = parts[4].strip() or "?"
                white_score = float(parts[5])
                exit_flag = int(float(parts[6]))
                device_sequence = int(float(parts[7])) if len(parts) > 7 else -1
                device_time = int(float(parts[8])) if len(parts) > 8 else -1

            except ValueError:
                state["bad_lines"] = state.get("bad_lines", 0) + 1
                continue

            if exit_flag:
                state["exit_found"] = True

            state["white_score"] = white_score
            state["device_packet_seq"] = device_sequence
            state["device_time_ms"] = device_time
            state["packets"] = state.get("packets", 0) + 1

            packet_times.append(time.time())

            if len(packet_times) > 1:
                span = packet_times[-1] - packet_times[0]
                if span > 0:
                    state["telemetry_hz"] = (len(packet_times) - 1) / span

            first_packet_event.set() # first valid telemetry has arrived so the controller can start
            update_pose_and_map(distance, yaw, left_encoder, right_encoder, motion)

    state["connected"] = False
    connected_event.clear()

# STARTING THE ROBOT CONNECTIONNN
def connect_robot(ip, port):
    global sock

    print(f"Connectin at {ip}:{port}")
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.settimeout(5)
    connection.connect((ip, port))
    connection.settimeout(0.05)
    sock = connection
    state["connected"] = True
    connected_event.set()
    threading.Thread(target=telemetry_loop, daemon=True).start()