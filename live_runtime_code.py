# LIVE MAPPPPPP
def draw_map(screen: pygame.Surface):
    screen.fill(COL_UNKNOWN)
    screen_w, screen_h = screen.get_size()
    x, y, h = current_pose()

    with map_lock:
        log_copy = log_odds_grid.copy()
    probs = logodds_to_probability(log_copy)

    cell_px = float(CELL_PX)
    robot_screen = (screen_w // 2, screen_h // 2)
    offset_x = robot_screen[0] - x * cell_px
    offset_y = robot_screen[1] - y * cell_px

    min_c = max(0, int((-offset_x) // cell_px) - 2)
    max_c = min(GRID_WIDTH, int((screen_w - offset_x) // cell_px) + 3)
    min_r = max(0, int((-offset_y) // cell_px) - 2)
    max_r = min(GRID_HEIGHT, int((screen_h - offset_y) // cell_px) + 3)
    draw_cell = max(1, int(math.ceil(cell_px)))
    for grid_y in range(min_r, max_r):
        sy = int(grid_y * cell_px + offset_y)
        if sy < 0 or sy > screen_h:
            continue
        for grid_x in range(min_c, max_c):
            value = float(log_copy[grid_y, grid_x])
            if abs(value) < 0.06:
                continue
            sx = int(grid_x * cell_px + offset_x)
            pygame.draw.rect(
                screen,
                probability_to_rgb(float(probs[grid_y, grid_x])),
                pygame.Rect(sx, sy, draw_cell, draw_cell),)

    display_trail = list(trail)
    if len(display_trail) > 1:
        points = [(int(px * cell_px + offset_x), int(py * cell_px + offset_y)) for px, py in display_trail]
        pygame.draw.lines(screen, COL_TRAIL, False, points, 2)

    frontier_path = controller.get("frontier_path", [])
    if len(frontier_path) > 1:
        points = [(int(grid_x * cell_px + offset_x), int(grid_y * cell_px + offset_y)) for grid_x, grid_y in frontier_path]
        pygame.draw.lines(screen, COL_PLAN, False, points, 2)

    sx = int(start_cell[0] * cell_px + offset_x)
    sy = int(start_cell[1] * cell_px + offset_y)
    pygame.draw.rect(screen, COL_START, pygame.Rect(sx - 5, sy - 5, 10, 10))

    if exit_cell:
        ex = int(exit_cell[0] * cell_px + offset_x)
        ey = int(exit_cell[1] * cell_px + offset_y)
        pygame.draw.rect(screen, COL_EXIT, pygame.Rect(ex - 7, ey - 7, 14, 14), 3)

    distance = float(state.get("dist", MAXIMUM_SENSOR_RANGE))
    ray_len_px = int((distance / CELL_SIZE) * cell_px)
    hx = int(robot_screen[0] + math.cos(math.radians(h)) * ray_len_px)
    hy = int(robot_screen[1] - math.sin(math.radians(h)) * ray_len_px)
    pygame.draw.line(screen, COL_SENSOR, robot_screen, (hx, hy), 1)

    rr = max(6, int((ROBOT_RADIUS / CELL_SIZE) * cell_px))
    pygame.draw.circle(screen, COL_ROBOT, robot_screen, rr, 2)
    fx = int(robot_screen[0] + math.cos(math.radians(h)) * (rr + 8))
    fy = int(robot_screen[1] - math.sin(math.radians(h)) * (rr + 8))
    pygame.draw.line(screen, COL_TRAIL, robot_screen, (fx, fy), 2)


pose_conf = {"trans_drift_cm": 0.0,"rot_drift_deg": 0.0,}
DRIFT_PER_CM = 0.04
DRIFT_PER_TURN_DEG = 0.03

def note_translation(cm: float):
    pose_conf["trans_drift_cm"] += abs(float(cm)) * DRIFT_PER_CM


def note_rotation(deg: float):
    pose_conf["rot_drift_deg"] += abs(float(deg)) * DRIFT_PER_TURN_DEG
def pose_confidence_pct() -> float:
    d = pose_conf["trans_drift_cm"] + pose_conf["rot_drift_deg"] * 0.5
    return max(5.0, 100.0 * math.exp(-d / 25.0))


def prepare_exit_results():
    global exit_cell
    if bool(state.get("result_ready")):
        return
    stop_robot("exit found")
    if exit_cell is None:
        x_exit, y_exit, unused_value = current_pose()
        exit_cell = (int(round(x_exit)), int(round(y_exit)))
    controller["frontier_path"] = []
    update_map_counts()
    state["result_ready"] = True
    state["mode"] = "FINISHED"
    state["behaviour"] = "exit found; live map held"
    state["finish_t"] = time.time()

# COMMAND LINE SETUPPPP
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ip_pos", nargs="?")
    parser.add_argument("--ip")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--hand", choices=["right", "left"], default=os.environ.get("MBOT2_HAND", "right"))
    args = parser.parse_args()

    def signal_stop(sig, frame):
        stop_event.set()
        stop_robot("signal stop")

    signal.signal(signal.SIGINT, signal_stop) # stop everything if the program gets interrupted
    signal.signal(signal.SIGTERM, signal_stop)

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    try:
        connect_robot(args.ip, args.port)
    except Exception as exc:
        print("Could not connect:", exc)
        pygame.quit()
        return

    try:
        while not stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_event.set()

                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        stop_event.set()

                    elif event.key == pygame.K_SPACE and not state.get("result_ready"):
                        state["paused"] = not state.get("paused", False)
                        if state["paused"]:
                            stop_robot("paused")

                    elif event.key == pygame.K_c and not state.get("result_ready"):
                        with map_lock:
                            occupancy_grid.fill(UNKNOWN)
                            stable_endpoint_heading.fill(np.nan)

                            for grid in (obstacle_hits, arc_obstacle_hits, turn_obstacle_hits,
                                         support_obstacle_hits, free_hits, visit_count, log_odds_grid):
                                grid.fill(0)

                        invalidate_likelihood_caches()

            if state.get("exit_found") and not state.get("result_ready"):
                prepare_exit_results()

            if state.get("result_ready"):
                set_command("S", "exit found")
            else:
                controller_tick(True) # update the exploration controller every cycle

            pump_command()
            update_map_counts()
            draw_map(screen)
            pygame.display.flip()
            clock.tick(FPS)

    finally:
        stop_event.set()

        try:
            stop_robot("shutdown") # always stop the robot before closing the program
            time.sleep(0.15)
            raw_send("S")

            if sock is not None:
                sock.close()
        except Exception:
            pass

        pygame.quit()


if __name__ == "__main__":
    main()
