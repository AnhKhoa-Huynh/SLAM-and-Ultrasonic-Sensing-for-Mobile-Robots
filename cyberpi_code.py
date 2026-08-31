import cyberpi
import mbot2
import socket
import time

# HOTSPOT PASSWORDDDDDDDD
NETWORK_NAME = "Sticky_Riice"
NETWORK_PASSWORD = "12343234"
NETWORK_PORT = 5000


FORWARD_SPEED = 18
REVERSE_SPEED = 14
TURNING_SPEED = 12
MAX_WHEEL_SPEED = 28
EMERGENCY_STOP = 12.5
FORWARD_GUARD = 12.0
COMMAND_TIMEOUT = 1.05   
SEND_INTERVAL = 0.045
SEND_FAILURE_LIMIT = 18
ENCODER_READ_INTERVAL = 0.055
RGB_READ_INTERVAL = 0.30
DISTANCE_READ_INTERVAL = 0.035
YAW_READ_INTERVAL = 0.035
TURN_CLEARANCE = 16.0 
TURN_CONTACT_DISTANCE = 7.5  
EXIT_ANNOUNCE_GRACE = 2.0
EXIT_REPEAT_INTERVAL = 0.12
EXIT_SEND_HOLD = 0.25
WHITE_MINIMUM = 175.0
WHITE_RISE = 55.0
WHITE_MAX_COLOUR_DIFFERENCE = 70.0
WHITE_CONFIRMATION_COUNT = 4

try:
    start_time = time.ticks_ms()

    def now_s():
        return time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
except Exception:
    start_time_fallback = time.time()

    def now_s():
        return time.time() - start_time_fallback

motion = "S"
contact_latched = False
turn_contact_count = 0
exit_found = False
white_count = 0
white_score = 0.0
floor_white = None
rgb_method = 0
last_rgb_time = 0.0
last_white_result = False
encoder_method = 0
first_encoder_readings = {}
last_encoder_time = 0.0
last_encoder_values = (0, 0)
last_distance = 300.0
last_distance_time = -999.0
last_yaw = 0.0
last_yaw_time = -999.0
telemetry_sequence = 0


# MOTORRRRRR

mbot2.drive_speed(0, 0)

def stop_robot():
    mbot2.drive_speed(0, 0)

def drive_forward(speed):
    mbot2.drive_speed(speed, -speed)
def drive_reverse(speed):
    mbot2.drive_speed(-speed, speed)
def turn_left(speed):
    mbot2.drive_speed(-speed, -speed)
def turn_right(speed):
    mbot2.drive_speed(speed, speed)
def drive_wheels(left_forward, right_forward):
    mbot2.drive_speed(left_forward, -right_forward)



def decode_text(data):

    try:
        return data.decode("utf-8")
    except Exception:
        return str(data)

def send_text(connection, text):

    try:
        payload = text.encode("utf-8")
        if hasattr(connection, "sendall"):
            connection.sendall(payload)
        else:
            connection.send(payload)
        return True
    except Exception:
        return False
def clamp_speed(v):
    try:
        v = int(float(v))
    except Exception:
        return 0
    return max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, v))


def parse_speed(command, default_speed):
    if ":" not in command:
        return default_speed
    try:
        v = int(float(command.split(":", 1)[1]))
        return max(0, min(MAX_WHEEL_SPEED, v))
    except Exception:
        return default_speed


# SENSORRRRRRRR
def read_distance():
    try:
        d = cyberpi.ultrasonic2.get(index=1)
        if d is None:
            return 300.0
        return float(d)
    except Exception:
        return 300.0


def get_distance(force=False):
    global last_distance, last_distance_time
    t = now_s()
    if force or t - last_distance_time >= DISTANCE_READ_INTERVAL:
        last_distance = read_distance()
        last_distance_time = t
    return last_distance

def get_yaw(force=False):
    global last_yaw, last_yaw_time
    t = now_s()
    if force or t - last_yaw_time >= YAW_READ_INTERVAL:
        try:
            last_yaw = float(cyberpi.get_yaw())
        except Exception:
            pass
        last_yaw_time = t
    return last_yaw


def read_encoder_method(method_number, wheel): # check api until it works
    if method_number == 1:
        return int(mbot2.get_encoder_value(wheel))
    if method_number == 2:
        return int(mbot2.EM_get_angle(wheel))
    if method_number == 3:
        return int(mbot2.EM_get_angle("EM" + str(wheel)))
    raise ValueError


def read_encoders():
    global encoder_method, last_encoder_time, last_encoder_values
    t = now_s()
    if t - last_encoder_time < ENCODER_READ_INTERVAL:
        return last_encoder_values
    last_encoder_time = t
# check api
    if encoder_method:
        try:
            last_encoder_values = (read_encoder_method(encoder_method, 1), read_encoder_method(encoder_method, 2))
        except Exception:
            pass
        return last_encoder_values
    best = None
    for method_number in (1, 2, 3):
        try:
            l = read_encoder_method(method_number, 1)
            r = read_encoder_method(method_number, 2)
        except Exception:
            continue
        if method_number not in first_encoder_readings:
            first_encoder_readings[method_number] = (l, r)
        f = first_encoder_readings[method_number]
        if abs(l - f[0]) > 3 or abs(r - f[1]) > 3:
            encoder_method = method_number
            last_encoder_values = (l, r)
            return last_encoder_values
        if best is None:
            best = (l, r)
    if best is not None:
        last_encoder_values = best
    return last_encoder_values


# RGB SENSORRRRR
def read_rgb_api(method_number):
    if method_number == 1:
        import mbuild
        return (mbuild.quad_rgb_sensor.get_red("all", index=1),
                mbuild.quad_rgb_sensor.get_green("all", index=1),
                mbuild.quad_rgb_sensor.get_blue("all", index=1))
    if method_number == 2:
        import mbuild
        return (mbuild.quad_rgb_sensor.get_red(1, index=1),
                mbuild.quad_rgb_sensor.get_green(1, index=1),
                mbuild.quad_rgb_sensor.get_blue(1, index=1))
    if method_number == 3:
        return (cyberpi.quad_rgb_sensor.get_red("all", index=1),
                cyberpi.quad_rgb_sensor.get_green("all", index=1),
                cyberpi.quad_rgb_sensor.get_blue("all", index=1))
    if method_number == 4:
        return (cyberpi.quad_rgb.get_red("all", index=1),
                cyberpi.quad_rgb.get_green("all", index=1),
                cyberpi.quad_rgb.get_blue("all", index=1))
    return (None, None, None)


def read_rgb():
    global rgb_method
    if rgb_method:
        try:
            r, g, b = read_rgb_api(rgb_method)
            return float(r), float(g), float(b)
        except Exception:
            rgb_method = 0
    for method_number in (1, 2, 3, 4):
        try:
            r, g, b = read_rgb_api(method_number)
            rgb_method = method_number
            return float(r), float(g), float(b)
        except Exception:
            pass
    return None, None, None
def check_white_exit():
    global white_count, white_score, exit_found, floor_white, last_rgb_time, last_white_result
    if exit_found:
        return True
    t = now_s()
    if t - last_rgb_time < RGB_READ_INTERVAL:
        return last_white_result
    last_rgb_time = t

    r, g, b = read_rgb()
    if r is None:
        white_score = 0.0
        white_count = 0
        last_white_result = False
        return False

    mn = min(r, g, b)
    mx = max(r, g, b)
    white_score = mn
    if floor_white is None:
        floor_white = mn

    threshold = max(WHITE_MINIMUM, floor_white + WHITE_RISE)
    balanced = (mx - mn) <= WHITE_MAX_COLOUR_DIFFERENCE
    is_white = mn >= threshold and balanced

    if not is_white and mn < threshold + 20:
        floor_white = 0.985 * floor_white + 0.015 * mn

    if is_white:
        white_count += 1
    else:
        white_count = 0

    if white_count >= WHITE_CONFIRMATION_COUNT:
        exit_found = True
        last_white_result = True
        return True

    last_white_result = False
    return False



def send_telemetry(connection):
    global telemetry_sequence
    d = get_distance()
    y = get_yaw()
    l, r = read_encoders()
    sequence = telemetry_sequence
    telemetry_sequence += 1
    device_time = int(now_s() * 1000.0)
    return send_text(connection, f"{d:.1f},{y:.2f},{l},{r},{motion},
                     {white_score:.1f},{int(exit_found)},{sequence},{device_time}\n")


def apply_command(command):
    global motion, contact_latched
    c = command.strip().upper()
    if not c:
        return
    if c == "Q":
        raise KeyboardInterrupt()
    if c == "S" or c == "STOP" or c == "PING":
        stop_robot()
        motion = "S"
        if get_distance() >= TURN_CLEARANCE:
            contact_latched = False
        return
    if c.startswith("W:"):
        parts = c.split(":")
        if len(parts) >= 3:
            left = clamp_speed(parts[1])
            right = clamp_speed(parts[2])
            avg_forward = (left + right) * 0.5
            current_range = get_distance()
            if avg_forward > 2 and (contact_latched or current_range <= FORWARD_GUARD):  
                
                stop_robot()
                motion = "S"
                if current_range >= TURN_CLEARANCE:
                    contact_latched = False
                return
            drive_wheels(left, right)
            if abs(left) < 2 and abs(right) < 2:
                motion = "S"
            elif left > 0 and right > 0:
                motion = "F" if abs(left - right) < 3 else "A"
            elif left < 0 and right < 0:
                motion = "B"
            else:
                motion = "T"
        return
    if c.startswith("F"):
        current_range = get_distance()
        if contact_latched or current_range <= FORWARD_GUARD:
            stop_robot()
            motion = "S"
            if current_range >= TURN_CLEARANCE:
                contact_latched = False
            return
        drive_forward(parse_speed(c, FORWARD_SPEED))
        motion = "F"
        return
    if c.startswith("B"):
        contact_latched = False
        drive_reverse(parse_speed(c, REVERSE_SPEED))
        motion = "B"
        return
    if c.startswith("L"):
        turn_left(parse_speed(c, TURNING_SPEED))
        motion = "T"
        return
    if c.startswith("R"):
        turn_right(parse_speed(c, TURNING_SPEED))
        motion = "T"
        return


def announce_exit_reliably(connection):
    global motion
    motion = "S"
    stop_robot()
    deadline = now_s() + EXIT_ANNOUNCE_GRACE
    last_announce = -999.0
    acked = False
    while now_s() < deadline and not acked:
        t = now_s()
        if t - last_announce >= EXIT_REPEAT_INTERVAL:
            send_telemetry(connection)
            send_text(connection, "EXIT\n")
            last_announce = t
        try:
            data = connection.recv(64)
            if data and "EXIT_ACK" in decode_text(data).upper():
                acked = True
        except Exception:
            pass
        time.sleep(0.015)

    time.sleep(EXIT_SEND_HOLD)
    return acked


# MAIN SERVER LOOPPPPPPPPPP
def serve_once(server):
    global motion, exit_found, contact_latched, turn_contact_count
    connection = None
    send_failures = 0
    buffer = ""
    last_send_time = now_s()
    last_command_time = now_s()

    connection, address = server.accept()
    connection.settimeout(0.01)
    motion = "S"
    contact_latched = False
    turn_contact_count = 0
    stop_robot()

    try:
        while True:
            try:
                data = connection.recv(128)
                if not data:
                    break
                buffer += decode_text(data)
            except Exception:
                pass

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                last_command_time = now_s()
                apply_command(line)
            safe_range = get_distance()     
            if motion in ("F", "A") and safe_range <= EMERGENCY_STOP:
                stop_robot()
                motion = "S"
                contact_latched = True
            elif motion == "T" and safe_range <= TURN_CONTACT_DISTANCE:
                turn_contact_count += 1
                if turn_contact_count >= 4:
                    stop_robot()
                    motion = "S"
                    contact_latched = True
            else:
                turn_contact_count = 0
            if motion != "S" and now_s() - last_command_time > COMMAND_TIMEOUT:  
                stop_robot()
                motion = "S"
            if now_s() - last_send_time >= SEND_INTERVAL: 
                if send_telemetry(connection):
                    send_failures = 0
                else:
                    send_failures += 1
                    if send_failures >= SEND_FAILURE_LIMIT:
                        break
                last_send_time = now_s()
            if check_white_exit():
                stop_robot()
                motion = "S"
                announce_exit_reliably(connection)
                break

            time.sleep(0.004)

    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        stop_robot()
        motion = "S"
        try:
            if connection:
                connection.close()
        except Exception:
            pass
        time.sleep(0.4)


@cyberpi.event.start
def on_start():
    global exit_found
    stop_robot()
    exit_found = False
    try:
        cyberpi.led.off()
    except Exception:
        pass
    cyberpi.wifi.connect(NETWORK_NAME, NETWORK_PASSWORD)
    while not cyberpi.wifi.is_connect():
        time.sleep(0.2)
    try:
        cyberpi.reset_yaw()
    except Exception:
        pass

    time.sleep(0.4)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", NETWORK_PORT))
    server.listen(1)

    while not exit_found:
        serve_once(server)

    try:
        server.close()
    except Exception:
        pass
    stop_robot()
