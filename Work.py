#!/usr/bin/env python3
import time
import math
import sys
import csv
from ev3dev2.motor import MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_1, INPUT_4

# Motors and sensor setup
MOTOR_R = MediumMotor(OUTPUT_B)
MOTOR_B = MediumMotor(OUTPUT_C)
MOTOR_L = MediumMotor(OUTPUT_A)

SONIC_L = UltrasonicSensor(INPUT_4)
SONIC_R = UltrasonicSensor(INPUT_1)

# ---------- Geometry and motion ----------
TARGET_DISTANCE_CM = 30.0
V_X_MAX = 20.0
V_X_MIN = 6.0
MIN_ANGLE_ERROR = 1.0

# Два разных порога вместо одного MAX_VALID_DISTANCE
CONTROL_MAX_DIST = 60.0     # выше этого значения показания обрезаются для регулятора
NO_ECHO_DIST = 150.0        # выше этого датчик считается "ничего не видит" (обычно 255)

# Debounce: сколько циклов подряд держим last_valid при отсутствии эха
LOST_THRESHOLD = 3

# ---------- V_Y PD ----------
Kv_p_close = 2.6
Kv_p_far = 1.5
Kv_d = 0.7
MAX_VY = 30.0
prev_v_error = 0.0
filtered_v_deriv = 0.0

# ---------- W PD ----------
Kw_p_pos = 1.75
Kw_d_pos = 0.8
Kw_p_neg = 0.35             # если теряется правый датчик, попробуйте 1.0 .. 1.5
Kw_d_neg = 0.25
MAX_W = 40.0
CORNER_SLOWDOWN_K = 1.4
prev_angle_error = 0.0
filtered_angle_deriv = 0.0
ALPHA_D = 0.4

# ---------- Режим потери цели (оба датчика не видят) ----------
LOST_V_X = 8.0              # продолжаем ползти вдоль объекта, а не стоим
LOST_V_Y = -12.0            # отрицательный V_Y это к объекту
LOST_W = 14.0
LOST_TIMEOUT = 3.0          # через сколько секунд слепой езды остановиться

# Precomputed trigonometry for 120-degree Kiwi drive
COS_120 = math.cos(2.0 * math.pi / 3.0)
SIN_120 = math.sin(2.0 * math.pi / 3.0)
COS_240 = math.cos(4.0 * math.pi / 3.0)
SIN_240 = math.sin(4.0 * math.pi / 3.0)

hist_L = [TARGET_DISTANCE_CM] * 3
hist_R = [TARGET_DISTANCE_CM] * 3

lost_count_L = 0
lost_count_R = 0
last_valid_L = TARGET_DISTANCE_CM
last_valid_R = TARGET_DISTANCE_CM


def get_median_3(new_val, history_list):
    history_list.pop(0)
    history_list.append(new_val)
    return sorted(history_list)[1]


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def stop_all():
    MOTOR_B.off()
    MOTOR_R.off()
    MOTOR_L.off()


# --- LOGGING SETUP ---
log_file = open('robot_log_v5.csv', mode='w')
csv_writer = csv.writer(log_file)
csv_writer.writerow(["time", "state", "raw_L", "raw_R", "lost_L", "lost_R",
                     "dist_L", "dist_R", "v_error", "angle_error",
                     "V_X", "V_Y", "W"])

loop_count = 0
start_time = time.time()
last_time = start_time
was_lost = False
lost_since = None

try:
    while True:
        # 1. Raw sensor distances
        raw_L = SONIC_L.distance_centimeters
        raw_R = SONIC_R.distance_centimeters

        # 2. Debounce только для настоящего "нет эха"
        if raw_L >= NO_ECHO_DIST:
            lost_count_L += 1
            if lost_count_L < LOST_THRESHOLD:
                raw_L = last_valid_L
        else:
            lost_count_L = 0
            last_valid_L = raw_L

        if raw_R >= NO_ECHO_DIST:
            lost_count_R += 1
            if lost_count_R < LOST_THRESHOLD:
                raw_R = last_valid_R
        else:
            lost_count_R = 0
            last_valid_R = raw_R

        lost_L = lost_count_L >= LOST_THRESHOLD
        lost_R = lost_count_R >= LOST_THRESHOLD
        both_lost = lost_L and lost_R

        # 3. Обрезка для регулятора (60 см, а не 35) и медиана
        dist_L = get_median_3(min(raw_L, CONTROL_MAX_DIST), hist_L)
        dist_R = get_median_3(min(raw_R, CONTROL_MAX_DIST), hist_R)

        # 4. Delta time
        current_time = time.time()
        d_t = current_time - last_time
        if d_t <= 0.0:
            d_t = 0.001

        v_error = 0.0
        angle_error = 0.0
        state_str = "NORMAL"

        if both_lost:
            state_str = "LOST"
            if lost_since is None:
                lost_since = current_time
            if current_time - lost_since > LOST_TIMEOUT:
                print("Target lost for too long, stopping.")
                stop_all()
                break
            V_X_cmd = LOST_V_X
            V_Y = LOST_V_Y
            W = LOST_W
            filtered_v_deriv = 0.0
            filtered_angle_deriv = 0.0
        else:
            lost_since = None
            if lost_L or lost_R:
                state_str = "ONE_LOST"

            min_dist = min(dist_L, dist_R)
            max_dist = max(dist_L, dist_R)
            dist_F = 0.75 * min_dist + 0.25 * max_dist

            # --- V_Y ---
            v_error = TARGET_DISTANCE_CM - dist_F
            if was_lost:
                prev_v_error = v_error      # чтобы не было пика производной после LOST
            raw_v_deriv = (v_error - prev_v_error) / d_t
            filtered_v_deriv = ALPHA_D * raw_v_deriv + (1.0 - ALPHA_D) * filtered_v_deriv
            filtered_v_deriv = clamp(filtered_v_deriv, -15.0, 15.0)

            if v_error >= 0.0:
                V_Y = Kv_p_close * v_error + Kv_d * filtered_v_deriv
            else:
                V_Y = Kv_p_far * v_error + Kv_d * filtered_v_deriv
            V_Y = clamp(V_Y, -MAX_VY, MAX_VY)
            prev_v_error = v_error

            # --- W ---
            raw_angle_error = dist_L - dist_R
            if abs(raw_angle_error) < MIN_ANGLE_ERROR:
                angle_error = 0.0
            elif raw_angle_error > 0.0:
                angle_error = raw_angle_error - MIN_ANGLE_ERROR
            else:
                angle_error = raw_angle_error + MIN_ANGLE_ERROR

            if was_lost:
                prev_angle_error = angle_error
            raw_angle_deriv = (angle_error - prev_angle_error) / d_t
            filtered_angle_deriv = ALPHA_D * raw_angle_deriv + (1.0 - ALPHA_D) * filtered_angle_deriv
            filtered_angle_deriv = clamp(filtered_angle_deriv, -15.0, 15.0)

            if angle_error >= 0.0:
                W = Kw_p_pos * angle_error + Kw_d_pos * filtered_angle_deriv
            else:
                W = Kw_p_neg * angle_error + Kw_d_neg * filtered_angle_deriv
            W = clamp(W, -MAX_W, MAX_W)
            prev_angle_error = angle_error

            if angle_error > 0.0:
                V_X_cmd = max(V_X_MIN, V_X_MAX - CORNER_SLOWDOWN_K * angle_error)
            else:
                V_X_cmd = V_X_MAX

        was_lost = both_lost

        # 5. Kiwi drive kinematics
        B_speed = V_X_cmd + W
        R_speed = V_X_cmd * COS_120 + V_Y * SIN_120 + W
        L_speed = V_X_cmd * COS_240 + V_Y * SIN_240 + W

        # 6. Proportional normalization
        max_speed = max(abs(B_speed), abs(R_speed), abs(L_speed))
        if max_speed > 100.0:
            k = 100.0 / max_speed
            B_speed *= k
            R_speed *= k
            L_speed *= k

        # --- LOGS ---
        t_now = current_time - start_time
        csv_writer.writerow(["{:.3f}".format(t_now), state_str, raw_L, raw_R,
                             int(lost_L), int(lost_R), dist_L, dist_R,
                             v_error, angle_error, V_X_cmd, V_Y, W])
        loop_count += 1
        if loop_count % 20 == 0:
            log_file.flush()
            print("[{:.1f}s] {} | Raw(L/R): {:.1f}/{:.1f} | Err(V/Ang): {:.1f}/{:.1f} | Cmd(X/Y/W): {:.1f}/{:.1f}/{:.1f}".format(
                t_now, state_str, raw_L, raw_R, v_error, angle_error, V_X_cmd, V_Y, W))
            sys.stdout.flush()

        # 7. Motors
        MOTOR_B.on(-B_speed)
        MOTOR_R.on(-R_speed)
        MOTOR_L.on(-L_speed)

        last_time = current_time
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nProgram interrupted.")
finally:
    stop_all()
    log_file.close()