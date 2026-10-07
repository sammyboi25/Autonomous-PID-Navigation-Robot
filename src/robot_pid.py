import time
import board
import adafruit_tcs34725
from mpu6050 import mpu6050
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# ——— System Setup ——————————————————————————————————————
Device.pin_factory = LGPIOFactory(chip=4)

# Left Motor (AIN1, AIN2, PWMA)
AIN1 = DigitalOutputDevice(17)
AIN2 = DigitalOutputDevice(24)
PWMA = PWMOutputDevice(12)

# Right Motor (BIN1, BIN2, PWMB)
BIN1 = DigitalOutputDevice(22)
BIN2 = DigitalOutputDevice(23)
PWMB = PWMOutputDevice(13)

# ——— Configuration —————————————————————————————————————
BLACK_R, BLACK_G, BLACK_B = 39, 74, 67
WHITE_R, WHITE_G, WHITE_B = 205, 304, 280

# ——— PID Constants ————————————————————————————————————
KP = 0.015
KI = 0.0008
KD = 0.004

# ——— Speed Constants ——————————————————————————————————
BASE_SPEED = 0.35
TURN_SPEED = 0.30

# ——— Constraints —————————————————————————————————————
MAX_CORRECTION = 0.15
MIN_MOTOR_SPEED = 0.15
MAX_MOTOR_SPEED = 0.60

# PID Integral Limits
MAX_INTEGRAL = 100.0

# Derivative filtering
DERIVATIVE_FILTER = 0.7

# ——— Hardware Setup ——————————————————————————————————
i2c = board.I2C()

tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 200

time.sleep(0.5)

mpu = mpu6050(0x68)

# ——— IMU Variables ———————————————————————————————————
gyro_x_angle = 0.0
last_imu_time = time.time()

target_angle = 0.0
bias = 0.0

# ——— PID State ———————————————————————————————————————
integral_error = 0.0
previous_error = 0.0
filtered_derivative = 0.0
pid_initialized = False


# ——— Motor Control Functions ——————————————————————————

def set_speeds(left_speed, right_speed):
    """Sets motor speeds from -1.0 to 1.0"""

    left_speed = max(min(left_speed, 1.0), -1.0)
    right_speed = max(min(right_speed, 1.0), -1.0)

    # Left Motor
    if left_speed >= 0:
        AIN1.on()
        AIN2.off()
        PWMA.value = left_speed
    else:
        AIN1.off()
        AIN2.on()
        PWMA.value = abs(left_speed)

    # Right Motor
    if right_speed >= 0:
        BIN1.on()
        BIN2.off()
        PWMB.value = right_speed
    else:
        BIN1.off()
        BIN2.on()
        PWMB.value = abs(right_speed)


def stop_motors():
    PWMA.value = 0
    PWMB.value = 0

    AIN1.off()
    AIN2.off()

    BIN1.off()
    BIN2.off()


# ——— PID Functions ————————————————————————————————————

def reset_pid():
    global integral_error
    global previous_error
    global filtered_derivative
    global pid_initialized

    integral_error = 0.0
    previous_error = 0.0
    filtered_derivative = 0.0
    pid_initialized = False


def calculate_pid(error, dt):
    global integral_error
    global previous_error
    global filtered_derivative
    global pid_initialized

    if dt <= 0:
        return 0.0

    # First loop
    if not pid_initialized:
        previous_error = error
        pid_initialized = True

    # Integral term
    integral_error += error * dt

    # Anti-windup
    integral_error = max(
        min(integral_error, MAX_INTEGRAL),
        -MAX_INTEGRAL
    )

    # Derivative term
    derivative = (error - previous_error) / dt

    # Low-pass filtering of derivative
    filtered_derivative = (
        DERIVATIVE_FILTER * filtered_derivative
        + (1.0 - DERIVATIVE_FILTER) * derivative
    )

    # PID
    p_term = KP * error
    i_term = KI * integral_error
    d_term = KD * filtered_derivative

    output = p_term + i_term + d_term

    previous_error = error

    # Limit correction
    output = max(
        min(output, MAX_CORRECTION),
        -MAX_CORRECTION
    )

    return output


# ——— IMU Function ————————————————————————————————————

def get_angle():
    global gyro_x_angle
    global last_imu_time

    try:
        now = time.time()

        dt = now - last_imu_time

        data = mpu.get_gyro_data()

        gyro_rate = data['x'] - bias

        # Deadzone
        if abs(gyro_rate) < 1.5:
            gyro_rate = 0.0

        gyro_x_angle += gyro_rate * dt

        last_imu_time = now

        return gyro_x_angle

    except:
        return gyro_x_angle


# ——— Color Detection —————————————————————————————————

def get_color():

    r, g, b, c = tcs.color_raw

    wr_cal = WHITE_R - BLACK_R
    wg_cal = WHITE_G - BLACK_G
    wb_cal = WHITE_B - BLACK_B

    max_val = max(wr_cal, wg_cal, wb_cal)

    scale_r = max_val / wr_cal if wr_cal > 0 else 1
    scale_g = max_val / wg_cal if wg_cal > 0 else 1
    scale_b = max_val / wb_cal if wb_cal > 0 else 1

    r_cal = max(0, (r - BLACK_R) * scale_r)
    g_cal = max(0, (g - BLACK_G) * scale_g)
    b_cal = max(0, (b - BLACK_B) * scale_b)

    threshold = 1.3
    min_val = 30.0

    if r_cal < min_val and g_cal < min_val and b_cal < min_val:
        return "Black"

    elif b_cal > r_cal * threshold and b_cal > g_cal * threshold:
        return "Blue"

    elif g_cal > r_cal * threshold and g_cal > b_cal * threshold:
        return "Green"

    elif r_cal > b_cal * threshold and g_cal > b_cal * 1.5:
        return "Yellow"

    elif r_cal > g_cal * threshold and r_cal > b_cal * threshold:
        return "Red"

    return "Unknown"


# ——— PID Straight Driving ————————————————————————————

def drive_with_pid_timed(duration):

    global target_angle

    reset_pid()

    start_time = time.time()
    previous_time = start_time

    print(f"[SYSTEM] Clearing card for {duration}s...")

    while time.time() - start_time < duration:

        current_time = time.time()

        dt = current_time - previous_time
        previous_time = current_time

        angle = get_angle()

        error = target_angle - angle

        correction = calculate_pid(error, dt)

        left_speed = BASE_SPEED + correction
        right_speed = BASE_SPEED - correction

        left_speed = max(
            min(left_speed, MAX_MOTOR_SPEED),
            MIN_MOTOR_SPEED
        )

        right_speed = max(
            min(right_speed, MAX_MOTOR_SPEED),
            MIN_MOTOR_SPEED
        )

        set_speeds(left_speed, right_speed)

        time.sleep(0.01)


# ——— PID 90 Degree Turn —————————————————————————————

def turn_90_smart(direction):

    global target_angle

    print(f"\n[TURN] Starting {direction} turn...")

    # Set new target
    if direction == "right":
        target_angle += 90

    else:
        target_angle -= 90

    reset_pid()

    previous_time = time.time()

    while True:

        current_time = time.time()

        dt = current_time - previous_time
        previous_time = current_time

        current_angle = get_angle()

        error = target_angle - current_angle

        # Stop when close enough
        if abs(error) < 2.0:
            break

        correction = calculate_pid(error, dt)

        turn_speed = max(
            min(abs(correction), TURN_SPEED),
            0.18
        )

        if error > 0:

            # Turn right
            set_speeds(
                turn_speed,
                -turn_speed
            )

        else:

            # Turn left
            set_speeds(
                -turn_speed,
                turn_speed
            )

        time.sleep(0.01)

    stop_motors()

    reset_pid()

    time.sleep(0.3)

    print(
        f"[TURN] Finished. Angle: "
        f"{get_angle():.1f}"
    )


# ——— Main Program ————————————————————————————————————

print("[SYSTEM] Calibrating IMU... STAY STILL.")

bias = sum(
    [mpu.get_gyro_data()['x'] for _ in range(500)]
) / 500

print(
    f"[SYSTEM] Calibration Done. "
    f"Bias: {bias:.2f}"
)

try:

    while True:

        color = get_color()

        angle = get_angle()

        # ——— RED ————————————————————————————————

        if color == "Red":

            time.sleep(0.05)

            if get_color() == "Red":

                print(
                    "\n[STOP] Red detected. "
                    "Powering down."
                )

                stop_motors()

                break


        # ——— YELLOW ————————————————————————————

        elif color == "Yellow":

            print(
                "\n[EVENT] Yellow Card -> Left Turn"
            )

            stop_motors()

            time.sleep(0.2)

            turn_90_smart("left")

            drive_with_pid_timed(1.2)


        # ——— BLUE —————————————————————————————

        elif color == "Blue":

            print(
                "\n[EVENT] Blue Card -> Right Turn"
            )

            stop_motors()

            time.sleep(0.2)

            turn_90_smart("right")

            drive_with_pid_timed(1.2)


        # ——— NORMAL DRIVING ————————————————————

        else:

            error = target_angle - angle

            correction = calculate_pid(
                error,
                0.01
            )

            left_speed = BASE_SPEED + correction

            right_speed = BASE_SPEED - correction

            left_speed = max(
                min(left_speed, MAX_MOTOR_SPEED),
                MIN_MOTOR_SPEED
            )

            right_speed = max(
                min(right_speed, MAX_MOTOR_SPEED),
                MIN_MOTOR_SPEED
            )

            set_speeds(
                left_speed,
                right_speed
            )

            print(
                f"[DRIVE] {color} | "
                f"Angle: {angle:.1f} | "
                f"Error: {error:.1f} | "
                f"PID: {correction:.3f}",
                end='\r'
            )

        time.sleep(0.01)


except KeyboardInterrupt:

    print("\n[STOP] User interrupted.")


finally:

    stop_motors()
