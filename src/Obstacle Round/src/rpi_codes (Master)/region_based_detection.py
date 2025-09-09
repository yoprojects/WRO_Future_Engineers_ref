# ==============================
# Region-Based Obstacle Navigation for RPi5 + ESP32
# Divides frame into 3 vertical regions for smart turning
# ==============================

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import RPi.GPIO as GPIO
import board
import busio
import adafruit_bno055
import logging
import os
from datetime import datetime

# ------------------ CONFIG ------------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
BUTTON_PIN = 17  # physical push button on RPi
LED_PIN = 18     # Status LED

FRAME_W, FRAME_H = 1280, 720

# Frame regions (divide into 3 vertical sections)
LEFT_REGION = (0, FRAME_W // 3)           # 0 to 426
CENTER_REGION = (FRAME_W // 3, 2 * FRAME_W // 3)  # 426 to 853
RIGHT_REGION = (2 * FRAME_W // 3, FRAME_W)        # 853 to 1280

# Servo calibration values (must match ESP32 sketch)
SERVO_LEFT = 25
SERVO_RIGHT = 145
SERVO_STRAIGHT = 90

# Turn intensity configuration based on block size
MIN_TURN = 10     # Minimum turn angle for small/distant blocks
MAX_TURN = 60     # Maximum turn angle for large/close blocks
SIZE_THRESHOLD_SMALL = 1000    # Small block area threshold
SIZE_THRESHOLD_LARGE = 8000    # Large block area threshold

# Speed values
BASE_SPEED = 140
STOP_SPEED = 0

# Color ranges (HSV) - corrected for proper red/green detection
lower_red1 = np.array([171, 57, 84])      # Lower red range
upper_red1 = np.array([184, 255, 255])
lower_red2 = np.array([171, 57, 84])    # Upper red range (wraps around hue)
upper_red2 = np.array([184, 255, 255])

lower_green = np.array([51, 75, 54])     # Green range
upper_green = np.array([85, 255, 255])

# ------------------ LOGGING CONFIG ------------------
# Create logs directory if it doesn't exist
LOGS_DIR = "/home/rpi5wro/wro_fe_project/wro_fe_25/obstacle_round_v2/logs"
os.makedirs(LOGS_DIR, exist_ok=True)

# Generate timestamp for this run
RUN_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(LOGS_DIR, f"navigation_log_{RUN_TIMESTAMP}.txt")
VIDEO_FILE = os.path.join(LOGS_DIR, f"navigation_video_{RUN_TIMESTAMP}.avi")

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()  # Also print to console
    ]
)
logger = logging.getLogger(_name_)

# Video recording setup
video_writer = None
RECORD_VIDEO = True  # Set to False to disable video recording

# --------------------------------------------

# System setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_PIN, GPIO.OUT)

# Init Serial to ESP32
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# Init Camera
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (FRAME_W, FRAME_H), "format": 'RGB888'})

# Init BNO055 IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Global variables
initial_yaw = 0
yaw = 0
front_dist, left_dist, right_dist = 100, 35, 35
last_turn_direction = ""  # Track last turn to help with recovery
recovery_mode = False     # Flag for recovery steering

# Obstacle memory system to handle brief detection losses
last_seen_obstacles = {"red": [], "green": [], "timestamp": 0}
OBSTACLE_MEMORY_TIMEOUT = 1.0  # Remember obstacles for 1 second


# ------------- Helper Functions -------------

def log_navigation_data(steering, state, yaw, front_dist, left_dist, right_dist, red_blocks, green_blocks):
    """Log comprehensive navigation data"""
    timestamp = time.time()
    
    # Count and summarize blocks
    red_count = len(red_blocks)
    green_count = len(green_blocks)
    
    red_areas = [w * h for x, y, w, h in red_blocks] if red_blocks else []
    green_areas = [w * h for x, y, w, h in green_blocks] if green_blocks else []
    
    log_entry = (
        f"T:{timestamp:.3f} | Steering:{steering:.1f} | State:{state} | "
        f"Yaw:{yaw:.1f} | Dist(F:{front_dist},L:{left_dist},R:{right_dist}) | "
        f"RED_blocks:{red_count}{red_areas} | GREEN_blocks:{green_count}{green_areas}"
    )
    
    logger.info(log_entry)


def led_blink(duration=1.5):
    """Blink status LED"""
    GPIO.output(LED_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(LED_PIN, GPIO.LOW)


def calibrate_imu():
    """Calibrate IMU by setting current heading as 0° reference"""
    global initial_yaw
    print("Calibrating IMU...")
    time.sleep(1)  # Let IMU settle
    raw_yaw = sensor.euler[0]
    if raw_yaw is not None:
        initial_yaw = raw_yaw
        print(f"IMU calibrated. Initial heading: {initial_yaw:.1f}° set as 0° reference")
    else:
        print("Warning: Could not read IMU for calibration")
        initial_yaw = 0


def read_yaw():
    """Read relative yaw from BNO055"""
    global initial_yaw
    raw_yaw = sensor.euler[0]
    if raw_yaw is not None:
        relative_yaw = raw_yaw - initial_yaw
        # Normalize to [-180, 180]
        if relative_yaw > 180:
            relative_yaw -= 360
        elif relative_yaw < -180:
            relative_yaw += 360
        return relative_yaw
    return 0.0


def read_from_esp32():
    """Read ultrasonic distances from ESP32"""
    global front_dist, left_dist, right_dist
    ser.write(b"GET\n")
    try:
        line = ser.readline().decode().strip()
        if line.startswith("DIST"):
            _, f, l, r = line.split(",")
            front_dist = int(float(f))
            left_dist = int(float(l))
            right_dist = int(float(r))
    except:
        pass


def send_to_esp32(speed, steering):
    """Send motor speed + steering angle to ESP32"""
    command = f"{int(speed)},{int(steering)}\n"
    ser.write(command.encode())


def update_obstacle_memory(red_blocks, green_blocks):
    """Update obstacle memory and return enhanced block lists"""
    global last_seen_obstacles
    
    current_time = time.time()
    
    # Update memory with current detections
    if red_blocks or green_blocks:
        last_seen_obstacles["red"] = red_blocks
        last_seen_obstacles["green"] = green_blocks
        last_seen_obstacles["timestamp"] = current_time
        logger.debug(f"Obstacle memory updated: {len(red_blocks)} red, {len(green_blocks)} green")
    else:
        # Check if we should use remembered obstacles
        time_since_last_detection = current_time - last_seen_obstacles["timestamp"]
        if time_since_last_detection < OBSTACLE_MEMORY_TIMEOUT:
            # Use remembered obstacles if recent enough
            red_blocks = last_seen_obstacles["red"]
            green_blocks = last_seen_obstacles["green"]
            logger.debug(f"Using remembered obstacles: {len(red_blocks)} red, {len(green_blocks)} green (age: {time_since_last_detection:.2f}s)")
        else:
            # Clear old memory
            last_seen_obstacles = {"red": [], "green": [], "timestamp": 0}
    
    return red_blocks, green_blocks


def detect_obstacles(frame):
    """Detect red and green obstacles and return their positions"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for red and green
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    # Find contours
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    red_blocks = []
    green_blocks = []
    min_area = 500  # Reduced minimum area for better detection of distant blocks
    
    # Process red contours
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            if h > 15 and w > 15:  # More permissive size requirements
                red_blocks.append((x, y, w, h))
                logger.debug(f"RED block detected: x={x}, y={y}, w={w}, h={h}, area={area}")
    
    # Process green contours
    for contour in green_contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            if h > 15 and w > 15:  # More permissive size requirements
                green_blocks.append((x, y, w, h))
                logger.debug(f"GREEN block detected: x={x}, y={y}, w={w}, h={h}, area={area}")
    
    # Log detection summary
    if red_blocks or green_blocks:
        logger.debug(f"Detection summary: {len(red_blocks)} red blocks, {len(green_blocks)} green blocks")
    
    return red_blocks, green_blocks


def get_region(x):
    """Determine which region the x-coordinate falls into"""
    if LEFT_REGION[0] <= x < LEFT_REGION[1]:
        return "LEFT"
    elif CENTER_REGION[0] <= x < CENTER_REGION[1]:
        return "CENTER"
    elif RIGHT_REGION[0] <= x < RIGHT_REGION[1]:
        return "RIGHT"
    return "UNKNOWN"


def calculate_turn_intensity(block_area):
    """Calculate turn intensity based on block size (area)"""
    # Larger blocks = closer = need sharper turns
    # Smaller blocks = farther = need gentler turns
    
    if block_area <= SIZE_THRESHOLD_SMALL:
        # Small block (far away) - gentle turn
        intensity = MIN_TURN
        size_category = "SMALL"
    elif block_area >= SIZE_THRESHOLD_LARGE:
        # Large block (close) - sharp turn
        intensity = MAX_TURN
        size_category = "LARGE"
    else:
        # Medium block - interpolate between min and max
        ratio = (block_area - SIZE_THRESHOLD_SMALL) / (SIZE_THRESHOLD_LARGE - SIZE_THRESHOLD_SMALL)
        intensity = MIN_TURN + (MAX_TURN - MIN_TURN) * ratio
        size_category = "MEDIUM"
    
    print(f"Block area: {block_area} -> {size_category} -> Turn intensity: {intensity:.1f}°")
    logger.debug(f"Turn calculation: area={block_area} -> {size_category} -> intensity={intensity:.1f}°")
    return intensity, size_category


def calculate_steering(blocks, color):
    """Calculate steering based on block positions, size, and WRO rules"""
    if not blocks:
        return 0, "NO_OBSTACLE"
    
    # Find the largest (closest) block
    largest_block = max(blocks, key=lambda b: b[2] * b[3])  # w * h
    x, y, w, h = largest_block
    block_center_x = x + w // 2
    block_area = w * h
    region = get_region(block_center_x)
    
    # Calculate turn intensity based on block size
    base_intensity, size_category = calculate_turn_intensity(block_area)
    
    steering_adjustment = 0
    turn_type = ""
    
    if color == "GREEN":
        # GREEN blocks: Must turn LEFT around them
        if region == "LEFT":
            # Green block on left - LEFT turn (good position)
            steering_adjustment = -base_intensity
            turn_type = f"{size_category}_LEFT"
        elif region == "CENTER":
            # Green block in center - MEDIUM LEFT turn (increase intensity slightly)
            steering_adjustment = -(base_intensity * 1.2)  # 20% more aggressive
            turn_type = f"{size_category}_CENTER_LEFT"
        elif region == "RIGHT":
            # Green block on right - GENTLE LEFT turn (need to get to left side)
            steering_adjustment = -(base_intensity * 0.8)  # 20% less aggressive
            turn_type = f"{size_category}_SLIGHT_LEFT"
            
    elif color == "RED":
        # RED blocks: Must turn RIGHT around them
        if region == "LEFT":
            # Red block on left - GENTLE RIGHT turn (need to get to right side)
            steering_adjustment = base_intensity * 0.8  # 20% less aggressive
            turn_type = f"{size_category}_SLIGHT_RIGHT"
        elif region == "CENTER":
            # Red block in center - MEDIUM RIGHT turn (increase intensity slightly)
            steering_adjustment = base_intensity * 1.2  # 20% more aggressive
            turn_type = f"{size_category}_CENTER_RIGHT"
        elif region == "RIGHT":
            # Red block on right - RIGHT turn (good position)
            steering_adjustment = base_intensity
            turn_type = f"{size_category}_RIGHT"
    
    # Ensure steering adjustment doesn't exceed limits
    max_adjustment = 55  # Maximum steering adjustment
    steering_adjustment = max(-max_adjustment, min(max_adjustment, steering_adjustment))
    
    print(f"{color} block in {region} region at ({block_center_x}, {y}) -> {turn_type} (area={block_area})")
    logger.debug(f"Steering calculation: {color} block in {region} at ({block_center_x},{y}) -> {turn_type} | area={block_area} | adjustment={steering_adjustment:.1f}")
    return steering_adjustment, turn_type


def decide_steering():
    """Main decision logic for steering"""
    global yaw, last_turn_direction, recovery_mode
    
    # Read current yaw
    yaw = read_yaw()
    
    # Get obstacle detections
    frame = picam2.capture_array()
    red_blocks, green_blocks = detect_obstacles(frame)
    
    # Update obstacle memory and get enhanced detection
    red_blocks, green_blocks = update_obstacle_memory(red_blocks, green_blocks)
    
    steering = SERVO_STRAIGHT
    state = "STRAIGHT"
    
    # Priority: Handle obstacles based on proximity (size), not color preference
    closest_block = None
    closest_color = None
    closest_area = 0
    
    # Log all detected blocks for debugging
    if red_blocks or green_blocks:
        all_red_areas = [w * h for x, y, w, h in red_blocks]
        all_green_areas = [w * h for x, y, w, h in green_blocks]
        logger.debug(f"All detections: RED blocks={len(red_blocks)}{all_red_areas}, GREEN blocks={len(green_blocks)}{all_green_areas}")
    
    # Find the closest block among all detected blocks
    if red_blocks:
        largest_red = max(red_blocks, key=lambda b: b[2] * b[3])
        red_area = largest_red[2] * largest_red[3]
        if red_area > closest_area:
            closest_block = largest_red
            closest_color = "RED"
            closest_area = red_area
            logger.debug(f"Largest RED block: area={red_area}")
    
    if green_blocks:
        largest_green = max(green_blocks, key=lambda b: b[2] * b[3])
        green_area = largest_green[2] * largest_green[3]
        if green_area > closest_area:
            closest_block = largest_green
            closest_color = "GREEN"
            closest_area = green_area
            logger.debug(f"Largest GREEN block: area={green_area}")
        elif green_blocks:  # Log even if not closest
            logger.debug(f"GREEN block present but smaller: area={green_area} vs RED area={closest_area}")
    
    # Handle the closest (most urgent) obstacle
    if closest_block is not None:
        # Create a list with just the closest block for calculate_steering
        closest_blocks = [closest_block]
        adjustment, turn_type = calculate_steering(closest_blocks, closest_color)
        steering = SERVO_STRAIGHT + adjustment
        state = f"{closest_color}_{turn_type}"
        
        if closest_color == "GREEN":
            last_turn_direction = "LEFT"
        else:  # RED
            last_turn_direction = "RIGHT"
            
        recovery_mode = False  # Reset recovery when actively avoiding
        
        logger.info(f"PRIORITY: {closest_color} block (area={closest_area}) is closest - handling first")
        print(f"PRIORITY: {closest_color} block (area={closest_area}) is closest - handling first")
        
    else:
        # No obstacles detected - apply recovery logic
        logger.debug(f"No obstacles detected - red_blocks={len(red_blocks)}, green_blocks={len(green_blocks)}")
        
        recovery_steering = 0
        
        # If we just finished avoiding an obstacle, apply counter-steering
        if abs(yaw) > 5:  # Robot is not straight
            if last_turn_direction == "RIGHT" and yaw > 5:
                # After right turn, robot is angled right - steer LEFT to center
                recovery_steering = -15  # Counter-steer left
                state = "RECOVERY_LEFT_AFTER_RIGHT"
                recovery_mode = True
                
            elif last_turn_direction == "LEFT" and yaw < -0:
                # After left turn, robot is angled left - steer RIGHT to center
                recovery_steering = 15   # Counter-steer right
                state = "RECOVERY_RIGHT_AFTER_LEFT"
                recovery_mode = True
                
        # Apply yaw correction to get back to 0°
        error = 0 - yaw  # Target is 0° (straight from start)
        yaw_correction = error * 0.8  # Stronger correction for faster recovery
        
        # Combine recovery steering with yaw correction
        total_correction = recovery_steering + yaw_correction
        steering = SERVO_STRAIGHT + total_correction
        
        if not recovery_mode:
            state = "STRAIGHT_YAW_CORRECTION"
            
        print(f"No obstacles - Yaw: {yaw:.1f}°, Recovery: {recovery_steering}, YawCorr: {yaw_correction:.1f}, Total: {total_correction:.1f}")
        logger.debug(f"Recovery mode: yaw={yaw:.1f}°, recovery_steering={recovery_steering}, yaw_correction={yaw_correction:.1f}, total={total_correction:.1f}")
        
        # Clear last turn direction if we're back to straight
        if abs(yaw) < 3:
            last_turn_direction = ""
            recovery_mode = False
    
    # Limit steering to valid range
    steering = min(max(SERVO_LEFT, steering), SERVO_RIGHT)
    
    # Log comprehensive navigation data
    log_navigation_data(steering, state, yaw, front_dist, left_dist, right_dist, red_blocks, green_blocks)
    
    # Draw debug visualization
    frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state)
    cv2.imshow("Region-Based Navigation", frame_debug)
    
    return steering, state


def draw_debug(frame, red_blocks, green_blocks, steering, state):
    """Draw debug visualization with regions and obstacles"""
    debug_frame = frame.copy()
    
    # Draw region dividers
    cv2.line(debug_frame, (FRAME_W // 3, 0), (FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    cv2.line(debug_frame, (2 * FRAME_W // 3, 0), (2 * FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    
    # Label regions
    cv2.putText(debug_frame, "LEFT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "CENTER", (FRAME_W // 2 - 50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "RIGHT", (FRAME_W - 150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Draw red blocks
    for x, y, w, h in red_blocks:
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        cv2.putText(debug_frame, f"RED-{region}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Draw green blocks
    for x, y, w, h in green_blocks:
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        cv2.putText(debug_frame, f"GREEN-{region}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
def draw_debug(frame, red_blocks, green_blocks, steering, state):
    """Draw debug visualization with regions and obstacles"""
    global video_writer
    
    debug_frame = frame.copy()
    
    # Draw region dividers
    cv2.line(debug_frame, (FRAME_W // 3, 0), (FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    cv2.line(debug_frame, (2 * FRAME_W // 3, 0), (2 * FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    
    # Label regions
    cv2.putText(debug_frame, "LEFT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "CENTER", (FRAME_W // 2 - 50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "RIGHT", (FRAME_W - 150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Draw red blocks
    for x, y, w, h in red_blocks:
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        cv2.putText(debug_frame, f"RED-{region}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Draw green blocks
    for x, y, w, h in green_blocks:
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        cv2.putText(debug_frame, f"GREEN-{region}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Add timestamp to frame
    timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Include milliseconds
    cv2.putText(debug_frame, timestamp_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Draw status info
    cv2.putText(debug_frame, f"Steering: {steering:.1f}", (10, FRAME_H - 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"State: {state}", (10, FRAME_H - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Yaw: {yaw:.1f}°", (10, FRAME_H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Last Turn: {last_turn_direction}", (10, FRAME_H - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Recovery: {recovery_mode}", (10, FRAME_H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # Record video frame if enabled
    if RECORD_VIDEO and video_writer is not None:
        # Convert RGB to BGR for video writer
        bgr_frame = cv2.cvtColor(debug_frame, cv2.COLOR_RGB2BGR)
        video_writer.write(bgr_frame)
    
    return debug_frame


def main():
    """Main navigation loop"""
    global yaw, video_writer
    
    try:
        # Initialize camera
        picam2.configure(config)
        picam2.start()
        
        # Initialize video recording
        if RECORD_VIDEO:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_writer = cv2.VideoWriter(VIDEO_FILE, fourcc, 20.0, (FRAME_W, FRAME_H))
            logger.info(f"Video recording initialized: {VIDEO_FILE}")
        
        # LED indicator
        led_blink()
        
        logger.info("System initialized successfully")
        logger.info(f"Log file: {LOG_FILE}")
        logger.info(f"Video file: {VIDEO_FILE}")
        logger.info("Waiting for button press...")
        
        print("Waiting for button press...")
        while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            time.sleep(0.1)
        
        print("Button pressed! Starting system...")
        logger.info("Button pressed! Starting navigation system...")
        
        # Calibrate IMU
        calibrate_imu()
        logger.info(f"IMU calibrated with initial heading: {initial_yaw:.1f}°")
        
        # Send START to ESP32
        ser.write(b"START\n")
        logger.info("START command sent to ESP32")
        
        print("Starting region-based navigation...")
        print(f"Regions: LEFT(0-{LEFT_REGION[1]}), CENTER({CENTER_REGION[0]}-{CENTER_REGION[1]}), RIGHT({RIGHT_REGION[0]}-{FRAME_W})")
        
        logger.info("Starting region-based navigation...")
        logger.info(f"Frame regions: LEFT(0-{LEFT_REGION[1]}), CENTER({CENTER_REGION[0]}-{CENTER_REGION[1]}), RIGHT({RIGHT_REGION[0]}-{FRAME_W})")
        logger.info(f"Turn intensity config: MIN={MIN_TURN}°, MAX={MAX_TURN}°, SMALL_THRESHOLD={SIZE_THRESHOLD_SMALL}, LARGE_THRESHOLD={SIZE_THRESHOLD_LARGE}")
        
        loop_count = 0
        start_time = time.time()
        
        while True:
            loop_count += 1
            
            # Read sensor data
            read_from_esp32()
            
            # Decide steering based on region logic
            steering, state = decide_steering()
            
            # Send commands to ESP32
            send_to_esp32(BASE_SPEED, steering)
            
            # Console output (reduced frequency for readability)
            if loop_count % 5 == 0:  # Print every 5th loop
                print(f"Loop {loop_count} | Yaw: {yaw:.1f}° | Steering: {steering:.1f} | State: {state}")
                print(f"Distances - Front: {front_dist}, Left: {left_dist}, Right: {right_dist}")
                print("-" * 60)
            
            # Exit conditions
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info("User pressed 'q' - exiting navigation loop")
                break
                
            time.sleep(0.05)  # Small delay for stability
        
        # Calculate session statistics
        end_time = time.time()
        total_runtime = end_time - start_time
        logger.info(f"Navigation session completed: {loop_count} loops in {total_runtime:.2f} seconds ({loop_count/total_runtime:.1f} Hz)")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
        logger.info("Navigation interrupted by user (Ctrl+C)")
        
    except Exception as e:
        print(f"Error: {e}")
        logger.error(f"Navigation error: {e}", exc_info=True)
        
    finally:
        # Cleanup
        print("Cleaning up...")
        logger.info("Starting cleanup process...")
        
        send_to_esp32(0, SERVO_STRAIGHT)  # Stop robot
        logger.info("Robot stopped - sent stop command to ESP32")
        
        if 'picam2' in globals():
            picam2.stop()
            logger.info("Camera stopped")
        
        if video_writer is not None:
            video_writer.release()
            logger.info(f"Video recording saved: {VIDEO_FILE}")
        
        cv2.destroyAllWindows()
        led_blink(1.5)  # Final LED blink
        GPIO.cleanup()
        
        logger.info("System shutdown complete")
        print("System shutdown complete")


if _name_ == "_main_":
    main()
