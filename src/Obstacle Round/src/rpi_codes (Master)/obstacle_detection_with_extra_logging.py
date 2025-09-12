# -*- coding: utf-8 -*-
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
import threading
from collections import deque

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
logger = logging.getLogger(__name__)

# Video recording setup
video_writer = None
RECORD_VIDEO = True  # Set to False to disable video recording

# --------------------------------------------

# System setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_PIN, GPIO.OUT)

# Init Serial to ESP32
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)  # Non-blocking with very short timeout

# Init Camera
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (FRAME_W, FRAME_H), "format": 'RGB888'})

# Init BNO055 IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Global variables
initial_yaw = 0
yaw = 0
straight_heading_reference = 0.0  # Current reference for "straight ahead" - updates after turns
front_dist, left_dist, right_dist = 100, 35, 35
last_turn_direction = ""  # Track last turn to help with recovery
recovery_mode = False     # Flag for recovery steering

# Obstacle memory system to handle brief detection losses
last_seen_obstacles = {"red": [], "green": [], "timestamp": 0}
OBSTACLE_MEMORY_TIMEOUT = 0.0  # Remember obstacles for 1 second

# Simple delay system for post-obstacle straightening
post_obstacle_delay_active = False  # Flag to indicate delay is active
post_obstacle_delay_start = 0       # When the delay started
POST_OBSTACLE_DELAY_DURATION = 1.0  # Delay duration in seconds

# Turn detection and execution variables
obstacle_count = 0  # Counter for obstacles encountered in current segment
max_obstacles_per_segment = 6  # Very conservative - Maximum obstacles before turn is required
in_turn_mode = False  # Flag indicating robot is currently turning
turn_start_time = 0  # Timestamp when turn started
target_yaw = 0  # Target heading after turn completion
TURN_TIMEOUT = 8.0  # Increased timeout for multi-phase turn (seconds)

# CRITICAL: Turn sequence control - ensures alternating pattern - BLOCKED BY DEFAULT
turn_just_completed = True   # Flag to prevent consecutive turns - DEFAULT BLOCKED
obstacle_deflection_after_turn = False  # Flag to track if obstacle deflection happened after last turn

# Turn completion counter - Stop after 12 turns
total_turns_completed = 0  # Counter for total completed turns
MAX_TURNS_BEFORE_STOP = 12  # Maximum turns before automatic stop

# Multi-phase turn variables
turn_phase = "NONE"  # Phases: "FORWARD", "REVERSE_TURN", "BACKWARD_POST_ARC", "COMPLETE"
turn_type = "MULTI_PHASE"  # Turn type: "MULTI_PHASE" or "ARC"
forward_phase_duration = 0.8  # Time to go forward before reversing (seconds)
reverse_turn_duration = 3.0  # Time to reverse while turning (seconds)
backward_post_arc_duration = 0.8  # Time to go backward after arc turn completion (seconds)
backward_post_arc_start = 0  # Timestamp when backward phase started

# Turn detection thresholds
FRONT_DISTANCE_TURN_THRESHOLD = 60  # Front distance to trigger turn consideration
SIDE_DISTANCE_THRESHOLD = 90  # Side distance threshold for turn direction

# Serial communication variables with improved management
serial_lock = threading.Lock()
command_queue = deque(maxlen=3)  # Small queue to prevent command piling up
last_command_time = 0
last_get_request_time = 0
COMMAND_INTERVAL = 0.05  # Minimum time between motor commands (20Hz max)
GET_REQUEST_INTERVAL = 0.1  # Minimum time between GET requests (10Hz max)
sensor_data_fresh = False
last_sensor_update = 0


# ------------- Improved Serial Communication Functions -------------

def serial_communication_handler():
    """Background thread to handle serial communication without blocking"""
    global front_dist, left_dist, right_dist, sensor_data_fresh, last_sensor_update
    
    while True:
        try:
            with serial_lock:
                # Check for incoming data
                if ser.in_waiting > 0:
                    line = ser.readline().decode().strip()
                    if line.startswith("DIST"):
                        parts = line.split(",")
                        if len(parts) == 4:
                            _, f, l, r = parts
                            front_dist = int(float(f))
                            left_dist = int(float(l))
                            right_dist = int(float(r))
                            sensor_data_fresh = True
                            last_sensor_update = time.time()
                            logger.debug(f"Distance update: F={front_dist}, L={left_dist}, R={right_dist}")
                
                # Process command queue (non-blocking)
                current_time = time.time()
                if (command_queue and 
                    current_time - last_command_time > COMMAND_INTERVAL):
                    
                    command = command_queue.popleft()
                    ser.write(command.encode())
                    ser.flush()
                    globals()['last_command_time'] = current_time
                    logger.debug(f"Sent command: {command.strip()}")
                    
        except Exception as e:
            logger.warning(f"Serial communication error: {e}")
            time.sleep(0.01)  # Brief pause on error
        
        time.sleep(0.005)  # Small delay to prevent excessive CPU usage


def send_to_esp32_improved(speed, steering):
    """Improved non-blocking command sending with queue management"""
    global last_command_time
    
    # Create command string
    command = f"{int(speed)},{int(steering)}\n"
    
    # Add to queue only if it's different from the last command or queue is empty
    with serial_lock:
        if not command_queue or command != list(command_queue)[-1]:
            # If queue is full, remove oldest command to make space
            if len(command_queue) >= command_queue.maxlen:
                removed = command_queue.popleft()
                logger.debug(f"Queue full, removed: {removed.strip()}")
            
            command_queue.append(command)
            logger.debug(f"Command queued: {command.strip()} (Queue size: {len(command_queue)})")


def request_sensor_data():
    """Request sensor data from ESP32 with rate limiting"""
    global last_get_request_time
    
    current_time = time.time()
    if current_time - last_get_request_time > GET_REQUEST_INTERVAL:
        with serial_lock:
            try:
                ser.write(b"GET\n")
                ser.flush()
                last_get_request_time = current_time
                logger.debug("GET request sent")
            except Exception as e:
                logger.warning(f"Failed to send GET request: {e}")


def get_fresh_sensor_data():
    """Get sensor data and return freshness status"""
    global sensor_data_fresh
    
    # Request new data if needed
    request_sensor_data()
    
    # Check if we have fresh data
    data_age = time.time() - last_sensor_update
    is_fresh = sensor_data_fresh and data_age < 0.5  # Data is fresh if less than 500ms old
    
    if sensor_data_fresh:
        sensor_data_fresh = False  # Mark as consumed
    
    return front_dist, left_dist, right_dist, is_fresh, data_age


# ------------- Turn Detection and Execution Functions -------------

def should_initiate_turn(front_d, left_d, right_d, red_blocks, green_blocks):
    """Determine if a turn should be initiated based on conditions - VERY CONSERVATIVE"""
    global obstacle_count, turn_just_completed, obstacle_deflection_after_turn
    
    # CRITICAL CONSTRAINT: Turn is ONLY allowed if there are absolutely NO obstacles to deflect
    no_obstacles_at_all = len(red_blocks) == 0 and len(green_blocks) == 0
    
    # Only proceed if there are no obstacles whatsoever
    if not no_obstacles_at_all:
        logger.debug(f"Turn BLOCKED: Obstacles still present - RED:{len(red_blocks)}, GREEN:{len(green_blocks)}")
        return None
    
    # CRITICAL SEQUENCE CONSTRAINT: Prevent consecutive turns
    # Turn is only allowed if:
    # 1. No turn was just completed, OR
    # 2. A turn was completed but obstacle deflection happened after it
    if turn_just_completed and not obstacle_deflection_after_turn:
        logger.debug(f"Turn BLOCKED: Turn completed but no active obstacle deflection occurred yet.")
        logger.debug(f"Sequence state: turn_just_completed={turn_just_completed}, obstacle_deflection_after_turn={obstacle_deflection_after_turn}")
        print("SEQUENCE CONSTRAINT: Must perform ACTIVE obstacle deflection before next turn!")
        return None
    elif turn_just_completed and obstacle_deflection_after_turn:
        logger.debug(f"Turn ALLOWED: Turn completed and obstacle deflection occurred.")
        print("SEQUENCE OK: Active obstacle deflection completed - turn now allowed")
    else:
        logger.debug(f"Turn ALLOWED: Initial state - no previous turn to constrain.")
        print("SEQUENCE OK: No previous turn constraint")
    
    # Now check other conditions only if no obstacles exist
    front_blocked = front_d < FRONT_DISTANCE_TURN_THRESHOLD
    turn_right_condition = left_d < SIDE_DISTANCE_THRESHOLD and right_d > SIDE_DISTANCE_THRESHOLD
    turn_left_condition = left_d > SIDE_DISTANCE_THRESHOLD and right_d < SIDE_DISTANCE_THRESHOLD
    max_obstacles_reached = obstacle_count >= max_obstacles_per_segment
    
    logger.debug(f"Turn evaluation: NO_OBSTACLES={no_obstacles_at_all}, front_blocked={front_blocked}, "
                f"obstacles_handled={obstacle_count}/{max_obstacles_per_segment}, "
                f"distances(F:{front_d}, L:{left_d}, R:{right_d})")
    
    # STRICT CONDITIONS: Must have NO obstacles AND either:
    # 1. Front blocked with clear turn direction, OR
    # 2. Maximum obstacles handled with clear turn direction
    
    clear_turn_direction = turn_right_condition or turn_left_condition
    
    # Condition 1: Front blocked and clear turn direction
    if front_blocked and clear_turn_direction:
        logger.info(f"TURN INITIATED: Front blocked ({front_d}) with clear path")
        if turn_right_condition:
            return "RIGHT"
        elif turn_left_condition:
            return "LEFT"
    
    # Condition 2: Max obstacles reached and clear turn direction  
    if max_obstacles_reached and clear_turn_direction:
        logger.info(f"TURN INITIATED: Max obstacles ({obstacle_count}) reached with clear path")
        if turn_right_condition:
            return "RIGHT"
        elif turn_left_condition:
            return "LEFT"
    
    # If we reach here, conditions not met for turn
    if not clear_turn_direction:
        logger.debug(f"Turn blocked: No clear direction - L:{left_d}, R:{right_d}")
    
    return None


def execute_turn(direction):
    """Execute adaptive turn based on ultrasonic readings: multi-phase or arc turn"""
    global in_turn_mode, turn_start_time, target_yaw, yaw, obstacle_count, turn_phase, turn_type
    global turn_just_completed, obstacle_deflection_after_turn, backward_post_arc_start
    
    if in_turn_mode:
        return False  # Already in turn
    
    # Get current sensor readings to decide turn type
    f_dist, l_dist, r_dist, data_fresh, data_age = get_fresh_sensor_data()
    
    # Adaptive turn selection based on opposite side ultrasonic reading
    if direction == "RIGHT":
        # For RIGHT turn, check LEFT ultrasonic reading
        opposite_reading = l_dist
        opposite_side = "LEFT"
    else:  # LEFT turn
        # For LEFT turn, check RIGHT ultrasonic reading  
        opposite_reading = r_dist
        opposite_side = "RIGHT"
    
    # Decide turn type based on opposite side reading
    if opposite_reading > 40:
        turn_type = "MULTI_PHASE"
        turn_description = "multi-phase (forward + reverse)"
    else:
        turn_type = "ARC"
        turn_description = "arc turn"
    
    print(f"INITIATING {direction} TURN: {opposite_side} ultrasonic = {opposite_reading}cm → {turn_description}")
    logger.info(f"Starting {direction} turn - {opposite_side} reading: {opposite_reading}cm → {turn_type} turn")
    
    # SEQUENCE CONTROL: Reset flags when starting a new turn
    turn_just_completed = False  # Clear the flag since we're starting a new turn
    obstacle_deflection_after_turn = False  # Reset for next cycle
    
    in_turn_mode = True
    turn_start_time = time.time()
    
    # Calculate target yaw - NO NORMALIZATION - let it accumulate naturally
    current_yaw = read_yaw()
    
    print(f"=== UNBOUNDED TURN CALCULATION ===")
    print(f"Current Yaw: {current_yaw:.1f} deg")
    print(f"Turn Direction: {direction}")
    
    # Calculate target WITHOUT any normalization
    if direction == "RIGHT":
        target_yaw = current_yaw + 90.0
        print(f"RIGHT turn: {current_yaw:.1f} deg + 90.0 deg = {target_yaw:.1f} deg")
    else:  # LEFT
        target_yaw = current_yaw - 90.0
        print(f"LEFT turn: {current_yaw:.1f} deg - 90.0 deg = {target_yaw:.1f} deg")
    
    # NO NORMALIZATION - keep the natural target
    # This allows targets like 270 deg, -90 deg, 450 deg etc.
    
    print(f"Final Target: {target_yaw:.1f} deg (NO normalization)")
    print(f"Expected angular change: 90.0 deg")
    print(f"=== END DEBUG ===")
    
    # Simple validation - should always be exactly 90 deg difference
    expected_diff = 90.0
    actual_diff = abs(target_yaw - current_yaw)
    
    if abs(actual_diff - expected_diff) > 1.0:
        print(f"ERROR: Turn calculation wrong! Expected 90 deg, got {actual_diff:.1f} deg")
        logger.error(f"Turn calculation error: {direction} from {current_yaw:.1f} deg should be 90 deg, got {actual_diff:.1f} deg")
    else:
        print(f"CORRECT: Turn calculation validated - {actual_diff:.1f} deg difference")
    
    print(f"TARGET YAW: Current={current_yaw:.1f} deg, Target={target_yaw:.1f} deg")
    logger.info(f"Target yaw calculation: Current={current_yaw:.1f}, Direction={direction}, Final target={target_yaw:.1f}")
    
    # Set initial phase based on turn type
    if turn_type == "MULTI_PHASE":
        turn_phase = "FORWARD"  # Start with forward phase
        print(f"MULTI-PHASE: Going forward for {forward_phase_duration}s before reverse turn")
        logger.info(f"Multi-phase turn: Current yaw={current_yaw:.1f}, Target yaw={target_yaw:.1f}")
    else:  # ARC turn
        turn_phase = "ARC"  # Single phase arc turn
        print(f"ARC TURN: Direct turning maneuver")
        logger.info(f"Arc turn: Current yaw={current_yaw:.1f}, Target yaw={target_yaw:.1f}")
    
    # Reset obstacle counter for new segment
    obstacle_count = 0
    
    return True


def update_turn_progress():
    """Update turn progress and return speed/steering commands for both multi-phase and arc turns"""
    global in_turn_mode, target_yaw, yaw, turn_phase, turn_type, backward_post_arc_start
    
    if not in_turn_mode:
        return None, None
    
    current_time = time.time()
    turn_duration = current_time - turn_start_time
    
    # Check for timeout
    if turn_duration > TURN_TIMEOUT:
        print("WARNING: Turn timeout - completing turn")
        logger.warning("Turn timeout reached")
        complete_turn()
        return BASE_SPEED, SERVO_STRAIGHT
    
    # Get current yaw first
    current_yaw = read_yaw()
    
    # Handle based on turn type
    if turn_type == "MULTI_PHASE":
        # MULTI-PHASE TURN LOGIC (existing logic)
        
        # PHASE 1: FORWARD - Move forward briefly before starting turn
        if turn_phase == "FORWARD":
            if turn_duration < forward_phase_duration:
                print(f"MULTI-PHASE FORWARD: ({turn_duration:.1f}/{forward_phase_duration:.1f}s)")
                return BASE_SPEED, SERVO_STRAIGHT  # Go straight forward
            else:
                # Transition to reverse turn phase
                turn_phase = "REVERSE_TURN"
                print(f"MULTI-PHASE REVERSE: Starting reverse turn phase (target: {target_yaw:.1f} deg)")
                logger.info(f"Transitioning to reverse turn phase - target yaw: {target_yaw:.1f}")
        
        # PHASE 2: REVERSE_TURN - Reverse while turning to reach target yaw
        if turn_phase == "REVERSE_TURN":
            # Calculate error to target with debugging
            error = debug_angle_calculation(current_yaw, target_yaw)
            
            # Enhanced debugging for stuck yaw detection
            if not hasattr(update_turn_progress, 'last_yaw_debug'):
                update_turn_progress.last_yaw_debug = current_yaw
                update_turn_progress.same_yaw_count = 0
            else:
                if abs(current_yaw - update_turn_progress.last_yaw_debug) < 0.5:  # Essentially same reading
                    update_turn_progress.same_yaw_count += 1
                    if update_turn_progress.same_yaw_count >= 10:  # Been stuck for ~0.5s
                        print(f"YAW STUCK WARNING: Yaw has been {current_yaw:.1f} deg for {update_turn_progress.same_yaw_count} consecutive readings!")
                        logger.warning(f"Yaw stuck at {current_yaw:.1f} deg for {update_turn_progress.same_yaw_count} readings - IMU may be filtering valid changes")
                        # Force IMU reset if we have the function
                        if hasattr(read_yaw, 'stuck_count'):
                            print(f"FORCING IMU UPDATE: Resetting stuck counter from {read_yaw.stuck_count}")
                            read_yaw.stuck_count = 0
                else:
                    update_turn_progress.same_yaw_count = 0
                    if abs(current_yaw - update_turn_progress.last_yaw_debug) > 2.0:  # Significant change
                        print(f"YAW PROGRESSING: {update_turn_progress.last_yaw_debug:.1f} deg -> {current_yaw:.1f} deg (change: {current_yaw - update_turn_progress.last_yaw_debug:.1f} deg)")
                update_turn_progress.last_yaw_debug = current_yaw
            
            # Check if turn is complete
            if abs(error) < 5.0:  # Within 5 degrees of target
                print(f"SUCCESS: Multi-phase turn completed! Final error: {error:.1f} deg")
                logger.info(f"Multi-phase turn completed - final error: {error:.1f} degrees")
                complete_turn()
                return BASE_SPEED, SERVO_STRAIGHT
            
            # Determine turn direction and calculate steering
            reverse_speed = -BASE_SPEED  # Reverse speed (negative, but not too fast)
            
            # CRITICAL FIX: When reversing, steering direction must be inverted
            # Use the error to determine steering direction (handles wraparound correctly)
            if error > 0:  # Need to turn right to reach target
                steering = SERVO_LEFT  # LEFT steering to achieve RIGHT turn in reverse
                turn_desc = "RIGHT turn (LEFT steering in reverse)"
            else:  # Need to turn left to reach target
                steering = SERVO_RIGHT  # RIGHT steering to achieve LEFT turn in reverse
                turn_desc = "LEFT turn (RIGHT steering in reverse)"
            
            # Log progress more frequently during critical moments
            if int(turn_duration * 4) % 3 == 0:  # Log more frequently
                logger.info(f"MULTI-PHASE PROGRESS: {turn_desc} | Current={current_yaw:.1f}deg | Target={target_yaw:.1f}deg | Error={error:.1f}deg")
                print(f"MULTI-PHASE: Current={current_yaw:.1f}deg, Target={target_yaw:.1f}deg, Error={error:.1f}deg, Duration={turn_duration:.1f}s")
            
            return reverse_speed, steering
    
    elif turn_type == "ARC":
        # ARC TURN LOGIC - NO normalization, work with raw error
        
        # Calculate error to target WITHOUT normalization
        error = target_yaw - current_yaw
        # NO BOUNDARY CORRECTION - work with raw difference
        
        # Check if turn is complete (lenient threshold)
        if abs(error) < 5.0:  # Within 5 degrees of target
            print(f"SUCCESS: Arc turn completed! Error: {error:.1f} deg - Starting backward phase")
            logger.info(f"Arc turn completed successfully - final error: {error:.1f} deg - transitioning to backward phase")
            
            # Transition to backward phase instead of completing immediately
            turn_phase = "BACKWARD_POST_ARC"
            backward_post_arc_start = time.time()
            print(f"ARC BACKWARD PHASE: Moving backward for {backward_post_arc_duration}s")
            logger.info(f"Arc turn backward phase started for {backward_post_arc_duration}s")
            
            # Return backward movement command
            return -BASE_SPEED + 20, SERVO_STRAIGHT  # Slow backward movement
        
        # Calculate steering for arc turn (forward speed with steering)
        speed = BASE_SPEED  # Forward speed for arc turn
        
        # Calculate steering based on error
        if error > 0:  # Need to turn right
            steering = SERVO_RIGHT  # Right turn
            turn_desc = "Arc RIGHT turn"
        else:  # Need to turn left
            steering = SERVO_LEFT  # Left turn  
            turn_desc = "Arc LEFT turn"
        
        # Ensure steering is within limits
        steering = max(SERVO_LEFT, min(SERVO_RIGHT, steering))
        
        # Log progress periodically
        if int(turn_duration * 2) % 2 == 0:  # Every 0.5 seconds
            logger.info(f"ARC PROGRESS: {turn_desc} | Current={current_yaw:.1f}deg | Target={target_yaw:.1f}deg | Error={error:.1f}deg")
            print(f"ARC: Current={current_yaw:.1f}deg, Target={target_yaw:.1f}deg, Error={error:.1f}deg")
        
        return speed, steering
    
    elif turn_phase == "BACKWARD_POST_ARC":
        # BACKWARD PHASE AFTER ARC TURN - Move backward briefly before completing
        backward_elapsed = current_time - backward_post_arc_start
        
        if backward_elapsed < backward_post_arc_duration:
            # Continue backward movement
            print(f"BACKWARD PHASE: {backward_elapsed:.1f}s / {backward_post_arc_duration}s")
            logger.debug(f"Arc backward phase: {backward_elapsed:.1f}s elapsed")
            return -BASE_SPEED + 20, SERVO_STRAIGHT  # Slow backward
        else:
            # Backward phase complete, finish the turn
            print(f"BACKWARD PHASE COMPLETE: Arc turn fully completed after backward movement")
            logger.info(f"Arc backward phase completed - finishing turn")
            complete_turn()
            return BASE_SPEED, SERVO_STRAIGHT
    
    # Fallback - should not reach here
    return BASE_SPEED, SERVO_STRAIGHT


def snap_to_cardinal_angle(current_angle):
    """Snap angle to nearest cardinal direction (0, 90, 180, 270, 360, etc.) to prevent error accumulation"""
    # Find the nearest multiple of 90 degrees
    nearest_cardinal = round(current_angle / 90.0) * 90.0
    
    # Calculate the error
    error = abs(current_angle - nearest_cardinal)
    
    # Only snap if the error is reasonable (within 25 degrees)
    if error <= 25.0:
        print(f"CARDINAL SNAP: {current_angle:.1f} deg -> {nearest_cardinal:.1f} deg (error: {error:.1f} deg)")
        logger.info(f"Cardinal angle correction: {current_angle:.1f} -> {nearest_cardinal:.1f} (error: {error:.1f})")
        return nearest_cardinal
    else:
        print(f"CARDINAL SKIP: {current_angle:.1f} deg error too large ({error:.1f} deg) - keeping original")
        logger.warning(f"Cardinal snap skipped: {current_angle:.1f} deg, error {error:.1f} deg too large")
        return current_angle


def complete_turn():
    """Complete the current turn and reset turn mode with cardinal angle correction"""
    global in_turn_mode, obstacle_count, target_yaw, straight_heading_reference, turn_phase, turn_type
    global turn_just_completed, obstacle_deflection_after_turn, backward_post_arc_start
    global total_turns_completed
    
    # Get current yaw at completion
    current_yaw_at_completion = read_yaw()
    old_reference = straight_heading_reference
    
    # CARDINAL ANGLE CORRECTION: Snap to nearest 90-degree increment
    corrected_yaw = snap_to_cardinal_angle(current_yaw_at_completion)
    
    # Force update the IMU system with the corrected angle
    if hasattr(read_yaw, 'last_valid_yaw'):
        read_yaw.last_valid_yaw = corrected_yaw
    if hasattr(read_yaw, 'cumulative_yaw'):
        read_yaw.cumulative_yaw = corrected_yaw
    
    # Update the straight reference to the corrected angle
    straight_heading_reference = corrected_yaw
    
    # INCREMENT TURN COUNTER
    total_turns_completed += 1
    
    print(f"{turn_type} turn completed - CORRECTED straight reference: {straight_heading_reference:.1f} deg (was: {current_yaw_at_completion:.1f} deg)")
    print(f"REFERENCE UPDATE: Target was {target_yaw:.1f} deg, actual was {current_yaw_at_completion:.1f} deg, corrected to {corrected_yaw:.1f} deg")
    print(f"TURN COUNTER: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP} turns completed")
    logger.info(f"{turn_type} turn completed - Updated straight reference from {old_reference:.1f} deg to {straight_heading_reference:.1f} deg (corrected from {current_yaw_at_completion:.1f})")
    logger.info(f"Turn completion: Target={target_yaw:.1f} deg, Actual={current_yaw_at_completion:.1f} deg, Corrected={corrected_yaw:.1f} deg")
    logger.info(f"Turn counter: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP} turns completed")
    
    # Check if maximum turns reached
    if total_turns_completed >= MAX_TURNS_BEFORE_STOP:
        print(f"MAXIMUM TURNS REACHED: {total_turns_completed} turns completed - Robot will stop")
        logger.info(f"Maximum turns ({MAX_TURNS_BEFORE_STOP}) reached - robot stopping")
        
    in_turn_mode = False
    turn_phase = "NONE"  # Reset turn phase
    turn_type = "MULTI_PHASE"  # Reset to default
    obstacle_count = 0  # Reset for new segment
    backward_post_arc_start = 0  # Reset backward phase timer
    
    # CRITICAL SEQUENCE CONTROL: Set flags to enforce alternating pattern
    turn_just_completed = True  # Mark that a turn just completed
    obstacle_deflection_after_turn = False  # Reset - need obstacle deflection before next turn
    
    print("SEQUENCE CONTROL: Turn completed - Next turn blocked until obstacle deflection occurs")
    logger.info("Turn sequence control activated - next turn requires obstacle deflection first")
    
    # Clear any stuck detection variables
    if hasattr(update_turn_progress, 'last_yaw'):
        delattr(update_turn_progress, 'last_yaw')
    if hasattr(update_turn_progress, 'stuck_counter'):
        delattr(update_turn_progress, 'stuck_counter')
    
    # Brief pause to stabilize
    time.sleep(0.1)  # Reduced pause
    
    print("TARGET: Turn sequence completed - resuming obstacle navigation with corrected heading")
    logger.info("Turn completed - resuming normal navigation with cardinal-corrected heading reference")


# ------------- Original Helper Functions (Updated) -------------

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
    """Calibrate IMU by setting current heading as 0 reference"""
    global initial_yaw, target_yaw, straight_heading_reference
    print("Calibrating IMU...")
    time.sleep(1)  # Let IMU settle
    raw_yaw = sensor.euler[0]
    if raw_yaw is not None:
        initial_yaw = raw_yaw
        target_yaw = 0.0  # Initial target is straight ahead
        straight_heading_reference = 0.0  # Initial straight reference
        print(f"IMU calibrated. Initial heading: {initial_yaw:.1f} set as 0 reference")
    else:
        print("Warning: Could not read IMU for calibration")
        initial_yaw = 0
        target_yaw = 0.0
        straight_heading_reference = 0.0


def debug_angle_calculation(current_yaw, target_yaw):
    """Helper function to calculate turn error with NO normalization"""
    error = target_yaw - current_yaw
    
    # NO NORMALIZATION - work with raw error directly
    # This allows proper tracking of 170 deg → 260 deg (90 deg turn) without confusion
    
    print(f"UNBOUNDED ANGLE: Current={current_yaw:.1f} deg, Target={target_yaw:.1f} deg, Raw Error={error:.1f} deg")
    logger.debug(f"Unbounded angle calculation: current={current_yaw:.1f}, target={target_yaw:.1f}, error={error:.1f}")
    
    return error


def read_yaw():
    """Read relative yaw from BNO055 with NO boundary normalization - let angles accumulate naturally"""
    global initial_yaw
    
    try:
        raw_yaw = sensor.euler[0]
        if raw_yaw is not None:
            # Calculate relative yaw WITHOUT any normalization
            relative_yaw = raw_yaw - initial_yaw
            
            # NO NORMALIZATION - let angles go beyond [-180, 180] naturally
            # This allows smooth progression: 170 deg → 180 deg → 190 deg → 200 deg → 270 deg etc.
            
            # Initialize on first reading
            if not hasattr(read_yaw, 'last_valid_yaw'):
                read_yaw.last_valid_yaw = relative_yaw
                read_yaw.cumulative_yaw = relative_yaw  # Track cumulative angle
                read_yaw.last_raw_yaw = raw_yaw
                print(f"IMU INIT: First reading = {relative_yaw:.1f} deg (raw: {raw_yaw:.1f} deg)")
                return relative_yaw
            
            # Handle raw sensor wrap-around by tracking cumulative change
            raw_diff = raw_yaw - read_yaw.last_raw_yaw
            
            # Handle raw sensor boundary crossing
            if raw_diff > 180:
                raw_diff -= 360  # Sensor wrapped from ~180 to ~-180
            elif raw_diff < -180:
                raw_diff += 360  # Sensor wrapped from ~-180 to ~180
            
            # Update cumulative yaw by adding the raw change
            read_yaw.cumulative_yaw += raw_diff
            
            # Calculate what the new relative yaw should be
            new_relative_yaw = read_yaw.cumulative_yaw
            
            # Simple validation: reject only massive impossible jumps (>45 deg per reading)
            change_from_last = abs(new_relative_yaw - read_yaw.last_valid_yaw)
            
            if change_from_last > 45:
                # This is likely a sensor glitch, use the last valid reading
                print(f"IMU REJECT: Too large change {change_from_last:.1f} deg - keeping {read_yaw.last_valid_yaw:.1f} deg")
                return read_yaw.last_valid_yaw
            
            # Accept the reading
            if abs(new_relative_yaw - read_yaw.last_valid_yaw) > 2:
                print(f"IMU UPDATE: {read_yaw.last_valid_yaw:.1f} deg → {new_relative_yaw:.1f} deg (raw: {read_yaw.last_raw_yaw:.1f} deg → {raw_yaw:.1f} deg)")
            
            read_yaw.last_valid_yaw = new_relative_yaw
            read_yaw.last_raw_yaw = raw_yaw
            
            return new_relative_yaw
        else:
            logger.warning("IMU returned None for yaw reading")
            if hasattr(read_yaw, 'last_valid_yaw'):
                return read_yaw.last_valid_yaw
    except Exception as e:
        logger.error(f"Error reading IMU: {e}")
        if hasattr(read_yaw, 'last_valid_yaw'):
            return read_yaw.last_valid_yaw
    
    return 0.0


def force_imu_reset():
    """Force reset IMU reading state when stuck"""
    if hasattr(read_yaw, 'last_valid_yaw'):
        print(f"IMU RESET: Clearing stuck state (was: {read_yaw.last_valid_yaw:.1f} deg)")
        delattr(read_yaw, 'last_valid_yaw')
    if hasattr(read_yaw, 'stuck_count'):
        print(f"IMU RESET: Clearing stuck count (was: {read_yaw.stuck_count})")
        delattr(read_yaw, 'stuck_count')
    if hasattr(read_yaw, 'reading_history'):
        print(f"IMU RESET: Clearing reading history")
        delattr(read_yaw, 'reading_history')
    logger.info("IMU reading state forcibly reset")


def get_imu_debug_info():
    """Get debug information about IMU state"""
    info = {}
    if hasattr(read_yaw, 'last_valid_yaw'):
        info['last_valid'] = read_yaw.last_valid_yaw
    if hasattr(read_yaw, 'stuck_count'):
        info['stuck_count'] = read_yaw.stuck_count
    if hasattr(read_yaw, 'reading_history'):
        info['history'] = read_yaw.reading_history[-3:]  # Last 3 readings
    return info


# Legacy function - now just calls the improved version
def read_from_esp32():
    """Legacy function - now uses improved sensor data retrieval"""
    return get_fresh_sensor_data()


# Legacy function - now just calls the improved version  
def send_to_esp32(speed, steering):
    """Legacy function - now uses improved command sending"""
    send_to_esp32_improved(speed, steering)


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
            if y + h < FRAME_H * 0.3:  
                continue
            if y + h >= FRAME_H * 0.95:  
                continue
            
            if h > 100 and w > 100:  # More permissive size requirements
                red_blocks.append((x, y, w, h))
                logger.debug(f"RED block detected: x={x}, y={y}, w={w}, h={h}, area={area}")
    
    # Process green contours
    for contour in green_contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            if h > 100 and w > 100:  # More permissive size requirements
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
    
    print(f"Block area: {block_area} -> {size_category} -> Turn intensity: {intensity:.1f}")
    logger.debug(f"Turn calculation: area={block_area} -> {size_category} -> intensity={intensity:.1f}")
    return intensity, size_category


def calculate_steering(blocks, color):
    """Calculate steering based on block positions, size, and WRO rules with edge avoidance optimization"""
    if not blocks:
        return 0, "NO_OBSTACLE"
    
    # Find the largest (closest) block
    largest_block = max(blocks, key=lambda b: b[2] * b[3])  # w * h
    x, y, w, h = largest_block
    block_center_x = x + w // 2
    block_area = w * h
    region = get_region(block_center_x)
    
    # OPTIMIZATION: Check if block is at extreme edge where going straight is sufficient
    edge_threshold = 100  # Pixels from frame edge to consider "extreme edge"
    
    # For RED blocks: if at leftmost edge, go straight (block will be naturally avoided)
    if color == "RED" and x < edge_threshold:
        print(f"RED block at LEFTMOST EDGE (x={x}) - Going STRAIGHT (natural avoidance)")
        logger.info(f"RED block at extreme left edge (x={x}) - no steering adjustment needed")
        return 0, "STRAIGHT_EDGE_AVOIDANCE"
    
    # For GREEN blocks: if at rightmost edge, go straight (block will be naturally avoided)
    if color == "GREEN" and (x + w) > (FRAME_W - edge_threshold):
        print(f"GREEN block at RIGHTMOST EDGE (x+w={x+w}) - Going STRAIGHT (natural avoidance)")
        logger.info(f"GREEN block at extreme right edge (x+w={x+w}) - no steering adjustment needed")
        return 0, "STRAIGHT_EDGE_AVOIDANCE"
    
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
    """Main decision logic for steering with improved sensor data handling"""
    global yaw, last_turn_direction, recovery_mode, obstacle_count, turn_phase
    global turn_just_completed, obstacle_deflection_after_turn
    
    # Read current yaw
    yaw = read_yaw()
    
    # Get fresh sensor data with status
    f_dist, l_dist, r_dist, data_fresh, data_age = get_fresh_sensor_data()
    
    # Log sensor data freshness
    if not data_fresh and data_age > 1.0:
        logger.warning(f"Sensor data is stale (age: {data_age:.2f}s)")
    
    # Get obstacle detections
    frame = picam2.capture_array()
    red_blocks, green_blocks = detect_obstacles(frame)
    
    # Update obstacle memory and get enhanced detection
    red_blocks, green_blocks = update_obstacle_memory(red_blocks, green_blocks)
    
    steering = SERVO_STRAIGHT
    state = "STRAIGHT"
    
    # PRIORITY 1: Check if currently in turn mode
    if in_turn_mode:
        turn_result = update_turn_progress()
        if turn_result is not None and len(turn_result) == 2:
            turn_speed, turn_steering = turn_result
            steering = turn_steering
            state = f"TURNING_{turn_phase}"
            
            # Send turn commands to ESP32
            send_to_esp32_improved(turn_speed, steering)
            
            # Log turn progress
            log_navigation_data(steering, state, yaw, f_dist, l_dist, r_dist, red_blocks, green_blocks)
            
            # Draw debug visualization
            frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state)
            cv2.imshow("Region-Based Navigation", frame_debug)
            
            # Return early since we're still turning
            return steering, state
        else:
            # Turn completed, fall through to normal navigation
            steering = SERVO_STRAIGHT
            state = "TURN_COMPLETED"
    
    # PRIORITY 2: Check if turn should be initiated - ONLY if NO obstacles present
    # Double safety check: Never initiate turns if any obstacles are detected
    if len(red_blocks) == 0 and len(green_blocks) == 0:
        turn_direction = should_initiate_turn(f_dist, l_dist, r_dist, red_blocks, green_blocks)
        if turn_direction is not None:
            if execute_turn(turn_direction):
                # Turn initiated, use turn steering
                turn_result = update_turn_progress()
                if turn_result is not None and len(turn_result) == 2:
                    turn_speed, turn_steering = turn_result
                    steering = turn_steering
                    state = f"INITIATING_{turn_direction}_TURN"
                    
                    # Send turn commands to ESP32
                    send_to_esp32_improved(turn_speed, steering)
                    
                    # Log turn initiation
                    log_navigation_data(steering, state, yaw, f_dist, l_dist, r_dist, red_blocks, green_blocks)
                    
                    # Draw debug visualization
                    frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state)
                    cv2.imshow("Region-Based Navigation", frame_debug)
                    
                    return steering, state
    else:
        # Reset obstacle counter if we're actively avoiding obstacles
        # This prevents premature turn initiation
        logger.debug(f"Obstacle avoidance active - turn logic disabled (R:{len(red_blocks)}, G:{len(green_blocks)})")
    
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
    
    # PRIORITY 3: Handle the closest (most urgent) obstacle
    if closest_block is not None:
        # CRITICAL: If we're handling obstacles, absolutely NO TURNS allowed
        if in_turn_mode:
            print("WARNING: Canceling turn due to obstacle detection!")
            logger.warning("Turn canceled - obstacle detected during turn execution")
            complete_turn()
        
        # CRITICAL SEQUENCE CONTROL: Mark that obstacle deflection is happening
        # This allows the next turn to be activated after this deflection
        if turn_just_completed and not obstacle_deflection_after_turn:
            obstacle_deflection_after_turn = True
            print("SEQUENCE CONTROL: Obstacle deflection detected after turn - Next turn now allowed!")
            logger.info("Obstacle deflection after turn detected - turn sequence constraint lifted")
        
        # Create a list with just the closest block for calculate_steering
        closest_blocks = [closest_block]
        adjustment, turn_type = calculate_steering(closest_blocks, closest_color)
        steering = SERVO_STRAIGHT + adjustment
        state = f"{closest_color}_{turn_type}"
        
        # SEQUENCE CONTROL VALIDATION: Count both steering adjustments and edge avoidance as deflection
        if turn_type == "STRAIGHT_EDGE_AVOIDANCE":
            logger.info(f"Edge avoidance maneuver - counts as active obstacle deflection")
            # Edge avoidance always counts as active deflection since we're actively handling an obstacle
        elif abs(adjustment) < 20:  # Minor adjustments don't count as full deflection
            logger.debug(f"Minor obstacle adjustment ({adjustment:.1f}) - not counted as active deflection")
        else:
            logger.debug(f"Significant obstacle adjustment ({adjustment:.1f}) - counts as active deflection")
        
        if closest_color == "GREEN":
            last_turn_direction = "LEFT"
        else:  # RED
            last_turn_direction = "RIGHT"
            
        recovery_mode = False  # Reset recovery when actively avoiding
        
        # Start post-obstacle delay when actively deflecting
        global post_obstacle_delay_active, post_obstacle_delay_start
        if abs(adjustment) > 10:  # Only for significant deflections
            post_obstacle_delay_active = True
            post_obstacle_delay_start = time.time()
            print(f"POST-OBSTACLE DELAY: Started {POST_OBSTACLE_DELAY_DURATION}s delay after deflecting {closest_color} obstacle")
            logger.debug(f"Post-obstacle delay activated: {POST_OBSTACLE_DELAY_DURATION}s delay after {closest_color} deflection")
        
        # More conservative obstacle counting to prevent premature turns
        # Only count if we're making a significant steering adjustment and this is a NEW obstacle
        if abs(adjustment) > 20:  # Higher threshold for counting obstacles
            # Use area-based key to better distinguish different obstacles
            current_obstacle_key = f"{closest_color}_{closest_area//1000}"  # Group by area ranges
            
            # Initialize tracking if needed
            if not hasattr(decide_steering, 'last_obstacle_key'):
                decide_steering.last_obstacle_key = ""
                decide_steering.obstacle_stable_count = 0
            
            # Only count if this is a significantly different obstacle
            if current_obstacle_key != decide_steering.last_obstacle_key:
                decide_steering.obstacle_stable_count = 1
                decide_steering.last_obstacle_key = current_obstacle_key
                logger.debug(f"New obstacle detected: {current_obstacle_key}")
            else:
                decide_steering.obstacle_stable_count += 1
                # Only increment count after seeing the same obstacle for multiple cycles
                if decide_steering.obstacle_stable_count == 5:  # Stable for 5 cycles
                    obstacle_count += 1
                    print(f"OBSTACLE #{obstacle_count} confirmed: {closest_color} block (area={closest_area})")
                    logger.info(f"Obstacle #{obstacle_count} confirmed and being avoided: {closest_color} block (area={closest_area})")
        
        logger.info(f"PRIORITY: {closest_color} block (area={closest_area}) is closest - handling first")
        print(f"AVOIDING: {closest_color} block (area={closest_area}) - Total obstacles: {obstacle_count}/{max_obstacles_per_segment}")
        
    else:
        # No obstacles detected - check delay before straightening
        logger.debug(f"No obstacles detected - red_blocks={len(red_blocks)}, green_blocks={len(green_blocks)}")
        
        # Reset obstacle tracking when no obstacles are present
        if hasattr(decide_steering, 'last_obstacle_key'):
            decide_steering.last_obstacle_key = ""
            decide_steering.obstacle_stable_count = 0
        
        # Check if post-obstacle delay is active
        if post_obstacle_delay_active:
            delay_elapsed = time.time() - post_obstacle_delay_start
            if delay_elapsed < POST_OBSTACLE_DELAY_DURATION:
                # Still in delay period - maintain last deflection direction
                if last_turn_direction == "RIGHT":
                    recovery_steering = 10  # Gentle right deflection
                    state = "POST_OBSTACLE_DELAY_RIGHT"
                elif last_turn_direction == "LEFT":
                    recovery_steering = -10  # Gentle left deflection  
                    state = "POST_OBSTACLE_DELAY_LEFT"
                else:
                    recovery_steering = 0
                    state = "POST_OBSTACLE_DELAY"
                    
                # Apply minimal yaw correction during delay
                error = straight_heading_reference - yaw
                yaw_correction = error * 0.3  # Gentle correction during delay
                total_correction = recovery_steering + yaw_correction
                steering = SERVO_STRAIGHT + total_correction
                
                remaining_delay = POST_OBSTACLE_DELAY_DURATION - delay_elapsed
                print(f"POST-OBSTACLE DELAY: {remaining_delay:.1f}s remaining, maintaining deflection")
                logger.debug(f"Post-obstacle delay active: {remaining_delay:.1f}s remaining")
                
                # Limit steering and return early
                steering = min(max(SERVO_LEFT, steering), SERVO_RIGHT)
                return steering, state
            else:
                # Delay period complete
                post_obstacle_delay_active = False
                print(f"POST-OBSTACLE DELAY: Complete after {delay_elapsed:.1f}s - resuming normal navigation")
                logger.debug(f"Post-obstacle delay completed after {delay_elapsed:.1f}s")
        
        recovery_steering = 0
        
        # If we just finished avoiding an obstacle, apply counter-steering relative to current reference
        heading_error = yaw - straight_heading_reference
        if abs(heading_error) > 5:  # Robot is not straight relative to current reference
            if last_turn_direction == "RIGHT" and heading_error > 5:
                # After right turn, robot is angled right from reference - steer LEFT to center
                recovery_steering = -15  # Counter-steer left
                state = "RECOVERY_LEFT_AFTER_RIGHT"
                recovery_mode = True
                
            elif last_turn_direction == "LEFT" and heading_error < -5:
                # After left turn, robot is angled left from reference - steer RIGHT to center
                recovery_steering = 15   # Counter-steer right
                state = "RECOVERY_RIGHT_AFTER_LEFT"
                recovery_mode = True
                
        # Apply yaw correction to get back to current straight reference
        error = straight_heading_reference - yaw  # Target is current straight reference (updates after turns)
        yaw_correction = error * 0.8  # Stronger correction for faster recovery
        
        # Combine recovery steering with yaw correction
        total_correction = recovery_steering + yaw_correction
        steering = SERVO_STRAIGHT + total_correction
        
        if not recovery_mode:
            state = "STRAIGHT_YAW_CORRECTION"
            
        print(f"No obstacles - Yaw: {yaw:.1f}, Target: {straight_heading_reference:.1f}, Recovery: {recovery_steering}, YawCorr: {yaw_correction:.1f}, Total: {total_correction:.1f}")
        logger.debug(f"Recovery mode: yaw={yaw:.1f}, target={straight_heading_reference:.1f}, recovery_steering={recovery_steering}, yaw_correction={yaw_correction:.1f}, total={total_correction:.1f}")
        
        # Clear last turn direction if we're back to straight (within tolerance of current reference)
        heading_error = abs(yaw - straight_heading_reference)
        if heading_error < 3:
            last_turn_direction = ""
            recovery_mode = False
    
    # Limit steering to valid range
    steering = min(max(SERVO_LEFT, steering), SERVO_RIGHT)
    
    # Log comprehensive navigation data
    log_navigation_data(steering, state, yaw, f_dist, l_dist, r_dist, red_blocks, green_blocks)
    
    # Draw debug visualization
    frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state)
    cv2.imshow("Region-Based Navigation", frame_debug)
    
    return steering, state


def draw_debug(frame, red_blocks, green_blocks, steering, state):
    """Draw debug visualization with regions and obstacles"""
    global video_writer, turn_just_completed, obstacle_deflection_after_turn
    
    debug_frame = frame.copy()
    
    # Draw region dividers
    cv2.line(debug_frame, (FRAME_W // 3, 0), (FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    cv2.line(debug_frame, (2 * FRAME_W // 3, 0), (2 * FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    
    # Draw edge avoidance zones (100px from edges)
    edge_threshold = 100
    cv2.line(debug_frame, (edge_threshold, 0), (edge_threshold, FRAME_H), (255, 0, 255), 1)  # Left edge zone
    cv2.line(debug_frame, (FRAME_W - edge_threshold, 0), (FRAME_W - edge_threshold, FRAME_H), (255, 0, 255), 1)  # Right edge zone
    
    # Label regions
    cv2.putText(debug_frame, "LEFT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "CENTER", (FRAME_W // 2 - 50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "RIGHT", (FRAME_W - 150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Label edge zones
    cv2.putText(debug_frame, "EDGE", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.putText(debug_frame, "EDGE", (FRAME_W - 60, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    
    # Draw red blocks with edge detection
    edge_threshold = 100
    for x, y, w, h in red_blocks:
        # Check if in edge zone for natural avoidance
        in_edge_zone = x < edge_threshold
        border_color = (255, 0, 255) if in_edge_zone else (0, 0, 255)  # Magenta if in edge zone
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), border_color, 3)
        
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        
        edge_label = "-EDGE" if in_edge_zone else ""
        cv2.putText(debug_frame, f"RED-{region}{edge_label}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_color, 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, border_color, 2)
    
    # Draw green blocks with edge detection
    for x, y, w, h in green_blocks:
        # Check if in edge zone for natural avoidance
        in_edge_zone = (x + w) > (FRAME_W - edge_threshold)
        border_color = (255, 0, 255) if in_edge_zone else (0, 255, 0)  # Magenta if in edge zone
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), border_color, 3)
        
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        
        edge_label = "-EDGE" if in_edge_zone else ""
        cv2.putText(debug_frame, f"GREEN-{region}{edge_label}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_color, 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, border_color, 2)
    
    # Add timestamp to frame
    timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Include milliseconds
    cv2.putText(debug_frame, timestamp_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Draw status info with improved data
    queue_size = len(command_queue) if command_queue else 0
    data_age = time.time() - last_sensor_update if last_sensor_update > 0 else 999
    
    # Main status info with special highlighting for edge avoidance
    cv2.putText(debug_frame, f"Steering: {steering:.1f}", (10, FRAME_H - 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # Highlight edge avoidance state in bright green
    state_color = (0, 255, 255) if "STRAIGHT_EDGE_AVOIDANCE" in state else (255, 255, 0)
    cv2.putText(debug_frame, f"State: {state}", (10, FRAME_H - 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)
    
    # Add special indicator for edge avoidance
    if "STRAIGHT_EDGE_AVOIDANCE" in state:
        cv2.putText(debug_frame, "EDGE AVOID!", (300, FRAME_H - 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 3)
    
    cv2.putText(debug_frame, f"Yaw: {yaw:.1f}", (10, FRAME_H - 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Target: {straight_heading_reference:.1f}", (10, FRAME_H - 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # Turn and obstacle info
    cv2.putText(debug_frame, f"Obstacles: {obstacle_count}/{max_obstacles_per_segment}", (10, FRAME_H - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    # Turn counter with color coding based on proximity to limit
    turns_color = (0, 0, 255) if total_turns_completed >= MAX_TURNS_BEFORE_STOP - 2 else (255, 255, 0)  # Red if close to limit
    cv2.putText(debug_frame, f"Turns: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP}", (300, FRAME_H - 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, turns_color, 2)
    
    if in_turn_mode:
        cv2.putText(debug_frame, f"{turn_type}: {turn_phase} -> Target: {target_yaw:.1f}", (10, FRAME_H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        cv2.putText(debug_frame, f"Last Turn: {last_turn_direction}", (10, FRAME_H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # Sequence control status with detailed info
    if turn_just_completed and not obstacle_deflection_after_turn:
        sequence_status = "BLOCKED"
        sequence_color = (0, 0, 255)  # Red
        sequence_detail = f"(Need deflection)"
    elif turn_just_completed and obstacle_deflection_after_turn:
        sequence_status = "ALLOWED"
        sequence_color = (0, 255, 0)  # Green  
        sequence_detail = f"(Post-deflection)"
    else:
        sequence_status = "BLOCKED"
        sequence_color = (0, 255, 255)  # Yellow
        sequence_detail = f"(Initial state)"
        
    cv2.putText(debug_frame, f"Next Turn: {sequence_status}", (300, FRAME_H - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, sequence_color, 2)
    cv2.putText(debug_frame, sequence_detail, (300, FRAME_H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, sequence_color, 2)
    
    # System info
    cv2.putText(debug_frame, f"Recovery: {recovery_mode}", (10, FRAME_H - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Cmd Queue: {queue_size}", (10, FRAME_H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Data Age: {data_age:.2f}s", (300, FRAME_H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    # Turn indicators with type and phase info
    if in_turn_mode:
        # Different colors for different turn types and phases
        if turn_type == "ARC":
            phase_color = (0, 255, 0)  # Green for arc turns
            display_text = f">> {turn_type} <<"
        else:  # MULTI_PHASE
            phase_color = (255, 255, 0) if turn_phase == "FORWARD" else (0, 255, 255) if turn_phase == "REVERSE_TURN" else (0, 255, 0)
            display_text = f">> {turn_type}: {turn_phase} <<"
        
        cv2.putText(debug_frame, display_text, (FRAME_W - 350, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, phase_color, 3)
    
    # Record video frame if enabled
    if RECORD_VIDEO and video_writer is not None:
        # Convert RGB to BGR for video writer
        bgr_frame = cv2.cvtColor(debug_frame, cv2.COLOR_RGB2BGR)
        video_writer.write(debug_frame)
    
    return debug_frame


def main():
    """Main navigation loop with improved serial communication"""
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
        
        # Test serial connection and reset ESP32 state
        print("Testing serial connection to ESP32...")
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"Serial port: {SERIAL_PORT}")
            print(f"Baud rate: {BAUD_RATE}")
            print(f"Port open: {ser.is_open}")
            logger.info(f"Serial connection test: Port={SERIAL_PORT}, Baud={BAUD_RATE}, Open={ser.is_open}")
            
            # Send a reset/stop command first to clear ESP32 state
            print("Resetting ESP32 state...")
            ser.write(b"RESET\n")
            ser.flush()
            time.sleep(0.5)
            
            # Send stop command to ensure motors are stopped
            ser.write(b"0,90\n")
            ser.flush()
            time.sleep(0.5)
            
            # Clear buffers again after reset commands
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print("ESP32 reset sequence completed")
            logger.info("ESP32 reset sequence completed")
            
        except Exception as e:
            print(f"ERROR: Serial connection issue: {e}")
            logger.error(f"Serial connection test failed: {e}")
        
        # Start serial communication thread
        serial_thread = threading.Thread(target=serial_communication_handler, daemon=True)
        serial_thread.start()
        logger.info("Serial communication thread started")
        
        # Give the thread a moment to start
        time.sleep(0.5)
        
        # LED indicator
        led_blink()
        
        logger.info("System initialized successfully")
        logger.info(f"Log file: {LOG_FILE}")
        logger.info(f"Video file: {VIDEO_FILE}")
        logger.info("Waiting for button press...")
        
        print("=== IMPROVED RPi NAVIGATION SYSTEM ===")
        print("Features:")
        print("- Non-blocking serial communication")
        print("- Command queuing to prevent pile-up")
        print("- Fresh sensor data monitoring")
        print("- Rate-limited communication (20Hz commands, 10Hz sensor requests)")
        print("Waiting for button press...")
        
        while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            time.sleep(0.1)
        
        print("Button pressed! Starting system...")
        logger.info("Button pressed! Starting navigation system...")
        
        # Calibrate IMU
        calibrate_imu()
        logger.info(f"IMU calibrated with initial heading: {initial_yaw:.1f}")
        
        # Initialize sequence control flags - TURNING BLOCKED BY DEFAULT
        global turn_just_completed, obstacle_deflection_after_turn
        turn_just_completed = True   # DEFAULT: Turn blocked until obstacle deflection occurs
        obstacle_deflection_after_turn = False
        print("SEQUENCE CONTROL: Initialized - Turning BLOCKED by default until obstacle deflection occurs")
        logger.info("Turn sequence control initialized - turns BLOCKED by default, requires obstacle deflection to enable")
        
        # Send START to ESP32 with timeout and retry logic
        start_success = False
        max_retries = 3
        retry_count = 0
        
        print("Initializing communication with ESP32...")
        
        while not start_success and retry_count < max_retries:
            try:
                with serial_lock:
                    # Clear any pending input
                    ser.reset_input_buffer()
                    
                    # First try to check if ESP32 is already started
                    ser.write(b"STATUS\n")
                    ser.flush()
                    time.sleep(0.2)
                    
                    # Check for any immediate response
                    if ser.in_waiting > 0:
                        response = ser.readline().decode().strip()
                        print(f"ESP32 status response: {response}")
                        if "DIST" in response:
                            print("SUCCESS: ESP32 already active and sending data!")
                            start_success = True
                            break
                    
                    # Send START command
                    ser.write(b"START\n")
                    ser.flush()
                
                print(f"Sending START command to ESP32 (attempt {retry_count + 1}/{max_retries})...")
                logger.info(f"START command sent to ESP32 (attempt {retry_count + 1})")
                
                # Wait for acknowledgment with timeout
                start_time = time.time()
                timeout = 2.0  # Reduced timeout
                
                while time.time() - start_time < timeout:
                    if ser.in_waiting > 0:
                        response = ser.readline().decode().strip()
                        logger.info(f"ESP32 response: {response}")
                        
                        if "ACK_START" in response or "READY" in response or "DIST" in response:
                            start_success = True
                            print("SUCCESS: ESP32 acknowledged START command!")
                            logger.info("ESP32 acknowledged START command successfully")
                            break
                        elif response:
                            print(f"ESP32 response: {response}")
                    
                    time.sleep(0.1)
                
                if not start_success:
                    retry_count += 1
                    if retry_count < max_retries:
                        print(f"WARNING: No response from ESP32, retrying...")
                        # Try a different approach - send a reset first
                        with serial_lock:
                            ser.write(b"0,90\n")  # Stop command
                            ser.flush()
                        time.sleep(1)
                    else:
                        print("WARNING: Proceeding without ESP32 acknowledgment...")
                        print("   (ESP32 might already be running or will start receiving commands)")
                        logger.warning("ESP32 not responding to START command, continuing without acknowledgment")
                        start_success = True  # Continue anyway
                        
            except Exception as e:
                retry_count += 1
                print(f"ERROR: Error communicating with ESP32: {e}")
                logger.error(f"Error sending START command: {e}")
                if retry_count < max_retries:
                    time.sleep(1)
        
        print("Starting region-based navigation with improved communication...")
        print(f"Regions: LEFT(0-{LEFT_REGION[1]}), CENTER({CENTER_REGION[0]}-{CENTER_REGION[1]}), RIGHT({RIGHT_REGION[0]}-{FRAME_W})")
        
        logger.info("Starting region-based navigation...")
        logger.info(f"Frame regions: LEFT(0-{LEFT_REGION[1]}), CENTER({CENTER_REGION[0]}-{CENTER_REGION[1]}), RIGHT({RIGHT_REGION[0]}-{FRAME_W})")
        logger.info(f"Turn intensity config: MIN={MIN_TURN}, MAX={MAX_TURN}, SMALL_THRESHOLD={SIZE_THRESHOLD_SMALL}, LARGE_THRESHOLD={SIZE_THRESHOLD_LARGE}")
        logger.info(f"Communication config: CMD_INTERVAL={COMMAND_INTERVAL}s, GET_INTERVAL={GET_REQUEST_INTERVAL}s, Queue_size={command_queue.maxlen}")
        
        loop_count = 0
        start_time = time.time()
        last_status_time = 0
        
        while True:
            loop_count += 1
            loop_start_time = time.time()
            
            # Check if maximum turns completed - STOP ROBOT
            if total_turns_completed >= MAX_TURNS_BEFORE_STOP:
                print(f"STOPPING: Maximum {MAX_TURNS_BEFORE_STOP} turns completed!")
                logger.info(f"Navigation stopped: Maximum {MAX_TURNS_BEFORE_STOP} turns completed")
                break
            
            # Decide steering based on region logic
            steering, state = decide_steering()
            
            # Send commands to ESP32 using improved method
            # Note: During turn mode, commands are sent within decide_steering()
            # For normal navigation, send base speed
            if not in_turn_mode:
                send_to_esp32_improved(BASE_SPEED, steering)
            
            # Console output (reduced frequency for readability) with improved stats
            current_time = time.time()
            if current_time - last_status_time > 1.0:  # Print every second
                queue_size = len(command_queue)
                data_age = current_time - last_sensor_update if last_sensor_update > 0 else 999
                loop_hz = loop_count / (current_time - start_time) if current_time > start_time else 0
                
                print(f"Loop {loop_count} ({loop_hz:.1f}Hz) | Yaw: {yaw:.1f} | Steering: {steering:.1f} | State: {state}")
                print(f"Distances - Front: {front_dist}, Left: {left_dist}, Right: {right_dist} (Age: {data_age:.2f}s)")
                print(f"Turns Completed: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP} | Command Queue: {queue_size}/{command_queue.maxlen} | Serial Thread: {'Active' if serial_thread.is_alive() else 'DEAD'}")
                print("-" * 80)
                last_status_time = current_time
            
            # Monitor serial thread health
            if not serial_thread.is_alive():
                logger.error("Serial communication thread died! Restarting...")
                serial_thread = threading.Thread(target=serial_communication_handler, daemon=True)
                serial_thread.start()
            
            # Exit conditions and manual overrides
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("User pressed 'q' - exiting navigation loop")
                break
            elif key == ord('t'):  # Manual turn escape
                if in_turn_mode:
                    print("Manual turn escape activated!")
                    logger.info("Manual turn escape - forcing turn completion")
                    complete_turn()
            elif key == ord('r'):  # Reset obstacle count
                obstacle_count = 0
                print(f"Obstacle count reset to 0")
                logger.info("Manual obstacle count reset")
            
            # Maintain consistent loop timing
            loop_duration = time.time() - loop_start_time
            if loop_duration < 0.05:  # Target 20Hz
                time.sleep(0.05 - loop_duration)
        
        # Calculate session statistics
        end_time = time.time()
        total_runtime = end_time - start_time
        avg_hz = loop_count / total_runtime if total_runtime > 0 else 0
        
        logger.info(f"Navigation session completed:")
        logger.info(f"  Total loops: {loop_count}")
        logger.info(f"  Runtime: {total_runtime:.2f} seconds")
        logger.info(f"  Average frequency: {avg_hz:.1f} Hz")
        logger.info(f"  Turns completed: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP}")
        logger.info(f"  Final queue size: {len(command_queue)}")
        
        print(f"\n=== SESSION STATISTICS ===")
        print(f"Total loops: {loop_count}")
        print(f"Runtime: {total_runtime:.2f} seconds")
        print(f"Average frequency: {avg_hz:.1f} Hz")
        print(f"Turns completed: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP}")
        print(f"Final queue size: {len(command_queue)}")
        
        # Check if stopped due to turn limit
        if total_turns_completed >= MAX_TURNS_BEFORE_STOP:
            print(f"MISSION COMPLETED: Robot stopped after {total_turns_completed} turns as configured")
            logger.info(f"Mission completed: Robot stopped after reaching {MAX_TURNS_BEFORE_STOP} turn limit")
        
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
        
        # Stop robot immediately
        try:
            with serial_lock:
                ser.write(b"0,90\n")  # Emergency stop
                ser.flush()
            logger.info("Emergency stop sent to ESP32")
        except Exception as e:
            logger.error(f"Failed to send emergency stop: {e}")
        
        # Clear command queue
        command_queue.clear()
        logger.info("Command queue cleared")
        
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


if __name__ == "__main__":
    main()
