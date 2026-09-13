#!/usr/bin/env python3
"""
Autonomous QR-Guided Drone System with PID Landing Control
Part 1: Core Classes and Configuration

Compatible with SITL simulation and real hardware (Raspberry Pi 4 + Pixhawk 2.4.8)
Features: QR detection, auto-to-guided switching, PID landing, payload drop, RTL
"""

import sys
import os
import cv2
import numpy as np
import time
import logging
import threading
from collections import deque
from pyzbar import pyzbar
from pymavlink import mavutil
from dataclasses import dataclass
from typing import Optional, Tuple, List

# Suppress zbar warnings
sys.stderr = open(os.devnull, 'w')

# ============================================================================
# CONFIGURATION
# ============================================================================

@dataclass
class SystemConfig:
    """Main system configuration"""
    # Hardware mode - CHANGED TO FALSE FOR REAL DRONE
    USE_SITL: bool = True  
    
    # Camera settings
    CAMERA_RESOLUTION: Tuple[int, int] = (640, 480)  # Lowered to 640x480 for stability
    CAMERA_ISO: int = 800
    CAMERA_SHUTTER_US: int = 3000
    CAMERA_FPS: int = 30 
    
    # QR detection
    QR_SIZE_CM: float = 59.4  # A2 paper size (width in cm)
    QR_DETECTION_CONFIDENCE: int = 2  # Reduced for faster detection
    QR_MIN_AREA: int = 500   # Reduced minimum area
    QR_SEARCH_TIMEOUT: float = 60.0  
    QR_SEARCH_PATTERN_ENABLED: bool = True  
    
    # PID tuning for landing alignment
    PID_KP: float = 0.3
    PID_KI: float = 0.05
    PID_KD: float = 0.15
    PID_MAX_VEL: float = 0.5  # m/s
    
    # Mission parameters
    APPROACH_ALTITUDE: float = 3.0  # meters
    LANDING_THRESHOLD_CM: float = 10.0  # Center within 10cm
    CENTERING_TIMEOUT: float = 30.0  # seconds
    
    # MAVLink settings
    MAVLINK_PORT_SITL: str = "tcp:192.168.29.8:5762"
    MAVLINK_PORT_REAL: str = "/dev/serial0"
    MAVLINK_BAUD: int = 57600
    
    # Display
    SHOW_GUI: bool = True  # Set to False for headless mode
    
    def get_mavlink_connection(self) -> str:
        return self.MAVLINK_PORT_SITL if self.USE_SITL else self.MAVLINK_PORT_REAL


# ============================================================================
# LOGGING SETUP
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("qr_drone.log"),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger("QRDrone")


# ============================================================================
# PID CONTROLLER
# ============================================================================

class PIDController:
    """PID controller for drone positioning"""
    
    def __init__(self, kp: float, ki: float, kd: float, max_output: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def update(self, error: float, current_time: float) -> float:
        """Calculate PID output"""
        if self.prev_time is None:
            self.prev_time = current_time
            return 0.0
            
        dt = current_time - self.prev_time
        if dt <= 0:
            return 0.0
            
        # PID calculations
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Clamp output
        output = max(-self.max_output, min(self.max_output, output))
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output


# ============================================================================
# QR DETECTOR
# ============================================================================

class QRDetector:
    """Optimized QR code detector for Raspberry Pi"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.detection_history = deque(maxlen=config.QR_DETECTION_CONFIDENCE)
        self.last_qr_data = None
        self.last_qr_bbox = None
        self.detection_scales = [1.0] # Use single scale for performance
        
    def detect(self, frame: np.ndarray) -> Optional[dict]:
        """
        Detect QR code in frame
        Returns: dict with 'data', 'bbox', 'center' or None
        """
        if frame is None:
            return None
            
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Try multiple scales for robustness
        for scale in self.detection_scales:
            if scale != 1.0:
                h, w = gray.shape
                scaled = cv2.resize(gray, (int(w * scale), int(h * scale)))
            else:
                scaled = gray
                
            # Detect QR codes
            barcodes = pyzbar.decode(scaled)
            
            if barcodes:
                for barcode in barcodes:
                    # Scale coordinates back
                    x = int(barcode.rect.left / scale)
                    y = int(barcode.rect.top / scale)
                    w = int(barcode.rect.width / scale)
                    h = int(barcode.rect.height / scale)
                    
                    # Filter small detections
                    if w * h < self.config.QR_MIN_AREA:
                        continue
                        
                    data = barcode.data.decode('utf-8', errors='ignore')
                    
                    # Calculate center
                    cx = x + w // 2
                    cy = y + h // 2
                    
                    result = {
                        'data': data,
                        'bbox': (x, y, w, h),
                        'center': (cx, cy),
                        'area': w * h
                    }
                    
                    # Update history
                    self.detection_history.append(result)
                    self.last_qr_data = data
                    self.last_qr_bbox = (x, y, w, h)
                    
                    return result
                    
        # No detection
        self.detection_history.append(None)
        return None
        
    def is_stable_detection(self) -> bool:
        """Check if QR detection is stable"""
        if len(self.detection_history) < self.config.QR_DETECTION_CONFIDENCE:
            return False
        return all(d is not None for d in self.detection_history)


# ============================================================================
# CAMERA MANAGER
# ============================================================================

class CameraManager:
    """Manages camera capture with Pi Camera 2 or OpenCV fallback"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.camera = None
        # On Real Hardware, prefer Picamera2, else OpenCV
        self.use_picamera = not config.USE_SITL
        self.cap = None
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = False
        
    def init_camera(self) -> bool:
        """Initialize camera"""
        logger.info("Initializing Camera Manager...")

        # 1. Try Picamera2 (Native Pi Camera)
        if self.use_picamera:
            try:
                from picamera2 import Picamera2
                logger.info("Attempting Picamera2 connection...")
                self.camera = Picamera2()
                
                # Basic configuration
                config = self.camera.create_preview_configuration(
                    main={"size": self.config.CAMERA_RESOLUTION, "format": "RGB888"}
                )
                self.camera.configure(config)
                self.camera.start()
                
                # Warmup
                time.sleep(2.0)
                
                # Test capture
                frame = self.camera.capture_array("main")
                if frame is not None:
                    # Picamera returns RGB, OpenCV needs BGR
                    test_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    logger.info(f"Picamera2 initialized successfully: {test_frame.shape}")
                    return True
            except Exception as e:
                logger.warning(f"Picamera2 failed: {e}. Falling back to OpenCV.")
                self.use_picamera = False
                if self.camera:
                    try:
                        self.camera.stop()
                        self.camera.close()
                    except:
                        pass
                    self.camera = None

        # 2. Fallback to OpenCV (USB Webcam or Legacy Pi Camera)
        logger.info("Initializing OpenCV VideoCapture...")
        
        # Try index 0, then 1
        for idx in range(2):
            try:
                # CRITICAL FIX: Use CAP_V4L2 backend for Raspberry Pi
                # This prevents hanging/freezing on initialization
                if not self.config.USE_SITL:
                    self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                else:
                    self.cap = cv2.VideoCapture(idx)

                if self.cap.isOpened():
                    # Force MJPG to ensure compatibility and speed
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_RESOLUTION[0])
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_RESOLUTION[1])
                    self.cap.set(cv2.CAP_PROP_FPS, self.config.CAMERA_FPS)
                    
                    # Read one frame to verify
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        logger.info(f"OpenCV Camera (Index {idx}) initialized.")
                        return True
                    else:
                        self.cap.release()
            except Exception as e:
                logger.error(f"Failed to open camera index {idx}: {e}")

        logger.error("CRITICAL: No camera could be initialized.")
        return False
            
    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a single frame"""
        try:
            if self.use_picamera and self.camera:
                frame = self.camera.capture_array("main")
                # Convert RGB (Picamera) to BGR (OpenCV)
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif self.cap:
                ret, frame = self.cap.read()
                return frame if ret else None
        except Exception as e:
            # Don't spam logs with capture errors
            pass
        return None
        
    def start_capture_thread(self):
        """Start background capture thread"""
        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        
    def _capture_loop(self):
        """Background capture loop"""
        logger.info("Camera capture thread started")
        frame_count = 0
        
        while self.running:
            frame = self.capture_frame()
            if frame is not None:
                with self.lock:
                    self.latest_frame = frame
                frame_count += 1
                
                # Log every 100 frames to show it's alive
                if frame_count % 100 == 0:
                    logger.debug(f"Captured {frame_count} frames")
            else:
                time.sleep(0.1)
                
            time.sleep(1.0 / self.config.CAMERA_FPS)
            
        logger.info("Camera capture thread stopped")
        
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get latest captured frame"""
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
            
    def stop(self):
        """Stop camera"""
        self.running = False
        time.sleep(0.5)
        
        if self.camera:
            try:
                self.camera.stop()
                self.camera.close()
            except:
                pass
        if self.cap:
            self.cap.release()

# ============================================================================
# MAVLINK INTERFACE - Part 2
# ============================================================================

class MAVLinkInterface:
    """MAVLink communication handler"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.master = None
        self.target_system = None
        self.target_component = 1
        self.current_mode = None
        self.mission_items = {}
        self.current_waypoint = -1
        
    def connect(self) -> bool:
        """Connect to flight controller"""
        try:
            connection_string = self.config.get_mavlink_connection()
            logger.info(f"Connecting to MAVLink: {connection_string}")
            
            if self.config.USE_SITL:
                self.master = mavutil.mavlink_connection(connection_string)
            else:
                self.master = mavutil.mavlink_connection(
                    connection_string, 
                    baud=self.config.MAVLINK_BAUD
                )
                
            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            msg = self.master.wait_heartbeat(timeout=10)
            if not msg:
                logger.error("No heartbeat received")
                return False
                
            self.target_system = self.master.target_system
            self.target_component = 1  # Autopilot
            
            logger.info(f"Connected to system {self.target_system}")
            
            # Request data streams
            self.request_data_streams(rate_hz=10)
            
            return True
            
        except Exception as e:
            logger.error(f"MAVLink connection failed: {e}")
            return False
            
    def request_data_streams(self, rate_hz: int = 10):
        """Request MAVLink data streams"""
        self.master.mav.request_data_stream_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1
        )
        
    def get_mode(self) -> Optional[str]:
        """Get current flight mode"""
        return self.current_mode
        
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode"""
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            logger.error(f"Mode '{mode_name}' not found")
            return False
            
        logger.info(f"Setting mode to {mode_name}")
        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait for confirmation
        for _ in range(20):
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg:
                mode_str = mavutil.mode_string_v10(msg)
                if mode_str and mode_name.upper() in mode_str.upper():
                    self.current_mode = mode_str
                    logger.info(f"Mode confirmed: {mode_str}")
                    return True
                    
        logger.error(f"Failed to set mode {mode_name}")
        return False
        
    def send_ned_velocity(self, vx: float, vy: float, vz: float):
        """Send velocity command in NED frame"""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # type_mask (velocities only)
            0, 0, 0,  # positions (not used)
            vx, vy, vz,  # velocities in m/s
            0, 0, 0,  # accelerations (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        
    def send_position_target(self, x: float, y: float, z: float):
        """Send position target in NED frame"""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (positions only)
            x, y, z,  # positions in meters
            0, 0, 0,  # velocities (not used)
            0, 0, 0,  # accelerations (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        
    def command_land(self):
        """Send land command"""
        logger.info("Commanding LAND")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
    def command_rtl(self):
        """Send RTL command"""
        logger.info("Commanding RTL")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
    def send_statustext(self, text: str, severity: int = 6):
        """Send status text to GCS"""
        self.master.mav.statustext_send(
            severity,
            text.encode('utf-8')[:50].ljust(50, b'\0')
        )
        logger.info(f"Status: {text}")
        
    def request_mission_item(self, seq: int):
        """Request specific mission item"""
        self.master.mav.mission_request_int_send(
            self.target_system,
            self.target_component,
            seq
        )
        
    def get_local_position(self) -> Optional[Tuple[float, float, float]]:
        """Get current local position NED"""
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            return (msg.x, msg.y, msg.z)
        return None
        
    def get_altitude(self) -> Optional[float]:
        """Get altitude above ground"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            return msg.relative_alt / 1000.0  # Convert to meters
        return None
        
    def close(self):
        """Close connection"""
        if self.master:
            self.master.close()
            logger.info("MAVLink connection closed")


# ============================================================================
# MISSION MONITOR - Part 2
# ============================================================================

class MissionMonitor:
    """Monitors mission progress and detects trigger commands"""
    
    def __init__(self, mavlink: MAVLinkInterface):
        self.mavlink = mavlink
        self.running = False
        self.trigger_detected = False
        self.trigger_callback = None
        self.last_waypoint = -1
        
    def set_trigger_callback(self, callback):
        """Set callback for when trigger is detected"""
        self.trigger_callback = callback
        
    def start(self):
        """Start monitoring thread"""
        self.running = True
        threading.Thread(target=self._monitor_loop, daemon=True).start()
        logger.info("Mission monitor started")
        
    def _monitor_loop(self):
        """Main monitoring loop"""
        last_stream_request = time.time()
        
        while self.running:
            # Keep-alive for data streams
            if time.time() - last_stream_request > 2.0:
                self.mavlink.request_data_streams(10)
                last_stream_request = time.time()
                
            msg = self.mavlink.master.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
                
            msg_type = msg.get_type()
            
            # Update current mode
            if msg_type == 'HEARTBEAT':
                mode_str = mavutil.mode_string_v10(msg)
                if mode_str and mode_str != self.mavlink.current_mode:
                    self.mavlink.current_mode = mode_str
                    logger.info(f"Mode: {mode_str}")
                    
            # Track waypoint progress
            elif msg_type == 'MISSION_CURRENT':
                wp = msg.seq
                if wp != self.last_waypoint:
                    logger.info(f"Waypoint: {wp}")
                    self.last_waypoint = wp
                    
                    # Peek ahead - check next waypoint
                    self.mavlink.request_mission_item(wp + 1)
                    
            # Check reached waypoints
            elif msg_type == 'MISSION_ITEM_REACHED':
                reached = msg.seq
                logger.info(f"Reached waypoint: {reached}")
                
                # Request this item and next
                self.mavlink.request_mission_item(reached)
                self.mavlink.request_mission_item(reached + 1)
                
            # Receive mission items
            elif msg_type == 'MISSION_ITEM_INT':
                seq = msg.seq
                cmd = msg.command
                self.mavlink.mission_items[seq] = cmd
                
                # Check for DO_SPRAYER trigger
                if cmd == mavutil.mavlink.MAV_CMD_DO_SPRAYER:
                    logger.warning(f"DO_SPRAYER detected at waypoint {seq}!")
                    self._handle_trigger()
                    
            # Backup: text-based trigger detection
            elif msg_type == 'STATUSTEXT':
                try:
                    text = msg.text.decode('utf-8', errors='ignore').strip() \
                           if isinstance(msg.text, bytes) else str(msg.text).strip()
                    
                    if 'Sprayer' in text or 'SPRAYER' in text:
                        logger.info(f"Sprayer text detected: {text}")
                        self._handle_trigger()
                        
                except:
                    pass
                    
    def _handle_trigger(self):
        """Handle trigger detection"""
        if not self.trigger_detected:
            self.trigger_detected = True
            logger.warning("TRIGGER ACTIVATED!")
            
            if self.trigger_callback:
                self.trigger_callback()
                
    def reset_trigger(self):
        """Reset trigger state"""
        self.trigger_detected = False
        
    def stop(self):
        """Stop monitoring"""
        self.running = False
# ============================================================================
# AUTONOMOUS CONTROLLER - Part 3
# ============================================================================

class AutonomousController:
    """Main autonomous control system"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.camera = CameraManager(config)
        self.qr_detector = QRDetector(config)
        self.mavlink = MAVLinkInterface(config)
        self.monitor = MissionMonitor(self.mavlink)
        
        # PID controllers for X and Y alignment
        self.pid_x = PIDController(config.PID_KP, config.PID_KI, config.PID_KD, config.PID_MAX_VEL)
        self.pid_y = PIDController(config.PID_KP, config.PID_KI, config.PID_KD, config.PID_MAX_VEL)
        
        self.running = False
        self.display_frame = None
        self.display_lock = threading.Lock()
        
    def initialize(self) -> bool:
        """Initialize all systems"""
        logger.info("Initializing systems...")
        
        # Connect MAVLink
        if not self.mavlink.connect():
            return False
            
        # Initialize camera
        if not self.camera.init_camera():
            return False
            
        # Start camera capture
        self.camera.start_capture_thread()
        time.sleep(1.0)
        
        # Set up mission monitor
        self.monitor.set_trigger_callback(self.on_trigger_detected)
        self.monitor.start()
        
        logger.info("All systems initialized")
        return True
        
    def on_trigger_detected(self):
        """Callback when DO_SPRAYER trigger is detected"""
        logger.warning("Trigger detected - starting autonomous sequence")
        threading.Thread(target=self.execute_autonomous_sequence, daemon=True).start()
        
    def execute_autonomous_sequence(self):
        """Main autonomous sequence"""
        try:
            logger.info("="*60)
            logger.info("  AUTONOMOUS SEQUENCE STARTED")
            logger.info("="*60)
            
            # Step 1: Switch to GUIDED mode
            if not self.switch_to_guided():
                logger.error("Failed to switch to GUIDED mode")
                return
                
            # Step 2: Search for QR code (never abort, keep searching)
            qr_data = self.search_for_qr()
            # QR will always be found eventually - no need to check for None
                
            # Step 3: Center and descend with PID
            if not self.center_on_qr():
                logger.error("Failed to center on QR - aborting")
                self.mavlink.send_statustext("CENTERING_FAILED")
                self.mavlink.command_rtl()
                return
                
            # Step 4: Land
            self.execute_landing()
            
            # Step 5: Drop payload (simulated)
            self.drop_payload()
            
            # Step 6: RTL
            self.mavlink.command_rtl()
            self.mavlink.send_statustext("MISSION_COMPLETE")
            
            logger.info("="*60)
            logger.info("  AUTONOMOUS SEQUENCE COMPLETE")
            logger.info("="*60)
            
            # Reset trigger for next mission
            self.monitor.reset_trigger()
            
        except Exception as e:
            logger.error(f"Autonomous sequence error: {e}")
            import traceback
            traceback.print_exc()
            self.mavlink.command_rtl()
            
    def switch_to_guided(self) -> bool:
        """Switch to GUIDED mode"""
        logger.info("Switching to GUIDED mode...")
        time.sleep(0.5)
        return self.mavlink.set_mode('GUIDED')
        
    def search_for_qr(self) -> Optional[str]:
        """Search for QR code with active scanning pattern"""
        logger.info("Searching for QR code...")
        self.mavlink.send_statustext("SEARCHING_QR")
        
        search_timeout = self.config.QR_SEARCH_TIMEOUT
        start_time = time.time()
        
        # Search pattern: hover and scan, then move if not found
        search_phase = 0  # 0=hover, 1=move_right, 2=move_left, 3=move_forward, 4=move_back
        phase_duration = 5.0  # seconds per phase
        phase_start = start_time
        
        consecutive_detections = 0
        required_detections = 5  # Need 5 consecutive frames
        
        while time.time() - start_time < search_timeout:
            frame = self.camera.get_latest_frame()
            if frame is None:
                logger.warning("No frame available from camera")
                time.sleep(0.2)
                continue
            
            # Verify frame is valid
            if frame.size == 0:
                logger.warning("Empty frame received")
                time.sleep(0.2)
                continue
            
            # Detect QR
            qr_result = self.qr_detector.detect(frame)
            
            # Update display
            if self.config.SHOW_GUI:
                self._update_display(frame, qr_result, 
                                   status=f"SEARCHING Phase {search_phase}")
            
            # Check for stable detection
            if qr_result:
                consecutive_detections += 1
                logger.info(f"QR detected ({consecutive_detections}/{required_detections})")
                
                if consecutive_detections >= required_detections:
                    qr_data = qr_result['data']
                    logger.info(f"QR CODE CONFIRMED: {qr_data}")
                    self.mavlink.send_statustext(f"QR: {qr_data[:40]}")
                    
                    # Stop movement
                    self.mavlink.send_ned_velocity(0, 0, 0)
                    return qr_data
            else:
                consecutive_detections = 0
            
            # Execute search pattern if enabled
            if self.config.QR_SEARCH_PATTERN_ENABLED:
                elapsed_phase = time.time() - phase_start
                
                if elapsed_phase > phase_duration:
                    # Move to next phase
                    search_phase = (search_phase + 1) % 5
                    phase_start = time.time()
                    logger.info(f"Search pattern: Phase {search_phase}")
                    self.mavlink.send_statustext(f"SEARCH_P{search_phase}")
                
                # Execute movement based on phase
                if search_phase == 0:
                    # Hover and scan
                    self.mavlink.send_ned_velocity(0, 0, 0)
                elif search_phase == 1:
                    # Move right
                    self.mavlink.send_ned_velocity(0, 0.2, 0)
                elif search_phase == 2:
                    # Move left
                    self.mavlink.send_ned_velocity(0, -0.2, 0)
                elif search_phase == 3:
                    # Move forward
                    self.mavlink.send_ned_velocity(0.2, 0, 0)
                elif search_phase == 4:
                    # Move back
                    self.mavlink.send_ned_velocity(-0.2, 0, 0)
            
            time.sleep(0.1)
        
        # Timeout reached - stop and continue anyway
        logger.warning(f"QR search timeout ({search_timeout}s) - continuing search...")
        self.mavlink.send_ned_velocity(0, 0, 0)
        self.mavlink.send_statustext("QR_SEARCH_TIMEOUT")
        
        # Don't abort - keep searching indefinitely until QR is found
        return self.search_for_qr_continuous()
    
    def search_for_qr_continuous(self) -> Optional[str]:
        """Continue searching for QR indefinitely until found"""
        logger.info("Entering continuous QR search mode...")
        self.mavlink.send_statustext("QR_CONTINUOUS_SEARCH")
        
        consecutive_detections = 0
        required_detections = 5
        
        while True:  # Search forever until found
            frame = self.camera.get_latest_frame()
            if frame is None or frame.size == 0:
                time.sleep(0.2)
                continue
            
            # Detect QR
            qr_result = self.qr_detector.detect(frame)
            
            # Update display
            if self.config.SHOW_GUI:
                self._update_display(frame, qr_result, 
                                   status="CONTINUOUS SEARCH")
            
            if qr_result:
                consecutive_detections += 1
                logger.info(f"QR detected ({consecutive_detections}/{required_detections})")
                
                if consecutive_detections >= required_detections:
                    qr_data = qr_result['data']
                    logger.info(f"QR CODE FOUND: {qr_data}")
                    self.mavlink.send_statustext(f"QR_FOUND: {qr_data[:40]}")
                    self.mavlink.send_ned_velocity(0, 0, 0)
                    return qr_data
            else:
                consecutive_detections = 0
            
            time.sleep(0.1)
        
    def center_on_qr(self) -> bool:
        """Center drone on QR code using PID control"""
        logger.info("Centering on QR code with PID...")
        self.mavlink.send_statustext("CENTERING")
        
        # Reset PID controllers
        self.pid_x.reset()
        self.pid_y.reset()
        
        start_time = time.time()
        centered_count = 0
        required_centered_frames = 30  # 1 second at 30 FPS
        
        while time.time() - start_time < self.config.CENTERING_TIMEOUT:
            frame = self.camera.get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue
                
            # Detect QR
            qr_result = self.qr_detector.detect(frame)
            
            if qr_result is None:
                # Lost QR - stop and search
                self.mavlink.send_ned_velocity(0, 0, 0)
                logger.warning("Lost QR during centering")
                time.sleep(0.1)
                continue
                
            # Get frame center and QR center
            frame_h, frame_w = frame.shape[:2]
            frame_center_x = frame_w / 2
            frame_center_y = frame_h / 2
            
            qr_center_x, qr_center_y = qr_result['center']
            
            # Calculate errors in pixels
            error_x_px = qr_center_x - frame_center_x
            error_y_px = qr_center_y - frame_center_y
            
            # Estimate distance using QR size (basic approximation)
            qr_width_px = qr_result['bbox'][2]
            # Rough estimate: assume focal length ~500px for 640x480
            focal_length = 500
            distance_m = (self.config.QR_SIZE_CM / 100.0) * focal_length / qr_width_px
            
            # Convert pixel errors to metric errors
            px_to_m = distance_m / focal_length
            error_x_m = error_x_px * px_to_m
            error_y_m = error_y_px * px_to_m
            
            # Check if centered
            error_distance = np.sqrt(error_x_m**2 + error_y_m**2)
            if error_distance < (self.config.LANDING_THRESHOLD_CM / 100.0):
                centered_count += 1
                if centered_count >= required_centered_frames:
                    logger.info("QR centered successfully!")
                    self.mavlink.send_ned_velocity(0, 0, 0)
                    return True
            else:
                centered_count = 0
                
            # PID control
            current_time = time.time()
            vx = self.pid_y.update(error_y_m, current_time)  # Forward/backward (NED X)
            vy = self.pid_x.update(error_x_m, current_time)  # Left/right (NED Y)
            vz = 0.05  # Slow descent
            
            # Send velocity command
            self.mavlink.send_ned_velocity(vx, vy, vz)
            
            # Update display
            if self.config.SHOW_GUI:
                self._update_display(frame, qr_result, error_x_m, error_y_m, distance_m)
                
            # Log progress
            if int(time.time()) % 2 == 0:
                logger.info(f"Centering: error={error_distance*100:.1f}cm, dist={distance_m:.2f}m")
                
            time.sleep(0.05)
            
        logger.error("Centering timeout")
        return False
        
    def execute_landing(self):
        """Execute landing sequence"""
        logger.info("Landing...")
        self.mavlink.send_statustext("LANDING")
        self.mavlink.command_land()
        time.sleep(10)  # Wait for landing
        
    def drop_payload(self):
        """Simulate payload drop"""
        logger.info("Dropping payload...")
        self.mavlink.send_statustext("PAYLOAD_DROP")
        # TODO: Add servo command for real payload release
        # self.mavlink.master.mav.command_long_send(...)
        time.sleep(2)
        
    def _update_display(self, frame, qr_result, error_x=None, error_y=None, distance=None, status=None):
        """Update display frame with overlays"""
        if not self.config.SHOW_GUI:
            return
        
        # Verify frame is valid
        if frame is None or frame.size == 0:
            logger.warning("Invalid frame for display")
            return
            
        display = frame.copy()
        h, w = display.shape[:2]
        
        # Add status text if provided
        if status:
            cv2.putText(display, status, (10, h - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Draw center crosshair
        cv2.line(display, (w//2 - 30, h//2), (w//2 + 30, h//2), (0, 255, 0), 2)
        cv2.line(display, (w//2, h//2 - 30), (w//2, h//2 + 30), (0, 255, 0), 2)
        cv2.circle(display, (w//2, h//2), 50, (0, 255, 0), 2)
        
        # Draw QR detection
        if qr_result:
            x, y, qw, qh = qr_result['bbox']
            cx, cy = qr_result['center']
            
            # QR bounding box
            cv2.rectangle(display, (x, y), (x + qw, y + qh), (0, 255, 255), 3)
            
            # QR center
            cv2.circle(display, (cx, cy), 8, (0, 0, 255), -1)
            cv2.circle(display, (cx, cy), 15, (0, 0, 255), 2)
            
            # Line from frame center to QR center
            cv2.line(display, (w//2, h//2), (cx, cy), (255, 0, 255), 2)
            
            # QR data
            data_text = qr_result['data'][:40]
            cv2.putText(display, f"QR: {data_text}", (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                       
        # Display errors and distance
        info_y = 30
        if error_x is not None and error_y is not None:
            info_text = f"Error: X={error_x*100:.1f}cm Y={error_y*100:.1f}cm"
            cv2.putText(display, info_text, (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            info_y += 30
                       
        if distance is not None:
            dist_text = f"Distance: {distance:.2f}m"
            cv2.putText(display, dist_text, (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            info_y += 30
        
        # Add frame counter
        frame_text = f"Frame: {display.shape}"
        cv2.putText(display, frame_text, (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                   
        with self.display_lock:
            self.display_frame = display
            
    def start_display(self):
        """Start display window (GUI mode)"""
        if not self.config.SHOW_GUI:
            return
            
        logger.info("Starting display window...")
        window_name = "QR Drone - 720p"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        
        no_frame_count = 0
        
        while self.running:
            with self.display_lock:
                frame = self.display_frame
            
            # Show camera feed even if no processing overlay yet
            if frame is None:
                frame = self.camera.get_latest_frame()
                
            if frame is not None and frame.size > 0:
                cv2.imshow(window_name, frame)
                no_frame_count = 0
            else:
                no_frame_count += 1
                if no_frame_count > 50:
                    logger.warning(f"No frames for display ({no_frame_count} iterations)")
                
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("User pressed 'q' - shutting down")
                self.running = False
                break
            elif key == ord('c'):
                # Capture screenshot
                if frame is not None:
                    filename = f"qr_drone_capture_{int(time.time())}.jpg"
                    cv2.imwrite(filename, frame)
                    logger.info(f"Screenshot saved: {filename}")
                
        cv2.destroyAllWindows()
        
    def run(self):
        """Main run loop"""
        if not self.initialize():
            logger.error("Initialization failed")
            return
            
        self.running = True
        
        print("\n" + "="*60)
        print("  QR-GUIDED AUTONOMOUS DRONE SYSTEM")
        print("="*60)
        print(f"  Mode: {'SITL Simulation' if self.config.USE_SITL else 'Real Hardware'}")
        print(f"  Display: {'GUI' if self.config.SHOW_GUI else 'Headless'}")
        print("="*60)
        print("  System Status: READY")
        print("  Waiting for DO_SPRAYER trigger in AUTO mission...")
        print("="*60 + "\n")
        
        try:
            if self.config.SHOW_GUI:
                # Run display in main thread
                self.start_display()
            else:
                # Headless - just wait
                while self.running:
                    time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt")
        finally:
            self.shutdown()
            
    def shutdown(self):
        """Shutdown all systems"""
        logger.info("Shutting down...")
        self.running = False
        
        self.monitor.stop()
        self.camera.stop()
        self.mavlink.close()
        
        cv2.destroyAllWindows()
        logger.info("Shutdown complete")


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """Main entry point"""
    # Configuration
    config = SystemConfig()
    
    # Parse command line arguments (optional)
    if len(sys.argv) > 1:
        if '--sitl' in sys.argv:
            config.USE_SITL = True
        if '--real' in sys.argv:
            config.USE_SITL = False
        if '--headless' in sys.argv:
            config.SHOW_GUI = False
        if '--gui' in sys.argv:
            config.SHOW_GUI = True
            
    # Create and run controller
    controller = AutonomousController(config)
    controller.run()


if __name__ == "__main__":
    main()
