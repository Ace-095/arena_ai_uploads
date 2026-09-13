#!/usr/bin/env python3
"""
integrated_qr_final_updated.py

Final integrated QR-guidance script (patched).
Features:
 - Robust mission trigger logic (accepts MAV_CMD_DO_SPRAYER and fallbacks)
 - MISSION_REQUEST_INT + MISSION_REQUEST fallback with caching (no races)
 - Pyzbar-only QR detection
 - Motion-adapted frame fusion
 - Simple test sequence (test4.py style) and full GuidedApproach
 - Improved land fallback when no GLOBAL_POSITION_INT available
 - Pi Camera 3 Wide (Picamera2) preferred, OpenCV fallback
 - Preview auto-disabled if no X DISPLAY
"""

import argparse
import time
import math
import threading
import queue
import sys
import os
import cv2
import numpy as np
from dataclasses import dataclass
from collections import deque
from pyzbar import pyzbar
from pymavlink import mavutil

# -------------------------
# Configurable parameters
# -------------------------
IMAGE_W = 1280
IMAGE_H = 720
HFOV_DEG = 120.0  # Pi Camera 3 Wide (approx)
FOCAL_PIXELS = (IMAGE_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))

SEARCH_ALT = 6.0
FINAL_LAND_ALT = 0.6
MAX_SEARCH_TIME = 45

CENTER_PIXEL_TOL = 40
APPROACH_RATE = 0.4

USE_FUSION = True
FUSION_AVG_FRAMES = 3
FUSION_MED_FRAMES = 5

VERBOSE = True
ABORT_ON_FAIL = True

PAYLOAD_SERVO_CHANNEL = 9
PAYLOAD_RELEASE_PWM = 1100
SERVO_RELEASE_DELAY = 2.0

MAVLINK_CONN_SIM = "tcp:127.0.0.1:5762"
MAVLINK_CONN_REAL = "serial:/dev/serial0:57600"

VELOCITY_CMD_RATE = 5.0

# -------------------------
# Shared queues & events
# -------------------------
detection_queue = queue.Queue(maxsize=4)
mission_event_queue = queue.Queue(maxsize=16)
abort_event = threading.Event()

# -------------------------
# Data classes
# -------------------------
@dataclass
class Detection:
    data: str
    bbox: tuple
    timestamp: float

# -------------------------
# DO_SPRAYER constant handling
# -------------------------
try:
    MAV_CMD_DO_SPRAYER = mavutil.mavlink.MAV_CMD_DO_SPRAYER
except Exception:
    MAV_CMD_DO_SPRAYER = None  # we'll compare to known integers later

# -------------------------
# Vision subsystem (pyzbar-only)
# -------------------------
class QRVision:
    def __init__(self, resolution=(IMAGE_W, IMAGE_H), show_preview=False):
        self.resolution = resolution
        self.show_preview = show_preview and bool(os.environ.get("DISPLAY"))
        self.camera = None
        self.running = False

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_detection = None

        self.prev_gray = None
        self.frame_buffer = deque(maxlen=FUSION_MED_FRAMES)

    def init_camera(self):
        # Picamera2 preferred
        try:
            from picamera2 import Picamera2
            self.camera = Picamera2()
            cfg = self.camera.create_preview_configuration(
                main={"size": self.resolution, "format": "RGB888"}
            )
            self.camera.configure(cfg)
            self.camera.start()
            time.sleep(0.6)
            if VERBOSE:
                print("[vision] Picamera2 initialized")
            return True
        except Exception as e:
            if VERBOSE:
                print("[vision] Picamera2 not available:", e)

        # Fallback to OpenCV
        try:
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            ret, _ = cap.read()
            if ret:
                self.camera = cap
                if VERBOSE:
                    print("[vision] OpenCV camera initialized")
                return True
        except Exception as e:
            if VERBOSE:
                print("[vision] OpenCV init failed:", e)

        print("[vision] Camera init failed")
        return False

    def capture_frame(self):
        try:
            if hasattr(self.camera, "capture_array"):
                arr = self.camera.capture_array("main")
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            ret, frame = self.camera.read()
            return frame if ret else None
        except Exception:
            return None

    def calc_motion(self, gray):
        if self.prev_gray is None:
            self.prev_gray = gray
            return 0.0
        try:
            flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None,
                                                0.5, 3, 15, 3, 5, 1.2, 0)
            mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            score = min(np.mean(mag) / 8.0, 1.0)
            self.prev_gray = gray
            return score
        except Exception:
            self.prev_gray = gray
            return 0.0

    def fusion(self, frame, motion):
        if not USE_FUSION:
            return frame
        self.frame_buffer.append(frame.astype(np.float32))
        if 0.25 <= motion < 0.6 and len(self.frame_buffer) >= FUSION_AVG_FRAMES:
            avg = np.mean(list(self.frame_buffer)[-FUSION_AVG_FRAMES:], axis=0)
            return np.clip(avg, 0, 255).astype(np.uint8)
        if motion >= 0.6 and len(self.frame_buffer) >= FUSION_MED_FRAMES:
            stack = np.stack(list(self.frame_buffer)[-FUSION_MED_FRAMES:], axis=0)
            med = np.median(stack, axis=0)
            return med.astype(np.uint8)
        return frame

    def detect_py(self, gray):
        out = []
        try:
            barcodes = pyzbar.decode(gray)
            for b in barcodes:
                try:
                    data = b.data.decode("utf-8", errors="ignore")
                except Exception:
                    data = str(b.data)
                x, y, w, h = int(b.rect.left), int(b.rect.top), int(b.rect.width), int(b.rect.height)
                out.append(Detection(data=data, bbox=(x, y, w, h), timestamp=time.time()))
        except Exception:
            pass
        return out

    def centroid(self, bbox):
        x, y, w, h = bbox
        return int(x + w / 2), int(y + h / 2)

    def pixel_to_meter(self, px, py, alt):
        cx = IMAGE_W / 2.0
        cy = IMAGE_H / 2.0
        dx = px - cx
        dy = py - cy
        north = -(dy / FOCAL_PIXELS) * alt
        east = (dx / FOCAL_PIXELS) * alt
        return north, east

    def detect_once(self):
        frame = self.capture_frame()
        if frame is None:
            return None, [], 0.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        motion = self.calc_motion(gray)
        fused = self.fusion(frame, motion)
        fused_gray = cv2.cvtColor(fused, cv2.COLOR_BGR2GRAY)
        if np.std(fused_gray) > 20:
            try:
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                proc = clahe.apply(fused_gray)
            except Exception:
                proc = fused_gray
        else:
            proc = fused_gray
        dets = self.detect_py(proc)
        with self.frame_lock:
            self.latest_frame = fused
            self.latest_detection = dets[0] if dets else None
        return fused, dets, motion

    def stop(self):
        try:
            if hasattr(self.camera, "stop"):
                self.camera.stop()
            elif hasattr(self.camera, "release"):
                self.camera.release()
        except Exception:
            pass

# -------------------------
# MAV Helper with mission download & cache
# -------------------------
class MAVHelper:
    def __init__(self, conn_str):
        self.conn_str = conn_str
        self.master = None
        self.lock = threading.Lock()
        self.last_gpos = None
        self.last_local = None
        self._reader = None
        self._stop_reader = threading.Event()
        self.mission_items = {}  # seq -> mission item message

    def connect(self, timeout=12):
        try:
            self.master = mavutil.mavlink_connection(self.conn_str, autoreconnect=True, robust_parsing=True)
            self.master.wait_heartbeat(timeout=timeout)
            if VERBOSE:
                print(f"[mav] Heartbeat from sys {self.master.target_system}")
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()
            return True
        except Exception as e:
            print("[mav] connect failed:", e)
            return False

    def _read_loop(self):
        # Background reader: collect positions and mission events
        while not self._stop_reader.is_set():
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                t = msg.get_type()
                if t == "GLOBAL_POSITION_INT":
                    self.last_gpos = msg
                elif t == "LOCAL_POSITION_NED":
                    self.last_local = msg
                elif t == "MISSION_ITEM_REACHED":
                    mission_event_queue.put(msg)
                # intentionally skip MISSION_ITEM / MISSION_ITEM_INT to avoid race
            except Exception:
                time.sleep(0.02)

    def download_mission(self, timeout=10.0):
        """Download and cache mission items robustly."""
        if self.master is None:
            print("[mon] download_mission: no connection")
            return False
        self.mission_items = {}
        try:
            with self.lock:
                self.master.mav.mission_request_list_send(self.master.target_system, self.master.target_component)
            if VERBOSE:
                print("[mon] Sent MISSION_REQUEST_LIST")
            end_t = time.time() + timeout
            mission_count = None
            while time.time() < end_t:
                m = self.master.recv_match(blocking=True, timeout=1.0)
                if m is None:
                    continue
                if m.get_type() == "MISSION_COUNT":
                    mission_count = getattr(m, "count", None)
                    if VERBOSE:
                        print(f"[mon] MISSION_COUNT = {mission_count}")
                    break
            if mission_count is None:
                print("[mon] download_mission: no MISSION_COUNT received")
                return False
            for seq in range(int(mission_count)):
                got = None
                # Try MISSION_REQUEST_INT (preferred)
                try:
                    with self.lock:
                        self.master.mav.mission_request_int_send(self.master.target_system, self.master.target_component, int(seq))
                    if VERBOSE:
                        print(f"[mon] Sent MISSION_REQUEST_INT({seq})")
                except Exception:
                    pass
                t0 = time.time()
                while time.time() - t0 < 1.2:
                    m = self.master.recv_match(blocking=True, timeout=0.9)
                    if m is None:
                        continue
                    mt = m.get_type()
                    if mt in ("MISSION_ITEM_INT", "MISSION_ITEM") and getattr(m, "seq", None) == seq:
                        got = m
                        break
                # Fallback to non-int request
                if got is None:
                    try:
                        with self.lock:
                            self.master.mav.mission_request_send(self.master.target_system, self.master.target_component, int(seq))
                        if VERBOSE:
                            print(f"[mon] Sent MISSION_REQUEST({seq})")
                    except Exception:
                        pass
                    t0 = time.time()
                    while time.time() - t0 < 1.2:
                        m = self.master.recv_match(blocking=True, timeout=0.9)
                        if m is None:
                            continue
                        mt = m.get_type()
                        if mt in ("MISSION_ITEM_INT", "MISSION_ITEM") and getattr(m, "seq", None) == seq:
                            got = m
                            break
                if got is None:
                    print(f"[mon] download_mission: failed to fetch seq {seq}")
                else:
                    self.mission_items[int(seq)] = got
                    if VERBOSE:
                        print(f"[mon] Cached seq={seq} cmd={getattr(got,'command',None)}")
            if VERBOSE:
                print(f"[mon] Mission download complete: cached {len(self.mission_items)} items")
            return True
        except Exception as e:
            print("[mon] download_mission error:", e)
            return False

    def get_mission_item(self, seq, timeout=4.0):
        """On-demand mission item retrieval if cache lacks it."""
        if self.master is None:
            return None
        end_t = time.time() + timeout
        candidates = [int(seq)]
        if seq > 0:
            candidates.append(int(seq - 1))
        candidates.append(int(seq + 1))
        # Try INT first
        for c in candidates:
            if time.time() > end_t:
                break
            try:
                with self.lock:
                    self.master.mav.mission_request_int_send(self.master.target_system, self.master.target_component, int(c))
                if VERBOSE:
                    print(f"[mon] Sent MISSION_REQUEST_INT({c})")
            except Exception:
                pass
            t0 = time.time()
            while time.time() - t0 < 1.2 and time.time() < end_t:
                try:
                    m = self.master.recv_match(blocking=True, timeout=0.8)
                    if m is None:
                        continue
                    if m.get_type() in ("MISSION_ITEM_INT", "MISSION_ITEM") and getattr(m, "seq", None) == c:
                        if VERBOSE:
                            print(f"[mon] Received {m.get_type()} seq={c}")
                        return m
                except Exception:
                    pass
        # fallback non-int
        for c in candidates:
            if time.time() > end_t:
                break
            try:
                with self.lock:
                    self.master.mav.mission_request_send(self.master.target_system, self.master.target_component, int(c))
                if VERBOSE:
                    print(f"[mon] Sent MISSION_REQUEST({c})")
            except Exception:
                pass
            t0 = time.time()
            while time.time() - t0 < 1.2 and time.time() < end_t:
                try:
                    m = self.master.recv_match(blocking=True, timeout=0.8)
                    if m is None:
                        continue
                    if m.get_type() in ("MISSION_ITEM_INT", "MISSION_ITEM") and getattr(m, "seq", None) == c:
                        if VERBOSE:
                            print(f"[mon] Received {m.get_type()} seq={c}")
                        return m
                except Exception:
                    pass
        if VERBOSE:
            print("[mon] get_mission_item timed out")
        return None

    def set_mode_guided(self):
        try:
            with self.lock:
                self.master.set_mode("GUIDED")
            if VERBOSE:
                print("[mav] GUIDED requested")
            return True
        except Exception as e:
            print("[mav] set_mode_guided error:", e)
            return False

    def send_velocity_body(self, vx, vy, vz, duration=1.0):
        if self.master is None:
            return
        # Build type_mask: we will set velocities, ignore positions/accelerations/yaw
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        end_t = time.time() + float(duration)
        while time.time() < end_t and not abort_event.is_set():
            try:
                with self.lock:
                    # time_boot_ms = 0 acceptable
                    self.master.mav.set_position_target_local_ned_send(
                        0,
                        int(self.master.target_system),
                        int(self.master.target_component),
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        int(type_mask),
                        0.0, 0.0, 0.0,  # x,y,z positions (ignored)
                        float(vx), float(vy), float(vz),  # vx,vy,vz
                        0.0, 0.0, 0.0,  # accelerations (ignored)
                        0.0, 0.0  # yaw, yaw_rate
                    )
                time.sleep(1.0 / VELOCITY_CMD_RATE)
            except Exception as e:
                print("[mav] velocity send error:", e)
                break

    def send_ned_position(self, north, east, down, relative=True):
        if self.master is None:
            return
        try:
            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
            if self.last_local:
                base_n = float(self.last_local.x)
                base_e = float(self.last_local.y)
                base_d = float(self.last_local.z)
            else:
                base_n = base_e = base_d = 0.0
            with self.lock:
                self.master.mav.set_position_target_local_ned_send(
                    0,
                    int(self.master.target_system),
                    int(self.master.target_component),
                    frame,
                    int(type_mask),
                    float(base_n + float(north)),
                    float(base_e + float(east)),
                    float(base_d + float(down)),
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0
                )
        except Exception as e:
            print("[mav] send_ned_position error:", e)

    def land_now(self):
        """
        Land using known global position if available; otherwise issue a generic LAND
        and request LAND mode as fallback (addresses "missing pos").
        """
        if self.master is None:
            print("[mav] cannot land: no connection")
            return False
        try:
            if self.last_gpos is not None:
                lat = float(self.last_gpos.lat) / 1e7
                lon = float(self.last_gpos.lon) / 1e7
                alt = float(self.last_gpos.alt) / 1000.0
                self.master.mav.command_long_send(
                    int(self.master.target_system),
                    int(self.master.target_component),
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0,
                    0, 0, 0, 0,
                    float(lat), float(lon), float(alt)
                )
                if VERBOSE:
                    print(f"[mav] LAND (lat,lon,alt): {lat:.6f},{lon:.6f},{alt:.2f} sent")
                return True
            else:
                # Fallback: send generic LAND + request LAND mode
                print("[mav] LAND fallback: no global pos available — sending generic LAND and requesting LAND mode")
                try:
                    self.master.mav.command_long_send(
                        int(self.master.target_system),
                        int(self.master.target_component),
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0,
                        0, 0, 0, 0,
                        0.0, 0.0, 0.0
                    )
                except Exception as e:
                    print("[mav] LAND fallback command failed:", e)
                try:
                    self.master.set_mode("LAND")
                    if VERBOSE:
                        print("[mav] Requested LAND mode")
                except Exception as e:
                    print("[mav] set LAND mode failed:", e)
                return True
        except Exception as e:
            print("[mav] LAND failed:", e)
            return False

    def release_payload(self, simulated=True):
        if simulated:
            print(f"[mav] (SIM) releasing payload servo ch{PAYLOAD_SERVO_CHANNEL} pwm={PAYLOAD_RELEASE_PWM}")
            return True
        try:
            self.master.mav.command_long_send(
                int(self.master.target_system),
                int(self.master.target_component),
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                int(PAYLOAD_SERVO_CHANNEL),
                float(PAYLOAD_RELEASE_PWM),
                0, 0, 0, 0, 0
            )
            print("[mav] Payload release sent")
            return True
        except Exception as e:
            print("[mav] payload release failed:", e)
            return False

    def rtl(self):
        try:
            self.master.mav.command_long_send(
                int(self.master.target_system),
                int(self.master.target_component),
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            print("[mav] RTL commanded")
        except Exception as e:
            print("[mav] RTL failed:", e)

# -------------------------
# Simple test flow (test4.py style)
# -------------------------
def simple_test_autonomous_sequence(mav: MAVHelper,
                                    approach_vx=0.3,
                                    approach_duration=8.0,
                                    approach_interval=0.5,
                                    simulated=True):
    """
    Simple approach -> land -> payload -> RTL flow.
    """
    print("\n" + "=" * 48)
    print("  SIMPLE AUTONOMOUS SEQUENCE (test4.py style) START")
    print("=" * 48 + "\n")

    # Notify GCS
    try:
        if mav.master:
            mav.master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                b"QR_DETECTED".ljust(50, b'\0')
            )
    except Exception:
        pass

    start_t = time.time()
    next_cmd = start_t
    while time.time() - start_t < approach_duration and not abort_event.is_set():
        now = time.time()
        if now >= next_cmd:
            mav.send_velocity_body(float(approach_vx), 0.0, 0.0, duration=float(approach_interval))
            next_cmd = now + approach_interval
        time.sleep(0.02)

    try:
        mav.send_velocity_body(0.0, 0.0, 0.0, duration=0.5)
    except Exception:
        pass

    print("[test] Approach finished; sending LAND")
    mav.land_now()

    time.sleep(6.0)

    if simulated:
        print("[test] (SIM) payload release simulated")
    else:
        print("[test] Releasing payload (servo)")
        mav.release_payload(simulated=False)

    time.sleep(1.0)
    print("[test] Commanding RTL")
    mav.rtl()

    print("\n" + "=" * 48)
    print("  SIMPLE AUTONOMOUS SEQUENCE COMPLETE")
    print("=" * 48 + "\n")

# -------------------------
# Guided approach (centering + precise landing)
# -------------------------
class GuidedApproach:
    def __init__(self, mav: MAVHelper, vision: QRVision, simulated=True):
        self.mav = mav
        self.vision = vision
        self.simulated = simulated

    def descend_to_search_alt(self, target_alt=SEARCH_ALT, timeout=12.0):
        print(f"[ctl] Descend to {target_alt} m")
        t_end = time.time() + timeout
        while time.time() < t_end and not abort_event.is_set():
            self.mav.send_velocity_body(0.0, 0.0, 0.4, duration=1.0)
            if not detection_queue.empty():
                return True
        return False

    def pixel_centering_loop(self, max_iters=8):
        print("[ctl] Centering loop")
        for i in range(max_iters):
            try:
                det = detection_queue.get(timeout=6.0)
            except queue.Empty:
                print("[ctl] Centering: no detection")
                return False
            x_c, y_c = self.vision.centroid(det.bbox)
            cx = IMAGE_W / 2.0; cy = IMAGE_H / 2.0
            dx_pix = x_c - cx; dy_pix = y_c - cy
            if abs(dx_pix) <= CENTER_PIXEL_TOL and abs(dy_pix) <= CENTER_PIXEL_TOL:
                print("[ctl] Centered")
                return True
            north_m, east_m = self.vision.pixel_to_meter(x_c, y_c, SEARCH_ALT)
            vx = max(-APPROACH_RATE, min(APPROACH_RATE, north_m))
            vy = max(-APPROACH_RATE, min(APPROACH_RATE, east_m))
            dur = min(3.0, max(0.5, math.hypot(north_m, east_m) / max(0.05, APPROACH_RATE)))
            print(f"[ctl] Correct N:{north_m:.2f} E:{east_m:.2f} -> vx:{vx:.2f} vy:{vy:.2f} dur:{dur:.1f}")
            self.mav.send_velocity_body(vx, vy, 0.0, duration=dur)
            time.sleep(0.3)
        print("[ctl] Center failed")
        return False

    def final_descend_and_land(self):
        print("[ctl] Final descend")
        t0 = time.time()
        while time.time() - t0 < 30 and not abort_event.is_set():
            if self.mav.last_gpos and hasattr(self.mav.last_gpos, "relative_alt"):
                try:
                    alt = float(self.mav.last_gpos.relative_alt) / 1000.0
                    if alt <= FINAL_LAND_ALT:
                        print(f"[ctl] Low alt {alt:.2f}m -> LAND")
                        self.mav.land_now()
                        return True
                except Exception:
                    pass
            self.mav.send_velocity_body(0.0, 0.0, 0.2, duration=1.0)
            time.sleep(0.2)
        print("[ctl] Final descend timeout -> LAND")
        self.mav.land_now()
        return True

    def execute_release_and_rtl(self):
        print("[ctl] Releasing payload")
        self.mav.release_payload(simulated=self.simulated)
        time.sleep(SERVO_RELEASE_DELAY)
        print("[ctl] Payload released -> RTL")
        self.mav.rtl()

    def run_full_sequence(self, timeout=MAX_SEARCH_TIME):
        print("[ctl] Running guided sequence")
        if not self.mav.set_mode_guided():
            print("[ctl] GUIDED set failed")
            if ABORT_ON_FAIL:
                self.mav.rtl()
            return False
        self.descend_to_search_alt(SEARCH_ALT)
        found = False
        start = time.time()
        while time.time() - start < timeout and not abort_event.is_set():
            try:
                det = detection_queue.get(timeout=2.0)
                if det:
                    print("[ctl] Detection:", det.data)
                    try:
                        detection_queue.put(det, timeout=0.5)
                    except Exception:
                        pass
                    found = True
                    break
            except queue.Empty:
                print("[ctl] No detection yet; small sweep")
                self.mav.send_velocity_body(0.2, 0.0, 0.0, duration=1.0)
                time.sleep(0.2)
        if not found:
            print("[ctl] Search timeout")
            if ABORT_ON_FAIL:
                self.mav.rtl()
            return False
        centered = self.pixel_centering_loop()
        if not centered:
            print("[ctl] Center failed")
            if ABORT_ON_FAIL:
                self.mav.rtl()
            return False
        landed = self.final_descend_and_land()
        if not landed:
            print("[ctl] Land failed")
            if ABORT_ON_FAIL:
                self.mav.rtl()
            return False
        self.execute_release_and_rtl()
        return True

# -------------------------
# Threads: vision & mission monitor
# -------------------------
def vision_thread_fn(vision: QRVision):
    vision.running = True
    while vision.running and not abort_event.is_set():
        frame, dets, motion = vision.detect_once()
        if dets:
            try:
                detection_queue.put_nowait(dets[0])
                if VERBOSE:
                    print(f"[vision] Queued detection: {dets[0].data} motion={motion:.2f}")
            except queue.Full:
                try:
                    _ = detection_queue.get_nowait()
                    detection_queue.put_nowait(dets[0])
                except Exception:
                    pass
        time.sleep(0.05)
    vision.stop()
    print("[vision] stopped")

def mission_monitor_thread(mav: MAVHelper, guided: GuidedApproach):
    print("[mon] Mission monitor started. Waiting for MISSION_ITEM_REACHED.")
    while True:
        try:
            msg = mission_event_queue.get(block=True)
        except Exception:
            continue
        if msg is None:
            continue
        seq = getattr(msg, "seq", None)
        print(f"[mav] Mission item reached: {seq}")

        # Try cached mission item first
        got = None
        if hasattr(mav, "mission_items") and mav.mission_items:
            try:
                got = mav.mission_items.get(int(seq), None)
            except Exception:
                got = None
            if got is not None and VERBOSE:
                print(f"[mon] Found cached mission item seq={seq} cmd={getattr(got,'command',None)}")

        # fallback to on-demand fetch
        if got is None:
            got = mav.get_mission_item(seq, timeout=5.0)
            if got is None:
                print("[mon] Could not retrieve mission item for seq", seq)
                continue

        cmd = getattr(got, "command", None)
        print(f"[mon] Item {seq} command ID = {cmd}")

        # Accept known fallbacks plus the MAV constant if available.
        valid_sprayer_cmds = {216, 222, 42600}
        if MAV_CMD_DO_SPRAYER is not None:
            try:
                valid_sprayer_cmds.add(int(MAV_CMD_DO_SPRAYER))
            except Exception:
                pass

        if cmd in valid_sprayer_cmds:
            print(f"[mon] DO_SPRAYER reached (cmd={cmd}) - launching handler")
            # Default: run simple test flow (from test4.py)
            t_simple = threading.Thread(target=simple_test_autonomous_sequence,
                                        args=(mav, 0.3, 8.0, 0.5, True),
                                        daemon=True)
            t_simple.start()

            # To use the full centering guided approach instead, comment the previous block and uncomment:
            # t_guided = threading.Thread(target=guided.run_full_sequence, daemon=True)
            # t_guided.start()

        else:
            print("[mon] Not sprayer -> ignoring")

# -------------------------
# Main entrypoint
# -------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="Run in SITL simulated mode")
    parser.add_argument("--mavlink", type=str, default=None, help="MAVLink connection string override")
    parser.add_argument("--preview", action="store_true", help="Show camera preview (requires X)")
    args = parser.parse_args()

    # default: if --sim given, simulated; else real. If omitted, default to sim.
    simulated = args.sim or True
    conn = args.mavlink if args.mavlink else (MAVLINK_CONN_SIM if simulated else MAVLINK_CONN_REAL)

    print(f"[main] Mode simulated = {simulated}  MAVLink = {conn}")

    vision = QRVision(resolution=(IMAGE_W, IMAGE_H), show_preview=args.preview)
    if not vision.init_camera():
        print("[main] Camera init failed - exiting")
        return

    mav = MAVHelper(conn)
    if not mav.connect():
        print("[main] MAV connect failed - ensure SITL reachable")
        vision.stop()
        return

    # Download & cache mission items (avoid race later)
    ok = mav.download_mission(timeout=10.0)
    if not ok:
        print("[main] Warning: mission download incomplete; will attempt on-demand fetches")

    guided = GuidedApproach(mav=mav, vision=vision, simulated=simulated)

    vt = threading.Thread(target=vision_thread_fn, args=(vision,), daemon=True)
    vt.start()

    mt = threading.Thread(target=mission_monitor_thread, args=(mav, guided), daemon=True)
    mt.start()

    print("[main] System ready. Trigger DO_SPRAYER mission item in Mission Planner to start approach.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[main] Keyboard interrupt - shutting down")
        abort_event.set()
        vision.stop()
        time.sleep(0.5)

if __name__ == "__main__":
    main()
