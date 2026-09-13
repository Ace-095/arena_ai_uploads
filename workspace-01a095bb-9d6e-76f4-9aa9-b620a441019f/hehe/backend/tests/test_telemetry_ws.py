import os
os.environ["MAVLINK20"] = "1"
import asyncio
import json
import threading
import time
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Set environment variables for config before importing app
os.environ["MAVLINK_DEVICE"] = f"udpout:127.0.0.1:14561"
os.environ["MAVLINK_BAUD"] = "115200"
os.environ["MAVLINK_TIMEOUT_S"] = "15.0"
os.environ["TELEMETRY_POSITION_HZ"] = "5.0"
os.environ["TELEMETRY_STATS_HZ"] = "1.0"
os.environ["FENCE_ACTION"] = "1"

import uvicorn
import requests
from pymavlink import mavutil

# Mock flight controller setup
PORT = 14561
SERVER_PORT = 8001
stop_event = threading.Event()
server_thread = None


def run_mock_fc():
    print(f"[Mock FC] Starting on udp:0.0.0.0:{PORT}")
    conn = mavutil.mavlink_connection(f'udp:0.0.0.0:{PORT}', input=True)
    last_hb = 0.0
    last_pos = 0.0
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0, 0, mavutil.mavlink.MAV_STATE_STANDBY,
    )
    last_hb = time.time()
    print("[Mock FC] Sent initial heartbeat")

    while not stop_event.is_set():
        now = time.time()
        msg = conn.recv_match(blocking=True, timeout=0.05)
        if msg:
            conn.target_system = msg.get_srcSystem()
            conn.target_component = msg.get_srcComponent()

        if now - last_hb > 1.0:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                0, 0, mavutil.mavlink.MAV_STATE_STANDBY,
            )
            last_hb = now

        if now - last_pos > 0.2 and getattr(conn, 'target_system', 0) != 0:
            conn.mav.global_position_int_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                int(12.9716 * 1e7),
                int(77.5946 * 1e7),
                10000,
                5000,
                0, 0, 0,
                0
            )
            last_pos = now


def run_server():
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import app
    uvicorn.run(app.app, host="127.0.0.1", port=SERVER_PORT, log_level="info")


async def test_telemetry_ws():
    global server_thread

    fc_thread = threading.Thread(target=run_mock_fc, daemon=True)
    fc_thread.start()
    time.sleep(2.0)

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    time.sleep(4.0)

    try:
        for _ in range(20):
            try:
                r = requests.get(f"http://127.0.0.1:{SERVER_PORT}/health", timeout=1)
                if r.status_code == 200:
                    print("[Test] Server is ready")
                    break
            except:
                time.sleep(0.5)
        else:
            print("[Test] Server didn't start in time")
            return False

        import websockets
        uri = f"ws://127.0.0.1:{SERVER_PORT}/ws/telemetry"
        print(f"[Test] Connecting to {uri}")

        try:
            async with websockets.connect(uri, origin=f"http://127.0.0.1:{SERVER_PORT}") as websocket:
                print("[Test] WebSocket connected successfully")

                # Verify 5 envelope messages
                channels_received = set()
                for i in range(5):
                    message = await asyncio.wait_for(websocket.recv(), timeout=10.0)
                    print(f"[Test] Received envelope {i+1}: {message}")
                    data = json.loads(message)
                    assert "channel" in data, "Envelope missing 'channel'"
                    assert "t" in data, "Envelope missing 't'"
                    assert "data" in data, "Envelope missing 'data'"
                    channels_received.add(data["channel"])

                print(f"[Test] Channels received: {channels_received}")

                # Test push_event
                from telemetry import push_event
                push_event("TEST_EVENT", "sample_payload")

                # Receive pushed event
                event_received = False
                for _ in range(5):
                    message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    data = json.loads(message)
                    if data.get("channel") == "event":
                        assert data["data"]["type"] == "TEST_EVENT"
                        assert data["data"]["value"] == "sample_payload"
                        event_received = True
                        print("[Test] Event envelope verified successfully!")
                        break

                assert event_received, "Event envelope was not received"
                print("[Test] All telemetry envelope tests passed successfully!")
                return True

        except Exception as e:
            print(f"[Test] Critical failure: {e}")
            return False

    finally:
        print("[Test] Stopping...")
        stop_event.set()
        fc_thread.join(timeout=2.0)
        print("[Test] Mock FC stopped")


if __name__ == "__main__":
    success = asyncio.run(test_telemetry_ws())
    exit(0 if success else 1)