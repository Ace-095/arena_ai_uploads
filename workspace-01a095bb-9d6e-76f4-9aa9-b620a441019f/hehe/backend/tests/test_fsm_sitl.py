import os
import time
import logging

os.environ.setdefault("MAVLINK20", "1")
os.environ["FSM_PREFLIGHT_TIMEOUT_S"] = "30.0"

from config import config
import state
from models import GeofenceStatus
import fsm
from mavlink_bus import MavlinkBus

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

def main():
    print("--- Starting Tier 1: Full FSM SITL + Gazebo Acceptance Test ---")
    
    # Needs Gazebo SITL running in the background (e.g. sim_vehicle.py -v ArduCopter -f gazebo-iris --console)
    bus = MavlinkBus(device="udp:127.0.0.1:14550", timeout_s=5.0)
    bus.start()
    
    time.sleep(2.0)
    
    # Fake geofence so it can arm
    fence = GeofenceStatus(loaded=True, armable=True, reason="SITL OK")
    state.set(fence)
    
    mission = fsm.init_fsm(bus)
    
    print("Starting FSM...")
    mission.start()
    
    try:
        while True:
            status = mission.get_status()
            print(f"Current State: {status.state} | Reason: {status.reason} | Alt: {mission._last_alt_rel:.2f}m")
            
            if status.state == "MISSION_COMPLETE":
                print("Test passed: Reached MISSION_COMPLETE successfully!")
                break
            elif status.state == "FAILSAFE":
                print(f"Test failed: Entered FAILSAFE. Reason: {status.reason}")
                break
                
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("Aborting...")
        mission.abort()
        time.sleep(2.0)
    finally:
        bus.stop()
        
if __name__ == "__main__":
    main()
