import time
import pytest
from unittest.mock import MagicMock, patch

from config import config
from models import GeofenceStatus
import fsm
import state

@pytest.fixture
def mock_bus():
    bus = MagicMock()
    bus.conn = MagicMock()
    bus.conn.target_system = 1
    bus.conn.target_component = 1
    return bus

def test_fsm_initial_state(mock_bus):
    mission_fsm = fsm.MissionFSM(mock_bus)
    status = mission_fsm.get_status()
    assert status.state == "INIT"

def test_fsm_abort(mock_bus):
    mission_fsm = fsm.MissionFSM(mock_bus)
    status = mission_fsm.abort()
    assert status.state == "FAILSAFE"

@patch('fsm.state.get')
def test_fsm_pre_flight_check_fail_no_fence(mock_state_get, mock_bus):
    mock_state_get.return_value = None
    config.FSM_PREFLIGHT_TIMEOUT_S = 0.5

    mission_fsm = fsm.MissionFSM(mock_bus)
    mission_fsm.start()

    time.sleep(1.0)
    status = mission_fsm.get_status()

    # FAILSAFE immediately triggers RTL as its own corrective action (see
    # fsm.py's FAILSAFE state handler: it calls stop_search() then
    # transitions straight to RTL on its very next loop tick). By the time
    # we check status a full second later, the FSM has almost certainly
    # already moved past FAILSAFE into RTL — that's correct, intentional
    # behavior (FAILSAFE should not sit idle), not a bug. A prior version
    # of this test asserted state == "FAILSAFE" only, which was really
    # asserting a timing race rather than the actual contract.
    assert status.state in ("FAILSAFE", "RTL"), (
        f"expected pre-flight failure to reach FAILSAFE or its immediate "
        f"RTL follow-up, got '{status.state}'"
    )

@patch('fsm.state.get')
def test_fsm_pre_flight_check_pass(mock_state_get, mock_bus):
    fence_status = GeofenceStatus(loaded=True, armable=True, reason="ok")
    mock_state_get.return_value = fence_status
    
    mission_fsm = fsm.MissionFSM(mock_bus)
    # Simulate a very recent heartbeat
    mission_fsm._last_heartbeat = time.time()
    
    mission_fsm.start()
    time.sleep(0.5)
    
    status = mission_fsm.get_status()
    assert status.state in ["ARM", "TAKEOFF"]

