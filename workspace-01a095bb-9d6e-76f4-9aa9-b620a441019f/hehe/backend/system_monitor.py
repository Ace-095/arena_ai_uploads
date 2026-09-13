"""
system_monitor.py — Pi hardware health monitoring (CPU/RAM/disk/temp).

Confirmed gap against the original project notes: the UI was supposed to
show "System Info" for the Pi's own health, and nothing existed for this
anywhere in the codebase (grepped for psutil/cpu_percent/system_info
across the whole repo before writing this — zero matches).

Uses psutil for the well-established cross-platform stats (CPU%, RAM%,
disk%) and reads the standard Linux thermal sysfs path for temperature —
this works on any Linux system with a thermal zone exposed, not a
Raspberry-Pi-specific trick, though thermal_zone0 is the specific zone
Raspberry Pi OS uses for the SoC.

AI accelerator (Hailo) status is best-effort and clearly marked as such:
NEED TEST — HailoRT's Python API surface for querying live
utilization/temperature was not verified against real hardware or its
SDK docs (no Hailo hardware available in this dev environment). Falls
back to shelling out to `hailortcli`, the documented CLI tool, and
degrades gracefully to "unavailable" if that's not installed either
(e.g. running this on a dev laptop with no Hailo hardware at all).
"""

import logging
import subprocess
import time
from typing import Optional, Tuple

import psutil
from pydantic import BaseModel

log = logging.getLogger("system_monitor")

_THERMAL_ZONE_PATH = "/sys/class/thermal/thermal_zone0/temp"


class SystemStats(BaseModel):
    cpu_percent: float
    ram_percent: float
    ram_used_mb: float
    ram_total_mb: float
    disk_percent: float
    cpu_temp_c: Optional[float] = None
    cpu_freq_mhz: Optional[float] = None
    uptime_s: float
    ai_accelerator_available: bool = False
    ai_accelerator_info: Optional[str] = None


def _read_cpu_temp_c() -> Optional[float]:
    """Standard Linux thermal sysfs interface, reported in millidegrees C."""
    try:
        with open(_THERMAL_ZONE_PATH, "r") as f:
            raw = int(f.read().strip())
        return raw / 1000.0
    except (FileNotFoundError, ValueError, PermissionError):
        # Not on a Pi, or the thermal zone path differs on this system —
        # this is an optional field, don't fail the whole stats read
        # over it.
        return None


def _get_ai_accelerator_info() -> Tuple[bool, Optional[str]]:
    """
    Best-effort Hailo status via `hailortcli` (the documented CLI tool —
    not a guessed Python API call). NEED TEST: exact output format not
    verified against real hardware. If this needs richer stats later
    (live utilization %, not just identify), check the actual HailoRT
    Python bindings' API once hardware is available rather than guessing
    function names now.
    """
    try:
        result = subprocess.run(
            ["hailortcli", "fw-control", "identify"],
            capture_output=True, text=True, timeout=3.0,
        )
        if result.returncode == 0:
            return True, result.stdout.strip()[:200]
        return False, None
    except (FileNotFoundError, subprocess.TimeoutExpired):
        # hailortcli not installed (e.g. dev laptop) or hung — either
        # way this is optional info, not a reason to fail anything.
        return False, None
    except Exception as exc:
        log.debug("AI accelerator status check failed: %s", exc)
        return False, None


def get_system_stats() -> SystemStats:
    """
    Note: psutil.cpu_percent(interval=None) reports 0.0 (or an
    inaccurate figure) on the very first call in a process — it needs a
    baseline. This is documented psutil behavior, not a bug here;
    subsequent periodic calls (this function is called on a timer by
    telemetry.py) are accurate.
    """
    vmem = psutil.virtual_memory()
    disk = psutil.disk_usage("/")
    cpu_freq = psutil.cpu_freq()
    ai_available, ai_info = _get_ai_accelerator_info()

    return SystemStats(
        cpu_percent=psutil.cpu_percent(interval=None),
        ram_percent=vmem.percent,
        ram_used_mb=vmem.used / (1024 * 1024),
        ram_total_mb=vmem.total / (1024 * 1024),
        disk_percent=disk.percent,
        cpu_temp_c=_read_cpu_temp_c(),
        cpu_freq_mhz=cpu_freq.current if cpu_freq else None,
        uptime_s=time.time() - psutil.boot_time(),
        ai_accelerator_available=ai_available,
        ai_accelerator_info=ai_info,
    )
