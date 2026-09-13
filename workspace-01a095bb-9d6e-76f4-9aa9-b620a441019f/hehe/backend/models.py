from typing import List, Optional, Literal
from pydantic import BaseModel, Field, field_validator


class Vertex(BaseModel):
    lat: float = Field(..., ge=-90, le=90)
    lon: float = Field(..., ge=-180, le=180)


class GeofenceUploadRequest(BaseModel):
    vertices: List[Vertex]

    @field_validator("vertices")
    @classmethod
    def min_three_points(cls, v: List[Vertex]) -> List[Vertex]:
        if len(v) < 3:
            raise ValueError("A polygon fence needs at least 3 vertices")
        if len(v) > 255:
            # AC_PolyFenceItem stores vertex_count in param1 as a uint8
            # (MAV_MISSION_INVALID_PARAM1 above 255) — see
            # MissionItemProtocol_Fence.cpp on ArduPilot.
            raise ValueError("Polygon exceeds 255 vertices, the ArduPilot fence storage limit")
        return v


class ENUPoint(BaseModel):
    x: float  # meters, east of origin
    y: float  # meters, north of origin


class Position(BaseModel):
    lat: float
    lon: float
    alt_msl: Optional[float] = None  # meters above mean sea level
    alt_rel: Optional[float] = None  # meters above home/launch


class GeofenceStatus(BaseModel):
    loaded: bool
    armable: bool
    reason: str
    vertex_count: int = 0
    vertices_latlon: List[Vertex] = []
    vertices_enu: List[ENUPoint] = []
    centroid_enu: Optional[ENUPoint] = None
    max_radius_m: Optional[float] = None
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    fc_readback_matched: Optional[bool] = None


class CameraModeRequest(BaseModel):
    mode: Literal["cam1", "cam2", "dual", "none"]


class CameraFocusRequest(BaseModel):
    position: float = Field(..., ge=0.0, le=1.0)


class SearchStartRequest(BaseModel):
    strategy: Optional[str] = None
    dry_run: bool = False
    altitude_m: Optional[float] = None
    step_m: Optional[float] = None


class SearchWaypoint(BaseModel):
    index: int
    enu: ENUPoint
    lat: float
    lon: float
    altitude_m: float


class SearchStatus(BaseModel):
    state: Literal["idle", "searching", "holding", "target_found", "approaching", "decoded", "completed", "error"] = "idle"
    dry_run: bool = False
    strategy: str = "expanding_square"
    altitude_m: float = 5.0
    current_waypoint_idx: int = 0
    total_waypoints: int = 0
    reason: str = "Search inactive"
    target: Optional[dict] = None
    waypoints: List[SearchWaypoint] = []


class BBox(BaseModel):
    x: int
    y: int
    w: int
    h: int
    confidence: float = 1.0


class QRDetectionResult(BaseModel):
    bbox: Optional[BBox] = None
    payload: Optional[str] = None
    confirmed: bool = False
    streak: int = 0
    required_streak: int = 3
    timestamp: float = 0.0


class FSMStatus(BaseModel):
    state: Literal[
        "INIT", "PRE_FLIGHT_CHECK", "ARM", "TAKEOFF", "SEARCH", "CACHE_DETECTED",
        "APPROACH", "DECODE", "CONFIRMED", "TRANSMIT_RESULT", "RTL", "LAND",
        "MISSION_COMPLETE", "FAILSAFE"
    ] = "INIT"
    previous_state: Optional[str] = None
    reason: str = "FSM initialized"
    armable: bool = False
    telemetry_link_healthy: bool = False
    payload: Optional[str] = None
    timestamp: float = 0.0
