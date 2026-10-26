# simulator/models/radiodevices.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional, Literal, Callable, NamedTuple

class RFResponse(NamedTuple):
    freq_hz: float
    bandwidth_hz: float
    power_w: float

@dataclass
class Node:
    id: int
    x: float
    y: float
    node_type: str = "passive"  # "passive" | "active" for future use
    radius: float = 2.0
    color: str = ""
    label: bool = True
    efficiency: float = 1.0
    pce: float | Callable[[float], float] = 1.0
    antenna_type: Literal["omni", "3GPP"] | str = "omni"
    antenna_gain_dbi: float = 0.0
    azimuth_deg: Optional[float] = 0.0
    beamwidth_deg: Optional[float] = 360.0
    height: float = 1.5
    sensitivity_dbm: float = -45.0
    subcarrier_shift: Optional[int] = 0
    response_model: Optional[Callable[[List[RFResponse]], List[RFResponse]]] = None
    state: str = field(default="listening", init=False, repr=False)
    fading_gain: float = field(default=1.0, init=False, repr=False)
    dl_ratio_db: float = field(default=float("inf"), init=False, repr=False)


@dataclass
class Sector:
    azimuth_deg: float
    beamwidth_deg: float
    facecolor: str = "C1"
    alpha: float = 0.3
    edgecolor: str = "k"
    power: float = 43.0 #dBm
    antenna_type: Literal["omni", "3GPP"] | str = "3GPP"
    sensitivity_dbm: float = -100.0
    height: float = 25.0
    antenna_gain_dbi: float = 17.0
    carrier_freq: Optional[float] = None


@dataclass
class BaseStation:
    id: int
    x: float
    y: float
    sectors: List[Sector] = field(default_factory=list)
    carrier_on: bool = True
    site_radius:float = 2.0
    site_color: str = "green"
    label: bool = True