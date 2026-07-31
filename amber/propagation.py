# amber/propagation.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle

try:
    from .radiodevices import BaseStation, Sector, Node
except Exception:  # fallbacks for type hints only
    BaseStation = Any  # noqa: N816
    Sector = Any  # noqa: N816

C = 3e8  # speed of light (m/s)


# ----------------------------
# Numeric helpers
# ----------------------------
def dbm_to_watts(p_dbm: float) -> float:
    """Convert dBm to Watts.

    Parameters
    ----------
    p_dbm : float
        Power in dBm

    Returns
    -------
    float
        Power in Watts
    """    
    return 10.0 ** ((p_dbm - 30.0) / 10.0)


def watts_to_dbm(p_w: float) -> float:
    """Convert Watts to dBm.

    Parameters
    ----------
    p_w : float
         Power in Watts

    Returns
    -------
    numpy array
        Power in dBm
    """    
    return 10.0 * np.log10(np.maximum(p_w, 1e-15)) + 30.0


def lin_from_dbi(g_dbi: float) -> float:
    """Converts gain from dBi to linear scale.

    Parameters
    ----------
    g_dbi : float
        Gain in dBi

    Returns
    -------
    float
        Linear gain
    """    
    return 10.0 ** (g_dbi / 10.0)


def compass_to_math(az_compass_deg: float) -> float:
    """Convert compass angle (0°=North, clockwise) to math angle (0°=East, CCW).

    Parameters
    ----------
    az_compass_deg : float
        Azimuth angle in compass degrees

    Returns
    -------
    float
        Azimuth angle in math degrees
    """  
    return (90.0 - float(az_compass_deg)) % 360.0


def bearing_deg(x0: float, y0: float, x: float, y: float) -> np.ndarray:
    """Calculate the bearing degree for given points. 

    Parameters
    ----------
    x0 : float
        X-coordinate of the first point
    y0 : float
        Y-coordinate of the first point
    x : float
        X-coordinate of the second point
    y : float
        Y-coordinate of the second point

    Returns
    -------
    np.ndarray
        Bearing angle in degrees
    """        
    return np.degrees(np.arctan2(y - y0, x - x0)) % 360.0


def angular_error_deg(bore_math_deg: float, target_math_deg: float) -> np.ndarray:
    """Calculate the smallest angular error between bore and target angles in degrees.

    Parameters
    ----------
    bore_math_deg : float
        Bore angle in math degrees
    target_math_deg : float
        Target angle in math degrees

    Returns
    -------
    np.ndarray
        Angular error in degrees
    """    
    return np.abs((target_math_deg - bore_math_deg + 180.0) % 360.0 - 180.0)


# ----------------------------
# Antenna patterns
# ----------------------------
@dataclass(frozen=True)
class OmniAntenna:
    gain_dbi: float = 0.0

    def gain_linear(self, _angle_err_deg: float) -> float:
        return lin_from_dbi(self.gain_dbi)


@dataclass(frozen=True)
class SectorAntenna3GPP:
    # 3GPP TR 38.901 horizontal pattern
    boresight_compass_deg: float
    beamwidth_deg: float  # HPBW_h
    Gmax_dBi: float = 17.0  # typical macro panel peak
    Am_dB: float = 30.0  # front-to-back cap

    def gain_linear(self, angle_err_deg: float) -> float:
        # Horizontal attenuation Ah (dB)
        Ah = np.minimum(
            12.0 * (abs(angle_err_deg) / np.maximum(1e-6, self.beamwidth_deg)) ** 2, self.Am_dB
        )
        G_dBi = self.Gmax_dBi - Ah
        return lin_from_dbi(G_dBi)


def antenna_from_sector(sec: Sector) -> Any:
    """Create antenna pattern object from Sector definition.

    Parameters
    ----------
    sec : Sector
        Sector object with attributes like antenna_type, azimuth_deg, beamwidth_deg, antenna_gain_dbi

    Returns
    -------
    Any
        Antenna pattern object (e.g., OmniAntenna or SectorAntenna3GPP)

    Raises
    ------
    ValueError
        If the sector's antenna_type is unknown
    """    
    at = str(getattr(sec, "antenna_type", "3GPP")).lower()
    if at == "omni":
        return OmniAntenna(gain_dbi=0.0)
    elif at in ("3gpp", "3gpp3901", "3gpp_38.901", "3gpp-38.901"):
        return SectorAntenna3GPP(
            boresight_compass_deg=float(sec.azimuth_deg),
            beamwidth_deg=float(sec.beamwidth_deg),
            Gmax_dBi=float(sec.antenna_gain_dbi),
            Am_dB=30.0,
        )
    else:
        raise ValueError(f"Unknown antenna_type on Sector: {sec.antenna_type}")

def antenna_from_node(node: Node) -> Any:
    """Create antenna pattern object from Node definition.

    Parameters
    ----------
    node : Node
        Node object with attributes like antenna_type, azimuth_deg, beamwidth_deg, antenna_gain_dbi

    Returns
    -------
    Any
        Antenna pattern object (e.g., OmniAntenna or SectorAntenna3GPP)

    Raises
    ------
    ValueError
        If the node's antenna_type is unknown
    """    
    at = str(getattr(node, "antenna_type", "3GPP")).lower()
    if at == "omni":
        return OmniAntenna(gain_dbi=float(node.antenna_gain_dbi))
    elif at in ("3gpp", "3gpp3901", "3gpp_38.901", "3gpp-38.901"):
        return SectorAntenna3GPP(
            boresight_compass_deg=float(node.azimuth_deg),
            beamwidth_deg=float(node.beamwidth_deg),
            Gmax_dBi=float(node.antenna_gain_dbi),
            Am_dB=30.0,
        )
    else:
        raise ValueError(f"Unknown antenna_type on Node: {node.antenna_type}")

# ----------------------------
# Path loss models
# ----------------------------
def fspl_lin(distance_m, freq_hz):
    """Friis transmission equation (linear scale, no dB).

    Parameters
    ----------
    distance_m : float
        Distance in meters
    freq_hz : float
        Frequency in Hz

    Returns
    -------
    numpy array
        Linear path loss (unitless ratio)
    """    
    d = np.maximum(distance_m, 1e-3)
    lam = C / freq_hz
    return (lam / (4.0 * np.pi * d)) ** 2


def urban_macro_loss_db_38_901(
    distance_m, freq_hz, h_bs=25.0, h_ut=1.5, los=True,
):
    """3GPP TR 38.901 urban macro path loss model (in dB). LOS or NLOS can be chosen.

    Parameters
    ----------
    distance_m : numpy array or float
        2D distance between transmitter and receiver in meters
    freq_hz : numpy array or float
        Frequency in Hz
    h_bs : float, optional
        Height of the base station in meters, by default 25.0
    h_ut : float, optional
        Height of the user terminal in meters, by default 1.5
    los : bool, optional
        Whether to use the LOS model, by default True

    Returns
    -------
    numpy array
        Path loss in dB
    """    
    fc_GHz = np.maximum(0.1, freq_hz / 1e9)  # spec expects GHz
    d2D = np.maximum(distance_m, 1.0)
    d3D = np.hypot(d2D, h_bs - h_ut)
    d3D = np.maximum(d3D, 10.0)  # stay within model validity; also removes near-BS artifact
    dBP = 4.0 * (h_bs - 1.0) * np.maximum(h_ut - 1.0, 0.1) * freq_hz / C

    # LOS
    PL1_LOS = 28.0 + 22.0 * np.log10(d3D) + 20.0 * np.log10(fc_GHz)
    PL2_LOS = (
        28.0
        + 40.0 * np.log10(d3D)
        + 20.0 * np.log10(fc_GHz)
        - 9.0 * np.log10(dBP**2 + (h_bs - h_ut) ** 2)
    )

    PL_LOS = np.where((d2D <= dBP) & (d2D >= 10.0), PL1_LOS, PL2_LOS)

    # NLOS
    PL_NLOS = (
        13.54 + 39.08 * np.log10(d3D) + 20.0 * np.log10(fc_GHz) - 0.6 * (h_ut - 1.5)
    )
    
    ### source: https://www.etsi.org/deliver/etsi_tr/138900_138999/138901/15.00.00_60/tr_138901v150000p.pdf page 27 ###

    if los:
        return PL_LOS
    else:
        return np.maximum(PL_LOS, PL_NLOS)


# ----------------------------
# Core Rx power computation
# ----------------------------
def compute_rx_power_watts(
    tx_power_dbm: float,
    tx_gain_linear: float,
    distance_m: float,
    freq_hz: float,
    pathloss: str = "fspl",
    los: bool = True,
    extra_np_per_m: float = 0.0,
    rx_gain_linear: float = 1.0,
    h_bs: float = 25.0,
    h_ut: float = 1.5,
) -> float:
    """Compute received power in Watts based on given parameters and path loss model.

    Parameters
    ----------
    tx_power_dbm : float
        Transmitter power in dBm
    tx_gain_linear : float
        Transmitter gain in linear scale
    distance_m : float
        Distance between transmitter and receiver in meters
    freq_hz : float
        Frequency in Hz
    pathloss : str, optional
        Path loss model to use, by default "fspl"
    los : bool, optional
        Whether to use the LOS model, by default True
    extra_np_per_m : float, optional
        Medium attenuation in Neper/m, by default 0.0
    rx_gain_linear : float, optional
        Receiver gain in linear scale, by default 1.0
    h_bs : float, optional
        Height of the base station in meters, by default 25.0
    h_ut : float, optional
        Height of the user terminal in meters, by default 1.5

    Returns
    -------
    float
        Received power in Watts

    Raises
    ------
    ValueError
        If an unknown path loss model is specified
    """    
    ### Friis transmission equation with medium attenuation ###

    pt_w = dbm_to_watts(tx_power_dbm)
    if pathloss.lower() == "fspl":
        h = fspl_lin(distance_m, freq_hz)
        loss_lin = h
    elif pathloss.lower() in ("macro", "3gpp", "urban"):
        pl_db = urban_macro_loss_db_38_901(
            distance_m, freq_hz, h_bs=h_bs, h_ut=h_ut, los=los
        )
        loss_lin = 10.0 ** (-pl_db / 10.0)
    else:
        raise ValueError(f"Unknown pathloss model: {pathloss}")

    med = np.exp(-extra_np_per_m * distance_m)
    return pt_w * tx_gain_linear * loss_lin * med * rx_gain_linear


# ----------------------------
# Coverage map
# ----------------------------

NodeExtPowerFn = Callable[[Any], float]


@dataclass
class CoverageMap:
    base_stations: Optional[Sequence[Any]] = (
        None  # expects .id, .x, .y, .sectors (list of Sector)
    )
    nodes: Optional[Sequence[Any]] = (
        None  # expects .id, .x, .y, .efficiency, .antenna_type
    )
    freq_hz: float = 925e6
    pathloss_model: str = "macro"  # "fspl" or "macro"
    los: bool = True  # if macro
    extra_np_per_m: float = 0.0  # optional medium attenuation
    bandwidth_hz: float = 100e6  # for SNR
    noise_figure_db: float = 6.0

    # power mode
    node_energy_mode: str = "wpt"  # "wpt" | "external" | "hybrid"
    node_ext_power_fn: Optional[NodeExtPowerFn] = None
    combine_mode: str = "max"

    # internal
    _grid: Optional[Dict[str, np.ndarray]] = field(init=False, default=None, repr=False)


    def compute_coverage_map(
        self,
        xmin: float,
        xmax: float,
        ymin: float,
        ymax: float,
        step_m: float,
    ) -> Dict[str, np.ndarray]:
        """Compute coverage map over a grid defined by the given parameters.

        Parameters
        ----------
        xmin : float
            Minimum x-coordinate of the grid
        xmax : float
            Maximum x-coordinate of the grid
        ymin : float
            Minimum y-coordinate of the grid
        ymax : float
            Maximum y-coordinate of the grid
        step_m : float
            Step size for the grid

        Returns
        -------
        Dict[str, np.ndarray]
            Dictionary containing the coverage map data
        """        
        xs = np.arange(xmin, xmax + 1e-9, step_m)
        ys = np.arange(ymin, ymax + 1e-9, step_m)
        nx, ny = len(xs), len(ys)
        X, Y = np.meshgrid(xs, ys)  # both shape (ny, nx)

        best_pw_w = np.zeros((ny, nx), dtype=float)
        total_pw_w = np.zeros((ny, nx), dtype=float)
        owner = np.full((ny, nx), "", dtype=object)

        if self.base_stations is not None:
            for bs in self.base_stations:
                secs: List[Sector] = list(getattr(bs, "sectors", []))
                d2D = np.hypot(X - bs.x, Y - bs.y)

                for s_idx, sec in enumerate(secs):
                    bore_math = compass_to_math(sec.azimuth_deg)
                    math_angle = bearing_deg(bs.x, bs.y, X, Y)
                    aerr = angular_error_deg(bore_math, math_angle)
                    ant = antenna_from_sector(sec)
                    tx_gain_lin = ant.gain_linear(aerr)
                    tx_dbm = float(getattr(sec, "power", 43.0))

                    p_w = compute_rx_power_watts(
                        tx_power_dbm=tx_dbm,
                        tx_gain_linear=tx_gain_lin,
                        distance_m=d2D,
                        freq_hz=self.freq_hz,
                        pathloss=self.pathloss_model,
                        los=self.los,
                        extra_np_per_m=self.extra_np_per_m,
                        rx_gain_linear=1.0,
                        h_bs=sec.height,
                        h_ut=1.5,
                    )

                    total_pw_w += p_w
                    bs_name = f"BS{bs.id}_S{s_idx}"
                    mask = p_w > best_pw_w
                    best_pw_w = np.where(mask, p_w, best_pw_w)
                    owner = np.where(mask, bs_name, owner)

        best_pw_dbm = watts_to_dbm(best_pw_w)
        total_pw_dbm = watts_to_dbm(total_pw_w)

        self._grid = {
            "xs": xs,
            "ys": ys,
            "owner": owner,
            "best_pw_dbm": best_pw_dbm,
            "best_pw_w": best_pw_w,
            "total_pw_dbm": total_pw_dbm,
        }
        return self._grid

    def compute_bs_to_point(
        self, points: Sequence[Tuple[float, float]]
    ) -> Dict[str, Dict]:
        """Compute received power at each BS sector from given points (DOWNLINK).

        Parameters
        ----------
        points : Sequence[Tuple[float, float]]
            List of points (x, y coordinates) for which to compute received power

        Returns
        -------
        Dict[str, Dict]
            Dictionary containing the received power data for each point contained in the following keys:
            - owner: Dict[int, str] - best serving BS sector for each point
            - best_pw_dbm: Dict[int, float] - best received power in dBm for each point
            - best_pw_w: Dict[int, float] - best received power in Watts for each point
            - total_pw_dbm: Dict[int, float] - total received power in dBm for each point
            - per_node_powers: Dict[int, Dict[str, float]] - power from each BS sector for each point
        """              
        if points is None or len(points) == 0:
            return {}

        N = len(points)
        px = np.array([p.x for p in points])
        py = np.array([p.y for p in points])
        p_hut = np.array([getattr(p, "height", 1.5) for p in points])
        node_antennas = [antenna_from_node(p) for p in points]
        p_ids = [getattr(p, "id", i) for i, p in enumerate(points)]

        best_pw_w = np.zeros(N)
        best_names = np.full(N, "", dtype=object)
        total_pw_w = np.zeros(N)
        per_node_powers = {pid: {} for pid in p_ids}

        if self.base_stations is not None:
            for bs in self.base_stations:
                secs: List[Sector] = list(getattr(bs, "sectors", []))
                d2D = np.hypot(px - bs.x, py - bs.y)
                angle_from_node_to_bs = bearing_deg(px, py, bs.x, bs.y)
                p_rx_gain = np.array([
                    node_antennas[j].gain_linear(
                        angular_error_deg(
                            compass_to_math(float(getattr(points[j], "azimuth_deg", 0.0))),
                            angle_from_node_to_bs[j],
                        )
                    )
                    for j in range(N)
                ])

                for s_idx, sec in enumerate(secs):
                    bore_math = compass_to_math(sec.azimuth_deg)
                    math_angle = bearing_deg(bs.x, bs.y, px, py)
                    aerr = angular_error_deg(bore_math, math_angle)
                    ant = antenna_from_sector(sec)
                    tx_gain_lin = ant.gain_linear(aerr)
                    tx_dbm = float(getattr(sec, "power", 43.0))

                    p_w = compute_rx_power_watts(
                        tx_power_dbm=tx_dbm,
                        tx_gain_linear=tx_gain_lin,
                        distance_m=d2D,
                        freq_hz=self.freq_hz,
                        pathloss=self.pathloss_model,
                        los=self.los,
                        extra_np_per_m=self.extra_np_per_m,
                        rx_gain_linear=p_rx_gain,
                        h_bs=sec.height,
                        h_ut=p_hut,
                    )

                    total_pw_w += p_w
                    bs_name = f"BS{bs.id}_S{s_idx}"
                    mask = p_w > best_pw_w
                    best_pw_w = np.where(mask, p_w, best_pw_w)
                    best_names = np.where(mask, bs_name, best_names)

                    pw_dbm_arr = watts_to_dbm(p_w)
                    for j, pid in enumerate(p_ids):
                        per_node_powers[pid][bs_name] = float(pw_dbm_arr[j])

        best_dbm_arr = watts_to_dbm(best_pw_w)
        total_dbm_arr = watts_to_dbm(total_pw_w)

        return {
            "owner":          {p_ids[j]: str(best_names[j]) for j in range(N)},
            "best_pw_dbm":    {p_ids[j]: float(best_dbm_arr[j]) for j in range(N)},
            "best_pw_w":      {p_ids[j]: float(best_pw_w[j]) for j in range(N)},
            "total_pw_dbm":   {p_ids[j]: float(total_dbm_arr[j]) for j in range(N)},
            "per_node_powers": per_node_powers,
        }


    def calculate_node_power(
        self,
        nodes: Optional[Sequence[Any]] = None,
        downlink_results: Optional[Dict] = None,
    ) -> None:
        """Calculate the harvesting and backscatter power for each node based on the downlink results and the specified energy mode.

        Parameters
        ----------
        nodes : Optional[Sequence[Any]], optional
            List of nodes for which to calculate power, by default None
        downlink_results : Optional[Dict], optional
            Pre-computed results from compute_bs_to_point(), by default None
        """        
        target_nodes = nodes if nodes is not None else self.nodes
        if target_nodes is None or len(target_nodes) == 0:
            return
        if downlink_results is None:
            downlink_results = self.compute_bs_to_point(target_nodes)

        if self.node_energy_mode == "wpt":
            for node in target_nodes:
                node_id = getattr(node, "id", None)
                received_w = downlink_results["best_pw_w"].get(node_id, 0.0)
                efficiency = getattr(node, "efficiency", 1.0)
                if node.state != "transmitting":
                    harvested_w = received_w
                else:
                    harvested_w = received_w * (1 - efficiency)
                backscatter_w = received_w * efficiency

                node.harvesting_power_dbm = watts_to_dbm(harvested_w)
                node.backscatter_power_dbm = watts_to_dbm(backscatter_w)

        elif self.node_energy_mode == "external":
            for node in target_nodes:
                if self.node_ext_power_fn is not None:
                    ext_w = self.node_ext_power_fn(node)
                    node.harvesting_power_dbm = watts_to_dbm(ext_w)
                else:
                    node.harvesting_power_dbm = watts_to_dbm(0.0)
                node_id = getattr(node, "id", None)
                received_w = downlink_results["best_pw_w"].get(node_id, 0.0)
                efficiency = getattr(node, "efficiency", 1.0)
                backscatter_w = received_w * efficiency
                node.backscatter_power_dbm = watts_to_dbm(backscatter_w)

        elif self.node_energy_mode == "hybrid":
            for node in target_nodes:
                node_id = getattr(node, "id", None)
                received_w = downlink_results["best_pw_w"].get(node_id, 0.0)
                efficiency = getattr(node, "efficiency", 1.0)
                if node.state != "transmitting":
                    harvested_w = received_w
                else:
                    harvested_w = received_w * (1 - efficiency)
                backscatter_w = received_w * efficiency

                ext_w = self.node_ext_power_fn(node) if self.node_ext_power_fn else 0.0
                total_w = harvested_w + ext_w if self.combine_mode != "max" else max(harvested_w, ext_w)

                node.harvesting_power_dbm = watts_to_dbm(total_w)
                node.backscatter_power_dbm = watts_to_dbm(backscatter_w)

    def compute_point_to_bs(
        self, points: Sequence[Tuple[float, float]]
    ) -> Dict[str, any]:
        """Compute power received at each BS sector from given points (UPLINK).

        Parameters
        ----------
        points : Sequence[Tuple[float, float]]
            List of points (x, y coordinates) for which to compute received power

        Returns
        -------
        Dict[str, any]
            Dictionary containing:
            - owner: Dict[str, int] - best serving node per BS sector
                Format: {"BS0_S0": node_id, "BS0_S1": node_id, ...}
            - best_pw_dbm: Dict[str, float] - best received power per BS sector (dBm)
                Format: {"BS0_S0": power_dbm, ...}
            - best_pw_w: Dict[str, float] - best received power per BS sector (Watts)
                Format: {"BS0_S0": power_w, ...}
            - total_pw_dbm: Dict[str, float] - total received power per BS sector (dBm)
                Format: {"BS0_S0": total_dbm, ...}
            - per_sector_powers: Dict[str, Dict[int, float]] - power from each node per BS sector
                Format: {"BS0_S0": {node_id: power_dbm, ...}, ...}
        """        

        if points is None or len(points) == 0:
            return {}

        N = len(points)
        px = np.array([p.x for p in points])
        py = np.array([p.y for p in points])
        p_hut = np.array([getattr(p, "height", 1.5) for p in points])
        p_ids = [getattr(p, "id", i) for i, p in enumerate(points)]
        p_tx_dbm = np.array([getattr(p, "backscatter_power_dbm", -999.0) for p in points])
        node_antennas = [antenna_from_node(p) for p in points]

        per_sector_powers: Dict[str, Dict[int, float]] = {}

        if self.base_stations is not None:
            for bs in self.base_stations:
                secs: List[Sector] = list(getattr(bs, "sectors", []))
                d2D = np.hypot(px - bs.x, py - bs.y)
                angle_from_node_to_bs = bearing_deg(px, py, bs.x, bs.y)
                p_tx_gain_lin = np.array([
                    node_antennas[j].gain_linear(
                        angular_error_deg(
                            compass_to_math(float(getattr(points[j], "azimuth_deg", 0.0))),
                            angle_from_node_to_bs[j],
                        )
                    )
                    for j in range(N)
                ])

                for s_idx, sec in enumerate(secs):
                    bore_math = compass_to_math(sec.azimuth_deg)
                    math_angle = bearing_deg(bs.x, bs.y, px, py)
                    aerr = angular_error_deg(bore_math, math_angle)
                    ant = antenna_from_sector(sec)
                    rx_gain_lin = ant.gain_linear(aerr)

                    p_w = compute_rx_power_watts(
                        tx_power_dbm=p_tx_dbm,
                        tx_gain_linear=p_tx_gain_lin,
                        distance_m=d2D,
                        freq_hz=self.freq_hz,
                        pathloss=self.pathloss_model,
                        los=self.los,
                        extra_np_per_m=self.extra_np_per_m,
                        rx_gain_linear=rx_gain_lin,
                        h_bs=sec.height,
                        h_ut=p_hut,
                    )

                    bs_name = f"BS{bs.id}_S{s_idx}"
                    pw_dbm_arr = watts_to_dbm(p_w)
                    per_sector_powers[bs_name] = {
                        p_ids[j]: float(pw_dbm_arr[j]) for j in range(N)
                    }

        # Aggregate per sector
        owner: Dict[str, int] = {}
        best_pw_dbm: Dict[str, float] = {}
        best_pw_w: Dict[str, float] = {}
        total_pw_dbm: Dict[str, float] = {}

        for sector_name, node_powers in per_sector_powers.items():
            vals = np.array(list(node_powers.values()))
            keys = list(node_powers.keys())
            total_w = np.sum(dbm_to_watts(vals))
            total_pw_dbm[sector_name] = float(watts_to_dbm(total_w))
            best_idx = int(np.argmax(vals))
            owner[sector_name] = keys[best_idx]
            best_pw_dbm[sector_name] = float(vals[best_idx])
            best_pw_w[sector_name] = float(dbm_to_watts(vals[best_idx]))

        return {
            "owner": owner,
            "best_pw_dbm": best_pw_dbm,
            "best_pw_w": best_pw_w,
            "per_sector_powers": per_sector_powers,
            "total_pw_dbm": total_pw_dbm,
        }

    # --- plotting ---
    def _draw_radio_devices(self, ax: plt.Axes, fontsize_radio: int = 8) -> None:
        """Draw base stations and nodes on the given Matplotlib Axes based on the current grid settings.

        Parameters
        ----------
        ax : plt.Axes
            Matplotlib Axes on which to draw the base stations and nodes
        fontsize_radio : int, optional
            Font size for the radio device labels, by default 8
        """        
        if self.base_stations is not None:
            for bs in self.base_stations:
                ax.add_patch(
                    Circle(
                        (bs.x, bs.y),
                        radius=getattr(bs, "site_radius", 3.0),
                        color="green",
                        zorder=4,
                    )
                )
                ax.text(
                    bs.x,
                    bs.y - (getattr(bs, "site_radius", 3.0) + 2*fontsize_radio),
                    f"BS{bs.id}",
                    color="Black",
                    ha="center",
                    va="center",
                    fontsize=fontsize_radio,
                    zorder=5,
                )
                for s in getattr(bs, "sectors", []):
                    theta1 = (90.0 - s.azimuth_deg - s.beamwidth_deg * 0.5) % 360.0
                    theta2 = (90.0 - s.azimuth_deg + s.beamwidth_deg * 0.5) % 360.0
                    ax.add_patch(
                        Wedge(
                            (bs.x, bs.y),
                            r=getattr(bs, "site_radius", 3.0)*5.0,
                            theta1=theta1,
                            theta2=theta2,
                            facecolor=getattr(s, "facecolor", "C1"),
                            alpha=getattr(s, "alpha", 0.15),
                            edgecolor=getattr(s, "edgecolor", "none"),
                            linewidth=0.5,
                            zorder=2,
                        )
                    )

        if self.nodes is not None:
            for node in self.nodes:
                node_type = getattr(node, "node_type", "passive")
                color = getattr(node, "color", "") or (
                    "C0" if node_type == "passive" else "C4"
                )
                ax.add_patch(
                    Circle(
                        (node.x, node.y),
                        radius=getattr(node, "radius", 3.0),
                        color=color,
                        zorder=3,
                    )
                )
                if getattr(node, "label", True):
                    ax.text(
                        node.x,
                        node.y - (getattr(node, "radius", 3.0) + fontsize_radio/2 - fontsize_radio/4),
                        f"Node {node.id}",
                        fontsize=fontsize_radio,
                        ha="center",
                        va="top",
                        color="black",
                        zorder=4,
                    )
                if node.antenna_type != "omni":
                    theta1 = (90.0 - node.azimuth_deg - node.beamwidth_deg * 0.5) % 360.0
                    theta2 = (90.0 - node.azimuth_deg + node.beamwidth_deg * 0.5) % 360.0
                    ax.add_patch(
                        Wedge(
                            (node.x, node.y),
                            r=getattr(node, "radius",3.0)*5.0,
                            theta1=theta1,
                            theta2=theta2,
                            facecolor=getattr(node, "facecolor", "C5"),
                            alpha=getattr(node, "alpha", 0.15),
                            edgecolor=getattr(node, "edgecolor", "k"),
                            linewidth=0.5,
                            zorder=2,
                        )
                    )

    def plot_rssi(
        self,
        ax: Optional[plt.Axes] = None,
        vmin: float = -120.0,
        vmax: float = -40.0,
        cmap: str = "viridis",
        threshold: Optional[float] = None,
        fontsize: int = 8,
        fontsize_radio: int = 8,
    ):
        """Plot the RSSI coverage map on the given Matplotlib Axes based on the computed grid data.

        Parameters
        ----------
        ax : Optional[plt.Axes], optional
            Matplotlib Axes on which to plot the RSSI coverage map, by default None
        vmin : float, optional
            Minimum value for the color scale, by default -120.0
        vmax : float, optional
            Maximum value for the color scale, by default -40.0
        cmap : str, optional
            Colormap to use for the plot, by default "viridis"
        threshold : Optional[float], optional
            Threshold value for masking the data, by default None
        fontsize : int, optional
            Font size for the plot labels, by default 8
        fontsize_radio : int, optional
            Font size for the radio device labels, by default 8

        Returns
        -------
        _type_
            The plotted image and colorbar objects

        Raises
        ------
        RuntimeError
            If the coverage map grid has not been computed yet
        """        

        if self._grid is None:
            raise RuntimeError("compute_coverage_map() must be called first")
        xs, ys, Z = self._grid["xs"], self._grid["ys"], self._grid["best_pw_dbm"]

        if threshold is not None:
            Z = np.ma.masked_where(Z < threshold, Z)

        if ax is None:
            fig, ax = plt.subplots(figsize=(8, 7))
        im = ax.imshow(
            Z,
            origin="lower",
            extent=(xs[0], xs[-1], ys[0], ys[-1]),
            vmin=vmin,
            vmax=vmax,
            cmap=cmap,
            aspect="equal",
            zorder=1,
        )
        self._draw_radio_devices(ax, fontsize_radio=fontsize_radio)
        ax.set_xlabel("Position X (m)", fontsize=fontsize)
        ax.set_ylabel("Position Y (m)", fontsize=fontsize)
        ax.tick_params(axis='both', labelsize=fontsize)
        cb = plt.colorbar(im, ax=ax, label="RSSI (dBm)")
        cb.ax.tick_params(labelsize=fontsize)
        cb.set_label("RSSI (dBm)", fontsize=fontsize)

        return im, cb
