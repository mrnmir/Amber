# amber/packet_analysis.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Iterable, Optional, Tuple, Set
import os
import math
import pandas as pd
from collections import defaultdict
from datetime import datetime
from .propagation import dbm_to_watts, lin_from_dbi


# Thermal noise constants
K_BOLTZMANN = 1.38e-23  # J/K
T_REF = 290.0  # Reference temperature (K)

def thermal_noise_watts(bandwidth_hz: float, noise_figure_db: float = 6.0) -> float:
    """Calculate thermal noise power: N = k * T * B * NF

    Parameters
    ----------
    bandwidth_hz : float
        Bandwidth in Hz
    noise_figure_db : float, optional
        Noise figure in dB, by default 6.0

    Returns
    -------
    float
        Thermal noise power in watts
    """    
    nf_lin = lin_from_dbi(noise_figure_db)
    return K_BOLTZMANN * T_REF * bandwidth_hz * nf_lin

# --------------------------------------------------------------------------------------
# Data models
# --------------------------------------------------------------------------------------

@dataclass
class CollisionEvent:
    timestamp: str
    bs_id: int
    subcarrier: int
    involved_nodes: List[int]
    intervals: List[Tuple[float, float]]
    powers_dbm: List[float]
    sinr_db: List[float]
    kind: str  # "total", "capture", "sic_success", "n_way"
    winner_ids: List[int] = field(default_factory=list)  # Can have multiple winners with SIC

@dataclass
class BSStats:
    transmitted: int = 0
    received: int = 0
    errors: int = 0
    collisions_total: int = 0
    der: float = 0.0

@dataclass
class NodeStats:
    sent: int = 0
    received_at_bs: int = 0
    total_errors: int = 0
    heard_by_bs: Set[int] = field(default_factory=set)

@dataclass
class PacketAnalysisResult:
    per_bs: Dict[int, BSStats] = field(default_factory=dict)
    per_node: Dict[int, NodeStats] = field(default_factory=dict)
    nodes_in_bs: Dict[int, List[int]] = field(default_factory=dict)
    nodes_multi_bs: List[int] = field(default_factory=list)
    collisions: List[CollisionEvent] = field(default_factory=list)

# --------------------------------------------------------------------------------------
# SIC (Successive Interference Cancellation) algorithm
# --------------------------------------------------------------------------------------

def apply_sic(
    powers_dbm: List[float],
    noise_w: float,
    required_sinr_db: float,
    cancellation_factor: float = 0.9,
) -> Tuple[List[int], List[float]]:
    """Apply Successive Interference Cancellation (SIC) algorithm.

    SIC Algorithm:

    - Sort packets by power (strongest first)
    - Try to decode strongest packet (check SINR >= threshold)
    - If successful, subtract signal (with imperfect cancellation)
    - Re-calculate SINR for remaining packets
    - Repeat until no more packets can be decoded

    Parameters
    ----------
    powers_dbm : List[float]
        List of received powers at BS in dBm
    noise_w : float
        Thermal noise power in watts
    required_sinr_db : float
        Minimum SINR for successful decode
    cancellation_factor : float, optional
        Cancellation factor is how much of a decoded signal can be subtracted out of the received signal before trying to decode the next packet. 
        It's a value between 0 and 1, by default 0.9:

        1.0 = perfect cancellation → the decoded signal is fully removed, leaving zero residual interference
        0.9 = 90% cancellation → 10% of the decoded signal still leaks through as interference
        0.0 = no cancellation → the decoded signal stays fully in the mix (equivalent to capture effect only)

    Returns
    -------
    Tuple[List[int], List[float]]
        Indices of successfully decoded packets (in decode order) and their final SINR values
    """    
    n = len(powers_dbm)
    if n == 0:
        return [], []

    powers_w = [dbm_to_watts(p) for p in powers_dbm]
    residual_w = list(powers_w)  # Remaining power after cancellation
    decoded_indices: List[int] = []
    final_sinr_db = [-999.0] * n

    # Track which packets are still candidates
    candidates = set(range(n))

    while candidates:
        # Find strongest remaining candidate
        strongest_idx = max(candidates, key=lambda i: residual_w[i])
        signal_w = residual_w[strongest_idx]

        # Calculate interference from other candidates
        interference_w = sum(residual_w[j] for j in candidates if j != strongest_idx)

        # SINR for strongest
        sinr_lin = signal_w / max(1e-18, interference_w + noise_w)
        sinr_db = 10.0 * math.log10(max(1e-18, sinr_lin))
        final_sinr_db[strongest_idx] = sinr_db

        if sinr_db >= required_sinr_db:
            # Successfully decoded - cancel this signal
            decoded_indices.append(strongest_idx)
            candidates.remove(strongest_idx)

            # Imperfect cancellation: residual = original * (1 - cancellation_factor)
            residual_w[strongest_idx] = powers_w[strongest_idx] * (1.0 - cancellation_factor)
        else:
            # Cannot decode strongest - no more decoding possible
            # Record SINR for remaining candidates
            for idx in candidates:
                if idx != strongest_idx:
                    sig = residual_w[idx]
                    interf = sum(residual_w[j] for j in candidates if j != idx)
                    s_lin = sig / max(1e-18, interf + noise_w)
                    final_sinr_db[idx] = 10.0 * math.log10(max(1e-18, s_lin))
            break

    return decoded_indices, final_sinr_db

# --------------------------------------------------------------------------------------
# Collision detection with subcarrier awareness
# --------------------------------------------------------------------------------------

def detect_collisions(
    df: pd.DataFrame,
    bs_id: int,
    noise_w: float,
    required_sinr_db: float = 10.0,
    enable_sic: bool = True,
) -> Tuple[List[CollisionEvent], Dict[int, str]]:
    """Detect collisions with subcarrier awareness and SINR-based resolution.

    Packets on different subcarriers do NOT collide.
    Packets on same subcarrier with time overlap -> check SINR for decode.

    Uses SIC (Successive Interference Cancellation) algorithm:

    - Sort packets by power (strongest first)
    - Try to decode strongest (SINR >= threshold)
    - If successful, subtract signal and repeat for next strongest
    - Continue until no more packets can be decoded


    Parameters
    ----------
    df : pd.DataFrame
        DataFrame with packet information
    bs_id : int
        Base station ID
    noise_w : float
        Thermal noise power in watts
    required_sinr_db : float, optional
        Minimum SINR for successful decode, by default 10.0
    enable_sic : bool, optional
        Enable successive interference cancellation, by default True

    Returns
    -------
    Tuple[List[CollisionEvent], Dict[int, str]]
        Tuple of (collision_events, packet_outcomes)
        packet_outcomes: {row_idx: "decoded" | "collision_loss" | "captured" | "sic_recovered"}
    """    
    collisions = []
    packet_outcomes: Dict[int, str] = {}

    if df.empty:
        return collisions, packet_outcomes

    # Group by subcarrier
    for subcarrier, group in df.groupby("subcarrier_shift"):
        if len(group) < 2:
            # No collision possible with single packet
            for idx in group.index:
                packet_outcomes[idx] = "decoded"
            continue

        # Sort by start time
        group = group.sort_values("start_ms")

        # Find overlapping packets using sweep line
        events = []
        for idx, row in group.iterrows():
            events.append((row["start_ms"], "start", idx, row))
            events.append((row["end_ms"], "end", idx, row))

        events.sort(key=lambda x: (x[0], x[1] == "start"))  # end before start at same time

        active: Dict[int, pd.Series] = {}
        processed_collisions: Set[frozenset] = set()

        for _, typ, idx, row in events:
            if typ == "start":
                # Check for collision with active packets
                if active:
                    # Collision detected
                    colliding_idxs = list(active.keys()) + [idx]
                    collision_key = frozenset(colliding_idxs)

                    if collision_key not in processed_collisions:
                        processed_collisions.add(collision_key)

                        colliding_rows = [active[i] for i in active.keys()] + [row]
                        node_ids = [int(r["node_id"]) for r in colliding_rows]
                        intervals = [(float(r["start_ms"]), float(r["end_ms"])) for r in colliding_rows]
                        powers_dbm = [float(r["rssi_dbm"]) for r in colliding_rows]

                        # Analyze outcome (SINR computed dynamically inside)
                        event, outcomes = _analyze_collision_outcome(
                            bs_id=bs_id,
                            subcarrier=int(subcarrier),
                            node_ids=node_ids,
                            intervals=intervals,
                            powers_dbm=powers_dbm,
                            required_sinr_db=required_sinr_db,
                            enable_sic=enable_sic,
                            packet_idxs=colliding_idxs,
                            noise_w=noise_w,
                        )

                        if event:
                            collisions.append(event)

                        # Update packet outcomes
                        for pkt_idx, outcome in outcomes.items():
                            packet_outcomes[pkt_idx] = outcome

                active[idx] = row
            else:
                # End event - remove from active
                if idx in active:
                    del active[idx]
                # Mark as decoded if not involved in collision
                if idx not in packet_outcomes:
                    packet_outcomes[idx] = "decoded"

    return collisions, packet_outcomes


def _analyze_collision_outcome(
    bs_id: int,
    subcarrier: int,
    node_ids: List[int],
    intervals: List[Tuple[float, float]],
    powers_dbm: List[float],
    required_sinr_db: float,
    enable_sic: bool,
    packet_idxs: List[int],
    noise_w: float,
    cancellation_factor: float = 0.9,
) -> Tuple[Optional[CollisionEvent], Dict[int, str]]:
    
    """
    Analyze collision outcome using SINR threshold and SIC.

    SIC includes capture effect as a special case (when only strongest is decoded).
    SINR is computed dynamically based on actual colliding packets.

    Parameters
    ----------
        bs_id: Base station ID
        subcarrier: Subcarrier index
        node_ids: List of node IDs involved in collision
        intervals: List of (start_ms, end_ms) for each packet
        powers_dbm: List of received powers
        required_sinr_db: Minimum SINR for successful decode
        enable_sic: Enable successive interference cancellation
        packet_idxs: DataFrame indices of packets
        noise_w: Thermal noise power in watts
        cancellation_factor: SIC cancellation efficiency (0.9 = 90%)

    Returns
    -------
        Tuple of (CollisionEvent, {packet_idx: outcome})
    """
    now_str = datetime.now().isoformat(timespec="seconds")
    n = len(node_ids)
    outcomes: Dict[int, str] = {}

    # Apply SIC algorithm (handles both SIC and non-SIC cases)
    # When enable_sic=False, we still use apply_sic but only the first packet matters
    decoded_indices, sinr_list = apply_sic(
        powers_dbm=powers_dbm,
        noise_w=noise_w,
        required_sinr_db=required_sinr_db,
        cancellation_factor=cancellation_factor if enable_sic else 0.0,  # No cancellation if SIC disabled
    )

    if not enable_sic:
        # Without SIC, only the first decoded (strongest) counts
        decoded_indices = decoded_indices[:1] if decoded_indices else []

    winner_ids = [node_ids[i] for i in decoded_indices]
    num_decoded = len(decoded_indices)

    # Determine collision kind and outcomes
    if num_decoded == 0:
        # Total collision - nothing decoded
        kind = "total" if n == 2 else "n_way"
        for i in range(n):
            outcomes[packet_idxs[i]] = "collision_loss"

    elif num_decoded == 1:
        # Only one decoded - capture effect
        kind = "capture"
        winner_idx = decoded_indices[0]
        outcomes[packet_idxs[winner_idx]] = "captured"
        for i in range(n):
            if i != winner_idx:
                outcomes[packet_idxs[i]] = "collision_loss"

    else:
        # Multiple decoded via SIC
        kind = "sic_success"
        decoded_set = set(decoded_indices)
        for i in range(n):
            if i in decoded_set:
                outcomes[packet_idxs[i]] = "sic_recovered"
            else:
                outcomes[packet_idxs[i]] = "collision_loss"

    return CollisionEvent(
        timestamp=now_str,
        bs_id=bs_id,
        subcarrier=subcarrier,
        involved_nodes=node_ids,
        intervals=intervals,
        powers_dbm=powers_dbm,
        sinr_db=sinr_list,
        kind=kind,
        winner_ids=winner_ids,
    ), outcomes

# --------------------------------------------------------------------------------------
# Main analysis function
# --------------------------------------------------------------------------------------

def analyze_packets(
    data_path: str,
    num_bs: int,
    num_nodes: int,
    packet_sent: Iterable[int],
    packet_received: Iterable[int],
    required_sinr_db: float = 10.0,
    enable_sic: bool = True,
    noise_figure_db: float = 6.0,
    bandwidth_hz: float = 100e6,
    write_collision_log: Optional[str] = "collision_out.txt",
    write_terminal_log: Optional[str] = "terminal_out.txt",
) -> PacketAnalysisResult:
    """
    Packet analysis with subcarrier-aware, SINR-based collision detection.

    Reads BSEngine rx log format (tab-separated):
        start_ms, end_ms, slot_id, node_id, payload, rssi_dbm,
        sector_idx, expected_node, matched, subcarrier_shift, collided, bs_id

    Uses SIC (Successive Interference Cancellation) for collision resolution:

    - SINR is computed dynamically based on actual colliding packets
    - Capture effect is a special case of SIC (only strongest decoded)
    - Multiple packets can be decoded if power differences allow

    Parameters
    ----------
    data_path : str
        Path to BSEngine rx log file.
    num_bs : int
        Number of base stations.
    num_nodes : int
        Number of nodes.
    packet_sent : Iterable[int]
        Packets sent per node.
    packet_received : Iterable[int]
        Packets received per node.
    required_sinr_db : float
        Minimum SINR for successful decode.
    enable_sic : bool
        Enable successive interference cancellation.
    noise_figure_db : float
        Receiver noise figure.
    bandwidth_hz : float
        System bandwidth.
    write_collision_log : str, optional
        Path for collision log file.
    write_terminal_log : str, optional
        Path for terminal log file.

    Returns
    -------
    PacketAnalysisResult
        Per-BS and per-node statistics.
    """
    noise_w = thermal_noise_watts(bandwidth_hz, noise_figure_db)

    # Load packet log
    df = load_packet_log(data_path)
    result = PacketAnalysisResult(
        per_bs={i: BSStats() for i in range(num_bs)},
        per_node={i: NodeStats() for i in range(num_nodes)},
    )

    if df is None or df.empty:
        if write_terminal_log:
            with open(write_terminal_log, "a") as f:
                print("--no packets logged--", file=f)
        return result

    # Filter valid nodes
    df = df[df["node_id"].between(0, num_nodes - 1)]

    if df.empty:
        if write_terminal_log:
            with open(write_terminal_log, "a") as f:
                print("--no valid packets--", file=f)
        return result

    # Check if bs_id column exists in data; if not, fall back to filename extraction
    if "bs_id" not in df.columns:
        # Legacy: Extract BS ID from filename (bs_0_rx.txt -> 0)
        bs_id = 0
        if "bs_" in data_path:
            try:
                bs_id = int(data_path.split("bs_")[1].split("_")[0])
            except (IndexError, ValueError):
                pass
        df["bs_id"] = bs_id

    # Group packets by BS and use pre-computed collision results
    grouped_by_bs: Dict[int, set] = defaultdict(set)

    # Check if 'collided' column exists (from BSEngine real-time collision detection)
    use_precomputed = "collided" in df.columns

    for bs_id, bs_df in df.groupby("bs_id"):
        bs_id = int(bs_id)

        # Ensure BS stats exist for this bs_id
        if bs_id not in result.per_bs:
            result.per_bs[bs_id] = BSStats()

        result.per_bs[bs_id].transmitted = len(bs_df)

        if use_precomputed:
            # Use pre-computed collision status from BSEngine (more accurate, done in real-time)
            decoded_count = len(bs_df[bs_df["collided"] == False])
            collision_loss_count = len(bs_df[bs_df["collided"] == True])
            # Count collisions for this BS (collisions_total = number of collided packets)
            result.per_bs[bs_id].collisions_total = collision_loss_count
        else:
            # Fall back to post-hoc collision detection
            collisions, packet_outcomes = detect_collisions(
                df=bs_df,
                bs_id=bs_id,
                noise_w=noise_w,
                required_sinr_db=required_sinr_db,
                enable_sic=enable_sic,
            )
            result.collisions.extend(collisions)
            decoded_count = sum(1 for o in packet_outcomes.values() if o in ("decoded", "captured", "sic_recovered"))
            collision_loss_count = sum(1 for o in packet_outcomes.values() if o == "collision_loss")

        result.per_bs[bs_id].received = decoded_count
        result.per_bs[bs_id].errors = collision_loss_count

        # Track which nodes were heard by this BS
        for _, row in bs_df.iterrows():
            node_id = int(row["node_id"])
            if 0 <= node_id < num_nodes:
                grouped_by_bs[bs_id].add(node_id)
                result.per_node[node_id].heard_by_bs.add(bs_id)

    result.nodes_in_bs = {bs: sorted(nodes) for bs, nodes in grouped_by_bs.items()}

    # Multi-BS nodes (nodes heard by more than one BS)
    node_bs_count: Dict[int, int] = defaultdict(int)
    for bs_nodes in result.nodes_in_bs.values():
        for nid in bs_nodes:
            node_bs_count[nid] += 1
    result.nodes_multi_bs = sorted([nid for nid, c in node_bs_count.items() if c > 1])

    # DER per BS
    for bs_id_iter in range(num_bs):
        tx = result.per_bs[bs_id_iter].transmitted
        rx = result.per_bs[bs_id_iter].received
        result.per_bs[bs_id_iter].der = (rx / tx) if tx > 0 else 0.0

    # Per-node stats
    pkt_sent = list(packet_sent)
    pkt_recv = list(packet_received)

    for nid in range(num_nodes):
        sent = pkt_sent[nid] if nid < len(pkt_sent) else 0
        rec = pkt_recv[nid] if nid < len(pkt_recv) else 0

        ns = result.per_node[nid]
        ns.sent = sent
        ns.received_at_bs = rec
        ns.total_errors = max(0, sent - rec)

    # Write logs
    if write_collision_log and result.collisions:
        with open(write_collision_log, "a") as f:
            for e in result.collisions:
                nodes_str = ", ".join(f"N{nid}" for nid in e.involved_nodes)
                sinr_str = ", ".join(f"{s:.1f}dB" for s in e.sinr_db)
                pwr_str = ", ".join(f"{p:.1f}dBm" for p in e.powers_dbm)
                winners_str = f", Winners={e.winner_ids}" if e.winner_ids else ""
                print(
                    f"{e.timestamp} [{e.kind.upper()}] BS{e.bs_id} SC{e.subcarrier}: "
                    f"Nodes=[{nodes_str}], SINR=[{sinr_str}], Power=[{pwr_str}]{winners_str}",
                    file=f,
                )

    if write_terminal_log:
        with open(write_terminal_log, "a") as f:
            print("\n" + "=" * 80, file=f)
            print("PACKET ANALYSIS RESULTS", file=f)
            print("=" * 80, file=f)

            print("\n--- BASE STATION STATISTICS ---", file=f)
            for bs_id_iter in range(num_bs):
                s = result.per_bs[bs_id_iter]
                print(f"BS {bs_id_iter}:", file=f)
                print(f"  TX={s.transmitted}, RX={s.received}, Errors={s.errors}, DER={s.der:.4f}", file=f)
                print(f"  Nodes: {result.nodes_in_bs.get(bs_id_iter, [])}", file=f)

            print("\n--- NODE STATISTICS ---", file=f)
            print(f"Multi-BS nodes: {result.nodes_multi_bs}", file=f)
            for nid in range(num_nodes):
                ns = result.per_node[nid]
                if ns.sent > 0:
                    print(f"Node {nid}:", file=f)
                    print(f"  Sent={ns.sent}, RX@BS={ns.received_at_bs}, Errors={ns.total_errors}", file=f)
                    print(f"  Heard by BS={sorted(ns.heard_by_bs)}", file=f)

            print("\n" + "=" * 80 + "\n", file=f)

    return result

# --------------------------------------------------------------------------------------
# File I/O
# --------------------------------------------------------------------------------------

def load_packet_log(path: str) -> Optional[pd.DataFrame]:
    """
    Load packet log from BSEngine rx log file.

    Expected format (tab-separated with headers):
        start_ms, end_ms, slot_id, node_id, payload, rssi_dbm,
        sector_idx, expected_node, matched, subcarrier_shift, collided
    """
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None

    try:
        df = pd.read_csv(path, sep="\t")
        # Ensure required columns exist
        required = ["start_ms", "end_ms", "node_id", "rssi_dbm", "subcarrier_shift"]
        if not all(col in df.columns for col in required):
            print(f"[WARNING] Missing required columns in {path}")
            return None
        return df
    except Exception as e:
        print(f"[ERROR] Failed to load {path}: {e}")
        return None
