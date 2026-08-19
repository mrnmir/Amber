"""
Example of a user-written protocol built on top of Amber.

Protocol: "Adaptive framed-slotted ALOHA" (EPC-Gen2 Q-algorithm style).
  - The base station broadcasts a `collect` command that advertises a frame of
    N reception slots.
  - Each energised node picks a random slot and backscatters its sensor value.
  - After the frame the BS looks at how many transmissions COLLIDED and adapts
    the frame size for the next round:
        collisions occurred  -> grow the frame  (spread nodes out)
        frame was empty       -> shrink the frame (stop wasting slots)
    so the frame size self-tunes to the number of active nodes.

Nothing in Amber's engine is edited.
"""
import os
import math
import random

import simpy
import matplotlib.pyplot as plt

from amber import backscatter, bsengine, capacitor, propagation, controller, energy, radiodevices

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
SOLAR_TRACE = os.path.join(SCRIPT_DIR, "..", "threehundred_seconds_stable.xlsx")

SIM_TIME_MS = 15_001
FREQ_HZ = 924e6
PATHLOSS = "macro"

V_THRESHOLD_LOW = 1.3
V_THRESHOLD_HIGH = 1.7

R_SERIES = 5e3
R_LEAKAGE = 0.1e6
CAPACITANCE = 300e-6
DT = 1e-3

N_NODES = 12         
NODE_MIN_R = 5.0      
NODE_MAX_R = 40.0

START_SLOTS = 20      
MIN_SLOTS = 2
MAX_SLOTS = 32
SEED = 7


# ===============
# NODE HALF
# ===============
class AdaptiveAlohaNode(backscatter.BackscatterModule):

    # Called by receive_command() after the framework has applied the sensitivity
    # gate and stored the frame the BS advertised into self.rx_slots.
    def handle_command(self, cmd, bs_id, data):
        if cmd == "collect":
            # Pick one slot uniformly at random from the advertised frame;
            # -1 means no frame was advertised, so the node stays silent.
            self.chosen_slot_idx = (
                random.randint(0, len(self.rx_slots) - 1) if self.rx_slots else -1
            )
            self.state = "active"
            self.last_tx_command_time = self.env.now

    # Called when the node's chosen slot comes up; returning None transmits nothing.
    def build_tx_payload(self, sensor_data):
        if self.state == "active":
            return {"type": "data", "payload": sensor_data}
        return None


# ===============
# BS HALF
# ===============
# Generator yielding one frame at a time. It must stay lazy: each frame is
# decided only after the previous one has finished, which is what makes the
# protocol adaptive.
def adaptive_aloha_policy(bs):
    n_slots = START_SLOTS
    prev_collided = 0              # bs.total_collided is cumulative, so keep the previous value to difference it
    frame = 0
    load_ema = 0.0                 # smoothed estimate of contending nodes / frame, so as not to chase noise
    bs.adapt_history = []

    while True:
        # One frame = a broadcast "collect" command plus n_slots reply slots.
        # Both go in the SAME yielded frame so the engine's lookahead can
        # advertise those slots to the nodes in the command itself.
        yield [("tx", 5, "collect", {"cmd": "collect", "target": -1})] + \
              [("rx", 8, "slot") for _ in range(n_slots)]

        # Resumes here only once the whole frame has played out.
        heard = len(bs.decoded_this_frame)              # replies decoded cleanly this frame
        collided = bs.total_collided - prev_collided    # replies lost to collisions this frame
        prev_collided = bs.total_collided
        frame += 1
        bs.adapt_history.append((bs.env.now / 1000.0, n_slots, heard, collided))
        print(f"  frame {frame:>3}: slots={n_slots:>2}  decoded={heard:>2}  collided={collided:>2}")

        # Each transmission increments exactly one of the two counters, so the
        # sum is how many nodes actually contended this frame.
        contenders = heard + collided
        load_ema = 0.75 * load_ema + 0.25 * contenders
        # Aim for roughly two slots per contender, clamped to [MIN_SLOTS, MAX_SLOTS].
        target = min(MAX_SLOTS, max(MIN_SLOTS, round(load_ema * 2)))
        # Move one slot per frame, with a one-slot deadband above target so the
        # frame size settles instead of oscillating around it.
        if n_slots < target:
            n_slots += 1
        elif n_slots > target + 1:
            n_slots -= 1

# Put randomly placed nodes in a circle around the BS, with random radius and angle.
def build_nodes():
    nodes = []
    for i in range(N_NODES):
        r = random.uniform(NODE_MIN_R, NODE_MAX_R)
        ang = random.uniform(0, 2 * math.pi)
        nodes.append(radiodevices.Node(
            id=i, x=r * math.cos(ang), y=r * math.sin(ang),
            height=1.5, sensitivity_dbm=-100, efficiency=0.7))
    return nodes

# Define a base station with three sectors, each 120 degrees apart, using 3GPP antenna model.
def make_bs():
    return radiodevices.BaseStation(
        id=0, x=0, y=0, site_radius=2,
        sectors=[radiodevices.Sector(azimuth_deg=a, beamwidth_deg=65, power=46,
                                     antenna_gain_dbi=15, sensitivity_dbm=-100, height=25)
                 for a in (0, 120, 240)])

# Define controller and capacitor parameters
CONTROLLER_PARAMS = controller.ControllerParams(
    currents=controller.CurrentsA(listening=1.4e-4, sensing=0.512e-3,
                                  processing=1.28e-3, transmitting=5e-3),
    durations_ms=controller.DurationsMs(listening=5, sensing=2,
                                        processing=5, transmitting=15),
    thresholds_v=controller.VoltageThresholdsV(low=V_THRESHOLD_LOW, high=V_THRESHOLD_HIGH))

CAP_PARAMS = capacitor.CapacitorParams(dt=DT, R_series=R_SERIES,
                                       R_leakage=R_LEAKAGE, C=CAPACITANCE)


def run_simulation():
    print("=" * 70)
    print("Custom protocol demo: ADAPTIVE framed-slotted ALOHA")
    print("=" * 70)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    random.seed(SEED)

    env = simpy.Environment()
    energy_source = energy.EnvEnergySource(env=env, file_path=SOLAR_TRACE,
                                           column="V_IM", resistance=R_SERIES)

    nodes = build_nodes()
    bs = make_bs()

    # Set up coverage map for all nodes, using the external energy source to determine node power
    cov = propagation.CoverageMap(
        base_stations=[bs], nodes=nodes, freq_hz=FREQ_HZ, pathloss_model=PATHLOSS,
        los=True, node_energy_mode="hybrid",
        node_ext_power_fn=lambda node: energy_source.ext_power, combine_mode="max")
    cov.compute_coverage_map(-120, 120, -120, 120, step_m=2.0)

    # Compute downlink and uplink results for all nodes
    dl = cov.compute_bs_to_point(nodes)
    cov.calculate_node_power(nodes, dl)
    ul = cov.compute_point_to_bs(nodes)

    # Set up Capacitor, BackscatterModule, and Controller for each node
    capacitors, backscatter_modules = [], []
    for node in nodes:
        cap = capacitor.Capacitor(env=env, id=node.id, params=CAP_PARAMS, initial_voltage=0.0)
        capacitors.append(cap)
        bsm = AdaptiveAlohaNode(env=env, node=node, bs_processes=[],
                                uplink_results=ul, downlink_results=dl)
        backscatter_modules.append(bsm)
        controller.Controller(env=env, capacitor_ctrl=cap, node=node,
                              params=CONTROLLER_PARAMS, backscatter=bsm,
                              coverage_map=cov, downlink_results=dl)

    bs_behavior = bsengine.BSBehavior(
        env=env, base_station=bs, policy=adaptive_aloha_policy,
        backscatter_modules=backscatter_modules, enable_sic=True)
    for bsm in backscatter_modules:
        bsm.bs_processes = [bs_behavior]

    print(f"Running {SIM_TIME_MS/1000:.1f} s, {N_NODES} nodes, frame {MIN_SLOTS}..{MAX_SLOTS} slots")
    env.run(until=SIM_TIME_MS)

    # Compute results
    hist = bs_behavior.adapt_history
    frames = len(hist)
    final_slots = hist[-1][1] if hist else 0
    total_sent = sum(b.packets_sent for b in backscatter_modules)
    decoded = sum(1 for p in bs_behavior.rx_packets if not p.collided)
    collided = sum(1 for p in bs_behavior.rx_packets if p.collided)

    print("\nRESULTS")
    print(f"  frames run={frames}   frame size: start={START_SLOTS} -> end={final_slots}")
    print(f"  sent={total_sent}  decoded={decoded}  collided={collided}")
    print(f"  throughput={decoded/(SIM_TIME_MS/1000):.1f} pkt/s")

    save_packets(bs_behavior, backscatter_modules)
    return {"bs": bs_behavior, "caps": capacitors, "bsms": backscatter_modules, "cov": cov}

# Save results to CSV files for later analysis
def save_packets(bs_behavior, backscatter_modules):
    import pandas as pd
    rx = [{"time_ms": p.start_ms, "node_id": p.node_id, "rssi_dbm": p.rssi_dbm,
           "collided": p.collided} for p in bs_behavior.rx_packets]
    if rx:
        pd.DataFrame(rx).sort_values("time_ms").to_csv(
            os.path.join(OUTPUT_DIR, "rx_packets.csv"), index=False)
    tx = [{"node_id": b.node.id, "time_ms": r.end_ms, "slot_idx": r.slot_idx,
           "delivered": r.delivered} for b in backscatter_modules for r in b.tx_records]
    if tx:
        pd.DataFrame(tx).sort_values("time_ms").to_csv(
            os.path.join(OUTPUT_DIR, "tx_records.csv"), index=False)
    if bs_behavior.adapt_history:
        pd.DataFrame(bs_behavior.adapt_history,
                     columns=["time_s", "n_slots", "heard", "collided"]).to_csv(
            os.path.join(OUTPUT_DIR, "adaptation.csv"), index=False)
    print(f"  saved rx/tx/adaptation CSVs to {OUTPUT_DIR}")

# Plot results
def plot_results(sim):
    import numpy as np
    import pandas as pd

    hist = sim["bs"].adapt_history
    if not hist:
        print("no adaptation history to plot")
        return
    df = pd.DataFrame(hist, columns=["t", "slots", "decoded", "collided"])

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10, 6.5), sharex=True,
        gridspec_kw={"height_ratios": [1.1, 1]})
    fig.suptitle("Adaptive framed-slotted ALOHA",
                 fontsize=13, y=0.98)

    # --- top: frame size, raw (faint) + rolling average (bold) ---
    smooth = df["slots"].rolling(15, min_periods=1, center=True).mean()
    ax1.step(df["t"], df["slots"], where="post", color="#9db4e0", lw=1, alpha=0.55,
             label="frame size (per frame)")
    ax1.plot(df["t"], smooth, color="#2c51a0", lw=2.6, label="frame size (rolling avg)")
    ax1.set_ylabel("frame size (slots)")
    ax1.set_ylim(0, df["slots"].max() * 1.1)
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="upper right", frameon=False)
    ax1.spines[["top", "right"]].set_visible(False)

    # --- bottom: decoded vs collided, aggregated into time bins ---
    nbins = 40
    edges = np.linspace(df["t"].min(), df["t"].max(), nbins + 1)
    df["bin"] = pd.cut(df["t"], bins=edges, include_lowest=True)
    g = df.groupby("bin", observed=True)[["decoded", "collided"]].sum()
    centers = np.array([iv.mid for iv in g.index])
    width = (edges[1] - edges[0]) * 0.9
    ax2.bar(centers, g["decoded"], width=width, color="#2ca02c", label="decoded")
    ax2.bar(centers, g["collided"], width=width, bottom=g["decoded"],
            color="#d62728", label="collided")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel(f"packets per {edges[1]-edges[0]:.2f}s bin")
    ax2.grid(True, alpha=0.25, axis="y")
    ax2.legend(loc="upper right", frameon=False)
    ax2.spines[["top", "right"]].set_visible(False)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = os.path.join(OUTPUT_DIR, "adaptation.png")
    fig.savefig(out, dpi=130)
    print(f"  saved plot to {out}")

    def _on_key(event):
        if event.key in ("q", "escape"):
            plt.close("all")
    fig.canvas.mpl_connect("key_press_event", _on_key)
    plt.show()


if __name__ == "__main__":
    sim = run_simulation()
    plot_results(sim)
