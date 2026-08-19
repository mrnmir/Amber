from amber import backscatter, bsengine, capacitor, propagation, controller, energy, radiodevices
import simpy
import matplotlib.pyplot as plt
import pandas as pd
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
SOLAR_TRACE = os.path.join(SCRIPT_DIR, "..", "threehundred_seconds_stable.xlsx")

SIM_TIME_MS = 20_001
FREQ_HZ = 924e6
PATHLOSS = "macro"

V_THRESHOLD_LOW = 1.3
V_THRESHOLD_HIGH = 1.7

R_SERIES = 5e3
R_LEAKAGE = 0.1e6
CAPACITANCE = 300e-6
DT = 1e-3

LISTENING_TIME = 5
LISTENING_CURRENT = 1.4e-4
SENSING_TIME = 2
SENSING_CURRENT = 0.512e-3
PROCESSING_TIME = 5
PROCESSING_CURRENT = 1.28e-3
TRANSMITTING_TIME = 15
TRANSMITTING_CURRENT = 5e-3

TX_DURATION_MS = 5
RX_DURATION_MS = 10

# Define nodes
node0 = radiodevices.Node(id=0, x=0, y=20, height=1.5, sensitivity_dbm=-100, efficiency=0.7)
node1 = radiodevices.Node(id=1, x=0, y=30, height=1.5, sensitivity_dbm=-100, efficiency=0.7)
node2 = radiodevices.Node(id=2, x=0, y=100, height=1.5, sensitivity_dbm=-100, efficiency=0.7)

# Define a base station with 3 sectors
bs = radiodevices.BaseStation(
    id=0,
    x=0,
    y=0,
    site_radius=2,
    sectors=[
        radiodevices.Sector(
            azimuth_deg=0,
            beamwidth_deg=65,
            power=46,
            antenna_gain_dbi=15,
            sensitivity_dbm=-100,
            height=25,
        ),
        radiodevices.Sector(
            azimuth_deg=120,
            beamwidth_deg=65,
            power=46,
            antenna_gain_dbi=15,
            sensitivity_dbm=-100,
            height=25,
        ),
        radiodevices.Sector(
            azimuth_deg=240,
            beamwidth_deg=30,
            power=46,
            antenna_gain_dbi=15,
            sensitivity_dbm=-100,
            height=25,
        )
    ],
)

# Controller and Capacitor parameters
controller_parameters = controller.ControllerParams(
    currents=controller.CurrentsA(
        listening=LISTENING_CURRENT,
        sensing=SENSING_CURRENT,
        processing=PROCESSING_CURRENT,
        transmitting=TRANSMITTING_CURRENT,
    ),
    durations_ms=controller.DurationsMs(
        listening=LISTENING_TIME,
        sensing=SENSING_TIME,
        processing=PROCESSING_TIME,
        transmitting=TRANSMITTING_TIME,
    ),
    thresholds_v=controller.VoltageThresholdsV(
        low=V_THRESHOLD_LOW,
        high=V_THRESHOLD_HIGH,
    ),
)

capacitor_params = capacitor.CapacitorParams(
    dt=DT,
    R_series=R_SERIES,
    R_leakage=R_LEAKAGE,
    C=CAPACITANCE,
)

# Unicast schedule: one TX command addressed to each node, each followed by an RX slot
unicast_schedule = [
    ("tx", TX_DURATION_MS, "cmd", {"target": 0, "cmd": "send_data"}),
    ("rx", RX_DURATION_MS, "listen"),
    ("tx", TX_DURATION_MS, "cmd", {"target": 1, "cmd": "send_data"}),
    ("rx", RX_DURATION_MS, "listen"),
    ("tx", TX_DURATION_MS, "cmd", {"target": 2, "cmd": "send_data"}),
    ("rx", RX_DURATION_MS, "listen"),
]


def run_simulation():
    print("=" * 80)
    print("6G IoT Unicast Demo (3 nodes: WPT, WPT, Hybrid)")
    print("=" * 80)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Set up the SimPy environment
    env = simpy.Environment()

    # Set up the energy source that reads from the solar trace Excel file
    energy_source = energy.EnvEnergySource(
        env=env,
        file_path=SOLAR_TRACE,
        column="V_IM",
        resistance=R_SERIES,
    )

    # Define nodes and categorize them as WPT or Hybrid
    wpt_nodes = [node0, node1]
    hybrid_nodes = [node2]
    all_nodes = wpt_nodes + hybrid_nodes

    # Set up coverage maps for WPT and Hybrid nodes
    cov_wpt = propagation.CoverageMap(
        base_stations=[bs],
        nodes=all_nodes,
        freq_hz=FREQ_HZ,
        pathloss_model=PATHLOSS,
        los=True,
        node_energy_mode="wpt",
        node_ext_power_fn=lambda node: energy_source.ext_power,
        combine_mode="max",
    )

    cov_hybrid = propagation.CoverageMap(
        base_stations=[bs],
        nodes=hybrid_nodes,
        freq_hz=FREQ_HZ,
        pathloss_model=PATHLOSS,
        los=True,
        node_energy_mode="hybrid",
        node_ext_power_fn=lambda node: energy_source.ext_power,
        combine_mode="max",
    )

    # Compute coverage maps for WPT and Hybrid nodes
    xmin = min(n.x for n in all_nodes) - 200
    xmax = max(n.x for n in all_nodes) + 200
    ymin = min(n.y for n in all_nodes) - 200
    ymax = max(n.y for n in all_nodes) + 200
    cov_wpt.compute_coverage_map(xmin, xmax, ymin, ymax, step_m=1.0)

    # Compute downlink and uplink results for WPT and Hybrid nodes
    dl_wpt = cov_wpt.compute_bs_to_point(wpt_nodes)
    cov_wpt.calculate_node_power(wpt_nodes, dl_wpt)
    ul_wpt = cov_wpt.compute_point_to_bs(wpt_nodes)

    dl_hybrid = cov_hybrid.compute_bs_to_point(hybrid_nodes)
    cov_hybrid.calculate_node_power(hybrid_nodes, dl_hybrid)
    ul_hybrid = cov_hybrid.compute_point_to_bs(hybrid_nodes)

    capacitors = []
    backscatter_modules = []
    controllers = []

    per_node_setup = [
        (node0, dl_wpt, ul_wpt, cov_wpt),
        (node1, dl_wpt, ul_wpt, cov_wpt),
        (node2, dl_hybrid, ul_hybrid, cov_hybrid),
    ]

    # Set up Capacitor, BackscatterModule, and Controller for each node
    for node, dl, ul, cov in per_node_setup:
        cap = capacitor.Capacitor(
            env=env, id=node.id, params=capacitor_params, initial_voltage=0.0
        )
        capacitors.append(cap)

        bsm = backscatter.BackscatterModule(
            env=env,
            node=node,
            bs_processes=[],
            uplink_results=ul,
            downlink_results=dl,
        )
        backscatter_modules.append(bsm)

        ctrl = controller.Controller(
            env=env,
            capacitor_ctrl=cap,
            node=node,
            params=controller_parameters,
            backscatter=bsm,
            coverage_map=cov,
            downlink_results=dl,
        )
        controllers.append(ctrl)

    bs_behavior = bsengine.BSBehavior(
        env=env,
        base_station=bs,
        schedule=unicast_schedule,
        backscatter_modules=backscatter_modules,
        loop=True,
    )
    bs_behavior.nodes_registered = [n.id for n in all_nodes]

    for bsm in backscatter_modules:
        bsm.bs_processes = [bs_behavior]
        bsm.state = "registered"

    print(f"Starting simulation: {SIM_TIME_MS} ms ({SIM_TIME_MS/1000:.1f} s)")
    env.run(until=SIM_TIME_MS)
    print("Simulation complete!")

    # Compute results
    total_sent = sum(bsm.packets_sent for bsm in backscatter_modules)
    decoded = set()
    for pkt in bs_behavior.rx_packets:
        if not pkt.collided:
            decoded.add((pkt.node_id, pkt.start_ms))
    total_received = len(decoded)

    der = 1.0 - (total_received / total_sent) if total_sent > 0 else 0.0
    throughput = total_received / (SIM_TIME_MS / 1000)

    print("\n" + "=" * 80)
    print("RESULTS")
    print("=" * 80)
    print(f"Sent: {total_sent}, Decoded: {total_received}")
    print(f"DER: {der:.2%}, Throughput: {throughput:.1f} pkt/s")
    for bsm in backscatter_modules:
        node_decoded = sum(
            1 for pkt in bs_behavior.rx_packets
            if not pkt.collided and pkt.node_id == bsm.node.id
        )
        print(
            f"  Node {bsm.node.id} (mode={'hybrid' if bsm.node is node2 else 'wpt'}): "
            f"sent={bsm.packets_sent}, decoded={node_decoded}"
        )

    save_packets(bs_behavior, backscatter_modules)

    return {
        "coverage_map": cov_wpt,
        "capacitors": capacitors,
        "controllers": controllers,
        "backscatter_modules": backscatter_modules,
        "bs_behavior": bs_behavior,
        "der": der,
        "throughput": throughput,
        "total_sent": total_sent,
        "total_received": total_received,
    }

# Save results to CSV files for later analysis
def save_packets(bs_behavior, backscatter_modules):
    rx_data = []
    for pkt in bs_behavior.rx_packets:
        rx_data.append({
            "bs_id": bs_behavior.id,
            "time_ms": pkt.start_ms,
            "node_id": pkt.node_id,
            "rssi_dbm": pkt.rssi_dbm,
            "expected_node": pkt.expected_node,
            "collided": pkt.collided,
            "matched": pkt.matched,
            "subcarrier_shift": pkt.subcarrier_shift,
        })
    df_rx = pd.DataFrame(rx_data).sort_values("time_ms")
    df_rx.to_csv(os.path.join(OUTPUT_DIR, "rx_packets.csv"), index=False)
    print(f"Saved {len(df_rx)} received packets to rx_packets.csv")

    tx_data = []
    for bsm in backscatter_modules:
        for rec in bsm.tx_records:
            tx_data.append({
                "node_id": bsm.node.id,
                "time_ms": rec.end_ms,
                "slot_idx": rec.slot_idx,
                "payload_type": rec.payload_type,
                "delivered": rec.delivered,
            })
    df_tx = pd.DataFrame(tx_data).sort_values("time_ms")
    df_tx.to_csv(os.path.join(OUTPUT_DIR, "tx_records.csv"), index=False)
    print(f"Saved {len(df_tx)} transmissions to tx_records.csv")

# Plot results: coverage map, capacitor voltages, DER, throughput, and per-node outcomes
def plot_results(sim):
    coverage_map = sim["coverage_map"]
    capacitors = sim["capacitors"]

    fig1, ax1 = plt.subplots(figsize=(8, 7))
    coverage_map.plot_rssi(ax=ax1, vmin=-60, vmax=-20)
    ax1.set_title("Unicast Demo - RSSI Coverage Map")
    plt.tight_layout()

    fig2, ax2 = plt.subplots(figsize=(12, 6))
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    labels = ["Node 0 (WPT)", "Node 1 (WPT)", "Node 2 (Hybrid)"]
    for cap, color, label in zip(capacitors, colors, labels):
        if cap.voltage_history:
            times, voltages = zip(*cap.voltage_history)
            ax2.plot(times, voltages, label=label, color=color, alpha=0.8)
    ax2.axhline(V_THRESHOLD_HIGH, color="#555555", linestyle="--", alpha=0.5, label="Active threshold")
    ax2.axhline(V_THRESHOLD_LOW, color="#999999", linestyle="--", alpha=0.5, label="Low threshold")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Capacitor Voltage (V)")
    ax2.set_title("Capacitor Voltage Over Time")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    fig3, axd = plt.subplot_mosaic(
        [["der", "thr"], ["outcome", "outcome"]],
        figsize=(10, 8), constrained_layout=True
    )

    ax_der = axd["der"]
    ax_der.bar(["DER"], [sim["der"] * 100], color="#d627d0", width=0.9)
    ax_der.set_ylabel("Delivery Error Rate (%)")
    ax_der.set_ylim(0, 110)
    ax_der.set_xlim(-1, 1)
    ax_der.text(0, sim["der"] * 100 + 2, f"{sim['der']*100:.1f}%", ha="center")
    ax_der.grid(True, alpha=0.3, axis="y")

    ax_thr = axd["thr"]
    ax_thr.bar(["Throughput"], [sim["throughput"]], color="#2c51a0", width=0.9)
    ax_thr.set_ylabel("Throughput (pkt/s)")
    ax_thr.set_ylim(0, sim["throughput"] * 1.1 if sim["throughput"] > 0 else 1)
    ax_thr.set_xlim(-1, 1)
    ax_thr.text(0, sim["throughput"] * 1.02, f"{sim['throughput']:.1f}", ha="center")
    ax_thr.grid(True, alpha=0.3, axis="y")

    ax_out = axd["outcome"]
    bs_behavior = sim["bs_behavior"]
    backscatter_modules = sim["backscatter_modules"]
    node_ids = [bsm.node.id for bsm in backscatter_modules]
    sent = [bsm.packets_sent for bsm in backscatter_modules]
    decoded = [
        sum(1 for pkt in bs_behavior.rx_packets
            if not pkt.collided and pkt.node_id == nid)
        for nid in node_ids
    ]
    collided = [
        sum(1 for pkt in bs_behavior.rx_packets
            if pkt.collided and pkt.node_id == nid)
        for nid in node_ids
    ]
    lost = [max(s - d - c, 0) for s, d, c in zip(sent, decoded, collided)]

    x_labels = ["Node 0 (WPT)", "Node 1 (WPT)", "Node 2 (Hybrid)"]
    ax_out.bar(x_labels, decoded, label="Decoded", color="#2ca02c", width=0.5)
    ax_out.bar(x_labels, collided, bottom=decoded, label="Collided", color="#d62728", width=0.5)
    ax_out.bar(x_labels, lost,
               bottom=[d + c for d, c in zip(decoded, collided)],
               label="Lost", color="#999999", width=0.5)
    for i, s in enumerate(sent):
        ax_out.text(i, s + max(sent) * 0.02, f"sent={s}", ha="center", fontsize=9)
    ax_out.set_ylabel("Packets")
    ax_out.set_ylim(0, max(sent) * 1.1 if sent and max(sent) > 0 else 1)
    ax_out.set_title("Per-node packet outcome")
    ax_out.legend()
    ax_out.grid(True, alpha=0.3, axis="y")

    fig3.suptitle("Unicast - DER, Throughput and Outcome")
    # plt.tight_layout()

    # press q or escape on any figure to close all and exit
    def _on_key(event):
        if event.key in ("q", "escape"):
            plt.close("all")
    for f in (fig1, fig2, fig3):
        f.canvas.mpl_connect("key_press_event", _on_key)

    plt.show()


if __name__ == "__main__":
    sim = run_simulation()
    plot_results(sim)
