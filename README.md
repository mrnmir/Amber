<picture>
  <source media="(prefers-color-scheme: dark)" srcset="assets/logo_white.svg">
  <source media="(prefers-color-scheme: light)" srcset="assets/logo_black.svg">
  <img src="assets/logo_black.svg" alt="AMBER Logo" width="150" align="left" hspace="20" vspace="14">
</picture>

### 6G Ambient IoT Simulator

A Python-based discrete-event simulator for 6G Ambient IoT Networks.

[![docs](https://github.com/mrnmir/Amber/actions/workflows/docs.yml/badge.svg)](https://github.com/mrnmir/Amber/actions/workflows/docs.yml)

<br clear="left">

---

## About

AMBER simulates battery-free Ambient IoT devices that harvest energy from their
environment and communicate with base stations via backscatter. Built on
[SimPy](https://simpy.readthedocs.io/).

## Installation

Requires Python 3.9+.

```bash
git clone https://github.com/mrnmir/Amber.git
cd Amber
pip install -e .
```

Or with [uv](https://docs.astral.sh/uv/):

```bash
uv sync
```

> **Note:** editable installs need pip 21.3 or newer. Upgrade with
> `python -m pip install --upgrade pip` if the install fails.

## Quickstart

Three example scenarios are included:

```bash
python demo_experiments/broadcast/broadcast_demo.py       # 3 nodes, broadcast downlink
python demo_experiments/broadcast_sic/broadcast_sic_demo.py  # 3 nodes, broadcast with SIC enabled
python demo_experiments/unicast/unicast_demo.py           # 3 nodes, per-node addressed downlink
```

Each prints a summary and writes `rx_packets.csv` and `tx_records.csv` to its
`output/` directory.

## Modules

| Module | Contents |
| --- | --- |
| `radiodevices` | `Node`, `Sector`, `BaseStation` topology dataclasses |
| `propagation` | Path loss, antenna patterns, `CoverageMap` |
| `capacitor` | Energy storage model (`Capacitor`, `CapacitorParams`) |
| `controller` | Voltage-threshold state machine driving node duty cycling |
| `energy` | `EnvEnergySource` — replays an external energy trace |
| `backscatter` | Node-side backscatter transmit/receive |
| `bsengine` | Base station behaviour, downlink scheduling |
| `packet_analysis` | Collision detection, SIC, throughput and DER statistics |

Full API reference, generated from the source docstrings, is at
**[mrnmir.github.io/Amber](https://mrnmir.github.io/Amber/)**.

## License

BSD 3-Clause — see [LICENSE](LICENSE).
