[![docs](https://github.com/mrnmir/Amber/actions/workflows/docs.yml/badge.svg)](https://github.com/mrnmir/Amber/actions/workflows/docs.yml)
[![license](https://img.shields.io/badge/license-BSD--3--Clause-blue)](LICENSE)
[![python](https://img.shields.io/badge/python-3.9%2B-blue)](pyproject.toml)

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="assets/logo_white.svg">
  <source media="(prefers-color-scheme: light)" srcset="assets/logo_black.svg">
  <img src="assets/logo_black.svg" alt="AMBER Logo" width="120" align="left" hspace="20" vspace="6">
</picture>

### 6G Ambient IoT Simulator

A Python-based discrete-event simulator for 6G Ambient IoT Networks.

<br clear="left">

---

## About

AMBER simulates battery-free Ambient IoT devices that harvest energy from their
environment and communicate with base stations via backscatter. Built on
[SimPy](https://simpy.readthedocs.io/).

Developed at Uppsala University, Sweden.

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

Four example scenarios are included:

```bash
python demo_experiments/broadcast/broadcast_demo.py              # 3 nodes, broadcast downlink
python demo_experiments/broadcast_sic/broadcast_sic_demo.py      # 3 nodes, broadcast with SIC enabled
python demo_experiments/unicast/unicast_demo.py                  # 3 nodes, per-node addressed downlink
python demo_experiments/custom_protocol/custom_protocol_demo.py  # 12 nodes, user-defined adaptive framed-slotted ALOHA
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

## Publication

A demo abstract describing AMBER:

> M. Manafova, T. Voigt, and L. Mottola. "Demo Abstract: Amber: 6G Ambient IoT
> Simulator." *2026 ACM/IEEE International Conference on Embedded Artificial
> Intelligence and Sensing Systems – Posters and Demos (SenSys-Adjunct)*,
> pp. 23–24, 2026.
> [doi:10.1109/SenSys-Adjunct71932.2026.00021](https://doi.org/10.1109/SenSys-Adjunct71932.2026.00021)

```bibtex
@INPROCEEDINGS{11574939,
  author={Manafova, Mirana and Voigt, Thiemo and Mottola, Luca},
  booktitle={2026 ACM/IEEE International Conference on Embedded Artificial Intelligence and Sensing Systems - Posters and Demos (SenSys-Adjunct)}, 
  title={Demo Abstract: Amber: 6G Ambient IoT Simulator}, 
  year={2026},
  volume={},
  number={},
  pages={23-24},
  keywords={Internet of Things;Modeling;Base stations;Backscatter;Energy harvesting;Energy;Printing;Ambient IoT;6G;IoT simulator;backscatter communication;energy harvesting},
  doi={10.1109/SenSys-Adjunct71932.2026.00021}}
```

## License

BSD 3-Clause — see [LICENSE](LICENSE).
