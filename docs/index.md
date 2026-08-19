# AMBER

A Python-based discrete-event simulator for 6G Ambient IoT Networks.

AMBER simulates battery-free Ambient IoT devices that harvest energy from their
environment and communicate with base stations via backscatter. Built on
[SimPy](https://simpy.readthedocs.io/).

## Installation

```bash
git clone https://github.com/mrnmir/Amber.git
cd Amber
pip install -e .
```

## Quickstart

```bash
python demo_experiments/broadcast/broadcast_demo.py
python demo_experiments/broadcast_sic/broadcast_sic_demo.py
python demo_experiments/unicast/unicast_demo.py
```

Each prints a summary and writes `rx_packets.csv` and `tx_records.csv` to its
`output/` directory.

## API Reference

The [API Reference](api/backscatter.md) is generated directly from the docstrings
in the `amber` package.
