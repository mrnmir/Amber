# AMBER

A Python-based discrete-event simulator for 6G Ambient IoT Networks.

AMBER simulates battery-free Ambient IoT devices that harvest energy from their
environment and communicate with base stations via backscatter. Built on
[SimPy](https://simpy.readthedocs.io/).

Developed at Uppsala University, Sweden.

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
python demo_experiments/custom_protocol/custom_protocol_demo.py
```

Each prints a summary and writes `rx_packets.csv` and `tx_records.csv` to its
`output/` directory.

## API Reference

The [API Reference](api/backscatter.md) is generated directly from the docstrings
in the `amber` package.

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
