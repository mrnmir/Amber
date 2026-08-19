# amber/energy.py
from __future__ import annotations
import simpy
import pandas as pd


class EnvEnergySource:
    """
    SimPy actor that replays an external energy (voltage/power) trace
    into the simulation environment.

    Each environment step (timeout=1) consumes the next row from
    the input file and updates `self.ext_power`. Loops indefinitely.

    Parameters
    ----------
    env : simpy.Environment
        The simulation environment.
    file_path : str
        Path to the Excel file containing the energy trace.
    column : str
        Column name to read from (default "V_IM").
    resistance : float
        Load resistance in Ohms for P = V^2 / R conversion (default 1.0).
    """

    def __init__(
        self,
        env: simpy.Environment,
        file_path: str,
        column: str = "V_IM",
        resistance: float = 1.0,
    ):
        """

        """
        self.env = env
        self.file_path = file_path
        self.column = column
        self.resistance = resistance
        self.ext_power: float = 0.0

        # Load once at init
        self._df = pd.read_excel(file_path)
        if column not in self._df.columns:
            raise ValueError(f"Column '{column}' not found in {file_path}")

        # Pre-extract voltage values for faster iteration
        self._voltages = self._df[column].values

        # Initialize with first value so power is available before first timeout
        if len(self._voltages) > 0:
            self.ext_power = (self._voltages[0] ** 2) / self.resistance

        # Create the process
        self.action = env.process(self.run())

    def run(self):
        while True:
            for voltage in self._voltages:
                yield self.env.timeout(1)
                self.ext_power = (voltage**2) / self.resistance  # P = V^2 / R (Watts)
