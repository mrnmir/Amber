# amber/capacitor.py
from __future__ import annotations
import math
import simpy
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass(frozen=True)
class CapacitorParams:
    """Physical + timing constants."""

    dt: float = 1e-3  # s
    R_series: float = 5e3  # ohm
    R_leakage: float = 0.1e6  # ohm
    C: float = 300e-6  # F

# ----------------------------
# Capacitor model
# ----------------------------


@dataclass
class Capacitor:
    """
    SimPy-driven capacitor with:
      - charging (with leakage),
      - self-discharge,
      - load discharge with arbitrary current.
    """

    env: simpy.Environment
    id: int
    params: CapacitorParams = field(default_factory=CapacitorParams)
    keep_logs: bool = True
    initial_voltage: float = 0.0  # V (initial charge state)
    voltage_max: float = 2.0  # V (physical clamp / PMIC max)

    # runtime state (initialized in __post_init__)
    voltage: float = field(init=False, default=0.0)
    internal_time: float = field(init=False, default=0.0)  # seconds
    energy: float = field(init=False, default=0.0)  # Joule
    voltage_source: float = field(init=False, default=0.0)  # V (source equivalent)
    current: float = field(init=False, default=0.0)  # A (load)
    voltage_history: List[Tuple[float, float]] = field(init=False, default_factory=list)
        
    def __post_init__(self):
        # Set initial voltage
        self.voltage = min(self.initial_voltage, self.voltage_max)
        self.energy = 0.5 * self.params.C * self.voltage * self.voltage
        # start the SimPy process
        self.action = self.env.process(self.run())

    # ---------------- core helpers ----------------
    def _tick_common(self):
        self.internal_time = self.env.now / 1000.0  # ms -> s (your sim uses ms ticks)
        # E = 1/2 C V^2
        self.energy = 0.5 * self.params.C * self.voltage * self.voltage
        if self.keep_logs:
            self.voltage_history.append((self.internal_time, self.voltage))

    # --------------- transitions ------------------
    def capacitor_charging(self) -> float:
        """Charge toward voltage_source with leakage during one dt step.

        Returns
        -------
        float
            New voltage after one dt step.
        """        
        p = self.params
        # explicit-Euler for source coupling + exact leakage decay over dt
        # v_{t+dt} = v_t + dt*( (Vsrc - v_t)/(R_series*C) ) + v_t*(e^{-dt/(R_leak*C)} - 1)
        self.voltage = (p.dt / (p.R_series * p.C)) * (
            self.voltage_source - self.voltage
        ) + self.voltage * math.exp(-p.dt / (p.R_leakage * p.C))
        self.voltage = min(max(self.voltage, 0.0), self.voltage_max)
        self._tick_common()
        return self.voltage

    def capacitor_selfdischarge(self) -> float:
        """Leakage-only decay during one dt step.

        Returns
        -------
        float
            New voltage after one dt step.
        """        
        p = self.params
        self.voltage *= math.exp(-p.dt / (p.R_leakage * p.C))
        self.voltage = min(max(self.voltage, 0.0), self.voltage_max)
        self._tick_common()
        return self.voltage

    def load_discharging(self, current: float) -> float:
        """Discharge under an external load current for one dt step.

        Parameters
        ----------
        current : float
            Load current to draw from the capacitor.

        Returns
        -------
        float
            New voltage after one dt step.
        """        
        p = self.params
        # energy drawn from cap by the load over dt
        energy_consumed = self.voltage * current * p.dt  # (approx) P*dt with P=V*I
        self.energy = max(self.energy - energy_consumed, 0.0)

        # ideal cap voltage drop from energy extraction:
        # E = 1/2 C V^2  =>  V' = sqrt(max(V^2 - 2*E/C, 0))
        v_after_ideal = math.sqrt(
            max(self.voltage * self.voltage - (2.0 * energy_consumed) / p.C, 0.0)
        )

        # plus leakage during dt (exact decay applied to the *current* voltage):
        # combine as in your original: v_next = v_after_ideal - v * (1 - e^{-dt/(R_leak*C)})
        self.voltage = v_after_ideal - self.voltage * (
            1.0 - math.exp(-p.dt / (p.R_leakage * p.C))
        )
        self.voltage = min(max(self.voltage, 0.0), self.voltage_max)

        self._tick_common()
        return self.voltage

    def charge_and_discharge(self, load_current: float) -> float:
        """Simultaneous charging from voltage_source and discharging to load during one dt step.
           Net power model: voltage changes based on (harvesting - consumption).

        Parameters
        ----------
        load_current : float
            Load current to draw from the capacitor.

        Returns
        -------
        float
            New voltage after one dt step.
        """
        p = self.params

        # Energy harvested from source during dt (charging current through R_series)
        # I_charge = (V_source - V_cap) / R_series
        charge_current = (self.voltage_source - self.voltage) / p.R_series
        charge_current = max(charge_current, 0.0)  # can't push current back to source
        energy_harvested = self.voltage_source * charge_current * p.dt

        # Energy consumed by load during dt
        energy_consumed = self.voltage * load_current * p.dt

        # Net energy change
        net_energy = energy_harvested - energy_consumed

        # Update capacitor energy
        self.energy = max(self.energy + net_energy, 0.0)

        # Compute new voltage from energy: V = sqrt(2*E/C)
        self.voltage = math.sqrt(2.0 * self.energy / p.C)

        # Apply leakage decay
        self.voltage *= math.exp(-p.dt / (p.R_leakage * p.C))
        self.voltage = min(max(self.voltage, 0.0), self.voltage_max)
        # keep stored energy consistent if we clipped
        self.energy = 0.5 * p.C * self.voltage * self.voltage

        self._tick_common()
        return self.voltage

    # ---------------- SimPy process ----------------
    def run(self):
        while True:
            try:
                # coarse idle scheduler; actual updates happen via interrupts below
                yield self.env.timeout(10)
            except simpy.Interrupt as it:
                cause = it.cause
                if cause == "capacitor_charging":
                    self.capacitor_charging()
                elif cause == "capacitor_full":
                    self.capacitor_selfdischarge()
                elif cause == "capacitor_discharge":
                    self.load_discharging(self.current)
                elif cause == "capacitor_charge_and_discharge":
                    self.charge_and_discharge(self.current)

    # ----------------------------
    # convenience API
    # ----------------------------
    def charge_step(self, source_voltage: float):
        """One dt charging step (imperative helper; optionally call without interrupts).

        Parameters
        ----------
        source_voltage : float
            Voltage of the source to charge the capacitor from.

        Returns
        -------
        function
            Callable function that performs the charging step and returns the new voltage.
        """        
        self.voltage_source = source_voltage
        return self.capacitor_charging()

    def discharge_step(self, load_current: float):
        """One dt load step (imperative helper; optionally call without interrupts).

        Parameters
        ----------
        load_current : float
            Current to draw from the capacitor.

        Returns
        -------
        function
            Callable function that performs the discharging step and returns the new voltage.
        """        
        self.current = load_current
        return self.load_discharging(load_current)

    def leak_step(self):
        """One dt leakage-only step (imperative helper; optionally call without interrupts).

        Returns
        -------
        function
            Callable function that performs the leakage step and returns the new voltage.
        """
        return self.capacitor_selfdischarge()
