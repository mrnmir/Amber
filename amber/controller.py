# simulator/models/controller.py
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import random
import simpy

from .propagation import dbm_to_watts


# ----------------------------
# Config dataclasses
# ----------------------------
@dataclass(frozen=True)
class CurrentsA:
    listening: float
    sensing: float
    processing: float
    transmitting: float


@dataclass(frozen=True)
class DurationsMs:
    listening: int
    sensing: int
    processing: int
    transmitting: int


@dataclass(frozen=True)
class VoltageThresholdsV:
    high: float  # e.g., 1.7  (above this: go active)
    low: float  # e.g., 1.3  (below this: idle/off, charge)


@dataclass(frozen=True)
class ControllerParams:
    currents: CurrentsA
    durations_ms: DurationsMs
    thresholds_v: VoltageThresholdsV
    max_startup_time_ms: int = 2000


# ----------------------------
# Controller class
# ----------------------------
class Controller:
    """
    Orchestrates a node's duty cycle based on the capacitor's voltage:
      listening -> sensing -> processing -> transmitting -> (loop back to listening)
    
    Goes idle (off, charging) when voltage drops below low threshold.

    Expects:
    
      - capacitor_ctrl: has .action, .current (A), .voltage_source (V), .voltage (V),
                        .internal_time (s), and responds to interrupts:
                        'capacitor_discharge', 'capacitor_charging', 'capacitor_full'
      - backscatter: has .state, .controller_tx_ready(sensor_data), .do_transmit(tx_info)
    """

    def __init__(
        self,
        env: simpy.Environment,
        capacitor_ctrl,
        node,
        backscatter,
        params: ControllerParams,
        coverage_map=None,
        downlink_results=None,
        keep_logs: bool = True,  # if False, skip per-step diagnostic logging (flat memory)
    ):
        self.env = env
        self.id = node.id
        self.node = node

        self.capacitor_ctrl = capacitor_ctrl
        self.backscatter = backscatter
        self.coverage_map = coverage_map
        self.downlink_results = downlink_results
        self.p = params
        self.keep_logs = keep_logs

        # FSM state
        self.state_name = "listening"
        self.is_active = False

        # data path
        self.sensed_value = 0
        self.processed_data = 0
        self.data_history: List[Tuple[float, int]] = []

        # counters
        self.counter = 0  # generic step counter for states
        self.last_current = 0.0
        self.pending_tx_info = None  # stored tx info from backscatter

        # state start times (ms)
        self.listening_start_ms = 0.0
        self.sensing_start_ms = 0.0
        self.processing_start_ms = 0.0
        self.transmitting_start_ms = 0.0

        # startup delay (s)
        self.startup_delay_s = (
            random.randrange(0, max(1, self.p.max_startup_time_ms)) / 1000.0
        )

        # current logging buckets (for optional analytics)
        self.acc_current_listening: List[float] = []
        self.acc_current_sensing: List[float] = []
        self.acc_current_processing: List[float] = []
        self.acc_current_transmitting: List[float] = []

        self.fsm_count_in_current_cap_cycle = 0
        self.fsm_counts_per_cap_cycle = []  # list of counts, one per cap cycle

        self.action = env.process(self.run())

    # ----------------------------
    # FSM state handlers
    # ----------------------------
    def _enter_charge_and_discharge(self, i_amps: float):
        """Tell the capacitor to model a 1 ms step with simultaneous charging and load."""
        self.capacitor_ctrl.current = i_amps
        self.capacitor_ctrl.action.interrupt("capacitor_charge_and_discharge")

    def _is_registered(self) -> bool:
        """Check if node is registered with BS (can send data)."""
        if self.backscatter is None:
            return True  # No backscatter = always ready
        return self.backscatter.state == "registered"

    def working_mode(self):
        """
        Working mode FSM: called when voltage above high threshold and active.
        Handles the main duty cycle: listening -> sensing -> processing -> transmitting.
        """        
        c = self.p.currents
        d = self.p.durations_ms

        if self.state_name == "listening":
            # Listen for incoming commands/data.
            if self.counter == 0:
                self.listening_start_ms = float(self.env.now)

            self.counter += 1
            if self.keep_logs:
                self.acc_current_listening.append(c.listening)
            self._enter_charge_and_discharge(c.listening)
            if self.counter >= d.listening:
                # If registered: sense data. If not: wait for slot to send ID
                if self._is_registered():
                    self.state_name = "sensing"
                else:
                    self.state_name = "wait_slot"
                self.last_current = c.listening
                self.counter = 0

        elif self.state_name == "sensing":
            # Sense data from the environment (only when registered).
            if self.counter == 0:
                self.sensing_start_ms = float(self.env.now)

            self.counter += 1
            if self.keep_logs:
                self.acc_current_sensing.append(c.sensing)
            self._enter_charge_and_discharge(c.sensing)
            # produce a sensed value (stub)

            self.sensed_value = random.randint(100, 255)
            assert self.sensed_value != 0
            if self.counter >= d.sensing:
                self.state_name = "processing"
                self.last_current = c.sensing
                self.counter = 0

        elif self.state_name == "processing":
            # Process sensed data (only when registered).
            if self.counter == 0:
                self.processing_start_ms = float(self.env.now)

            self.counter += 1
            if self.keep_logs:
                self.acc_current_processing.append(c.processing)
            self._enter_charge_and_discharge(c.processing)
            if self.counter >= d.processing:
                self.state_name = "wait_slot"
                self.last_current = c.processing
                assert self.sensed_value != 0
                self.processed_data = self.sensed_value
                assert self.processed_data != 0
                self.counter = 0

        elif self.state_name == "wait_slot":
            # Waiting for backscatter slot - charge while waiting
            self.capacitor_ctrl.action.interrupt("capacitor_charging")

            # Just registered but no data yet — go sense/process first
            if self._is_registered() and self.processed_data == 0:
                self.state_name = "sensing"
                self.counter = 0
                return

            # Check if backscatter says we can transmit now
            if self.backscatter is not None:
                tx_info = self.backscatter.controller_tx_ready(self.processed_data)
                if tx_info is not None:
                    # Deliver at the exact validated in-slot instant (no 1-tick slip).
                    # controller_tx_ready() just confirmed `now` is inside the
                    # advertised RX slot, so the BS is listening right now. The
                    # `transmitting` state below only models the transmit energy
                    # draw over d.transmitting ms; the reception has already happened.
                    if self._is_registered() and self.keep_logs:
                        self.data_history.append(
                            (self.capacitor_ctrl.internal_time, self.processed_data)
                        )
                    self.backscatter.do_transmit(tx_info)
                    self.pending_tx_info = None
                    self.state_name = "transmitting"
                    self.counter = 0

        elif self.state_name == "transmitting":
            # Energy-draw phase only. The backscatter was already delivered at the
            # validated in-slot instant in wait_slot; here we just model the
            # transmit current over d.transmitting ms.
            if self.counter == 0:
                self.transmitting_start_ms = float(self.env.now)

            self.counter += 1
            if self.keep_logs:
                self.acc_current_transmitting.append(c.transmitting)
            self._enter_charge_and_discharge(c.transmitting)

            if self.counter >= d.transmitting:
                self.state_name = "listening"
                self.last_current = c.transmitting
                self.counter = 0
                self.fsm_count_in_current_cap_cycle += 1

    # ----------------------------
    # Main process loop
    # ----------------------------
    def run(self):
        thr = self.p.thresholds_v

        while True:
            self.node.state = self.state_name  # for external visibility

            # Recalculate harvesting power for this node every millisecond
            if self.coverage_map is not None and self.downlink_results is not None:
                self.coverage_map.calculate_node_power(
                    [self.node], self.downlink_results
                )
            # else wpt is tatic, no need to recalculate every ms

            # Convert power (dBm) to voltage: V = sqrt(P * R)
            power_w = dbm_to_watts(self.node.harvesting_power_dbm)
            R_series = getattr(self.capacitor_ctrl, "R_series", 5e3)
            self.capacitor_ctrl.voltage_source = (power_w * R_series) ** 0.5

            # handle startup delay: charge during startup delay before becoming active
            if self.capacitor_ctrl.internal_time <= self.startup_delay_s:
                self.capacitor_ctrl.action.interrupt("capacitor_charging")
            else:
                v = float(getattr(self.capacitor_ctrl, "voltage", 0.0))

                # transitions based on thresholds
                if (v >= thr.high) and (self.is_active is False):
                    # capacitor charged enough: go active
                    self.capacitor_ctrl.action.interrupt("capacitor_full")
                    self.state_name = "listening"
                    self.counter = 0
                    self.is_active = True

                elif self.is_active is False:
                    # still charging up (no controller current)
                    self.capacitor_ctrl.action.interrupt("capacitor_charging")
                    self.state_name = "listening"  # reset to default state while charging

                elif (v > thr.low) and (self.is_active is True):
                    # normal duty-cycling (charge_and_discharge handles both)
                    self.working_mode()

                elif (v <= thr.low) and (self.is_active is True):
                    # idle - voltage too low, turn off and charge
                    self.is_active = False
                    if self.keep_logs:
                        self.fsm_counts_per_cap_cycle.append(self.fsm_count_in_current_cap_cycle)
                    self.fsm_count_in_current_cap_cycle = 0
                    self.capacitor_ctrl.action.interrupt("capacitor_charging")

            # advance one millisecond
            yield self.env.timeout(1)
