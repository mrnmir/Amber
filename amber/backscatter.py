# amber/backscatter.py
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, Any, Tuple
import pandas as pd
import random
import simpy


@dataclass
class TxRecord:
    end_ms: float
    payload_type: str  # "id" or "data"
    payload: int
    slot_idx: int
    delivered: bool


@dataclass
class RxRecord:
    end_ms: float
    cmd: str
    from_bs: int


class BackscatterModule:
    """Backscatter communication module with protocol logic.

    Protocol:

    - Node receives "send_id" from BS → transmit ID in random slot
    - If ACK received → transition to registered state
    - On "send_data" → transmit sensor data (registered nodes only)

    Works with Controller - Controller handles energy/timing,
    BackscatterModule handles protocol.
    """    

    def __init__(
        self,
        env: simpy.Environment,
        node: Any,
        bs_processes: List[Any],
        uplink_results: dict = None,
        downlink_results: dict = None,
        # params: BackscatterParams = None,
        log_file: Optional[str] = None, 
        keep_logs: bool = True,  # if False, skip storing per-packet rx/tx records (flat memory)
    ):
        self.env = env
        self.node = node
        self.bs_processes = list(bs_processes) if bs_processes else []
        self.uplink_results = uplink_results or {}
        self.downlink_results = downlink_results or {}
        # self.params = params or BackscatterParams()
        self.log_file = log_file
        self.keep_logs = keep_logs

        self.state = "idle"  # idle, wait_ack, registered

        # RX slot timing (set by BS command)
        # List of (start_ms, end_ms) tuples for upcoming RX windows
        self.rx_slots: List[Tuple[float, float]] = []
        self.chosen_slot_idx: int = -1  # Index into rx_slots

        # Logs
        self.tx_records: List[TxRecord] = []
        self.rx_records: List[RxRecord] = []

        # Counters
        self.packets_sent: int = 0
        self.packets_delivered: int = 0
        self.acks_received: int = 0

        # Timestamp of last command that enables transmission
        self.last_tx_command_time: float = -1.0

    def receive_command(self, cmd: str, bs_id: int, data: dict = None):
        """Called by BSEngine when BS transmits a command to this node.

        Commands:
        - "send_id": BS requests all nodes to send their IDs. Node picks a random
          RX slot from the provided list and transitions to wait_ack state.
        - "ack": BS acknowledges receipt of this node's ID. Node transitions to
          registered state and can now send data.
        - "send_data": BS requests sensor data from registered nodes.

        Args:
            cmd: Command string ("send_id", "ack", "send_data")
            bs_id: ID of the base station sending the command
            data: Optional dict with rx_slots list of (start_ms, end_ms) tuples

        Parameters
        ----------
        cmd : str
            Command string ("send_id", "ack", "send_data")
        bs_id : int
            ID of the base station sending the command
        data : dict, optional
            Optional dict with rx_slots list of (start_ms, end_ms) tuples, by default None
        """
        data = data or {}

        # Check if received power is above node sensitivity
        node_sensitivity = getattr(self.node, "sensitivity_dbm", -45.0)
        best_pw_dbm = self.downlink_results.get("best_pw_dbm", {})
        received_power = best_pw_dbm.get(self.node.id, -999.0)
        if received_power < node_sensitivity:
            # Signal too weak, ignore command
            return

        # Log
        if self.keep_logs:
            self.rx_records.append(
                RxRecord(
                    end_ms=self.env.now,
                    cmd=cmd,
                    from_bs=bs_id,
                )
            )

        # Store RX slots from BS
        self.rx_slots = data.get("rx_slots", [])

        # Protocol logic: how this node reacts to the command. Everything above
        # is framework-level (sensitivity gate, logging, slot bookkeeping) and
        # applies to every protocol. Override handle_command() in a subclass to
        # design your own protocol.
        self.handle_command(cmd, bs_id, data)

    def handle_command(self, cmd: str, bs_id: int, data: dict):
        """Decide how this node reacts to a base-station command.

        Called by receive_command() after the framework has applied the
        sensitivity gate, logged the reception, and stored data["rx_slots"]
        into self.rx_slots. Override in a subclass to implement your own
        protocol; you can define arbitrary command strings on the BS side and
        dispatch on them here. The default implementation is the built-in
        registration + polling protocol (send_id / ack / send_data).

        Parameters
        ----------
        cmd : str
            Command string sent by the base station.
        bs_id : int
            ID of the base station that issued the command.
        data : dict
            Command payload (self.rx_slots has already been set from it).
        """
        if cmd == "send_id":
            # BS wants nodes to send their IDs - pick random RX slot
            # Random slot reduces collisions during discovery phase
            # Ignore if already registered
            if self.state == "registered":
                # Clear slot and rx_slots to prevent accidental transmission
                self.chosen_slot_idx = -1
                self.rx_slots = []
                return
            if self.rx_slots:
                self.chosen_slot_idx = random.randint(0, len(self.rx_slots) - 1)
            else:
                self.chosen_slot_idx = -1
            self.state = "wait_ack"
            self.last_tx_command_time = self.env.now

        elif cmd == "ack":
            # BS acknowledged our ID
            if self.state == "wait_ack":
                self.state = "registered"
                self.acks_received += 1

        elif cmd == "send_data":
            # BS wants sensor data - use first available RX slot
            # Registered nodes respond immediately, no need to randomize
            target = data.get("target", -1)
            if target != -1 and target != self.node.id:
                return  # not for me
            if self.state == "registered":
                # self.chosen_slot_idx = 0 if self.rx_slots else -1 
                if self.rx_slots:
                    if target == self.node.id:
                        self.chosen_slot_idx = len(self.rx_slots) - 1  # Unicast: pick earliest slot deterministically
                    else:
                        self.chosen_slot_idx = random.randint(0, len(self.rx_slots) - 1)  # Broadcast: random
                else:
                    self.chosen_slot_idx = -1
                self.last_tx_command_time = self.env.now

    def controller_tx_ready(self, sensor_data: int) -> Optional[dict]:
        """Called by Controller when it enters TX state.
        Returns what to transmit, or None if not ready.

        Checks:
        
        - Node must have received a recent TX command
        - Current time must be within the chosen RX slot window
        - Slot must have been assigned after last command (not stale)

        Parameters
        ----------
        sensor_data : int
            Sensor data to transmit if in registered state

        Returns
        -------
        Optional[dict]
            Dict with "type" ("id" or "data") and "payload", or None if not ready to transmit
        """        
        # Must have received a TX command recently
        if self.last_tx_command_time < 0:
            return None

        # Must have a valid chosen slot
        if self.chosen_slot_idx < 0 or self.chosen_slot_idx >= len(self.rx_slots):
            return None

        # Check if current time is within chosen RX slot
        slot_start, slot_end = self.rx_slots[self.chosen_slot_idx]
        now = self.env.now
        # Reject now == slot_end: the BS _do_rx window uses `while now < end_time`,
        # so at exactly slot_end it has already closed and would miss the packet.
        if now < slot_start or now >= slot_end:
            return None  # Not in our slot yet (or past it)

        # Verify slot was assigned after the command (not stale from previous cycle)
        if slot_start < self.last_tx_command_time:
            return None  # Stale slot from before the command

        return self.build_tx_payload(sensor_data)

    def build_tx_payload(self, sensor_data: int) -> Optional[dict]:
        """Decide what this node transmits in its slot.

        Called by controller_tx_ready() once the framework has confirmed the
        node is inside a valid transmit slot. Override in a subclass to change
        the payload. The default sends the node ID while waiting for an ack,
        and sensor data once registered.

        Parameters
        ----------
        sensor_data : int
            Sensor value available to transmit if registered.

        Returns
        -------
        Optional[dict]
            Dict with "type" and "payload", or None if nothing to send.
        """
        if self.state == "wait_ack":
            # Send our ID
            return {"type": "id", "payload": self.node.id}

        elif self.state == "registered":
            # Send sensor data
            return {"type": "data", "payload": sensor_data}

        return None

    def do_transmit(self, tx_info: dict) -> bool:
        """Perform the actual transmission to BS.
        Parameters
        ----------
        tx_info : dict
            Dict with "type" ("id" or "data") and "payload"

        Returns
        -------
        bool
            True if transmission was attempted, False if not ready to transmit
        """        

        payload_type = tx_info["type"]
        payload = tx_info["payload"]

        # Use pre-computed uplink results
        per_sector = self.uplink_results.get("per_sector_powers", {})

        delivered = False
        for bs in self.bs_processes:
            # Find best sector for this BS where this node is visible
            best_sector_name = None
            best_rssi_dbm = -999.0

            for sector_name, node_powers in per_sector.items():
                 if sector_name.startswith(f"BS{bs.id}_"):
                    if self.node.id in node_powers:
                        rssi = node_powers[self.node.id]
                        if rssi > best_rssi_dbm:
                            best_rssi_dbm = rssi
                            best_sector_name = sector_name

            # Skip this BS if node not visible to any of its sectors
            if best_sector_name is None:
                continue

            # Set scratch and interrupt
            bs.received_tag_id = self.node.id
            bs.data_from_tag = payload
            bs.rssi_dbm = best_rssi_dbm
            bs.best_sector_idx = int(best_sector_name.split("_S")[1])
            bs.subcarrier_shift = getattr(self.node, "subcarrier_shift", 0)

            bs.action.interrupt("receive_data")
            delivered = True

        # Log
        if self.keep_logs:
            self.tx_records.append(
                TxRecord(
                    end_ms=self.env.now,
                    payload_type=payload_type,
                    payload=payload,
                    slot_idx=self.chosen_slot_idx,
                    delivered=delivered,
                )
            )

        self.packets_sent += 1
        if delivered:
            self.packets_delivered += 1

        # Clear state after transmission to prevent re-transmission
        # Node must receive a new command to transmit again
        self.chosen_slot_idx = -1
        self.rx_slots = []
        self.last_tx_command_time = -1.0

        return delivered

    def save_logs(self, clear: bool = True):
        """Save logs to file using pandas and optionally clear lists.

        Args:
            clear: If True, clear the lists after saving to free memory.
        """
        if not self.log_file:
            return

        # Save TX records
        if self.tx_records:
            tx_data = [
                {
                    "end_ms": r.end_ms,
                    "payload_type": r.payload_type,
                    "payload": r.payload,
                    "slot_idx": r.slot_idx,
                    "delivered": r.delivered,
                }
                for r in self.tx_records
            ]
            df_tx = pd.DataFrame(tx_data)
            df_tx.to_csv(f"node_{self.log_file}_tx.txt", index=False, sep="\t")

        # Save RX records
        if self.rx_records:
            rx_data = [
                {
                    "end_ms": r.end_ms,
                    "cmd": r.cmd,
                    "from_bs": r.from_bs,
                }
                for r in self.rx_records
            ]
            df_rx = pd.DataFrame(rx_data)
            df_rx.to_csv(f"node_{self.log_file}_rx.txt", index=False, sep="\t")

        # Clear lists to free memory
        if clear:
            self.tx_records.clear()
            self.rx_records.clear()
