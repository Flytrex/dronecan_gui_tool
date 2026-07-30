# Flytrex Delivery Controller helpers

from dataclasses import dataclass
import dronecan
import threading
from dronecan.transport import get_active_union_field
from typing import Optional, Any, Callable
from logging import getLogger
from enum import IntEnum
import math


logger = getLogger(__name__)


class DeliveryControllerMode(IntEnum):
    INITIAL                 = 0
    ALIGN_ENCODER           = 1
    HOMING                  = 2
    RESERVED1               = 3
    DIRECT_OVERRIDE         = 4
    HALT                    = 5
    GROUND_UNLOAD           = 6
    RESERVED2               = 7
    HOOK_STAGING            = 8
    LIFT_PACKAGE            = 9
    RESERVED3               = 10
    LANDING                 = 11
    PREPARE_FOR_DELIVERY    = 12
    DELIVERY                = 13
    RELEASE_WIRE            = 14


@dataclass
class DeliveryControllerCommand:
    mode : DeliveryControllerMode
    wire_extension_m : float = 0.0


class NodeParametersHelper:
    # Base copied over from the TCA (drone_hw_validator) code
    def __init__(self, node):
        self._node = node

    def request(self,
                message: Any,
                dest_node_id: int,
                callback: Optional[Callable[..., None]] = None,
                timeout: Optional[float] = None,
                **kwargs: Any) -> Any:

        node = self._node

        try:
            if callback:
                # Asynchronous request
                node.request(message, dest_node_id, callback, canfd=True, **kwargs)
                logger.debug("Sent async request: %s to node %s", message.__class__, dest_node_id)
                return None
            # Synchronous request
            # pylint: disable=assignment-from-no-return
            response = node.request(message, dest_node_id, timeout=timeout, canfd=True, **kwargs)
            logger.debug("Sent sync request: %s to node %s", message.__class__, dest_node_id)
            return response
        except Exception as ex:
            raise RuntimeError("Failed to send request") from ex

    @staticmethod
    def _make_param_value(param_value: Any):
        v = dronecan.uavcan.protocol.param.Value()  # pylint: disable=no-member
        if isinstance(param_value, bool):
            v.boolean_value = int(param_value)
        elif isinstance(param_value, int):
            v.integer_value = param_value
        elif isinstance(param_value, float):
            v.real_value = param_value
        return v

    @staticmethod
    def _extract_param_value(value) -> Any:
        field = get_active_union_field(value)
        raw = getattr(value, field)
        return {'integer_value': int, 'real_value': float, 'boolean_value': bool}.get(field, lambda x: None)(raw)

    def set_param(self, node_id: int, name: str, value: Any) -> None:
        req = dronecan.uavcan.protocol.param.GetSet.Request(  # pylint: disable=no-member
            name=name, value=self._make_param_value(value))
        response_event = threading.Event()
        def _callback(event: Any) -> None:
            if event:
                response_event.set()
        logger.info("Setting parameter %s to %s on node %d", name, value, node_id)
        self.request(req, node_id, callback=_callback, timeout=2.0)
        assert response_event.wait(timeout=3.0), f"No response to param set for {name}"

    def set_verify_param(self, node_id: int, name: str, value: Any) -> None:
        """Set a parameter and verify by reading it back."""
        self.set_param(node_id, name, value)
        read_back = self.get_param(node_id, name)
        if isinstance(read_back, float):
            assert math.isclose(read_back, value, rel_tol=1e-4), (
                f"{name} verification failed: expected {value}, got {read_back}"
            )
        else:
            assert read_back == value, (
                f"{name} verification failed: expected {value}, got {read_back}"
            )

    def get_param(self, node_id: int, name: str) -> Any:
        req = dronecan.uavcan.protocol.param.GetSet.Request(name=name)  # pylint: disable=no-member
        response_event = threading.Event()
        response_data = {}

        def _callback(event: Any) -> None:
            if event:
                response_data['value'] = self._extract_param_value(event.response.value)
            response_event.set()

        self.request(req, node_id, callback=_callback, timeout=2.0)
        assert response_event.wait(timeout=3.0), f"No response to param get for {name}"
        return response_data.get('value')

    def delcon_mode_command(self, node_id : int, cmd : DeliveryControllerCommand):
        threading.Thread(target=self._execute_thread, args=(node_id, cmd,)).start()

    def _execute_thread(self, node_id : int, cmd : DeliveryControllerCommand):
        self.set_verify_param(node_id, 'API_ARG_MODE_SELECT', cmd.mode.value)
        self.set_verify_param(node_id, 'API_ARG_WIRE_EXTENSION_M', cmd.wire_extension_m)
        latch = self.get_param(node_id, 'API_EXECUTE_LATCH')
        self.set_verify_param(node_id, 'API_EXECUTE_LATCH', latch + 1)
