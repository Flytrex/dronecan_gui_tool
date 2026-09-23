#
# Copyright (C) 2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import logging
import threading
import dronecan
from PyQt5.QtCore import pyqtSignal, QObject


logger = logging.getLogger(__name__)


class ActiveDataTypeDetector(QObject):
    message_types_updated = pyqtSignal([])
    service_types_updated = pyqtSignal([])

    def __init__(self, node):
        super(ActiveDataTypeDetector, self).__init__()
        self._node = node
        self._state_lock = threading.RLock()
        self._active_messages = set()
        self._active_services = set()
        self._hook_handle = node.add_transfer_hook(self._on_transfer)

    def close(self):
        self._hook_handle.remove()

    def reset(self):
        with self._state_lock:
            self._active_messages.clear()
            self._active_services.clear()

    def _on_transfer(self, tr):
        try:
            dtname = dronecan.get_dronecan_data_type(tr.payload).full_name
        except Exception:
            try:
                kind = dronecan.dsdl.CompoundType.KIND_SERVICE if tr.service_not_message else \
                    dronecan.dsdl.CompoundType.KIND_MESSAGE
                dtname = dronecan.DATATYPES[(tr.data_type_id, kind)].full_name
            except Exception:
                logger.error('Could not detect data type name from transfer %r', tr, exc_info=True)
                return

        if tr.service_not_message:
            with self._state_lock:
                if dtname in self._active_services:
                    return
                self._active_services.add(dtname)
                self.service_types_updated.emit()
        else:
            with self._state_lock:
                if dtname in self._active_messages:
                    return
                self._active_messages.add(dtname)
                self.message_types_updated.emit()

    def get_names_of_active_messages(self):
        with self._state_lock:
            return list(sorted(self._active_messages))

    def get_names_of_active_services(self):
        with self._state_lock:
            return list(sorted(self._active_services))

    @staticmethod
    def get_names_of_all_message_types_with_data_type_id():
        message_types = []
        for (dtid, kind), dtype in dronecan.DATATYPES.items():
            if dtid is not None and kind == dronecan.dsdl.CompoundType.KIND_MESSAGE:
                message_types.append(str(dtype))
        return list(sorted(message_types))
