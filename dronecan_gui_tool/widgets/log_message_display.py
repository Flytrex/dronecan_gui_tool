#
# Copyright (C) 2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import dronecan
import datetime
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QHeaderView, QPushButton, QLabel
from PyQt5.QtCore import Qt, QObject, pyqtSignal
from logging import getLogger
from . import BasicTable, RealtimeLogWidget


logger = getLogger(__name__)


class LogMessageController(QObject):
    message_received = pyqtSignal(object)

    def __init__(self, node, parent=None):
        super(LogMessageController, self).__init__(parent)
        self._updates_enabled = True
        self._subscriber = node.add_handler(dronecan.uavcan.protocol.debug.LogMessage, self._on_message)

    @property
    def updates_enabled(self):
        return self._updates_enabled

    def set_updates_enabled(self, enabled):
        self._updates_enabled = bool(enabled)

    def _on_message(self, event):
        if self._updates_enabled:
            self.message_received.emit(event)

    def close(self):
        self._subscriber.remove()


def log_level_to_color(level):
    return {
        level.DEBUG: None,
        level.INFO: Qt.green,
        level.WARNING: Qt.yellow,
        level.ERROR: Qt.red,
    }.get(level.value)


class LogMessageDisplayWidget(QGroupBox):
    COLUMNS = [
        BasicTable.Column('NID',
                          lambda e: e.transfer.source_node_id),
        BasicTable.Column('Local Time',
                          lambda e: datetime.datetime.fromtimestamp(e.transfer.ts_real)
                          .strftime('%H:%M:%S.%f')[:-3],
                          searchable=False),
        BasicTable.Column('Level',
                          lambda e: (dronecan.value_to_constant_name(e.message.level, 'value'),
                                     log_level_to_color(e.message.level))),
        BasicTable.Column('Source',
                          lambda e: e.message.source),
        BasicTable.Column('Text',
                          lambda e: e.message.text,
                          resize_mode=QHeaderView.Stretch),
    ]

    def __init__(self, parent, node):
        super(LogMessageDisplayWidget, self).__init__(parent)
        self.setTitle('Log messages (dronecan.uavcan.protocol.debug.LogMessage)')

        if isinstance(node, LogMessageController):
            self._controller = node
            self._owns_controller = False
        else:
            self._controller = LogMessageController(node, self)
            self._owns_controller = True

        self._log_widget = RealtimeLogWidget(self, columns=self.COLUMNS, multi_line_rows=True, started_by_default=True)
        self._log_widget.table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._log_widget.table.setWordWrap(True)
        self._controller.message_received.connect(self._log_widget.add_item_async)

        layout = QVBoxLayout(self)
        layout.addWidget(self._log_widget, 1)
        self.setLayout(layout)

    def close(self):
        if self._owns_controller:
            self._controller.close()
