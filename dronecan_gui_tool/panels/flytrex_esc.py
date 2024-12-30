#
# Copyright (C) 2023  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Grisha Revzin
#
import datetime
import time

import dronecan
from functools import partial
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QLabel, QDialog, \
    QGridLayout, QPushButton, QLineEdit, QFileDialog, QComboBox, QHBoxLayout, QSpinBox, QGroupBox, QCheckBox
from PyQt5.QtCore import QTimer, Qt
from logging import getLogger
from ..widgets import make_icon_button, get_icon, get_monospace_font
from ..widgets import table_display
import random
import base64
import struct


__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'Flytrex Propulsion Controller'

logger = getLogger(__name__)

_singleton = None


class _ReadinessLabel(QLabel):
    def __init__(self, parent):
        super(_ReadinessLabel, self).__init__(parent)
        self.set(False)

    def set(self, value):
        if value:
            self.setText('OK')
            self.setStyleSheet('background-color: lightgreen')
        else:
            self.setText('PENDING')
            self.setStyleSheet('background-color: yellow')


class _FPCWidget(QGroupBox):
    MOTOR_INDEX_PARAM = 'MOTOR_INDEX'
    REVERSE_PARAM = 'REVERSE_DIRECTION'

    def __init__(self, parent, fpc_node, dronecan_node):
        super(_FPCWidget, self).__init__(parent)
        self._fpc_node = fpc_node
        self._dronecan_node = dronecan_node
        self.setDisabled(True)
        self.setTitle('FPC ' + str(self._fpc_node.node_id))

        # Motor Index
        self.index_selector = QComboBox()
        for a in ['DISABLED', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']:
            self.index_selector.addItem(a)
        self.index_selector.currentIndexChanged.connect(self._on_set_index)
        self._last_index = -1

        self.index_label = _ReadinessLabel(self)
        self.index_label.set(False)
        self._read_index()

        # Direction
        self.flip_checkbox = QCheckBox(self)
        self.flip_checkbox.clicked.connect(self._on_set_flip)
        self.flip_label = _ReadinessLabel(self)
        self.flip_label.set(False)
        self._last_direction = -1
        self._read_flipped()

        # Ident
        self.ident_button = QPushButton('Ident', self)
        self.ident_button.clicked.connect(self._on_ident_clicked)

        # Layout
        layout = QGridLayout()
        # Row 0
        layout.addWidget(QLabel("Motor Index"), 0, 0, 1, 1)
        layout.addWidget(self.index_selector, 0, 1, 1, 1)
        layout.addWidget(self.index_label, 0, 2, 1, 1)
        # Row 1
        layout.addWidget(QLabel("Reverse Rotation"), 1, 0, 1, 1)
        layout.addWidget(self.flip_checkbox, 1, 1, 1, 1)
        layout.addWidget(self.flip_label, 1, 2, 1, 1)
        # Row 2
        layout.addWidget(self.ident_button, 2, 0, 1, 3)

        self.setLayout(layout)

        self.default_stylesheet = self.styleSheet()

        QTimer.singleShot(500, self._update_state)

        self._handlers = [self._dronecan_node.add_handler(dronecan.uavcan.equipment.esc.Status,
                                                          self._on_status_message)]

    def _on_status_message(self, message):
        if message.transfer.source_node_id != self._fpc_node.node_id:
            pass
        else:
            pass # TODO info display later

    def _on_set_index(self):
        self._write_index()

    def _on_set_flip(self):
        self._write_flipped()

    def _on_ident_clicked(self):
        request = dronecan.uavcan.equipment.esc.Ident()
        self._dronecan_node.defer(0.1, lambda: self._dronecan_node.request(request,
                                                                           self._fpc_node.node_id,
                                                                           self._on_ident_response,
                                                                           timeout=0.5))

    def _on_ident_response(self, e):
        pass

    def _read_index(self):
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.MOTOR_INDEX_PARAM)
        self._dronecan_node.defer(0.1, lambda: self._dronecan_node.request(request,
                                                                           self._fpc_node.node_id,
                                                                           self._on_index_read_response,
                                                                           timeout=0.5))

    def _on_index_read_response(self, e):
        if e is None:
            self._read_index()
        else:
            self._last_index = e.response.value.integer_value
            if not self.isEnabled():
                self.index_selector.setCurrentIndex(self._last_index)

    def _write_index(self):
        self.index_label.set(False)
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.MOTOR_INDEX_PARAM)
        request.value.integer_value = int(self.index_selector.currentIndex())
        self._dronecan_node.defer(0.1, lambda: self._dronecan_node.request(request,
                                                                           self._fpc_node.node_id,
                                                                           self._on_index_write_response,
                                                                           timeout=3))

    def _on_index_write_response(self, e):
        if e is None:
            self._write_index()
        else:
            self.index_label.set(True)

    def _read_flipped(self):
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.REVERSE_PARAM)
        self._dronecan_node.defer(0.1, lambda: self._dronecan_node.request(request,
                                                                           self._fpc_node.node_id,
                                                                           self._on_flipped_read_response,
                                                                           timeout=0.5))

    def _on_flipped_read_response(self, e):
        if e is None:
            self._read_flipped()
        else:
            self._last_direction = e.response.value.boolean_value != 0
            if not self.isEnabled():
                self.flip_checkbox.setChecked(self._last_direction)

    def _write_flipped(self):
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.REVERSE_PARAM)
        request.value.boolean_value = bool(self.flip_checkbox.checkState())
        self._dronecan_node.defer(0.1, lambda: self._dronecan_node.request(request,
                                                                           self._fpc_node.node_id,
                                                                           self._on_flipped_write_response,
                                                                           timeout=3))
        self.flip_label.set(False)

    def _on_flipped_write_response(self, e):
        if e is None:
            self._write_flipped()
        else:
            self.flip_label.set(True)

    def _update_state(self):
        if self._last_direction != -1 and self._last_index != -1:
            self.setDisabled(False)
            self.index_label.set(True)
            self.flip_label.set(True)

        QTimer.singleShot(500, self._update_state)

    def __del__(self):
        for h in self._handlers:
            h.remove()

    def closeEvent(self, event):
        super(_FPCWidget, self).closeEvent(event)
        self.__del__()


class FlytrexPropulsionControllerPanel(QDialog):
    DEFAULT_INTERVAL = 0.1

    def __init__(self, parent, node):
        super(FlytrexPropulsionControllerPanel, self).__init__(parent)
        self.setWindowTitle('Flytrex Propulsion Controllers')
        self.setAttribute(Qt.WA_DeleteOnClose)  # This is required to stop background timers!

        self._node = node
        self._monitor = dronecan.app.node_monitor.NodeMonitor(node)

        self._widgets = dict()

        self._container = QGroupBox("FPC")
        layout = QVBoxLayout(self)
        layout.addWidget(self._container)

        self._layout = QGridLayout()
        self._container.setLayout(self._layout)

        QTimer.singleShot(500, self._update_data)

    def _update_data(self):
        if self._node.is_anonymous:
            self.setDisabled(True)
        else:
            self.setEnabled(True)

        COUNT_ROW = 4

        QTimer.singleShot(500, self._update_data)

        count = 0
        # Create a widget for each node
        for node in self._monitor.find_all(lambda node_:
                                           True if node_.info and str(node_.info.name).startswith('com.flytrex.fpc')
                                           else False):
            count += 1
            if node.node_id not in self._widgets.keys():
                widget = _FPCWidget(self, dronecan_node=self._node, fpc_node=node)
                self._widgets[node.node_id] = widget
                self._layout.addWidget(widget,
                                       len(self._widgets) // COUNT_ROW,
                                       len(self._widgets) % COUNT_ROW)

        self._container.setTitle(f'Total {count} FPCs')

    def __del__(self):
        global _singleton
        _singleton = None

    def closeEvent(self, event):
        super(FlytrexPropulsionControllerPanel, self).closeEvent(event)
        self.__del__()


def spawn(parent, node):
    global _singleton
    if _singleton is None:
        try:
            _singleton = FlytrexPropulsionControllerPanel(parent, node)
        except Exception as ex:
            print(ex)

    _singleton.show()
    _singleton.raise_()
    _singleton.activateWindow()

    return _singleton

get_icon = partial(get_icon, 'asterisk')
