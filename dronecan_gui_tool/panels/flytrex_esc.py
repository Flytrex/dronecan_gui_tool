#
# Copyright (C) 2023  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Grisha Revzin
#
import datetime

import dronecan
from functools import partial
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QDialog, \
    QGridLayout, QPushButton, QComboBox, QHBoxLayout, QGroupBox, QCheckBox
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QSizePolicy
from logging import getLogger
from ..widgets import make_icon_button, get_icon


__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'Flytrex Propulsion Controller'

logger = getLogger(__name__)

_singleton = None


class _ReadinessLabel(QLabel):
    def __init__(self, parent):
        super(_ReadinessLabel, self).__init__(parent)
        self.set(False)
        self.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.setFixedSize(70, 18)

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

        self._last_index = -1
        self._last_direction = -1

        self._fpc_node = fpc_node
        self._dronecan_node = dronecan_node
        self.setDisabled(True)
        self.setTitle('FPC ' + str(self._fpc_node.node_id))

        # Motor Index
        self._index_selector = QComboBox()
        for a in ['DISABLED', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']:
            self._index_selector.addItem(a)
        self._index_selector.currentIndexChanged.connect(self._on_set_index)

        self._index_label = _ReadinessLabel(self)
        self._index_label.set(False)

        # Direction
        self._flip_checkbox = QCheckBox(self)
        self._flip_checkbox.clicked.connect(self._on_set_flip)
        self._flip_label = _ReadinessLabel(self)
        self._flip_label.set(False)

        # Ident
        self._ident_button = QPushButton('Ident', self)
        self._ident_button.clicked.connect(self._on_ident_clicked)

        # Data Age
        self._age_label = QLabel('Unknown')
        self._last_status = datetime.datetime.now()

        # Layout
        layout = QGridLayout()

        # Row 0
        layout.addWidget(QLabel("Motor Index"), 0, 0, 1, 1)
        layout.addWidget(self._index_selector, 0, 1, 1, 1)
        layout.addWidget(self._index_label, 0, 2, 1, 1)

        # Row 1
        layout.addWidget(QLabel("Reverse Rotation"), 1, 0, 1, 1)
        layout.addWidget(self._flip_checkbox, 1, 1, 1, 1)
        layout.addWidget(self._flip_label, 1, 2, 1, 1)

        # Row 2
        layout.addWidget(self._ident_button, 2, 0, 1, 3)

        # Row 3
        layout.addWidget(QLabel("Data Age"), 3, 0, 1, 1)
        layout.addWidget(self._age_label, 3, 1, 1, 1)

        self.setLayout(layout)

        self.default_stylesheet = self.styleSheet()
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self._handlers = [self._dronecan_node.add_handler(dronecan.uavcan.equipment.esc.Status,
                                                          self._on_status_message)]
        self._saved = True

        self.reset()
        self.fetch()
        self._update_state()

    def saved(self):
        return self._saved

    def _on_status_message(self, message):
        if message.transfer.source_node_id != self._fpc_node.node_id:
            pass
        else:
            self._last_status = datetime.datetime.now()
            pass  # TODO info display later

    def _on_set_index(self):
        self._write_index()

    def _on_set_flip(self):
        self._write_flipped()

    def _on_ident_clicked(self):
        request = dronecan.uavcan.protocol.AccessCommandShell.Request(input='ident')
        self._dronecan_node.request(request,
                                    self._fpc_node.node_id,
                                    self._on_ident_response,
                                    timeout=0.5)

    def _on_ident_response(self, e):
        pass

    def _read_index(self):
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.MOTOR_INDEX_PARAM)
        self._dronecan_node.request(request,
                                    self._fpc_node.node_id,
                                    self._on_index_read_response,
                                    timeout=0.5)

    def _on_index_read_response(self, e):
        if e is None:
            self._read_index()
        else:
            self._last_index = e.response.value.integer_value
            if not self.isEnabled():
                self._index_selector.blockSignals(True)
                self._index_selector.setCurrentIndex(self._last_index)
                self._index_selector.blockSignals(False)
                self._index_label.set(True)

        self.repaint()

    def _write_index(self):
        self._index_label.set(False)
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.MOTOR_INDEX_PARAM)
        request.value.integer_value = int(self._index_selector.currentIndex())
        self._dronecan_node.request(request,
                                    self._fpc_node.node_id,
                                    self._on_index_write_response,
                                    timeout=.5)
        self._index_selector.setEnabled(False)

    def _on_index_write_response(self, e):
        if e is None:
            self._write_index()
        else:
            self._saved = False
            self._index_selector.setEnabled(True)
            self._index_label.set(True)

    def _read_flipped(self):
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.REVERSE_PARAM)
        self._dronecan_node.request(request,
                                    self._fpc_node.node_id,
                                    self._on_flipped_read_response,
                                    timeout=0.5)

    def _on_flipped_read_response(self, e):
        if e is None:
            self._read_flipped()
        else:
            self._last_direction = e.response.value.boolean_value != 0
            if not self.isEnabled():
                self._flip_checkbox.blockSignals(True)
                self._flip_checkbox.setChecked(self._last_direction)
                self._flip_checkbox.blockSignals(False)
                self._flip_label.set(True)

    def _write_flipped(self):
        request = dronecan.uavcan.protocol.param.GetSet.Request(name=self.REVERSE_PARAM)
        request.value.boolean_value = bool(self._flip_checkbox.checkState())
        self._dronecan_node.request(request,
                                    self._fpc_node.node_id,
                                    self._on_flipped_write_response,
                                    timeout=.5)
        self._flip_label.set(False)
        self._flip_checkbox.setEnabled(False)

    def _on_flipped_write_response(self, e):
        if e is None:
            self._write_flipped()
        else:
            self._flip_label.set(True)
            self._saved = False
            self._flip_checkbox.setEnabled(True)

    def _update_state(self):
        if self._last_direction != -1 and self._last_index != -1:
            self.setDisabled(False)
        else:
            self.setDisabled(True)

        diff = datetime.datetime.now() - self._last_status
        millis = diff / datetime.timedelta(milliseconds=1)
        self._age_label.setText("{:2.3f}".format(millis / 1000))

        if diff > datetime.timedelta(seconds=1.5):
            self._age_label.setStyleSheet("font-weight: bold; color: red")
        else:
            self._age_label.setStyleSheet("font-weight: normal; color: black")

        QTimer.singleShot(500, self._update_state)

    def __del__(self):
        for h in self._handlers:
            h.remove()

    def closeEvent(self, event):
        super(_FPCWidget, self).closeEvent(event)
        self.__del__()

    def reset(self):
        self._last_index = -1
        self._last_direction = -1

    def fetch(self):
        self.setEnabled(False)
        self._read_index()
        self._read_flipped()

    def save(self):
        self.setEnabled(False)
        opcodes = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
        request = dronecan.uavcan.protocol.param.ExecuteOpcode.Request(opcode=opcodes.OPCODE_SAVE)
        self._dronecan_node.request(request,
                                    self._fpc_node.node_id,
                                    self._on_save_response,
                                    timeout=3.0)

    def _on_save_response(self, e):
        if e is None:
            self.save()
        else:
            self._saved = True
            self.setEnabled(True)


class FlytrexPropulsionControllerPanel(QDialog):
    COUNT_ROW = 4

    def __init__(self, parent, node):
        super(FlytrexPropulsionControllerPanel, self).__init__(parent)
        self.setWindowTitle('Flytrex Propulsion Controllers')
        self.setAttribute(Qt.WA_DeleteOnClose)  # This is required to stop background timers!

        self._node = node
        self._monitor = dronecan.app.node_monitor.NodeMonitor(node)

        self._widgets = dict()

        buttons_container = QGroupBox("Configuration", self)

        save_button = make_icon_button('database', 'Upload FPC configs', self,
                                       text='Store All', on_clicked=self._on_upload_clicked)
        fetch_button = make_icon_button('refresh', 'Download FPC configs', self,
                                        text='Fetch All', on_clicked=self._on_download_clicked)
        self._status_label = QLabel("Status")
        self._status_label.setAlignment(Qt.AlignCenter)

        buttons_layout = QHBoxLayout(buttons_container)
        buttons_layout.addWidget(save_button)
        buttons_layout.addWidget(fetch_button)
        buttons_layout.addWidget(self._status_label)
        buttons_layout.addStretch()

        self._widget_container = QGroupBox("FPC")
        self._widget_layout = QGridLayout()
        self._widget_container.setLayout(self._widget_layout)

        self._anon_warning = QLabel("Press 'Set Local Node ID' in the main window")
        self._anon_warning.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self._anon_warning.setStyleSheet('font-weight: bold; color: red')

        layout = QVBoxLayout(self)
        layout.addWidget(self._anon_warning)
        layout.addWidget(buttons_container)
        layout.addWidget(self._widget_container)
        self._update_data()

        # Smallest size policy for this widget
        self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)

    def _on_upload_clicked(self):
        for widget in self._widgets.values():
            widget.save()

    def _on_download_clicked(self):
        for widget in self._widgets.values():
            widget.fetch()

    def _update_data(self):
        QTimer.singleShot(500, self._update_data)

        if self._node.is_anonymous:
            self._anon_warning.setHidden(False)
            self.setDisabled(True)
            return
        else:
            self._anon_warning.setHidden(True)
            self._widget_container.setLayout(self._widget_layout)
            self.setEnabled(True)

        count = 0
        # Create a widget for each node once
        for node in self._monitor.find_all(lambda node_:
                                           True if node_.info and str(node_.info.name).startswith('com.flytrex.fpc')
                                           else False):
            count += 1
            if node.node_id not in self._widgets.keys():
                widget = _FPCWidget(self, dronecan_node=self._node, fpc_node=node)
                self._widgets[node.node_id] = widget
                self._widget_layout.addWidget(widget,
                                              (len(self._widgets) - 1) // self.COUNT_ROW,
                                              (len(self._widgets) - 1) % self.COUNT_ROW)

        config_pending = False
        for widget in self._widgets.values():
            if not widget.saved():
                config_pending = True

        if config_pending:
            self._status_label.setText('[UNSAVED]')
            self._status_label.setStyleSheet("font-weight: bold; color: red")
        else:
            self._status_label.setText('[SAVED]')
            self._status_label.setStyleSheet('font-weight: bold; color: green')

        self._widget_container.setTitle(f'Total {count} FPCs')

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
