#
# Copyright (C) 2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import dronecan
from PyQt5.QtWidgets import QGroupBox, QLabel, QSpinBox, QHBoxLayout, QCheckBox
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from logging import getLogger
from . import make_icon_button, flash

logger = getLogger(__name__)


NODE_ID_MIN = 1
NODE_ID_MAX = 127

def setup_filtering(node):
    '''setup to filter messages for low bandwidth'''
    driver = node.can_driver
    ids = []
    ids.append(0) # anonymous frames
    ids.append(dronecan.uavcan.protocol.NodeStatus.default_dtid)
    ids.append(dronecan.uavcan.protocol.GetNodeInfo.default_dtid)
    ids.append(dronecan.uavcan.protocol.RestartNode.default_dtid)
    ids.append(dronecan.uavcan.protocol.param.GetSet.default_dtid)
    ids.append(dronecan.uavcan.protocol.param.ExecuteOpcode.default_dtid)
    ids.append(dronecan.uavcan.protocol.file.BeginFirmwareUpdate.default_dtid)
    ids.append(dronecan.uavcan.protocol.file.Read.default_dtid)
    ids.append(dronecan.uavcan.protocol.file.GetInfo.default_dtid)
    ids.append(dronecan.uavcan.protocol.dynamic_node_id.Allocation.default_dtid)
    ids.append(dronecan.uavcan.protocol.debug.LogMessage.default_dtid)
    ids.append(dronecan.uavcan.tunnel.Targetted.default_dtid)
    ids.append(dronecan.dronecan.protocol.Stats.default_dtid)
    ids.append(dronecan.dronecan.protocol.CanStats.default_dtid)
    ids.append(dronecan.uavcan.equipment.gnss.Fix2.default_dtid)
    logger.info("Setup %u filter IDs" % len(ids))
    driver.set_filter_list(ids)


class LocalNodeController(QObject):
    state_changed = pyqtSignal(object)

    def __init__(self, node, parent=None):
        super(LocalNodeController, self).__init__(parent)
        self._node = node
        self._node_id_collector = dronecan.app.message_collector.MessageCollector(
            self._node,
            dronecan.uavcan.protocol.NodeStatus,
            timeout=dronecan.uavcan.protocol.NodeStatus().OFFLINE_TIMEOUT_MS * 1e-3)
        self._update_timer = QTimer(self)
        self._update_timer.setSingleShot(False)
        self._update_timer.timeout.connect(self.refresh)
        self._update_timer.start(500)

    @property
    def node(self):
        return self._node

    def close(self):
        self._update_timer.stop()
        self._node_id_collector.close()

    @pyqtSlot()
    def refresh(self):
        driver = self._node.can_driver
        bus_number = driver.get_bus()
        filter_list = driver.get_filter_list() if bus_number is not None else None
        self.state_changed.emit({
            'anonymous': self._node.is_anonymous,
            'node_id': self._node.node_id,
            'prohibited_node_ids': set(self._node_id_collector),
            'bus_number': bus_number,
            'filtering_enabled': bool(filter_list),
        })

    @pyqtSlot(int)
    def apply_node_id(self, node_id):
        if node_id > 0:
            self._node.node_id = node_id
            logger.info('Node ID: %s', self._node.node_id)
        self.refresh()

    @pyqtSlot(int)
    def set_bus(self, bus_number):
        self._node.can_driver.set_bus(bus_number)
        self.refresh()

    @pyqtSlot(bool)
    def set_filtering(self, enabled):
        if enabled:
            setup_filtering(self._node)
        else:
            self._node.can_driver.set_filter_list([])
        self.refresh()

    @pyqtSlot(bool)
    def set_canfd(self, enabled):
        self._node.set_canfd(enabled)
        self.refresh()

class LocalNodeWidget(QGroupBox):
    apply_node_id_requested = pyqtSignal(int)

    def __init__(self, parent, node):
        super(LocalNodeWidget, self).__init__(parent)
        self.setTitle('Local node properties')

        if isinstance(node, LocalNodeController):
            self._controller = node
            self._owns_controller = False
        else:
            self._controller = LocalNodeController(node, self)
            self._owns_controller = True
        self._current_state = {'anonymous': True, 'node_id': None, 'prohibited_node_ids': set()}

        self._node_id_label = QLabel('Set local node ID:', self)

        self._node_id_spinbox = QSpinBox(self)
        self._node_id_spinbox.setMaximum(NODE_ID_MAX)
        self._node_id_spinbox.setMinimum(NODE_ID_MIN)
        self._node_id_spinbox.setValue(NODE_ID_MAX)
        self._node_id_spinbox.valueChanged.connect(self._update)

        self._node_id_apply = make_icon_button('fa6s.hand', 'Apply local node ID', self, text='Set',
                                               on_clicked=self._on_node_id_apply_clicked)
        self._node_id_spinbox.valueChanged.connect(self._update)
        self.apply_node_id_requested.connect(self._controller.apply_node_id)
        self._controller.state_changed.connect(self._apply_state)

        layout = QHBoxLayout(self)
        layout.addWidget(self._node_id_label)
        layout.addWidget(self._node_id_spinbox)
        layout.addWidget(self._node_id_apply)

        layout.addStretch(1)

        self.setLayout(layout)

        flash(self, 'Some functions will be unavailable unless local node ID is set')
        self._controller.refresh()

    def close(self):
        if self._owns_controller:
            self._controller.close()

    def _apply_state(self, state):
        was_anonymous = self._current_state.get('anonymous', True)
        self._current_state = state

        if not state['anonymous']:
            self._node_id_spinbox.setEnabled(False)
            self._node_id_spinbox.blockSignals(True)
            self._node_id_spinbox.setValue(state['node_id'])
            self._node_id_spinbox.blockSignals(False)
            self._node_id_apply.hide()
            self._node_id_label.setText('Local node ID:')
            if was_anonymous:
                flash(self, 'Local node ID set to %d, all functions should be available now', state['node_id'])
        else:
            self._node_id_spinbox.setEnabled(True)
            self._node_id_apply.show()
            self._node_id_label.setText('Set local node ID:')
            self._update()

    def _update(self):
        if not self._current_state.get('anonymous', True):
            return

        prohibited_node_ids = set(self._current_state.get('prohibited_node_ids', set()))
        while True:
            nid = int(self._node_id_spinbox.value())
            if not (set(range(nid, NODE_ID_MAX + 1)) - prohibited_node_ids):
                self._node_id_spinbox.blockSignals(True)
                self._node_id_spinbox.setValue(nid - 1)
                self._node_id_spinbox.blockSignals(False)
            else:
                break

        if nid in prohibited_node_ids:
            if self._node_id_apply.isEnabled():
                self._node_id_apply.setEnabled(False)
                flash(self, 'Selected node ID is used by another node, try different one', duration=3)
        else:
            self._node_id_apply.setEnabled(True)

    def _on_node_id_apply_clicked(self):
        nid = int(self._node_id_spinbox.value())
        if nid > 0:
            self.apply_node_id_requested.emit(nid)

class AdapterSettingsWidget(QGroupBox):
    bus_requested = pyqtSignal(int)
    filtering_requested = pyqtSignal(bool)
    canfd_requested = pyqtSignal(bool)

    def __init__(self, parent, node):
        super(AdapterSettingsWidget, self).__init__(parent)
        self.setTitle('Adapter Settings')

        if isinstance(node, LocalNodeController):
            self._controller = node
            self._owns_controller = False
        else:
            self._controller = LocalNodeController(node, self)
            self._owns_controller = True
        self._current_state = {'bus_number': None, 'filtering_enabled': False}

        self._canfd_label = QLabel('Send CANFD:', self)
        self._canfd = QCheckBox(self)
        self._canfd.setTristate(False)
        self._canfd.stateChanged.connect(self.change_canfd)

        self._busnum_label = QLabel('Bus Number:', self)
        self._busnum = QSpinBox(self)
        self._busnum.setMaximum(2)
        self._busnum.setMinimum(1)
        self._busnum.valueChanged.connect(self.change_bus)

        self._filtering_label = QLabel('Low Bandwidth:', self)
        self._filtering = QCheckBox(self)
        self._filtering.setTristate(False)
        self._filtering.stateChanged.connect(self.change_filtering)

        self.bus_requested.connect(self._controller.set_bus)
        self.filtering_requested.connect(self._controller.set_filtering)
        self.canfd_requested.connect(self._controller.set_canfd)
        self._controller.state_changed.connect(self._apply_state)

        layout = QHBoxLayout(self)
        layout.addWidget(self._canfd_label)
        layout.addWidget(self._canfd)
        layout.addWidget(self._busnum_label)
        layout.addWidget(self._busnum)
        layout.addWidget(self._filtering_label)
        layout.addWidget(self._filtering)

        layout.addStretch(1)

        self.setLayout(layout)
        self._controller.refresh()

    def _apply_state(self, state):
        self._current_state = state
        bus_supported = state.get('bus_number') is not None

        self._busnum_label.setVisible(bus_supported)
        self._busnum.setVisible(bus_supported)
        self._filtering_label.setVisible(bus_supported)
        self._filtering.setVisible(bus_supported)

        if bus_supported:
            self._busnum.blockSignals(True)
            self._busnum.setValue(state['bus_number'])
            self._busnum.blockSignals(False)
            self._filtering.blockSignals(True)
            self._filtering.setChecked(state.get('filtering_enabled', False))
            self._filtering.blockSignals(False)

    def change_bus(self):
        '''update bus number'''
        self.bus_requested.emit(self._busnum.value())

    def change_filtering(self):
        '''update filtering'''
        self.filtering_requested.emit(self._filtering.isChecked())

    def change_canfd(self):
        '''change canfd settings'''
        self.canfd_requested.emit(self._canfd.isChecked())

    def close(self):
        if self._owns_controller:
            self._controller.close()
