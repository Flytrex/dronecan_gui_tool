#
# Copyright (C) 2023  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ilan Graidy
# Date:   2026-01-19
#
import datetime
import os
import time
from dataclasses import dataclass
from threading import Lock
from typing import Callable, List, Optional, Sequence

import dronecan
from functools import partial
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QDialog, \
    QGridLayout, QPushButton, QComboBox, QHBoxLayout, QGroupBox, QCheckBox, QFileDialog, QApplication, QMessageBox, \
    QSizePolicy
from PyQt5.QtCore import QTimer, Qt, QObject, QThread, pyqtSignal, pyqtSlot, QMetaObject
from logging import getLogger
from ..widgets import BasicTable, make_icon_button, get_icon, node_properties
from ..widgets.node_monitor import NodeTable

__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'BMS Auto Check'

logger = getLogger(__name__)

_singleton = None


@dataclass(frozen=True)
class BmsTest:
    idx: str
    name: str
    timeout_sec: float
    # Callback contract:
    # - Return True  -> test completed successfully
    # - Return False -> test failed
    # - Return None  -> test still running / not completed yet
    # Exceptions are treated as failure.
    callback: Callable[[], Optional[bool]]
    critical: bool


class BmsTestSuite:
    """Owns the BMS test definitions and tracks which test is currently running."""

    def __init__(self):
        self._tests: Sequence[BmsTest] = []
        self._current_index: int = 0
        self._current_started_at: Optional[float] = None

    def reset(self) -> None:
        """Start testing from the beginning (no current test running)."""
        self._current_index = 0
        self._current_started_at = None

    def set_tests(self, tests: Sequence[BmsTest]) -> None:
        self._tests = list(tests)
        self.reset()

    def get_test(self) -> Optional[BmsTest]:
        """Returns the currently running test (or the next test if not started yet)."""
        if self._current_index >= len(self._tests):
            return None
        return self._tests[self._current_index]

    def start_current(self) -> Optional[BmsTest]:
        test = self.get_test()
        if test is None:
            return None
        self._current_started_at = time.monotonic()
        return test

    def is_current_timed_out(self) -> bool:
        test = self.get_test()
        if test is None:
            return False
        if self._current_started_at is None:
            return False
        return (time.monotonic() - self._current_started_at) > float(test.timeout_sec)

    def advance(self) -> Optional[BmsTest]:
        """Mark current test as finished and move to the next test."""
        if self._current_index < len(self._tests):
            self._current_index += 1
        self._current_started_at = None
        return self.get_test()


class _BmsTestRunnerWorker(QObject):
    test_started = pyqtSignal(object)                 # BmsTest
    test_updated = pyqtSignal(object, str, str)       # BmsTest, status, elapsed_str
    run_finished = pyqtSignal(bool)                   # stopped

    def __init__(self, suite: BmsTestSuite, poll_interval_sec: float = 0.2):
        super().__init__()
        self._suite = suite
        self._poll_interval_sec = float(poll_interval_sec)
        self._stop_requested = False

    @pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True

    @pyqtSlot()
    def run(self) -> None:
        try:
            logger.info('BMS test runner thread started')
            while True:
                if self._stop_requested:
                    self.run_finished.emit(True)
                    return

                test = self._suite.get_test()
                if test is None:
                    self.run_finished.emit(False)
                    return

                if self._suite._current_started_at is None:
                    self._suite.start_current()
                    self.test_started.emit(test)

                elapsed = 0.0
                if self._suite._current_started_at is not None:
                    elapsed = max(0.0, time.monotonic() - float(self._suite._current_started_at))
                elapsed_str = f'{elapsed:.1f}s'

                if self._suite.is_current_timed_out():
                    self.test_updated.emit(test, 'Failed', elapsed_str)
                    self.run_finished.emit(False)
                    return

                self.test_updated.emit(test, 'Running', elapsed_str)

                try:
                    result = test.callback()
                except Exception:
                    self.test_updated.emit(test, 'Failed', elapsed_str)
                    self.run_finished.emit(False)
                    return

                if isinstance(result, bool):
                    if result is True:
                        logger.info('Test "%s" passed', test.name)
                        self.test_updated.emit(test, 'Pass', elapsed_str)
                        self._suite.advance()
                        continue
                    else:
                        logger.info('Test "%s" failed', test.name)
                        self.test_updated.emit(test, 'Failed', elapsed_str)
                        self.run_finished.emit(False)
                        return

                time.sleep(self._poll_interval_sec)
        except Exception:
            # Last-resort safety: never explode the QThread.
            self.run_finished.emit(False)
            return

class _BmsAutoCheckTests:
    def __init__(self, node, ui):
        self._node = node
        self._ui = ui
        self._manual_lock = Lock()
        # Manual test state (kept simple per-test; avoids generic key maps)
        self._batteries_toggle_asked: bool = False
        self._batteries_toggle_answer: Optional[bool] = None

    def _set_batteries_toggle_answer(self, answer: bool) -> None:
        with self._manual_lock:
            self._batteries_toggle_answer = bool(answer)

    def critical_tests(self):
        # Timeout defaults are placeholders; adjust per test as you implement them.
        return [
            BmsTest(idx='1',  name='Batteries turn on and off normally', timeout_sec=30.0, callback=self._test_critical_1_batteries_toggle, critical=True),
            BmsTest(idx='2',  name='BQ communications work',            timeout_sec=30.0, callback=self._test_critical_2_bq_comms,          critical=True),
            BmsTest(idx='3',  name='Firmware update works',             timeout_sec=90.0, callback=self._test_critical_3_firmware_update,  critical=True),
            BmsTest(idx='4a', name='Board identification works',        timeout_sec=30.0, callback=self._test_critical_4a_board_id,        critical=True),
            BmsTest(idx='4b', name='Battery identification works',      timeout_sec=30.0, callback=self._test_critical_4b_battery_id,      critical=True),
            BmsTest(idx='5',  name='DroneCAN Parameters Check',         timeout_sec=60.0, callback=self._test_critical_5_param_check,      critical=True),
            BmsTest(idx='6',  name='Backwards Compatibility Check',     timeout_sec=60.0, callback=self._test_critical_6_backcompat,       critical=True),
            BmsTest(idx='7',  name='Lifetime Charge Tracker Validation',timeout_sec=60.0, callback=self._test_critical_7_lifetime_tracker, critical=True),
        ]

    def noncritical_tests(self):
        return [
            BmsTest(idx='1',  name='Smart Charger',                        timeout_sec=60.0, callback=self._test_noncritical_1_smart_charger,        critical=False),
            BmsTest(idx='1a', name='Pre-0.8: Full Charge',                 timeout_sec=60.0, callback=self._test_noncritical_1a_pre08_full_charge,   critical=False),
            BmsTest(idx='1b', name='Post-0.8: Initial Charge Reset',       timeout_sec=60.0, callback=self._test_noncritical_1b_post08_initial_reset, critical=False),
            BmsTest(idx='1c', name='Post-0.8: Alternate Charging Control', timeout_sec=60.0, callback=self._test_noncritical_1c_post08_alt_control,   critical=False),
            BmsTest(idx='2',  name='ATP can be completed',                 timeout_sec=120.0, callback=self._test_noncritical_2_atp_complete,         critical=False),
            BmsTest(idx='3',  name='Voltage protection works',             timeout_sec=60.0, callback=self._test_noncritical_3_voltage_protection,    critical=False),
        ]

    def all_tests(self):
        return list(self.critical_tests()) + list(self.noncritical_tests())

    # --- Critical test callbacks (placeholders) ---
    def _test_critical_1_batteries_toggle(self):
        # Called repeatedly by the worker thread. Non-blocking:
        # - schedules a UI dialog once
        # - returns None until the user answers
        # - then returns True/False
        with self._manual_lock:
            if isinstance(self._batteries_toggle_answer, bool):
                return self._batteries_toggle_answer

            if not self._batteries_toggle_asked:
                self._batteries_toggle_asked = True
                self._ui.request_batteries_toggle_dialog.emit()

        return None

    def _test_critical_2_bq_comms(self):
        pass

    def _test_critical_3_firmware_update(self):
        pass

    def _test_critical_4a_board_id(self):
        pass

    def _test_critical_4b_battery_id(self):
        pass

    def _test_critical_5_param_check(self):
        pass

    def _test_critical_6_backcompat(self):
        pass

    def _test_critical_7_lifetime_tracker(self):
        pass

    # --- Non-critical test callbacks (placeholders) ---
    def _test_noncritical_1_smart_charger(self):
        pass

    def _test_noncritical_1a_pre08_full_charge(self):
        pass

    def _test_noncritical_1b_post08_initial_reset(self):
        pass

    def _test_noncritical_1c_post08_alt_control(self):
        pass

    def _test_noncritical_2_atp_complete(self):
        pass

    def _test_noncritical_3_voltage_protection(self):
        pass


class BmsNodeTable(NodeTable):
    @staticmethod
    def _name_contains_bms(entry) -> bool:
        if not getattr(entry, 'info', None) or not getattr(entry.info, 'name', None):
            return False

        name = entry.info.name
        if isinstance(name, (bytes, bytearray)):
            name = name.decode(errors='ignore')
        else:
            name = str(name)

        return 'bms' in name.lower()

    def _update(self):
        all_nodes = self._monitor.find_all(lambda _: True)
        known_nodes = {e.node_id: e for e in all_nodes if self._name_contains_bms(e)}

        displayed_nodes = set()
        rows_to_remove = []

        # Updating existing entries
        for row in range(self.rowCount()):
            nid = int(self.item(row, 0).text())
            displayed_nodes.add(nid)
            if nid not in known_nodes:
                rows_to_remove.append(row)
            else:
                self.set_row(row, known_nodes[nid])

        # Removing nonexistent entries
        for row in rows_to_remove[::-1]:     # It is important to traverse from end
            logger.info('Removing row %d', row)
            self.removeRow(row)

        # Adding new entries
        def find_insertion_pos_for_node_id(target_nid):
            for row in range(self.rowCount()):
                nid = int(self.item(row, 0).text())
                if nid > target_nid:
                    return row
            return self.rowCount()

        for nid in set(known_nodes.keys()) - displayed_nodes:
            row = find_insertion_pos_for_node_id(nid)
            logger.info('Adding new row %d for node %d', row, nid)
            self.insertRow(row)
            self.set_row(row, known_nodes[nid])

class BMSAutoCheckPanel(QDialog):

    request_batteries_toggle_dialog = pyqtSignal()

    def __init__(self, parent, node):
        super(BMSAutoCheckPanel, self).__init__(parent)
        self.setWindowTitle(PANEL_NAME)
        self.setAttribute(Qt.WA_DeleteOnClose)              # This is required to stop background timers!

        self._node = node
        self._tests = _BmsAutoCheckTests(node, self)

        self._suite = BmsTestSuite()
        self._suite.set_tests(self._tests.all_tests())

        self._runner_thread: Optional[QThread] = None
        self._runner_worker: Optional[_BmsTestRunnerWorker] = None
        self._is_running: bool = False

        layout = QVBoxLayout(self)

        self._test_group = QGroupBox('BMS Test', self)
        test_layout = QVBoxLayout(self._test_group)

        self._test_table_headline = QLabel('Critical Tests', self._test_group)
        test_layout.addWidget(self._test_table_headline)

        self._test_table = BasicTable(
            self._test_group,
            columns=[
                BasicTable.Column('#', lambda r: r.get('idx', '')),
                BasicTable.Column('Test Name', lambda r: r.get('name', '')),
                BasicTable.Column('Status', lambda r: r.get('status', '')),
                BasicTable.Column('Time', lambda r: r.get('time', '')),
            ],
        )
        self._test_table.setRowCount(0)
        self._test_table.setMinimumHeight(120)

        self._critical_tests: List[BmsTest] = list(self._tests.critical_tests())
        self._critical_row_by_id = {}
        self._critical_row_data: List[dict] = []
        self._test_table.setRowCount(len(self._critical_tests))
        for row, test in enumerate(self._critical_tests):
            self._critical_row_by_id[str(test.idx)] = row
            data = {'idx': test.idx, 'name': test.name, 'status': 'Pending', 'time': ''}
            self._critical_row_data.append(data)
            self._test_table.set_row(row, data)

        self._noncritical_table_headline = QLabel('Non-Critical Tests', self._test_group)

        self._noncritical_table = BasicTable(
            self._test_group,
            columns=[
                BasicTable.Column('#', lambda r: r.get('idx', '')),
                BasicTable.Column('Test Name', lambda r: r.get('name', '')),
                BasicTable.Column('Status', lambda r: r.get('status', '')),
                BasicTable.Column('Time', lambda r: r.get('time', '')),
            ],
        )
        self._noncritical_table.setRowCount(0)
        self._noncritical_table.setMinimumHeight(120)

        self._noncritical_tests: List[BmsTest] = list(self._tests.noncritical_tests())

        self._noncritical_row_by_id = {}
        self._noncritical_row_data: List[dict] = []
        self._noncritical_table.setRowCount(len(self._noncritical_tests))
        for row, test in enumerate(self._noncritical_tests):
            self._noncritical_row_by_id[str(test.idx)] = row
            data = {'idx': test.idx, 'name': test.name, 'status': 'Pending', 'time': ''}
            self._noncritical_row_data.append(data)
            self._noncritical_table.set_row(row, data)

        self._start_button = QPushButton('Start', self._test_group)
        self._start_button.clicked.connect(self._on_start_stop_clicked)

        test_layout.addWidget(self._test_table)
        test_layout.addWidget(self._noncritical_table_headline)
        test_layout.addWidget(self._noncritical_table)
        test_layout.addWidget(self._start_button)
        self._test_group.setLayout(test_layout)
        layout.addWidget(self._test_group)

        self._nodes_group = QGroupBox('Online nodes', self)
        self._nodes_group.setToolTip('Online nodes (double click for more options)')
        nodes_layout = QVBoxLayout(self._nodes_group)

        self._node_table = BmsNodeTable(self._nodes_group, node)
        self._status_label = QLabel(self._nodes_group)

        nodes_layout.addWidget(self._node_table)
        nodes_layout.addWidget(self._status_label)
        self._nodes_group.setLayout(nodes_layout)
        layout.addWidget(self._nodes_group)

        self.setLayout(layout)
        self.resize(800, 600)

        self._status_update_timer = QTimer(self)
        self._status_update_timer.setSingleShot(False)
        self._status_update_timer.timeout.connect(self._update_status)
        self._status_update_timer.start(500)

        self._monitor_handle = self._node_table.monitor.add_update_handler(lambda _: self._update_status())
        self._update_status()

        self.request_batteries_toggle_dialog.connect(self._show_batteries_toggle_dialog)

    @pyqtSlot()
    def _show_batteries_toggle_dialog(self) -> None:
        # If the run was stopped before we got scheduled, ignore.
        if not self._is_running:
            return

        result = QMessageBox.question(
            self,
            'BMS Auto Check',
            'Did the batteries turned on and off?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )

        self._tests._set_batteries_toggle_answer(result == QMessageBox.Yes)

    def _reset_test_tables(self) -> None:
        for row, test in enumerate(self._critical_tests):
            if row < len(self._critical_row_data):
                self._critical_row_data[row].update({'idx': test.idx, 'name': test.name, 'status': 'Pending', 'time': ''})
                self._test_table.set_row(row, self._critical_row_data[row])

        for row, test in enumerate(self._noncritical_tests):
            if row < len(self._noncritical_row_data):
                self._noncritical_row_data[row].update({'idx': test.idx, 'name': test.name, 'status': 'Pending', 'time': ''})
                self._noncritical_table.set_row(row, self._noncritical_row_data[row])

    def _set_test_row(self, test: BmsTest, status: str, elapsed_str: str) -> None:
        idx_key = str(test.idx)

        if test.critical:
            row = self._critical_row_by_id.get(idx_key)
            if row is None or row >= len(self._critical_row_data):
                return
            self._critical_row_data[row].update({'status': status, 'time': elapsed_str})
            self._test_table.set_row(row, self._critical_row_data[row])
        else:
            row = self._noncritical_row_by_id.get(idx_key)
            if row is None or row >= len(self._noncritical_row_data):
                return
            self._noncritical_row_data[row].update({'status': status, 'time': elapsed_str})
            self._noncritical_table.set_row(row, self._noncritical_row_data[row])

    def _start_runner(self) -> None:
        if self._is_running:
            return

        self._reset_test_tables()
        self._suite.reset()

        # Mark running before starting the worker thread so queued UI prompts aren't ignored.
        self._is_running = True
        self._start_button.setText('Stop')
        self._start_button.setEnabled(True)
        logger.info('Starting BMS test runner')

        self._runner_thread = QThread(self)
        self._runner_worker = _BmsTestRunnerWorker(self._suite)
        self._runner_worker.moveToThread(self._runner_thread)

        self._runner_thread.started.connect(self._runner_worker.run, Qt.QueuedConnection)
        self._runner_worker.test_started.connect(self._on_test_started)
        self._runner_worker.test_updated.connect(self._on_test_updated)
        self._runner_worker.run_finished.connect(self._on_run_finished)

        self._runner_thread.start()

    def _stop_runner(self) -> None:
        if not self._is_running:
            return

        if self._runner_worker is not None:
            QMetaObject.invokeMethod(self._runner_worker, 'request_stop', Qt.QueuedConnection)

    def _cleanup_runner(self) -> None:
        if self._runner_thread is not None:
            self._runner_thread.quit()
            self._runner_thread.wait(1500)
            self._runner_thread = None
        self._runner_worker = None
        self._is_running = False
        self._start_button.setText('Start')
        self._start_button.setEnabled(True)

    def _on_start_stop_clicked(self):
        if self._is_running:
            self._stop_runner()
        else:
            self._start_runner()

    @pyqtSlot(object)
    def _on_test_started(self, test: BmsTest) -> None:
        self._set_test_row(test, 'Running', '0.0s')

    @pyqtSlot(object, str, str)
    def _on_test_updated(self, test: BmsTest, status: str, elapsed_str: str) -> None:
        self._set_test_row(test, status, elapsed_str)

    @pyqtSlot(bool)
    def _on_run_finished(self, stopped: bool) -> None:
        # If stopped is False, the run completed or failed (worker already set Failed status on failure).
        self._cleanup_runner()

    def _update_status(self):
        if self._node.is_anonymous:
            self._status_label.setText('Discovery is not possible - local node is configured in anonymous mode')
        else:
            num_undiscovered = len(list(self._node_table.monitor.find_all(lambda e: not e.discovered)))
            if num_undiscovered > 0:
                self._status_label.setText('Node discovery is in progress, %d left...' % num_undiscovered)
            else:
                self._status_label.setText('All nodes are discovered')

    def __del__(self):
        global _singleton
        _singleton = None

    def closeEvent(self, event):
        global _singleton
        _singleton = None

        try:
            self._stop_runner()
        except Exception:
            pass

        try:
            if hasattr(self, '_monitor_handle') and self._monitor_handle is not None:
                self._monitor_handle.remove()
        except Exception:
            pass

        try:
            if hasattr(self, '_status_update_timer') and self._status_update_timer is not None:
                self._status_update_timer.stop()
        except Exception:
            pass

        try:
            if hasattr(self, '_node_table') and self._node_table is not None:
                self._node_table.close()
        except Exception:
            pass

        try:
            self._cleanup_runner()
        except Exception:
            pass

        super(BMSAutoCheckPanel, self).closeEvent(event)


def spawn(parent, node):
    global _singleton
    if _singleton is None:
        _singleton = BMSAutoCheckPanel(parent, node)

    _singleton.show()
    _singleton.raise_()
    _singleton.activateWindow()

    return _singleton


get_icon = partial(get_icon, 'fa6s.car-battery')