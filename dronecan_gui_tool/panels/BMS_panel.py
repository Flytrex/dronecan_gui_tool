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
from dataclasses import dataclass, replace
from threading import Lock
from typing import Callable, List, Optional, Sequence

import dronecan
from functools import partial
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QDialog, \
    QGridLayout, QPushButton, QComboBox, QHBoxLayout, QGroupBox, QCheckBox, QFileDialog, QApplication, QMessageBox, \
    QSizePolicy, QMenu, QAction, QLineEdit, QDialogButtonBox
from PyQt5.QtCore import QTimer, Qt, QObject, QThread, pyqtSignal, pyqtSlot, QMetaObject
from logging import getLogger
from ..widgets import BasicTable, make_icon_button, get_icon, node_properties
from ..widgets.node_monitor import NodeTable
from ..AutoTests.firmware_update_test import FirmwareUpdateTestDialog

__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'BMS Auto Check'

logger = getLogger(__name__)

_singleton = None


@dataclass(frozen=True)
class BmsTest:
    """
    @brief          Immutable descriptor for a single BMS test.
    @param[in]      idx             Short identifier (e.g. '1', '4a').
    @param[in]      name            Human-readable test name.
    @param[in]      timeout_sec     Maximum allowed time for the test.
    @param[in]      callback        Callable that drives the test:
                                    - Return True  -> test completed successfully
                                    - Return False -> test failed
                                    - Return None  -> test still running / not completed yet
                                    Exceptions are treated as failure.
    @param[in]      critical        Whether the test is critical.
    """
    idx: str
    name: str
    timeout_sec: float
    callback: Callable[[], Optional[bool]]
    critical: bool


class BmsTestSuite:
    """
    @brief          Owns the BMS test definitions and tracks which test
                    is currently running.
    """

    def __init__(self):
        """
        @brief          Initialise with an empty test list.
        """
        self._tests: Sequence[BmsTest] = []
        self._current_index: int = 0
        self._current_started_at: Optional[float] = None

    def reset(self) -> None:
        """
        @brief          Start testing from the beginning (no current test running).
        """
        self._current_index = 0
        self._current_started_at = None

    def set_tests(self, tests: Sequence[BmsTest]) -> None:
        """
        @brief          Replace the test list and reset to the beginning.
        @param[in]      tests       Sequence of BmsTest to run.
        """
        self._tests = list(tests)
        self.reset()

    def get_test(self) -> Optional[BmsTest]:
        """
        @brief          Get the currently running test (or the next test
                        if not started yet).
        @return         The current BmsTest, or None if all tests finished.
        """
        if self._current_index >= len(self._tests):
            return None
        return self._tests[self._current_index]

    def start_current(self) -> Optional[BmsTest]:
        """
        @brief          Mark the current test as started and record the
                        start time.
        @return         The started BmsTest, or None if no test to start.
        """
        test = self.get_test()
        if test is None:
            return None
        self._current_started_at = time.monotonic()
        return test

    def is_current_timed_out(self) -> bool:
        """
        @brief          Check whether the current test has exceeded its
                        timeout.
        @return         True if timed out, False otherwise.
        """
        test = self.get_test()
        if test is None:
            return False
        if self._current_started_at is None:
            return False
        return (time.monotonic() - self._current_started_at) > float(test.timeout_sec)

    def advance(self) -> Optional[BmsTest]:
        """
        @brief          Mark current test as finished and move to the
                        next test.
        @return         The next BmsTest, or None if all tests finished.
        """
        if self._current_index < len(self._tests):
            self._current_index += 1
        self._current_started_at = None
        return self.get_test()


class _BmsTestRunnerWorker(QObject):
    """
    @brief          Worker that runs on a QThread, polling the test suite
                    and emitting progress signals.
    """
    test_started = pyqtSignal(object)                 # BmsTest
    test_updated = pyqtSignal(object, str, str)       # BmsTest, status, elapsed_str
    run_finished = pyqtSignal(bool)                   # stopped

    def __init__(self, suite: BmsTestSuite, poll_interval_sec: float = 0.2):
        """
        @brief          Initialise the worker.
        @param[in]      suite               BmsTestSuite to run.
        @param[in]      poll_interval_sec   Seconds between callback polls.
        """
        super().__init__()
        self._suite = suite
        self._poll_interval_sec = float(poll_interval_sec)
        self._stop_requested = False

    @pyqtSlot()
    def request_stop(self) -> None:
        """
        @brief          Request the worker to stop after the current poll.
        """
        self._stop_requested = True

    @pyqtSlot()
    def run(self) -> None:
        """
        @brief          Main loop executed on the worker thread. Polls each
                        test callback in sequence, emitting signals for
                        progress and completion.
        """
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
    """
    @brief          Holds all BMS test definitions and manual-test state.
    """
    def __init__(self, node, ui):
        """
        @brief          Initialise test definitions and manual-test state.
        @param[in]      node    Local DroneCAN node instance.
        @param[in]      ui      The BMSAutoCheckPanel (used for emitting
                                UI prompt signals).
        """
        self._node = node
        self._ui = ui
        self._manual_lock = Lock()
        # Manual test state (kept simple per-test; avoids generic key maps)
        self._batteries_toggle_asked: bool = False
        self._batteries_toggle_answer: Optional[bool] = None
        self._firmware_update_asked: bool = False
        self._firmware_update_result: Optional[bool] = None
        self._firmware_update_repeat: int = 1
        self._firmware_update_dialog: Optional[FirmwareUpdateTestDialog] = None
        self._firmware_update_continue_on_failure: bool = False

    def _set_batteries_toggle_answer(self, answer: bool) -> None:
        """
        @brief          Record the user's answer to the batteries-toggle prompt.
        @param[in]      answer      True if batteries toggled normally.
        """
        with self._manual_lock:
            self._batteries_toggle_answer = bool(answer)

    def _set_firmware_update_result(self, result: bool) -> None:
        """
        @brief          Record the firmware update test result.
        @param[in]      result      True if the FW update test passed.
        """
        with self._manual_lock:
            self._firmware_update_result = bool(result)

    def reset(self) -> None:
        """
        @brief          Reset all manual test state for a fresh run.
        """
        with self._manual_lock:
            self._batteries_toggle_asked = False
            self._batteries_toggle_answer = None
            self._firmware_update_asked = False
            self._firmware_update_result = None
            self._firmware_update_repeat = 1
            self._firmware_update_dialog = None
            self._firmware_update_continue_on_failure = False

    def critical_tests(self):
        """
        @brief          Return the list of critical BmsTest definitions.
        @return         List of BmsTest with critical=True.
        """
        # Timeout defaults are placeholders; adjust per test as you implement them.
        return [
            BmsTest(idx='1',  name='Batteries turn on and off normally', timeout_sec=30.0, callback=self._test_critical_1_batteries_toggle, critical=True),
            BmsTest(idx='2',  name='BQ communications work',            timeout_sec=30.0, callback=self._test_critical_2_bq_comms,          critical=True),
            BmsTest(idx='3',  name='Firmware update works',             timeout_sec=3600.0, callback=self._test_critical_3_firmware_update,  critical=True),
            BmsTest(idx='4a', name='Board identification works',        timeout_sec=30.0, callback=self._test_critical_4a_board_id,        critical=True),
            BmsTest(idx='4b', name='Battery identification works',      timeout_sec=30.0, callback=self._test_critical_4b_battery_id,      critical=True),
            BmsTest(idx='5',  name='DroneCAN Parameters Check',         timeout_sec=60.0, callback=self._test_critical_5_param_check,      critical=True),
            BmsTest(idx='6',  name='Backwards Compatibility Check',     timeout_sec=60.0, callback=self._test_critical_6_backcompat,       critical=True),
            BmsTest(idx='7',  name='Lifetime Charge Tracker Validation',timeout_sec=60.0, callback=self._test_critical_7_lifetime_tracker, critical=True),
        ]

    def noncritical_tests(self):
        """
        @brief          Return the list of non-critical BmsTest definitions.
        @return         List of BmsTest with critical=False.
        """
        return [
            BmsTest(idx='1',  name='Smart Charger',                        timeout_sec=60.0, callback=self._test_noncritical_1_smart_charger,        critical=False),
            BmsTest(idx='1a', name='Pre-0.8: Full Charge',                 timeout_sec=60.0, callback=self._test_noncritical_1a_pre08_full_charge,   critical=False),
            BmsTest(idx='1b', name='Post-0.8: Initial Charge Reset',       timeout_sec=60.0, callback=self._test_noncritical_1b_post08_initial_reset, critical=False),
            BmsTest(idx='1c', name='Post-0.8: Alternate Charging Control', timeout_sec=60.0, callback=self._test_noncritical_1c_post08_alt_control,   critical=False),
            BmsTest(idx='2',  name='ATP can be completed',                 timeout_sec=120.0, callback=self._test_noncritical_2_atp_complete,         critical=False),
            BmsTest(idx='3',  name='Voltage protection works',             timeout_sec=60.0, callback=self._test_noncritical_3_voltage_protection,    critical=False),
        ]

    def all_tests(self):
        """
        @brief          Return all tests (critical followed by non-critical).
        @return         Combined list of all BmsTest definitions.
        """
        return list(self.critical_tests()) + list(self.noncritical_tests())

    # --- Critical test callbacks (placeholders) ---
    def _test_critical_1_batteries_toggle(self):
        """
        @brief          Critical test 1: Batteries turn on and off normally.
                        Called repeatedly by the worker thread (non-blocking).
                        Schedules a UI dialog once, then returns None until
                        the user answers. Returns True/False afterwards.
        @return         True if passed, False if failed, None if pending.
        """
        with self._manual_lock:
            if isinstance(self._batteries_toggle_answer, bool):
                return self._batteries_toggle_answer

            if not self._batteries_toggle_asked:
                self._batteries_toggle_asked = True
                self._ui.request_batteries_toggle_dialog.emit()

        return None

    def _test_critical_2_bq_comms(self):
        """
        @brief          Critical test 2: BQ communications work.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_critical_3_firmware_update(self):
        """
        @brief          Critical test 3: Firmware update works.
                        Schedules a FirmwareUpdateTestDialog once, then
                        returns None until the dialog finishes.
        @return         True if passed, False if failed, None if pending.
        """
        with self._manual_lock:
            if isinstance(self._firmware_update_result, bool):
                return self._firmware_update_result

            if not self._firmware_update_asked:
                self._firmware_update_asked = True
                self._ui.request_firmware_update_dialog.emit(self._firmware_update_repeat)

        return None

    def _test_critical_4a_board_id(self):
        """
        @brief          Critical test 4a: Board identification works.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_critical_4b_battery_id(self):
        """
        @brief          Critical test 4b: Battery identification works.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_critical_5_param_check(self):
        """
        @brief          Critical test 5: DroneCAN Parameters Check.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_critical_6_backcompat(self):
        """
        @brief          Critical test 6: Backwards Compatibility Check.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_critical_7_lifetime_tracker(self):
        """
        @brief          Critical test 7: Lifetime Charge Tracker Validation.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_noncritical_1_smart_charger(self):
        """
        @brief          Non-critical test 1: Smart Charger.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_noncritical_1a_pre08_full_charge(self):
        """
        @brief          Non-critical test 1a: Pre-0.8 Full Charge.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_noncritical_1b_post08_initial_reset(self):
        """
        @brief          Non-critical test 1b: Post-0.8 Initial Charge Reset.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_noncritical_1c_post08_alt_control(self):
        """
        @brief          Non-critical test 1c: Post-0.8 Alternate Charging Control.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_noncritical_2_atp_complete(self):
        """
        @brief          Non-critical test 2: ATP can be completed.
        @return         True if passed, False if failed, None if pending.
        """
        pass

    def _test_noncritical_3_voltage_protection(self):
        """
        @brief          Non-critical test 3: Voltage protection works.
        @return         True if passed, False if failed, None if pending.
        """
        pass


class BmsNodeTable(NodeTable):
    """
    @brief          Node table filtered to show only BMS nodes.
    """
    @staticmethod
    def _name_contains_bms(entry) -> bool:
        """
        @brief          Check whether a node monitor entry has 'bms'
                        in its name.
        @param[in]      entry       Node monitor entry to inspect.
        @return         True if the entry name contains 'bms' (case-insensitive).
        """
        if not getattr(entry, 'info', None) or not getattr(entry.info, 'name', None):
            return False

        name = entry.info.name
        if isinstance(name, (bytes, bytearray)):
            name = name.decode(errors='ignore')
        else:
            name = str(name)

        return 'bms' in name.lower()

    def _update(self):
        """
        @brief          Refresh the table rows to match the current set
                        of online BMS nodes.
        """
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

        # Auto-select the first row when nothing is selected
        if self.rowCount() > 0 and not self.selectedItems():
            self.selectRow(0)

class BMSAutoCheckPanel(QDialog):
    """
    @brief          Main panel dialog for BMS automated testing.
                    Contains critical / non-critical test tables, a node
                    table, and Start/Stop controls.
    """

    request_batteries_toggle_dialog = pyqtSignal()
    request_firmware_update_dialog = pyqtSignal(int)

    def __init__(self, parent, node):
        """
        @brief          Initialise the BMS Auto Check panel.
        @param[in]      parent      Parent QWidget.
        @param[in]      node        Local DroneCAN node instance.
        """
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
        self._single_test_running: Optional[BmsTest] = None
        self._selected_node_id: Optional[int] = None

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
        self._test_table.setContextMenuPolicy(Qt.CustomContextMenu)
        self._test_table.customContextMenuRequested.connect(
            lambda pos: self._on_table_context_menu(self._test_table, pos, is_critical=True))

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
        self._noncritical_table.setContextMenuPolicy(Qt.CustomContextMenu)
        self._noncritical_table.customContextMenuRequested.connect(
            lambda pos: self._on_table_context_menu(self._noncritical_table, pos, is_critical=False))

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
        self._node_table.itemSelectionChanged.connect(self._on_node_selected)
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
        self.request_firmware_update_dialog.connect(self._show_firmware_update_dialog)

    def _on_node_selected(self) -> None:
        """
        @brief          Update _selected_node_id and test group title
                        when a node row is selected.
        """
        selected = self._node_table.selectedItems()
        if not selected:
            self._selected_node_id = None
            self._test_group.setTitle('BMS Test')
            return

        row = selected[0].row()
        item = self._node_table.item(row, 0)
        if item is not None:
            self._selected_node_id = int(item.text())
            self._test_group.setTitle(f'BMS Test (ID={self._selected_node_id})')

    def _on_table_context_menu(self, table, pos, is_critical: bool) -> None:
        """
        @brief          Show a context menu with Run Test / Stop Test for
                        the clicked row.
        @param[in]      table           The BasicTable that was right-clicked.
        @param[in]      pos             Click position inside the table viewport.
        @param[in]      is_critical     True if the table is the critical-test table.
        """
        item = table.itemAt(pos)
        if item is None:
            return

        row = item.row()
        tests = self._critical_tests if is_critical else self._noncritical_tests
        if row < 0 or row >= len(tests):
            return

        test = tests[row]
        menu = QMenu(self)

        # Check if this specific test is currently running
        is_this_test_running = (
            self._is_running
            and self._single_test_running is not None
            and self._single_test_running.idx == test.idx
            and self._single_test_running.critical == test.critical
        )

        if is_this_test_running:
            stop_action = QAction('Stop Test', self)
            stop_action.triggered.connect(self._stop_runner)
            menu.addAction(stop_action)
        else:
            run_action = QAction('Run Test', self)
            run_action.triggered.connect(lambda: self._run_single_test(test))
            if self._is_running:
                run_action.setEnabled(False)
            menu.addAction(run_action)

            run_n_action = QAction('Run Test...', self)
            run_n_action.triggered.connect(lambda: self._run_test_with_count(test))
            if self._is_running:
                run_n_action.setEnabled(False)
            menu.addAction(run_n_action)

        menu.exec_(table.viewport().mapToGlobal(pos))

    def _show_run_count_dialog(self):
        """
        @brief          Show a dialog asking for the number of test repeats.
        @return         A tuple (count, continue_on_failure), or None if cancelled.
        """
        dialog = QDialog(self)
        dialog.setWindowTitle('Run Test')
        layout = QVBoxLayout(dialog)

        label = QLabel('Repeats:', dialog)
        layout.addWidget(label)

        text_box = QLineEdit('1', dialog)
        layout.addWidget(text_box)

        continue_cb = QCheckBox('Continue on failure', dialog)
        layout.addWidget(continue_cb)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, dialog)
        button_box.accepted.connect(dialog.accept)
        button_box.rejected.connect(dialog.reject)
        layout.addWidget(button_box)

        if dialog.exec_() == QDialog.Accepted:
            try:
                count = int(text_box.text())
                if count > 0:
                    return count, continue_cb.isChecked()
            except ValueError:
                pass
        return None

    def _run_test_with_count(self, test: BmsTest) -> None:
        """
        @brief          Show the run-count dialog, then run the test N times.
        @param[in]      test    The BmsTest to run.
        """
        if self._is_running:
            return
        result = self._show_run_count_dialog()
        if result is None:
            return
        count, continue_on_failure = result
        self._run_single_test(test, repeat=count, continue_on_failure=continue_on_failure)

    def _run_single_test(self, test: BmsTest, repeat: int = 1, continue_on_failure: bool = False) -> None:
        """
        @brief          Run a test (optionally repeated) using a dedicated
                        worker thread.
        @param[in]      test                The BmsTest to run.
        @param[in]      repeat              Number of times to repeat the test.
        @param[in]      continue_on_failure  If True, don't stop on failure.
        """
        if self._is_running:
            return

        # Reset only the row for this test
        self._set_test_row(test, 'Pending', '')

        self._single_test_running = test
        scaled_test = replace(test, timeout_sec=test.timeout_sec * repeat) if repeat > 1 else test
        self._suite.set_tests([scaled_test] * repeat)
        self._tests.reset()
        self._tests._firmware_update_repeat = repeat
        self._tests._firmware_update_continue_on_failure = continue_on_failure

        self._is_running = True
        self._start_button.setText('Stop')
        self._start_button.setEnabled(True)
        logger.info('Starting single test: %s - %s', test.idx, test.name)

        self._runner_thread = QThread(self)
        self._runner_worker = _BmsTestRunnerWorker(self._suite)
        self._runner_worker.moveToThread(self._runner_thread)

        self._runner_thread.started.connect(self._runner_worker.run, Qt.QueuedConnection)
        self._runner_worker.test_started.connect(self._on_test_started)
        self._runner_worker.test_updated.connect(self._on_test_updated)
        self._runner_worker.run_finished.connect(self._on_run_finished)

        self._runner_thread.start()

    @pyqtSlot()
    def _show_batteries_toggle_dialog(self) -> None:
        """
        @brief          Show a Yes/No dialog asking whether the batteries
                        toggled on and off normally.
        """
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

    @pyqtSlot(int)
    def _show_firmware_update_dialog(self, repeat: int) -> None:
        """
        @brief          Open a FirmwareUpdateTestDialog for the selected node.
        @param[in]      repeat      Number of firmware update cycles.
        """
        if not self._is_running:
            return

        if self._selected_node_id is None:
            logger.error('Cannot start firmware update test: no node selected')
            self._tests._set_firmware_update_result(False)
            return

        # Access file_server_widget from the main window (same pattern as other panels)
        main_window = self.parent()
        file_server_widget = getattr(main_window, '_file_server_widget', None)

        dialog = FirmwareUpdateTestDialog(
            repeat=repeat,
            node=self._node,
            target_node_id=self._selected_node_id,
            file_server_widget=file_server_widget,
            continue_on_failure=self._tests._firmware_update_continue_on_failure,
            parent=self,
        )
        dialog.test_finished.connect(self._tests._set_firmware_update_result)
        self._tests._firmware_update_dialog = dialog  # prevent GC
        dialog.show()

    def _reset_test_tables(self) -> None:
        """
        @brief          Reset all rows in both test tables to 'Pending'.
        """
        for row, test in enumerate(self._critical_tests):
            if row < len(self._critical_row_data):
                self._critical_row_data[row].update({'idx': test.idx, 'name': test.name, 'status': 'Pending', 'time': ''})
                self._test_table.set_row(row, self._critical_row_data[row])

        for row, test in enumerate(self._noncritical_tests):
            if row < len(self._noncritical_row_data):
                self._noncritical_row_data[row].update({'idx': test.idx, 'name': test.name, 'status': 'Pending', 'time': ''})
                self._noncritical_table.set_row(row, self._noncritical_row_data[row])

    def _set_test_row(self, test: BmsTest, status: str, elapsed_str: str) -> None:
        """
        @brief          Update a single test row in the appropriate table.
        @param[in]      test            The BmsTest whose row to update.
        @param[in]      status          Status text (e.g. 'Running', 'Pass', 'Failed').
        @param[in]      elapsed_str     Formatted elapsed time string.
        """
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
        """
        @brief          Reset the suite and start the full test run on a
                        worker thread.
        """
        if self._is_running:
            return

        self._reset_test_tables()
        self._suite.reset()
        self._tests.reset()

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
        """
        @brief          Request the worker to stop and clean up the thread.
        """
        if not self._is_running:
            return

        if self._runner_worker is not None:
            # Set the flag directly — the worker polls it in its loop.
            # QueuedConnection won't work here because the worker's run()
            # never returns to the thread's event loop.
            self._runner_worker._stop_requested = True

        # Force-close any open firmware update dialog so the file server
        # stops serving the firmware file and the node can no longer download it.
        if self._tests._firmware_update_dialog is not None:
            try:
                self._tests._firmware_update_dialog.close()
            except Exception:
                logger.exception('Could not close firmware update dialog')
            self._tests._firmware_update_dialog = None

        # Synchronously clean up so the next Start works immediately
        self._cleanup_runner()

    def _cleanup_runner(self) -> None:
        """
        @brief          Terminate the worker thread and restore UI state.
        """
        if self._runner_thread is not None:
            self._runner_thread.quit()
            self._runner_thread.wait(1500)
            self._runner_thread = None
        self._runner_worker = None
        self._is_running = False
        self._single_test_running = None
        # Restore the full test list in case a single test was run
        self._suite.set_tests(self._tests.all_tests())
        self._start_button.setText('Start')
        self._start_button.setEnabled(True)

    def _on_start_stop_clicked(self):
        """
        @brief          Handle Start/Stop button click.
        """
        if self._is_running:
            self._stop_runner()
        else:
            self._start_runner()

    @pyqtSlot(object)
    def _on_test_started(self, test: BmsTest) -> None:
        """
        @brief          Slot called when a test starts running.
        @param[in]      test    The BmsTest that just started.
        """
        self._set_test_row(test, 'Running', '0.0s')

    @pyqtSlot(object, str, str)
    def _on_test_updated(self, test: BmsTest, status: str, elapsed_str: str) -> None:
        """
        @brief          Slot called when a test's status or timer updates.
        @param[in]      test            The BmsTest being updated.
        @param[in]      status          Current status text.
        @param[in]      elapsed_str     Formatted elapsed time.
        """
        self._set_test_row(test, status, elapsed_str)

    @pyqtSlot(bool)
    def _on_run_finished(self, stopped: bool) -> None:
        """
        @brief          Slot called when the worker thread finishes.
        @param[in]      stopped     True if the run was stopped by the user.
        """
        self._cleanup_runner()

    def _update_status(self):
        """
        @brief          Refresh the discovery status label.
        """
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
        """
        @brief          Clean up timers, handlers, and the worker thread
                        when the panel is closed.
        @param[in]      event       The QCloseEvent.
        """
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
    """
    @brief          Create or raise the singleton BMSAutoCheckPanel.
    @param[in]      parent      Parent QWidget.
    @param[in]      node        Local DroneCAN node instance.
    @return         The panel instance.
    """
    global _singleton
    if _singleton is None:
        _singleton = BMSAutoCheckPanel(parent, node)

    _singleton.show()
    _singleton.raise_()
    _singleton.activateWindow()

    return _singleton


get_icon = partial(get_icon, 'fa6s.car-battery')