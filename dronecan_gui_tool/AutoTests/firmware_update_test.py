import datetime
import os
from logging import getLogger

import dronecan
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QLineEdit,
                              QFileDialog, QProgressBar, QMessageBox, QTableWidget,
                              QTableWidgetItem, QHeaderView)
from PyQt5.QtCore import pyqtSignal, Qt, QTimer

from dronecan_gui_tool.widgets.file_server import FileServer_PathKey
from dronecan_gui_tool.AutoTests.firmware_binary_parser import FirmwareBinaryParser

logger = getLogger(__name__)

REQUEST_PRIORITY = 30
POST_REBOOT_TIMEOUT = 20  # seconds to wait for node to come back after FW update


class FirmwareUpdateTestDialog(QDialog):
    """
    @brief          Dialog for the firmware update test.
                    Shows OK / Cancel buttons.
                    - OK starts the test (dialog stays open until the test finishes).
                    - Cancel aborts the test immediately.
                    The dialog closes automatically when the test completes.
    """

    test_finished = pyqtSignal(bool)  # True = pass, False = fail

    def __init__(self, repeat: int = 1, node=None, target_node_id=None, file_server_widget=None, continue_on_failure: bool = False, parent=None):
        """
        @brief          Initialise the firmware update test dialog.
        @param[in]      repeat              Number of firmware update cycles to run.
        @param[in]      node                Local DroneCAN node instance.
        @param[in]      target_node_id      Node ID of the target device.
        @param[in]      file_server_widget  File server widget for serving firmware.
        @param[in]      parent              Parent QWidget.
        """
        super().__init__(parent)
        self.setWindowTitle('Firmware Update Test (0/%d)' % max(1, repeat))
        self.setModal(False)
        self._repeat = max(1, repeat)
        self._node = node
        self._target_node_id = target_node_id
        self._file_server_widget = file_server_widget
        self._current_step = 0
        self._total_steps = 1
        self._deferred_request_handle = None
        self._node_status_handle = None

        layout = QVBoxLayout(self)

        # --- Load File 1 row ---
        file1_layout = QHBoxLayout()
        file1_label = QLabel('Load File 1:', self)
        self._file1_textbox = QLineEdit(self)
        self._file1_load_button = QPushButton('Load', self)
        self._file1_load_button.clicked.connect(self._on_load_file1)
        file1_layout.addWidget(file1_label)
        file1_layout.addWidget(self._file1_textbox, 1)
        file1_layout.addWidget(self._file1_load_button)
        layout.addLayout(file1_layout)

        # --- Load File 2 row ---
        file2_layout = QHBoxLayout()
        file2_label = QLabel('Load File 2:', self)
        self._file2_textbox = QLineEdit(self)
        self._file2_load_button = QPushButton('Load', self)
        self._file2_load_button.clicked.connect(self._on_load_file2)
        file2_layout.addWidget(file2_label)
        file2_layout.addWidget(self._file2_textbox, 1)
        file2_layout.addWidget(self._file2_load_button)
        layout.addLayout(file2_layout)

        button_layout = QHBoxLayout()
        self._ok_button = QPushButton('OK', self)
        self._cancel_button = QPushButton('Cancel', self)
        button_layout.addWidget(self._ok_button)
        button_layout.addWidget(self._cancel_button)
        layout.addLayout(button_layout)

        self._progress_bar = QProgressBar(self)
        self._progress_bar.setRange(0, 100)
        self._progress_bar.setValue(0)
        self._progress_bar.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._progress_bar)

        self._ok_button.clicked.connect(self._on_ok)
        self._cancel_button.clicked.connect(self._on_cancel)

        self._test_started = False
        self._result = None  # None = not decided, True = pass, False = fail
        self._closed = False

        # File-transfer progress tracking
        self._fw_file_size = 0
        self._fw_max_offset = 0       # updated from transfer hook, read from Qt timer
        self._transfer_hook_handle = None
        self._progress_timer = QTimer(self)
        self._progress_timer.setInterval(200)
        self._progress_timer.timeout.connect(self._update_transfer_progress)

        # Firmware update completion tracking
        self._timeout_handle = None
        self._update_accepted = False
        self._fw_file_path = None  # Normalized path added to file server, removed on close
        self._saw_software_update_mode = False
        self._last_target_uptime = None
        self._verify_status_handle = None
        self._verify_timeout_handle = None

        # Continue-on-failure tracking
        self._continue_on_failure = continue_on_failure
        self._step_results = []
        self._current_step_fw_file = ''
        self._current_step_start_time = ''

    def _on_load_file1(self):
        """
        @brief          Open a file dialog to select firmware file 1.
        """
        path, _ = QFileDialog.getOpenFileName(
            self, 'Select Firmware File', '', 'BIN Files (*.bin);;All Files (*)')
        if path:
            self._file1_textbox.setText(path)
            self._file1_load_button.setToolTip(path)

    def _on_load_file2(self):
        """
        @brief          Open a file dialog to select firmware file 2.
        """
        path, _ = QFileDialog.getOpenFileName(
             self, 'Select Firmware File', '', 'BIN Files (*.bin);;All Files (*)')
        if path:
            self._file2_textbox.setText(path)
            self._file2_load_button.setToolTip(path)

    def _on_ok(self):
        """
        @brief          Handle OK button click. Validates file selection
                        and starts the firmware update test.
        """
        if self._test_started:
            return

        self._has_file1 = bool(self._file1_textbox.text().strip())
        self._has_file2 = bool(self._file2_textbox.text().strip())

        if not self._has_file1 and not self._has_file2:
            return

        self._test_started = True
        self._ok_button.setEnabled(False)
        self._file1_load_button.setEnabled(False)
        self._file2_load_button.setEnabled(False)
        self._current_step = 0
        if self._has_file1 and self._has_file2:
            self._total_steps = self._repeat * 2
        else:
            self._total_steps = max(1, self._repeat)
        self._run_firmware_update()

    def _on_cancel(self):
        """
        @brief          Handle Cancel button click. Aborts the test and closes the dialog.
        """
        self._result = False
        self.test_finished.emit(False)
        self.close()

    def _run_firmware_update(self):
        """
        @brief          Start (or continue) the firmware update sequence
                        for the current step.
                        When repeat > 1 each repeat cycle consists of two
                        FW updates: file 1 then file 2.  When repeat == 1
                        only file 1 is used (single step).
        """
        if self._current_step >= self._total_steps:
            logger.info('FW_TEST [node %s] All %d steps completed — finishing with success',
                        self._target_node_id, self._total_steps)
            if self._continue_on_failure and self._step_results:
                self._complete_all_steps()
            else:
                self._finish_test(True)
            return

        self._current_step += 1
        self._current_step_start_time = datetime.datetime.now().strftime('%H:%M:%S')
        # Each step occupies an equal slice of the 0-100 bar
        self._run_base = (self._current_step - 1) / self._total_steps * 100
        self._run_size = 100.0 / self._total_steps
        self._progress_bar.setValue(int(self._run_base))

        # Determine which file to use for this step
        if self._has_file1 and self._has_file2:
            # Both files: alternate (odd steps = file 1, even steps = file 2)
            file_index = 1 if (self._current_step % 2 == 1) else 2
        elif self._has_file1:
            file_index = 1
        else:
            file_index = 2

        fw_path = (self._file1_textbox.text().strip() if file_index == 1
                   else self._file2_textbox.text().strip())
        self._current_step_fw_file = fw_path or ('File %d' % file_index)

        if self._has_file1 and self._has_file2:
            repeat_num = (self._current_step + 1) // 2
        else:
            repeat_num = self._current_step
        self.setWindowTitle('Firmware Update Test (step %d/%d — file %d, repeat %d/%d)'
                           % (self._current_step, self._total_steps, file_index,
                              repeat_num, self._repeat))

        if not fw_path:
            logger.error('Firmware file %d path is empty', file_index)
            self._finish_test(False)
            return

        fw_file = os.path.normcase(os.path.abspath(fw_path))

        # Verify file is readable
        try:
            with open(fw_file, 'rb') as f:
                f.read(100)
        except Exception:
            logger.error('Firmware file not readable: %s', fw_file)
            self._finish_test(False)
            return

        # Check prerequisites
        if self._node is None or self._target_node_id is None:
            logger.error('Node or target_node_id not set')
            self._finish_test(False)
            return

        if self._node.is_anonymous:
            logger.error('Local node is anonymous, cannot request firmware update')
            self._finish_test(False)
            return

        # Configure file server
        if self._file_server_widget is not None:
            try:
                self._file_server_widget.add_path(fw_file)
                self._file_server_widget.force_start()
                self._fw_file_path = fw_file
            except Exception:
                logger.exception('Could not configure file server')
                self._finish_test(False)
                return

        remote_fw_file = FileServer_PathKey(fw_file)
        logger.info('Firmware update step %d/%d  node=%d  file%d=%s',
                    self._current_step, self._total_steps, self._target_node_id,
                    file_index, remote_fw_file)

        # Set up file-transfer progress monitoring
        self._fw_file_size = os.path.getsize(fw_file)
        self._fw_max_offset = 0
        self._update_accepted = False
        self._saw_software_update_mode = False
        self._last_target_uptime = None
        self._install_transfer_hook()
        self._progress_timer.start()

        TIMEOUT_SECONDS = 300  # 5 minutes per run
        total_requests = 4
        num_remaining = [total_requests]  # mutable counter for closures

        # --- Phase 1 helpers: initiate the update ---

        def on_update_accepted():
            """
            @brief      Handle node accepting the firmware update. Stops
                        sending requests and waits for transfer + reboot.
            """
            if self._closed:
                logger.debug('on_update_accepted: ignored (dialog closed)')
                return
            if self._update_accepted:
                logger.debug('on_update_accepted: ignored (already accepted)')
                return
            self._update_accepted = True
            # Cancel any pending deferred BeginFirmwareUpdate retry
            if self._deferred_request_handle is not None:
                self._deferred_request_handle.remove()
                self._deferred_request_handle = None
            logger.info('FW_TEST [node %d] Update ACCEPTED — waiting for transfer and reboot...',
                        self._target_node_id)

        # --- Phase 2 helper: transfer done, node rebooted ---

        def on_run_complete():
            """
            @brief      Handle run completion when the node reboots
                        after the firmware update.
            """
            if self._closed:
                logger.debug('on_run_complete: ignored (dialog closed)')
                return
            logger.info('FW_TEST [node %d] Step %d/%d COMPLETE — fw_max_offset=%d/%d, '
                        'saw_sw_update=%s',
                        self._target_node_id, self._current_step, self._total_steps,
                        self._fw_max_offset, self._fw_file_size,
                        self._saw_software_update_mode)
            self._uninstall_transfer_hook()
            self._progress_timer.stop()
            self._cleanup_handlers()
            progress_val = int(self._run_base + self._run_size)
            self._progress_bar.setValue(min(progress_val, 100))
            # If the firmware transfer was clearly incomplete, fail immediately
            if (self._fw_file_size > 0
                    and self._fw_max_offset + 256 < self._fw_file_size):
                logger.error('FW_TEST [node %d] Firmware transfer incomplete: '
                             'max_offset=%d, file_size=%d — treating as failure',
                             self._target_node_id, self._fw_max_offset, self._fw_file_size)
                self._finish_test(False)
                return
            self._verify_post_reboot(fw_file)

        def on_timeout():
            if self._closed:
                logger.debug('on_timeout: ignored (dialog closed)')
                return
            self._timeout_handle = None
            logger.error('FW_TEST [node %d] TIMEOUT step %d/%d — accepted=%s, saw_sw_update=%s, '
                         'last_uptime=%s, fw_offset=%d/%d',
                         self._target_node_id, self._current_step, self._total_steps,
                         self._update_accepted, self._saw_software_update_mode,
                         self._last_target_uptime, self._fw_max_offset, self._fw_file_size)
            self._finish_test(False)

        # --- DroneCAN callbacks ---

        def on_response(e):
            if self._closed:
                logger.debug('on_response: ignored (dialog closed)')
                return
            if self._update_accepted:
                logger.debug('on_response: ignored (already accepted)')
                return
            if e is None:
                logger.warning('FW_TEST [node %d] BeginFirmwareUpdate request timed out (remaining=%d)',
                               self._target_node_id, num_remaining[0])
                self._deferred_request_handle = self._node.defer(2, send_request)
            else:
                logger.info('FW_TEST [node %d] BeginFirmwareUpdate response: error=%d (%s)',
                            self._target_node_id, e.response.error, e.response)
                if e.response.error in (0, e.response.ERROR_IN_PROGRESS):
                    on_update_accepted()
                else:
                    logger.warning('FW_TEST [node %d] Rejected (error=%d), retrying...',
                                   self._target_node_id, e.response.error)
                    self._deferred_request_handle = self._node.defer(2, send_request)

        def on_node_status(e):
            if self._closed:
                return
            if e.transfer.source_node_id != self._target_node_id:
                return

            current_uptime = e.message.uptime_sec
            current_mode = e.message.mode
            current_health = e.message.health

            logger.debug('FW_TEST [node %d] NodeStatus: uptime=%d mode=%d health=%d | '
                         'accepted=%s saw_sw_update=%s last_uptime=%s',
                         self._target_node_id, current_uptime, current_mode, current_health,
                         self._update_accepted, self._saw_software_update_mode,
                         self._last_target_uptime)

            # Detect MODE_SOFTWARE_UPDATE → accept the update if not already
            if (current_mode == e.message.MODE_SOFTWARE_UPDATE
                    and current_health < e.message.HEALTH_ERROR):
                if not self._saw_software_update_mode:
                    logger.info('FW_TEST [node %d] Entered MODE_SOFTWARE_UPDATE (health=%d)',
                                self._target_node_id, current_health)
                self._saw_software_update_mode = True
                if not self._update_accepted:
                    on_update_accepted()

            # After update accepted, detect completion (node rebooted)
            if self._update_accepted:
                rebooted = (self._last_target_uptime is not None
                            and current_uptime < self._last_target_uptime)
                left_update_mode = (self._saw_software_update_mode
                                    and current_mode != e.message.MODE_SOFTWARE_UPDATE)
                if rebooted or left_update_mode:
                    # Only treat as completion if we actually saw firmware
                    # transfer or SOFTWARE_UPDATE mode.  Otherwise the node
                    # just rebooted into its bootloader and the real transfer
                    # hasn't started yet — keep waiting.
                    if not self._saw_software_update_mode and self._fw_max_offset == 0:
                        logger.info('FW_TEST [node %d] Reboot detected (uptime %s->%d) but no '
                                    'transfer yet — assuming bootloader entry, continuing…',
                                    self._target_node_id,
                                    self._last_target_uptime, current_uptime)
                        self._last_target_uptime = current_uptime
                        return
                    logger.info('FW_TEST [node %d] Reboot detected! uptime %s->%d, mode=%d, '
                                'rebooted=%s, left_update_mode=%s',
                                self._target_node_id,
                                self._last_target_uptime, current_uptime, current_mode,
                                rebooted, left_update_mode)
                    on_run_complete()
                    return

            self._last_target_uptime = current_uptime

        def send_request():
            if self._closed:
                logger.debug('send_request: ignored (dialog closed)')
                return
            if self._update_accepted:
                logger.debug('send_request: ignored (already accepted)')
                return
            self._deferred_request_handle = None
            if num_remaining[0] > 0:
                num_remaining[0] -= 1
                request = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
                    source_node_id=self._node.node_id,
                    image_file_remote_path=dronecan.uavcan.protocol.file.Path(path=remote_fw_file))
                logger.info('FW_TEST [node %d] Sending BeginFirmwareUpdate (%d remaining): %s',
                            self._target_node_id, num_remaining[0], request)
                try:
                    self._node.request(request, self._target_node_id, on_response, priority=REQUEST_PRIORITY)
                except Exception:
                    logger.exception('FW_TEST [node %d] Could not send firmware update request',
                                     self._target_node_id)
                    self._finish_test(False)
            else:
                # All requests exhausted — assume the update was accepted
                logger.info('FW_TEST [node %d] All %d requests exhausted, assuming accepted',
                            self._target_node_id, total_requests)
                on_update_accepted()

        self._timeout_handle = self._node.defer(TIMEOUT_SECONDS, on_timeout)
        self._node_status_handle = self._node.add_handler(
            dronecan.uavcan.protocol.NodeStatus, on_node_status)
        send_request()

    # --- File-transfer progress helpers ---

    def _install_transfer_hook(self):
        """
        @brief          Register a transfer hook to monitor incoming
                        file.Read requests from the target node.
        """
        self._uninstall_transfer_hook()
        if self._node is None:
            return

        def on_transfer(transfer):
            try:
                # We only care about incoming service requests from the target node
                if not (transfer.service_not_message and transfer.request_not_response):
                    return
                if transfer.source_node_id != self._target_node_id:
                    return
                # Check if the payload has an 'offset' attribute (file.Read.Request)
                offset = getattr(transfer.payload, 'offset', None)
                if offset is not None:
                    self._fw_max_offset = max(self._fw_max_offset, offset)
            except Exception:
                pass

        self._transfer_hook_handle = self._node.add_transfer_hook(on_transfer)

    def _uninstall_transfer_hook(self):
        """
        @brief          Remove the transfer hook.
        """
        if self._transfer_hook_handle is not None:
            try:
                self._transfer_hook_handle.remove()
            except Exception:
                pass
            self._transfer_hook_handle = None

    def _update_transfer_progress(self):
        """
        @brief          Update the progress bar from the Qt thread.
                        Called by the QTimer periodically.
        """
        if self._closed or self._fw_file_size <= 0:
            return
        frac = min(self._fw_max_offset / self._fw_file_size, 1.0)
        progress_val = int(self._run_base + self._run_size * frac)
        self._progress_bar.setValue(min(progress_val, 100))

    # --- DroneCAN handler cleanup ---

    def _remove_fw_from_file_server(self):
        """
        @brief          Remove the firmware file from the file server so the
                        remote node can no longer download it.
        """
        if self._fw_file_path is not None and self._file_server_widget is not None:
            try:
                self._file_server_widget.remove_path(self._fw_file_path)
                logger.info('FW_TEST removed firmware path from file server: %s', self._fw_file_path)
            except Exception:
                logger.exception('FW_TEST could not remove firmware path from file server')
            self._fw_file_path = None

    def _cleanup_handlers(self):
        """
        @brief          Remove any pending DroneCAN handlers.
        """
        if self._deferred_request_handle is not None:
            self._deferred_request_handle.remove()
            self._deferred_request_handle = None
        if self._node_status_handle is not None:
            self._node_status_handle.remove()
            self._node_status_handle = None
        if self._timeout_handle is not None:
            self._timeout_handle.try_remove()
            self._timeout_handle = None
        if self._verify_status_handle is not None:
            self._verify_status_handle.remove()
            self._verify_status_handle = None
        if self._verify_timeout_handle is not None:
            self._verify_timeout_handle.try_remove()
            self._verify_timeout_handle = None

    def _verify_post_reboot(self, fw_file_path: str):
        """
        @brief          After a firmware update step completes, verify that
                        the node comes back online and reports the correct
                        firmware version in its name.
        @param[in]      fw_file_path    Path to the .bin file that was just
                                        sent to the node.
        """
        logger.info('FW_TEST [node %d] Starting post-reboot verification (timeout=%ds, fw=%s)',
                    self._target_node_id, POST_REBOOT_TIMEOUT, fw_file_path)

        parser = FirmwareBinaryParser(fw_file_path)
        if not parser.match_found:
            logger.warning('FW_TEST [node %d] Could not parse version from firmware file %s — skipping version check',
                           self._target_node_id, fw_file_path)

        def on_verify_timeout():
            if self._closed:
                return
            self._verify_timeout_handle = None
            logger.error('FW_TEST [node %d] Post-reboot verification TIMEOUT — '
                         'no NodeStatus received within %d s',
                         self._target_node_id, POST_REBOOT_TIMEOUT)
            self._finish_test(False)

        def on_verify_node_status(e):
            if self._closed:
                return
            if e.transfer.source_node_id != self._target_node_id:
                return
            # Ignore if node is still in software update mode — it hasn't finished yet
            if e.message.mode == e.message.MODE_SOFTWARE_UPDATE:
                logger.debug('FW_TEST [node %d] Post-reboot: ignoring NodeStatus still in MODE_SOFTWARE_UPDATE',
                             self._target_node_id)
                return
            # Node is alive and out of update mode — cancel timeout
            logger.info('FW_TEST [node %d] Post-reboot NodeStatus received (uptime=%d, mode=%d)',
                        self._target_node_id, e.message.uptime_sec, e.message.mode)
            # Clean up verification handlers
            if self._verify_status_handle is not None:
                self._verify_status_handle.remove()
                self._verify_status_handle = None
            if self._verify_timeout_handle is not None:
                self._verify_timeout_handle.try_remove()
                self._verify_timeout_handle = None

            if not parser.match_found:
                logger.info('FW_TEST [node %d] No version to verify — advancing',
                            self._target_node_id)
                self._advance_to_next_step()
                return

            # Send GetNodeInfo to check the node's name contains the expected version
            def on_node_info(msg):
                if self._closed:
                    return
                if msg is None:
                    logger.error('FW_TEST [node %d] GetNodeInfo request timed out',
                                 self._target_node_id)
                    self._finish_test(False)
                    return

                node_name = msg.response.name
                if isinstance(node_name, (bytes, bytearray)):
                    node_name = node_name.decode('utf-8', errors='replace')
                else:
                    node_name = str(node_name)
                node_name = node_name.rstrip('\x00').strip()

                logger.info('FW_TEST [node %d] GetNodeInfo name: "%s"',
                            self._target_node_id, node_name)

                if parser.is_match(node_name):
                    logger.info('FW_TEST [node %d] Version verification PASSED — '
                                'node name matches expected "%s"',
                                self._target_node_id, parser.full_match)
                    self._advance_to_next_step()
                else:
                    logger.error('FW_TEST [node %d] Version verification FAILED — '
                                 'node name "%s" does not match expected "%s"',
                                 self._target_node_id, node_name, parser.full_match)
                    self._finish_test(False)

            try:
                req = dronecan.uavcan.protocol.GetNodeInfo.Request()
                self._node.request(req, self._target_node_id, on_node_info,
                                   priority=REQUEST_PRIORITY)
            except Exception:
                logger.exception('FW_TEST [node %d] Could not send GetNodeInfo',
                                 self._target_node_id)
                self._finish_test(False)

        self._verify_timeout_handle = self._node.defer(POST_REBOOT_TIMEOUT, on_verify_timeout)
        self._verify_status_handle = self._node.add_handler(
            dronecan.uavcan.protocol.NodeStatus, on_verify_node_status)

    def _advance_to_next_step(self):
        """
        @brief          Advance to the next firmware update step, or finish
                        if all steps are complete.
        """
        self._record_step_result(True)
        if self._closed:
            return
        if self._current_step < self._total_steps:
            logger.info('FW_TEST [node %d] Waiting %d s for node to stabilize before next step…',
                        self._target_node_id, POST_REBOOT_TIMEOUT)
            QTimer.singleShot(POST_REBOOT_TIMEOUT * 1000, self._run_firmware_update)
        else:
            if self._continue_on_failure:
                self._complete_all_steps()
            else:
                self._run_firmware_update()  # will see current_step >= total_steps and finish

    def _record_step_result(self, success: bool):
        """
        @brief          Record the result of the current firmware update step.
        @param[in]      success     True if the step passed, False if failed.
        """
        self._step_results.append({
            'fw_file': self._current_step_fw_file or 'N/A',
            'status': 'Pass' if success else 'Failed',
            'start_time': self._current_step_start_time or 'N/A',
        })

    def _complete_all_steps(self):
        """
        @brief          Finalize when all steps are done in continue-on-failure
                        mode. Shows a report dialog, then finishes the test.
        """
        if self._closed:
            return
        all_passed = all(r['status'] == 'Pass' for r in self._step_results)
        if not all_passed:
            self._show_report_dialog()
        self._continue_on_failure = False
        self._finish_test(all_passed)

    def _show_report_dialog(self):
        """
        @brief          Show a modal report dialog with results of all
                        firmware update steps.
        """
        dialog = QDialog(self)
        dialog.setWindowTitle('Firmware Update Report')
        dialog.setModal(True)
        layout = QVBoxLayout(dialog)

        table = QTableWidget(len(self._step_results), 3, dialog)
        table.setHorizontalHeaderLabels(['FW File', 'Status', 'Start Time'])
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)

        for row, result in enumerate(self._step_results):
            for col, value in enumerate([result['fw_file'], result['status'], result['start_time']]):
                item = QTableWidgetItem(value)
                item.setFlags(item.flags() | Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                table.setItem(row, col, item)

        table.setSelectionMode(QTableWidget.ContiguousSelection)
        layout.addWidget(table)

        ok_button = QPushButton('OK', dialog)
        ok_button.clicked.connect(dialog.accept)
        layout.addWidget(ok_button)

        dialog.resize(600, 400)
        dialog.exec_()

    def _finish_test(self, success: bool):
        """
        @brief          Finalize the firmware update test.
        @param[in]      success     True if the test passed, False otherwise.
        """
        if self._closed:
            logger.debug('_finish_test(%s): ignored (dialog already closed)', success)
            return
        # Continue-on-failure: record failure and advance instead of stopping
        if not success and self._continue_on_failure:
            self._record_step_result(False)
            self._progress_timer.stop()
            self._uninstall_transfer_hook()
            self._cleanup_handlers()
            progress_val = int(self._run_base + self._run_size)
            self._progress_bar.setValue(min(progress_val, 100))
            if self._current_step < self._total_steps:
                QTimer.singleShot(2000, self._run_firmware_update)
            else:
                self._complete_all_steps()
            return

        logger.info('FW_TEST [node %s] _finish_test(%s) — step=%d/%d, fw_offset=%d/%d',
                    self._target_node_id, success, self._current_step, self._total_steps,
                    self._fw_max_offset, self._fw_file_size)
        self._progress_timer.stop()
        self._uninstall_transfer_hook()
        self._cleanup_handlers()
        self._result = success
        self._progress_bar.setValue(100 if success else self._progress_bar.value())
        self.test_finished.emit(success)
        self.close()

    def closeEvent(self, event):
        """
        @brief          Handle the dialog close event. Cleans up handlers
                        and emits failure if no result was set.
        @param[in]      event       The QCloseEvent.
        """
        logger.debug('FW_TEST closeEvent: result=%s, closed=%s', self._result, self._closed)
        self._closed = True
        self._progress_timer.stop()
        self._uninstall_transfer_hook()
        self._cleanup_handlers()
        self._remove_fw_from_file_server()
        # If closed without a result (e.g. the X button), treat as cancel.
        if self._result is None:
            logger.info('FW_TEST closeEvent: no result set — treating as cancel (emitting False)')
            self._result = False
            self.test_finished.emit(False)
        super().closeEvent(event)
