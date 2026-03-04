import os
from logging import getLogger

import dronecan
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QLineEdit, QFileDialog, QProgressBar, QMessageBox
from PyQt5.QtCore import pyqtSignal, Qt, QTimer

from dronecan_gui_tool.widgets.file_server import FileServer_PathKey

logger = getLogger(__name__)

REQUEST_PRIORITY = 30


class FirmwareUpdateTestDialog(QDialog):
    """
    @brief          Dialog for the firmware update test.
                    Shows OK / Cancel buttons.
                    - OK starts the test (dialog stays open until the test finishes).
                    - Cancel aborts the test immediately.
                    The dialog closes automatically when the test completes.
    """

    test_finished = pyqtSignal(bool)  # True = pass, False = fail

    def __init__(self, repeat: int = 1, node=None, target_node_id=None, file_server_widget=None, parent=None):
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
        self._current_run = 0
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
        self._saw_software_update_mode = False
        self._last_target_uptime = None

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

        fw_file = self._file1_textbox.text().strip()
        if not fw_file:
            return

        if self._repeat > 1:
            fw_file2 = self._file2_textbox.text().strip()
            if not fw_file or not fw_file2:
                msg = QMessageBox(self)
                msg.setIcon(QMessageBox.Warning)
                msg.setWindowTitle('Missing File')
                msg.setText('Two files need to be selected when the number of tests is above 1')
                msg.setStandardButtons(QMessageBox.Ok)
                msg.exec_()
                return

        self._test_started = True
        self._ok_button.setEnabled(False)
        self._file1_load_button.setEnabled(False)
        self._file2_load_button.setEnabled(False)
        self._current_run = 0
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
                        for the current run.
        """
        if self._current_run >= self._repeat:
            logger.info('FW_TEST [node %s] All %d runs completed — finishing with success',
                        self._target_node_id, self._repeat)
            self._finish_test(True)
            return

        self._current_run += 1
        self.setWindowTitle('Firmware Update Test (%d/%d)' % (self._current_run, self._repeat))
        # Each run occupies an equal slice of the 0-100 bar
        self._run_base = (self._current_run - 1) / self._repeat * 100  # start of this run's slice
        self._run_size = 100.0 / self._repeat                         # width of each run's slice
        self._progress_bar.setValue(int(self._run_base))

        fw_path = self._file1_textbox.text().strip()
        if not fw_path:
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
            except Exception:
                logger.exception('Could not configure file server')
                self._finish_test(False)
                return

        remote_fw_file = FileServer_PathKey(fw_file)
        logger.info('Firmware update run %d/%d  node=%d  file=%s',
                    self._current_run, self._repeat, self._target_node_id, remote_fw_file)

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
            logger.info('FW_TEST [node %d] Run %d/%d COMPLETE — fw_max_offset=%d/%d',
                        self._target_node_id, self._current_run, self._repeat,
                        self._fw_max_offset, self._fw_file_size)
            self._uninstall_transfer_hook()
            self._progress_timer.stop()
            self._cleanup_handlers()
            progress_val = int(self._run_base + self._run_size)
            self._progress_bar.setValue(min(progress_val, 100))
            self._run_firmware_update()  # advance to next run or finish

        def on_timeout():
            if self._closed:
                logger.debug('on_timeout: ignored (dialog closed)')
                return
            self._timeout_handle = None
            logger.error('FW_TEST [node %d] TIMEOUT run %d/%d — accepted=%s, saw_sw_update=%s, '
                         'last_uptime=%s, fw_offset=%d/%d',
                         self._target_node_id, self._current_run, self._repeat,
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

    def _finish_test(self, success: bool):
        """
        @brief          Finalise the firmware update test.
        @param[in]      success     True if the test passed, False otherwise.
        """
        if self._closed:
            logger.debug('_finish_test(%s): ignored (dialog already closed)', success)
            return
        logger.info('FW_TEST [node %s] _finish_test(%s) — run=%d/%d, fw_offset=%d/%d',
                    self._target_node_id, success, self._current_run, self._repeat,
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
        # If closed without a result (e.g. the X button), treat as cancel.
        if self._result is None:
            logger.info('FW_TEST closeEvent: no result set — treating as cancel (emitting False)')
            self._result = False
            self.test_finished.emit(False)
        super().closeEvent(event)
