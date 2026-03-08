
#
# Copyright (C) 2026  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ilan Graidy
# Date:   2026-02-24
#

import dronecan
from functools import partial
from logging import getLogger
import threading
import os
import re
import json
import random
import xml.etree.ElementTree as ET

from PyQt5.QtCore import Qt, QRect, QSize, QPoint, QTimer
from PyQt5.QtGui import QIntValidator, QColor, QFont
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView, \
	QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QComboBox, QGridLayout, QSizePolicy, QFrame, QScrollArea, QWidget, QLayout, QMessageBox, QProgressBar
import numpy as np

from ..widgets import get_icon, show_error
from ..widgets.file_server import FileServer_PathKey

__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'Spool Controller'                    # Main panel window title
SPOOL_CONTROLLER_TUNE_NAME = 'Spool Controller Tuning'  # Header label for the tuning section
PARAM_FILE_MANAGE_NAME = 'Parameter File Management'    # Label for the file upload/download section
DESIGN_CONSTANTS_TUNE_NAME = 'DesignConstantsSet Tuning'  # Label for the design constants section header
DESIGN_CONSTANTS_SET_NAME = 'DesignConstantsSet'    # Title of the design constants groupbox
PARAM_SET_EDIT_NAME = 'ParamSet Editing'            # Label for the ParamSet editing section
PARAM_SET_ID_NAME = 'ParamSet ID'                   # Label next to the ParamSet ID textbox
PARAM_SET_NAME = 'ParamSet'                         # Prefix for individual ParamSet groupbox titles

BUTTON_HORIZONTAL_SPACING = 3                       # Horizontal spacing (px) between buttons in button rows
PARAM_SET_GROUPBOX_HEIGHT = 200                     # Fixed height (px) for each ParamSet editing groupbox
PARAM_SET_GROUPBOX_WIDTH = 400                      # Fixed width (px) for each ParamSet editing groupbox
RESPONSE_TIMEOUT = 3                                # Seconds to wait for a response to a sent message before showing a timeout error dialog
CONFIG_FILE_TRANSFER_TIMEOUT = 30                   # Number of seconds to wait for a config file upload/download to complete before showing a timeout error dialog

_PARAM_SET_LIGHT_COLORS = [                         # Pool of light background colors assigned to ParamSet groupboxes
	'#FFFFCC',  # light yellow
	'#CCFFCC',  # light green
	'#CCE5FF',  # light blue
	'#FFCCCC',  # light red / pink
	'#E5CCFF',  # light purple
	'#FFDDCC',  # light orange
	'#CCFFFF',  # light cyan
	'#FFE5CC',  # light peach
	'#D5FFCC',  # light lime
	'#FFCCFF',  # light magenta
]

logger = getLogger(__name__)

_singleton = None


class FlowLayout(QLayout):
	"""Layout that arranges child widgets left-to-right, wrapping to the next row when out of space."""

	def __init__(self, parent=None, margin=5, hSpacing=8, vSpacing=8):
		super().__init__(parent)
		self._hSpacing = hSpacing
		self._vSpacing = vSpacing
		self._items = []
		self.setContentsMargins(margin, margin, margin, margin)

	def addItem(self, item):
		self._items.append(item)

	def count(self):
		return len(self._items)

	def itemAt(self, index):
		if 0 <= index < len(self._items):
			return self._items[index]
		return None

	def takeAt(self, index):
		if 0 <= index < len(self._items):
			return self._items.pop(index)
		return None

	def expandingDirections(self):
		return Qt.Orientations(0)

	def hasHeightForWidth(self):
		return True

	def heightForWidth(self, width):
		return self._doLayout(QRect(0, 0, width, 0), True)

	def setGeometry(self, rect):
		super().setGeometry(rect)
		self._doLayout(rect, False)

	def sizeHint(self):
		return self.minimumSize()

	def minimumSize(self):
		size = QSize()
		for item in self._items:
			size = size.expandedTo(item.minimumSize())
		m = self.contentsMargins()
		size += QSize(m.left() + m.right(), m.top() + m.bottom())
		return size

	def _doLayout(self, rect, testOnly):
		m = self.contentsMargins()
		effective = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
		x = effective.x()
		y = effective.y()
		lineHeight = 0

		for item in self._items:
			wid = item.widget()
			if wid is not None and wid.isHidden():
				continue

			itemSize = item.sizeHint()
			nextX = x + itemSize.width() + self._hSpacing
			if nextX - self._hSpacing > effective.right() + 1 and lineHeight > 0:
				x = effective.x()
				y += lineHeight + self._vSpacing
				nextX = x + itemSize.width() + self._hSpacing
				lineHeight = 0

			if not testOnly:
				item.setGeometry(QRect(QPoint(x, y), itemSize))

			x = nextX
			lineHeight = max(lineHeight, itemSize.height())

		return y + lineHeight - rect.y() + m.bottom()


class _FlowContainer(QWidget):
	"""Container widget that updates its minimum height from the FlowLayout when resized."""

	def resizeEvent(self, event):
		super().resizeEvent(event)
		layout = self.layout()
		if layout and hasattr(layout, 'heightForWidth'):
			h = layout.heightForWidth(self.width())
			if h >= 0:
				self.setMinimumHeight(h)


class SpoolControllerPanel(QDialog):
	def __init__(self, parent, node):
		super().__init__(parent)
		self.setWindowTitle(PANEL_NAME)
		self.setWindowIcon(get_icon())
		self.setAttribute(Qt.WA_DeleteOnClose)
		self.resize(900, 600)
		self.setMinimumSize(700, 400)

		self._node = node                      # Local DroneCAN node used for broadcasting messages and registering handlers
		self._param_set_id_list = []           # List of ParamSet IDs currently being edited
		self._param_set_color_map = {}         # param_set_id -> background color string assigned to its groupbox
		self._available_colors = list(_PARAM_SET_LIGHT_COLORS)  # Colors from the palette not currently assigned to any groupbox
		self._param_set_dirty = {}             # param_set_id -> bool indicating whether any field was edited since opening
		self._param_set_field_inputs = {}      # param_set_id -> {field_name: (QLineEdit, type_str)} for each ParamSet groupbox
		self._param_set_groupboxes = {}        # param_set_id -> QGroupBox widget for each ParamSet editing groupbox

		# Load the design constants definition file
		self._design_const_set_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'DesignConstantsSet.json')
		self._design_constants_fields = self._load_design_constants_fields()  # Parsed DesignConstantsSet.json field definitions
		# Load the param set definition file
		self._param_set_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'ParamSet.json')
		self._param_set_fields = self._load_param_set_fields()  # Parsed ParamSet.json field definitions

		self._recall_response_handle = None            # DroneCAN handler handle for DesignConstantsSet (active during recall)
		self._recall_constants_timeout_timer = None    # QTimer for DesignConstantsSet recall timeout

		self._recall_param_set_handle = None           # Shared DroneCAN handler for ParamSet responses (active when any ParamSet recall is pending)
		self._pending_param_set_recalls = {}           # param_set_id -> QTimer for each pending ParamSet recall
		self._pending_param_set_compare = {}           # param_set_id -> {field_name: value} for store-then-recall compare

		self._upload_response_handle = None            # DroneCAN handler handle for WriteConfigFile (active during upload)
		self._upload_timeout_timer = None              # QTimer for WriteConfigFile upload timeout
		self._download_response_handle = None          # DroneCAN handler handle for ReadConfigFile (active during download)
		self._download_timeout_timer = None            # QTimer for ReadConfigFile download timeout
		self._config_transfer_timer = None             # QTimer for config file transfer timeout
		self._config_transfer_key = None               # File server key used to track transfer activity
		self._config_transfer_start_hits = 0           # Hit count at transfer start

		self._setup_ui()

	def _setup_ui(self):
		'''
		@brief    Main UI setup function that creates the window layout.
		@return   None
		'''
		layout = QVBoxLayout(self)

		# Create groupbox with header labels and sub-groupboxes
		header_group = QGroupBox(self)
		header_layout = QVBoxLayout(header_group)

		# Top row: Spool Controller Tuning label
		spool_tune_label = QLabel(SPOOL_CONTROLLER_TUNE_NAME, header_group)
		font_main = QFont()
		font_main.setBold(True)
		font_main.setPointSize(12)
		spool_tune_label.setFont(font_main)
		header_layout.addWidget(spool_tune_label, 0, Qt.AlignTop)

		# Grid layout for two columns + ParamSet Editing below
		columns_grid = QGridLayout()
		columns_grid.setColumnStretch(0, 1)
		columns_grid.setColumnStretch(1, 1)

		# Row 0, Col 0: Left column - Parameter File Management section
		left_column = self._create_param_file_manage_section(header_group)
		columns_grid.addLayout(left_column, 0, 0)

		# Row 0, Col 1: Right column - Design Constants section
		right_column = self._create_design_constants_section(header_group)
		columns_grid.addLayout(right_column, 0, 1)

		# Row 1, spanning both columns: ParamSet Editing label
		param_set_edit_label = QLabel(PARAM_SET_EDIT_NAME, header_group)
		font_secondary = QFont()
		font_secondary.setBold(True)
		param_set_edit_label.setFont(font_secondary)
		columns_grid.addWidget(param_set_edit_label, 1, 0, 1, 2, Qt.AlignLeft)

		# Row 2, spanning both columns: sunken line
		param_set_line = QFrame(header_group)
		param_set_line.setFrameShape(QFrame.HLine)
		param_set_line.setFrameShadow(QFrame.Sunken)
		columns_grid.addWidget(param_set_line, 2, 0, 1, 2)

		# Row 3: ParamSet ID label, textbox, and Edit button
		param_set_id_row = QHBoxLayout()
		param_set_id_label = QLabel(PARAM_SET_ID_NAME + ':', header_group)
		param_set_id_row.addWidget(param_set_id_label)

		self._text_box_param_set_id = QLineEdit(header_group)
		self._text_box_param_set_id.setFixedWidth(120)
		param_set_id_row.addWidget(self._text_box_param_set_id)

		self._edit_button = QPushButton('Edit', header_group)
		self._edit_button.clicked.connect(self._on_edit_clicked)
		param_set_id_row.addWidget(self._edit_button)

		param_set_id_row.addStretch(1)
		columns_grid.addLayout(param_set_id_row, 3, 0, 1, 2)

		# Row 4: Scrollable area spanning full width for ParamSet editing content
		self._param_set_scroll_area = QScrollArea(header_group)
		self._param_set_scroll_area.setWidgetResizable(True)
		self._param_set_scroll_area.setFrameShape(QFrame.StyledPanel)
		self._param_set_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
		self._param_set_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

		self._param_set_container = _FlowContainer()
		self._param_set_container_layout = FlowLayout(self._param_set_container, margin=2, hSpacing=4, vSpacing=4)
		self._param_set_scroll_area.setWidget(self._param_set_container)

		columns_grid.addWidget(self._param_set_scroll_area, 4, 0, 1, 2)

		# Row 5: stretch to absorb remaining space
		columns_grid.setRowStretch(4, 1)

		header_layout.addLayout(columns_grid)

		layout.addWidget(header_group)

	def _create_param_file_manage_section(self, parent):
		'''
		@brief    Create the Parameter File Management section.
		@param    parent - Parent widget.
		@return   QVBoxLayout containing the section.
		'''
		left_column = QVBoxLayout()
		left_column.setContentsMargins(0, 0, 0, 0)
		left_column.setSpacing(6)

		font_secondary = QFont()
		font_secondary.setBold(True)
		param_file_manage_label = QLabel(PARAM_FILE_MANAGE_NAME, parent)
		param_file_manage_label.setFont(font_secondary)
		param_file_manage_label.setFixedHeight(20)
		left_column.addWidget(param_file_manage_label, 0, Qt.AlignTop)

		# Horizontal line below param_file_manage_label
		param_line = QFrame(parent)
		param_line.setFrameShape(QFrame.HLine)
		param_line.setFrameShadow(QFrame.Sunken)
		left_column.addWidget(param_line)

		BUTTON_WIDTH = 110

		upload_row = QHBoxLayout()
		self._upload_button = QPushButton('Upload', parent)
		self._upload_button.setFixedWidth(BUTTON_WIDTH)
		self._upload_button.clicked.connect(self._on_upload_clicked)
		upload_row.addWidget(self._upload_button)

		self._upload_textbox = QLineEdit(parent)
		upload_row.addWidget(self._upload_textbox)

		self._upload_browse_button = QPushButton('Browse', parent)
		self._upload_browse_button.setFixedWidth(BUTTON_WIDTH)
		self._upload_browse_button.clicked.connect(self._on_upload_browse_clicked)
		upload_row.addWidget(self._upload_browse_button)
		left_column.addLayout(upload_row)

		download_row = QHBoxLayout()
		self._download_button = QPushButton('Download', parent)
		self._download_button.setFixedWidth(BUTTON_WIDTH)
		self._download_button.clicked.connect(self._on_download_clicked)
		download_row.addWidget(self._download_button)

		self._download_textbox = QLineEdit(parent)
		download_row.addWidget(self._download_textbox)

		self._download_browse_button = QPushButton('Browse', parent)
		self._download_browse_button.setFixedWidth(BUTTON_WIDTH)
		self._download_browse_button.clicked.connect(self._on_download_browse_clicked)
		download_row.addWidget(self._download_browse_button)
		left_column.addLayout(download_row)

		STATUS_LABEL_WIDTH = 60
		STATUS_TEXTBOX_WIDTH = 80

		status_row = QHBoxLayout()
		status_row.setSpacing(0)

		version_label = QLabel('Version:', parent)
		version_label.setFixedWidth(STATUS_LABEL_WIDTH)
		status_row.addWidget(version_label)

		self._version_textbox = QLineEdit(parent)
		self._version_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		self._version_textbox.setReadOnly(True)
		status_row.addWidget(self._version_textbox)

		status_row.addSpacing(23)

		crc32_label = QLabel('CRC32:', parent)
		crc32_label.setFixedWidth(STATUS_LABEL_WIDTH)
		status_row.addWidget(crc32_label)

		self._crc32_textbox = QLineEdit(parent)
		self._crc32_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		self._crc32_textbox.setReadOnly(True)
		status_row.addWidget(self._crc32_textbox)

		status_row.addSpacing(23)

		dirty_label = QLabel('Dirty:', parent)
		dirty_label.setFixedWidth(STATUS_LABEL_WIDTH)
		status_row.addWidget(dirty_label)

		self._dirty_textbox = QLineEdit(parent)
		self._dirty_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		self._dirty_textbox.setReadOnly(True)
		status_row.addWidget(self._dirty_textbox)

		status_row.addStretch(1)

		left_column.addLayout(status_row)

		left_column.addStretch(1)

		progress_row = QHBoxLayout()
		progress_row.setSpacing(0)
		self._config_transfer_progress = QProgressBar(parent)
		self._config_transfer_progress.setRange(0, 100)
		self._config_transfer_progress.setValue(0)
		self._config_transfer_progress.setAlignment(Qt.AlignCenter)
		self._config_transfer_progress.setFixedWidth(
			(STATUS_LABEL_WIDTH * 3) + (STATUS_TEXTBOX_WIDTH * 3) + 46
		)
		progress_row.addWidget(self._config_transfer_progress)
		progress_row.addStretch(1)
		left_column.addLayout(progress_row)

		left_column.addSpacing(2)

		return left_column

	def _on_upload_browse_clicked(self):
		'''
		@brief    Handle upload browse button click to select a file.
		@return   None
		'''
		filename, _ = QFileDialog.getOpenFileName(
			self,
			'Select file to upload',
			'',
			'All files (*.*)'
		)
		if filename:
			self._upload_textbox.setText(filename)

	def _on_upload_clicked(self):
		'''
		@brief    Handle upload button click: validate local file, configure file server, and send WriteConfigFile.
		@return   None
		'''
		upload_path = self._upload_textbox.text().strip()
		if not upload_path or not os.path.isfile(upload_path):
			self._show_ok_dialog('Upload', 'Choose a file to upload!')
			return

		self._cleanup_config_transfer_timeout()

		# Get the file server widget from the main window
		try:
			file_server_widget = self._get_file_server_widget()
			if file_server_widget is None:
				show_error('File Server Error', 'File server widget not available.', '', parent=self, blocking=True)
				return
		except Exception as ex:
			logger.exception('Could not access file server widget: %s', ex)
			show_error('File Server Error', 'Could not access file server.', str(ex), parent=self, blocking=True)
			return

		# Add the file to the file server
		try:
			file_server_widget.add_path(upload_path)
			file_server_widget.force_start()
			logger.info('File server configured for: %s', upload_path)
		except Exception as ex:
			logger.exception('Could not configure file server: %s', ex)
			show_error('File Server Error', 'Could not configure file server.', str(ex), parent=self, blocking=True)
			return

		# Get the remote path that the file server will use
		remote_config_file = FileServer_PathKey(upload_path)
		logger.info('Remote config file path: %r', remote_config_file)
		self._config_transfer_key = remote_config_file

		# Create and send WriteConfigFile message
		try:
			msg = dronecan.flytrex.delcon.WriteConfigFile()
			msg.destination_node_id = 0
			msg.image_file_remote_path.path = remote_config_file
		except Exception as ex:
			logger.exception('WriteConfigFile DSDL type not available: %s', ex)
			show_error(
				'DSDL type not loaded',
				'Could not access dronecan.flytrex.delcon.WriteConfigFile.',
				str(ex),
				parent=self,
				blocking=True,
			)
			return

		# Clean up any previous upload handler
		self._cleanup_upload_handler()

		# Disable the upload button while waiting for response
		self._upload_button.setEnabled(False)

		try:
			self._node.broadcast(msg)
			logger.info('Broadcast WriteConfigFile for %s', remote_config_file)
		except Exception as ex:
			logger.exception('Failed to broadcast WriteConfigFile: %s', ex)
			show_error('Broadcast failed', 'Could not broadcast WriteConfigFile.', str(ex), parent=self, blocking=True)
			self._upload_button.setEnabled(True)
			return

		# Register handler for the response message
		try:
			self._upload_response_handle = self._node.add_handler(
				dronecan.flytrex.delcon.WriteConfigFile,
				self._on_upload_response,
			)
		except Exception as ex:
			logger.exception('Could not register WriteConfigFile handler: %s', ex)
			self._upload_button.setEnabled(True)
			self._cleanup_upload_handler()
			return

		# Start timeout timer
		self._upload_timeout_timer = QTimer(self)
		self._upload_timeout_timer.setSingleShot(True)
		self._upload_timeout_timer.timeout.connect(
			lambda: (
				self._cleanup_upload_handler(),
				self._on_recall_timeout(
					f'No WriteConfigFile response was received within {RESPONSE_TIMEOUT} seconds.\n\n'
					f'The spool controller may be offline or not responding.'
				),
			)
		)
		self._upload_timeout_timer.start(RESPONSE_TIMEOUT * 1000)

		self._show_message('Upload request sent. Waiting for spool controller response...')

	def _on_download_clicked(self):
		'''
		@brief    Handle download button click: validate destination path and send ReadConfigFile.
		@return   None
		'''
		download_path = self._download_textbox.text().strip()
		if not download_path:
			self._show_ok_dialog('Download', 'Choose a file to download!')
			return

		download_dir = os.path.dirname(download_path)
		if download_dir and not os.path.isdir(download_dir):
			self._show_ok_dialog('Download', 'Choose a valid download directory!')
			return

		try:
			msg = dronecan.flytrex.delcon.ReadConfigFile()
			msg.destination_node_id = 0
		except Exception as ex:
			logger.exception('ReadConfigFile DSDL type not available: %s', ex)
			show_error(
				'DSDL type not loaded',
				'Could not access dronecan.flytrex.delcon.ReadConfigFile.',
				str(ex),
				parent=self,
				blocking=True,
			)
			return

		# Clean up any previous download handler
		self._cleanup_download_handler()

		# Disable the download button while waiting for response
		self._download_button.setEnabled(False)

		try:
			self._node.broadcast(msg)
			logger.info('Broadcast ReadConfigFile for %s', download_path)
		except Exception as ex:
			logger.exception('Failed to broadcast ReadConfigFile: %s', ex)
			show_error('Broadcast failed', 'Could not broadcast ReadConfigFile.', str(ex), parent=self, blocking=True)
			self._download_button.setEnabled(True)
			return

		# Register handler for the response message
		try:
			self._download_response_handle = self._node.add_handler(
				dronecan.flytrex.delcon.ReadConfigFile,
				self._on_download_response,
			)
		except Exception as ex:
			logger.exception('Could not register ReadConfigFile handler: %s', ex)
			self._download_button.setEnabled(True)
			self._cleanup_download_handler()
			return

		# Start timeout timer
		self._download_timeout_timer = QTimer(self)
		self._download_timeout_timer.setSingleShot(True)
		self._download_timeout_timer.timeout.connect(
			lambda: (
				self._cleanup_download_handler(),
				self._on_recall_timeout(
					f'No ReadConfigFile response was received within {RESPONSE_TIMEOUT} seconds.\n\n'
					f'The spool controller may be offline or not responding.'
				),
			)
		)
		self._download_timeout_timer.start(RESPONSE_TIMEOUT * 1000)

	def _on_edit_clicked(self):
		'''
		@brief    Handle Edit button click for ParamSet editing.
		@return   None
		'''
		param_set_id = self._text_box_param_set_id.text().strip()
		if not param_set_id:
			show_error('Edit Error', 'Please enter a ParamSet ID.', '', parent=self, blocking=True)
			return
		logger.info('Edit clicked for ParamSet ID: %s', param_set_id)
		self._add_param_set_editing_content(param_set_id)

	def _add_param_set_editing_content(self, param_set_id):
		'''
		@brief    Add content to the ParamSet editing area based on the given ID.
		@param    param_set_id - The ID of the ParamSet to edit.
		@return   None
		'''

		# Check if this ID is already being edited
		if param_set_id in self._param_set_id_list:
			# Show a popup message that this ID is already being edited
			show_error('Edit Error', f'ParamSet with ID "{param_set_id}" is already being edited.', '', parent=self, blocking=True)
			return

		# Register this ID as being edited
		self._param_set_id_list.append(param_set_id)
		self._param_set_dirty[param_set_id] = False
		self._add_param_set_editing_groupbox(param_set_id)

	def _add_param_set_editing_groupbox(self, param_set_id):
		'''
		@brief    Create and add a groupbox for editing a specific ParamSet ID.
		@param    param_set_id - The ID of the ParamSet to edit.
		@return   None
		'''

		groupbox = QGroupBox(f'{PARAM_SET_NAME} {param_set_id}', self._param_set_container)
		self._param_set_groupboxes[param_set_id] = groupbox
		groupbox.setFixedHeight(PARAM_SET_GROUPBOX_HEIGHT)
		groupbox.setFixedWidth(PARAM_SET_GROUPBOX_WIDTH)

		# Pick a unique light background color
		if self._available_colors:
			color = random.choice(self._available_colors)
			self._available_colors.remove(color)
		else:
			# All colors in use — pick a random one from the full palette
			color = random.choice(_PARAM_SET_LIGHT_COLORS)
		self._param_set_color_map[param_set_id] = color

		groupbox.setStyleSheet(f"""
			QGroupBox {{
				border: 1px solid gray;
				border-radius: 3px;
				margin-top: 0px;
				padding-top: 15px;
				background-color: {color};
			}}
			QGroupBox::title {{
				subcontrol-origin: margin;
				subcontrol-position: top left;
				padding: 2px 5px;
				background-color: palette(window);
				border: 1px solid gray;
				top: 0px;
				left: 0px;
			}}
		""")

		# Layout for groupbox
		groupbox_layout = QGridLayout(groupbox)
		groupbox_layout.setColumnStretch(0, 1)
		groupbox_layout.setColumnStretch(1, 0)
		groupbox_layout.setSpacing(10)
		groupbox_layout.setContentsMargins(0, 10, 5, 5)
		groupbox_layout.setRowMinimumHeight(0, 30)

		# Buttons row (fixed at top)
		buttons_layout = QHBoxLayout()
		buttons_layout.setSpacing(BUTTON_HORIZONTAL_SPACING)
		buttons_layout.addStretch(1)

		execute_button = QPushButton('Execute', groupbox)
		execute_button.clicked.connect(lambda: self._on_param_set_execute(param_set_id))
		buttons_layout.addWidget(execute_button)

		store_button = QPushButton('Store', groupbox)
		store_button.clicked.connect(lambda: self._on_param_set_store(param_set_id))
		buttons_layout.addWidget(store_button)

		recall_button = QPushButton('Recall', groupbox)
		recall_button.clicked.connect(lambda: self._on_param_set_recall(param_set_id))
		buttons_layout.addWidget(recall_button)

		close_button = QPushButton('Close', groupbox)
		close_button.clicked.connect(lambda: self._on_param_set_groupbox_close(param_set_id, groupbox))
		buttons_layout.addWidget(close_button)

		groupbox_layout.addLayout(buttons_layout, 0, 0)

		# X button at top-right corner (absolute positioning over the groupbox)
		close_x_button = QPushButton('\u2715', groupbox)
		close_x_button.setFixedSize(20, 20)
		close_x_button.setStyleSheet("""
			QPushButton {
				background-color: palette(window);
				border: 1px solid gray;
				font-weight: bold;
				color: red;
				font-size: 12px;
			}
			QPushButton:hover {
				background-color: #ffcccc;
			}
		""")
		close_x_button.clicked.connect(lambda: self._on_param_set_groupbox_close(param_set_id, groupbox))
		close_x_button.move(groupbox.width() - 20, 0)
		close_x_button.raise_()

		# Re-position the X button when the groupbox is resized
		groupbox.resizeEvent = lambda event, btn=close_x_button, gb=groupbox: (
			btn.move(gb.width() - 20, 0),
			type(gb).__base__.resizeEvent(gb, event)
		)

		# Scrollable area for ParamSet fields
		param_set_scroll = QScrollArea(groupbox)
		param_set_scroll.setWidgetResizable(True)
		param_set_scroll.setFrameShape(QFrame.NoFrame)
		param_set_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

		param_set_fields_container = QWidget()
		param_set_fields_container.setStyleSheet(f"background-color: {color};")
		param_set_fields_layout = QGridLayout(param_set_fields_container)
		param_set_fields_layout.setColumnStretch(0, 0)
		param_set_fields_layout.setColumnStretch(1, 1)
		param_set_fields_layout.setSpacing(5)
		param_set_fields_layout.setContentsMargins(5, 0, 5, 0)

		self._parse_param_set_file(param_set_id, param_set_fields_container, param_set_fields_layout)

		# Mark groupbox dirty when any field textbox is edited
		for textbox in param_set_fields_container.findChildren(QLineEdit):
			textbox.textChanged.connect(lambda _text, _id=param_set_id: self._param_set_dirty.__setitem__(_id, True))

		param_set_scroll.setWidget(param_set_fields_container)
		groupbox_layout.addWidget(param_set_scroll, 1, 0, 1, 2)
		groupbox_layout.setRowStretch(1, 1)

		self._param_set_container_layout.addWidget(groupbox)

	def _on_param_set_groupbox_close(self, param_set_id, groupbox):
		'''
		@brief    Remove a ParamSet editing groupbox and deregister the ID.
		@param    param_set_id - The ID of the ParamSet to close.
		@param    groupbox - The QGroupBox widget to remove.
		@return   None
		'''
		# Prompt if any field was edited
		if self._param_set_dirty.get(param_set_id, False):
			result = QMessageBox.question(
				self,
				'Close ParamSet',
				'A change was made to this ParamSet. Do you want to close the window anyway?',
				QMessageBox.Yes | QMessageBox.No,
				QMessageBox.No,
			)
			if result != QMessageBox.Yes:
				return

		self._param_set_dirty.pop(param_set_id, None)
		self._param_set_field_inputs.pop(param_set_id, None)
		self._param_set_groupboxes.pop(param_set_id, None)
		if param_set_id in self._param_set_id_list:
			self._param_set_id_list.remove(param_set_id)
		# Return the color to the available pool
		used_color = self._param_set_color_map.pop(param_set_id, None)
		if used_color and used_color in _PARAM_SET_LIGHT_COLORS and used_color not in self._available_colors:
			self._available_colors.append(used_color)
		self._param_set_container_layout.removeWidget(groupbox)
		groupbox.deleteLater()

	@staticmethod
	def _parse_value(field_type, raw_value):
		'''
		@brief    Parse a raw string value into the appropriate Python type based on field_type.
		@param    field_type - Type string (e.g. 'float32', 'uint16', 'bool').
		@param    raw_value - The raw string from the textbox.
		@return   Parsed value.
		'''
		ft = (field_type or '').strip()
		raw = '' if raw_value is None else str(raw_value).strip()

		if ft == 'bool':
			v = raw.lower()
			if v in ('1', 'true', 'yes', 'y', 'on'):
				return True
			if v in ('0', 'false', 'no', 'n', 'off', ''):
				return False
			raise ValueError(f'Invalid bool: {raw!r}')

		if ft.startswith('float') or ft in ('float', 'double'):
			return 0.0 if raw == '' else float(raw)

		if re.fullmatch(r'(u?int)(\d+)', ft) or re.fullmatch(r'(u?int)(\d+)_t', ft):
			return 0 if raw == '' else int(raw, 0)

		# Fallback: try int then float
		if raw == '':
			return 0
		try:
			return int(raw, 0)
		except Exception:
			return float(raw)

	def _send_param_set_msg(self, param_set_id, operation_name):
		'''
		@brief    Helper function to send a ParamSet message with the given operation.
		@param    param_set_id - The ID of the ParamSet.
		@param    operation_name - The DSDL constant name (e.g. 'OPERATION_EXECUTE').
		@return   True if the message was broadcast successfully.
		'''
		field_inputs = self._param_set_field_inputs.get(param_set_id)
		if not field_inputs:
			show_error('Error', 'No fields found for this ParamSet.', '', parent=self, blocking=True)
			return False

		try:
			msg = dronecan.flytrex.delcon.ParamSet()
		except Exception as ex:
			logger.exception('ParamSet DSDL type not available: %s', ex)
			show_error(
				'DSDL type not loaded',
				'Could not access dronecan.flytrex.delcon.ParamSet.',
				str(ex),
				parent=self,
				blocking=True,
			)
			return False

		operation = getattr(msg, operation_name)
		msg.operation = operation

		try:
			msg.param_id = int(param_set_id)
		except ValueError:
			show_error('Error', f'ParamSet ID "{param_set_id}" is not a valid integer.', '', parent=self, blocking=True)
			return False

		# Only set the fields if operation is EXECUTE or STORE (not RECALL)
		if operation_name == 'OPERATION_RECALL':
			msg.param_values = []
			msg.param_value_types = []
			for field_name, (textbox, field_type) in field_inputs.items():
				if not hasattr(msg, field_name):
					logger.warning('Field "%s" not found on ParamSet message, skipping', field_name)
					continue
				msg.param_values.append(0)  # placeholder value for recall
				msg.param_value_types.append(field_type)

		else:
			for field_name, (textbox, field_type) in field_inputs.items():
				raw_value = textbox.text().strip()
				if not hasattr(msg, field_name):
					logger.warning('Field "%s" not found on ParamSet message, skipping', field_name)
					continue
				try:
					value = self._parse_value(field_type, raw_value)
					setattr(msg, field_name, value)
				except Exception as ex:
					show_error(
						'Invalid field value',
						f'Could not parse field "{field_name}".',
						f'Type: {field_type}\nValue: {raw_value}\nError: {ex}',
						parent=self,
						blocking=True,
					)
					return False

		try:
			self._node.broadcast(msg)
			logger.info('Broadcast ParamSet with operation %s for param_id=%s', operation, param_set_id)
		except Exception as ex:
			logger.exception('Failed to broadcast ParamSet: %s', ex)
			show_error('Broadcast failed', 'Could not broadcast ParamSet.', str(ex), parent=self, blocking=True)
			return False

		return True

	def _capture_param_set_values(self, param_set_id):
		'''
		@brief    Capture current ParamSet UI values for comparison after a recall.
		@param    param_set_id - The ParamSet ID to capture.
		@return   Dict of field values or None if capture failed.
		'''
		field_inputs = self._param_set_field_inputs.get(param_set_id)
		if not field_inputs:
			show_error('Error', 'No fields found for this ParamSet.', '', parent=self, blocking=True)
			return None

		snapshot = {}
		for field_name, (textbox, field_type) in field_inputs.items():
			raw_value = textbox.text().strip()
			try:
				snapshot[field_name] = self._parse_value(field_type, raw_value)
			except Exception as ex:
				show_error(
					'Invalid field value',
					f'Could not parse field "{field_name}" for compare.',
					f'Type: {field_type}\nValue: {raw_value}\nError: {ex}',
					parent=self,
					blocking=True,
				)
				return None

		return snapshot

	@staticmethod
	def _values_match(left_value, right_value):
		'''
		@brief    Compare values with a small tolerance for floats.
		@param    left_value - First value.
		@param    right_value - Second value.
		@return   True if values match.
		'''
		if isinstance(left_value, float) or isinstance(right_value, float):
			try:
				return abs(float(left_value) - float(right_value)) <= 1e-6
			except Exception:
				return False
		return left_value == right_value

	def _show_ok_dialog(self, title, message):
		'''
		@brief    Show a warning dialog with a title, message, and a single OK button.
		@param    title - Dialog window title.
		@param    message - Dialog body text.
		@return   None
		'''
		dlg = QMessageBox(self)
		dlg.setIcon(QMessageBox.Warning)
		dlg.setWindowTitle(str(title))
		dlg.setText(str(message))
		dlg.setStandardButtons(QMessageBox.Ok)
		dlg.exec_()

	def _show_store_failed(self):
		'''
		@brief    Show a warning dialog when stored values do not match recalled values.
		@return   None
		'''
		self._show_ok_dialog('Store Failed', "Store didn't work!")


	def _on_param_set_execute(self, param_set_id):
		'''
		@brief    Handle Execute button click: broadcast a flytrex.delcon.ParamSet message with OPERATION_EXECUTE.
		@param    param_set_id - The ParamSet ID to execute.
		@return   None
		'''

		self._send_param_set_msg(param_set_id, 'OPERATION_EXECUTE')

	def _on_param_set_store(self, param_set_id):
		'''
		@brief    Handle Store button click: broadcast a flytrex.delcon.ParamSet message with OPERATION_STORE.
		@param    param_set_id - The ParamSet ID to store.
		@return   None
		'''
		snapshot = self._capture_param_set_values(param_set_id)
		if snapshot is None:
			return

		if not self._send_param_set_msg(param_set_id, 'OPERATION_STORE'):
			return

		self._pending_param_set_compare[param_set_id] = snapshot
		self._on_param_set_recall(param_set_id)

	def _on_param_set_recall(self, param_set_id):
		'''
		@brief    Handle Recall button click: broadcast a flytrex.delcon.ParamSet message with OPERATION_RECALL
		          and register a listener for ParamSet OPERATION_RESPONSE.
		          Each groupbox recall is independent — multiple recalls can be pending simultaneously.
		@param    param_set_id - The ParamSet ID to recall.
		@return   None
		'''
		# If this param_set_id already has a pending recall, clean it up first
		self._cleanup_param_set_recall(param_set_id)

		# Disable the groupbox while waiting for the response
		groupbox = self._param_set_groupboxes.get(param_set_id)
		if groupbox is not None:
			groupbox.setEnabled(False)

		self._send_param_set_msg(param_set_id, 'OPERATION_RECALL')

		# Register the shared handler if this is the first pending recall
		if not self._pending_param_set_recalls and self._recall_param_set_handle is None:
			try:
				self._recall_param_set_handle = self._node.add_handler(
					dronecan.flytrex.delcon.ParamSet,
					self._on_param_set_response,
				)
			except Exception as ex:
				logger.exception('Could not register ParamSet handler: %s', ex)
				if groupbox is not None:
					groupbox.setEnabled(True)
				return

		# Start a per-ID timeout timer
		timer = QTimer(self)
		timer.setSingleShot(True)
		timer.timeout.connect(
			lambda _id=param_set_id: (
				self._cleanup_param_set_recall(_id),
				self._on_recall_timeout(
					f'No ParamSet OPERATION_RESPONSE for ParamSet ID {_id} was received within {RESPONSE_TIMEOUT} seconds.\n\n'
					f'The spool controller may be offline or not responding.'
				),
			)
		)
		self._pending_param_set_recalls[param_set_id] = timer
		timer.start(RESPONSE_TIMEOUT * 1000)

	def _on_design_constants_recall(self):
		'''
		@brief    Handle Recall button click: broadcast a flytrex.delcon.DesignConstantsSet message with OPERATION_RECALL
		          and register a 3-second listener for DesignConstantsSet.
		@return   None
		'''
		# Clean up any previous recall listener
		self._cleanup_recall_handler()

		# Disable the groupbox while waiting for the report
		if self._design_const_set_group is not None:
			self._design_const_set_group.setEnabled(False)

		try:
			msg = dronecan.flytrex.delcon.DesignConstantsSet()
		except Exception as ex:
			logger.exception('DesignConstantsSet DSDL type not available: %s', ex)
			show_error(
				'DSDL type not loaded',
				'Could not access dronecan.flytrex.delcon.DesignConstantsSet.',
				str(ex),
				parent=self,
				blocking=True,
			)
			# Clean up any previous recall listener
			self._cleanup_recall_handler()
			return

		msg.operation = msg.OPERATION_RECALL

		try:
			self._node.broadcast(msg)
			logger.info('Broadcast DesignConstantsSet OPERATION_RECALL')
		except Exception as ex:
			logger.exception('Failed to broadcast DesignConstantsSet: %s', ex)
			show_error('Broadcast failed', 'Could not broadcast DesignConstantsSet.', str(ex), parent=self, blocking=True)
			# Clean up any previous recall listener
			self._cleanup_recall_handler()
			return

		# Register handler for the report message
		try:
			self._recall_response_handle = self._node.add_handler(
				dronecan.flytrex.delcon.DesignConstantsSet,
				self._on_design_constants,
			)
		except Exception as ex:
			logger.exception('Could not register DesignConstants handler: %s', ex)
			# Clean up any previous recall listener
			self._cleanup_recall_handler()
			return

		# Start a 3-second timeout timer
		self._recall_constants_timeout_timer = QTimer(self)
		self._recall_constants_timeout_timer.setSingleShot(True)
		self._recall_constants_timeout_timer.timeout.connect(
			lambda: (
				self._cleanup_recall_handler(),
				self._on_recall_timeout(
					f'No DesignConstantsSet OPERATION_RESPONSE was received within {RESPONSE_TIMEOUT} seconds.\n\n'
					f'The spool controller may be offline or not responding.'
				),
			)
		)
		self._recall_constants_timeout_timer.start(RESPONSE_TIMEOUT * 1000)

	def _on_design_constants(self, event):
		'''
		@brief    Handle an incoming DesignConstantsSet message and populate
		          the design constants fields in the UI.
		@param    event - DroneCAN transfer event containing the report message.
		@return   None
		'''
		self._cleanup_recall_handler()

		msg = event.message
		# Check that the 'operation' field is OPERATION_RESPONSE
		if msg.operation != msg.OPERATION_RESPONSE:
			logger.warning('DesignConstantsSet received with unexpected operation: %s', msg.operation)
			return

		for field_name, textbox in self._field_inputs.items():
			value = getattr(msg, field_name, None)
			if value is not None:
				textbox.setText(str(value))
		logger.info('DesignConstantsSet received — fields populated')

	def _on_param_set_response(self, event):
		'''
		@brief    Handle an incoming ParamSet message with OPERATION_RESPONSE and populate
		          the ParamSet fields in the UI. Matches the response to the correct
		          pending recall by msg.param_id.
		@param    event - DroneCAN transfer event containing the ParamSet message.
		@return   None
		'''
		msg = event.message
		# Check that the 'operation' field is OPERATION_RESPONSE
		if msg.operation != msg.OPERATION_RESPONSE:
			return

		param_set_id = str(msg.param_id)
		if param_set_id not in self._pending_param_set_recalls:
			logger.warning('ParamSet response for param_id=%s but no pending recall', param_set_id)
			return

		compare_snapshot = self._pending_param_set_compare.get(param_set_id)
		self._cleanup_param_set_recall(param_set_id)

		field_inputs = self._param_set_field_inputs.get(param_set_id)
		if not field_inputs:
			logger.warning('ParamSet response received but no field inputs found for ParamSet ID %s', param_set_id)
			return

		if compare_snapshot is not None:
			for field_name, (textbox, field_type) in field_inputs.items():
				if not hasattr(msg, field_name):
					continue
				recalled_value = getattr(msg, field_name, None)
				if recalled_value is None:
					continue
				stored_value = compare_snapshot.get(field_name)
				if not self._values_match(stored_value, recalled_value):
					self._show_store_failed()
					logger.warning('Store compare failed for ParamSet %s field %s', param_set_id, field_name)
					return
			logger.info('Store compare matched for ParamSet %s', param_set_id)
			return

		for field_name, (textbox, field_type) in field_inputs.items():
			value = getattr(msg, field_name, None)
			if value is not None:
				textbox.setText(str(value))
		logger.info('ParamSet OPERATION_RESPONSE received — ParamSet %s fields populated', param_set_id)

	def _on_recall_timeout(self, message):
		'''
		@brief    Show a recall timeout warning dialog. The caller is responsible
		          for invoking the appropriate cleanup handler before calling this.
		@param    message - The text to display in the warning dialog.
		@return   None
		'''
		logger.warning('Recall timeout: %s', message)
		self._show_ok_dialog('Recall Timeout', message)

	def _on_upload_response(self, event):
		'''
		@brief    Handle an incoming WriteConfigFile response message.
		          The spool controller will automatically request the file from the file server.
		@param    event - DroneCAN transfer event containing the response message.
		@return   None
		'''
		self._cleanup_upload_handler()

		msg = event.message
		logger.info('WriteConfigFile response received: status=%s', msg.status)

		try:
			if msg.status == msg.STATUS_OK:
				logger.info('Upload successful. Spool controller is reading the config file.')
				self._start_config_transfer_timeout()
				self._show_ok_dialog('Upload Complete', 'Config file upload request accepted. The spool controller is reading the file.')
			elif msg.status == msg.STATUS_BUSY:
				self._show_ok_dialog('Upload Status', 'The spool controller is busy. Please try again later.')
			elif msg.status == msg.STATUS_LOW_MEM:
				self._show_ok_dialog('Upload Status', 'The spool controller has low memory. Please try again later.')
			elif msg.status == msg.STATUS_UNKNOWN_ERR:
				self._show_ok_dialog('Upload Error', 'An unknown error occurred during upload.')
			else:
				self._show_ok_dialog('Upload Error', f'Upload failed with status code: {msg.status}')
		except Exception as ex:
			logger.exception('Error processing upload response: %s', ex)
			self._show_ok_dialog('Upload Error', f'Error processing upload response: {ex}')

	def _start_config_transfer_timeout(self):
		'''
		@brief    Start a timeout for config file transfer activity.
		@return   None
		'''
		self._cleanup_config_transfer_timeout()
		key = self._config_transfer_key
		if not key:
			return

		self._config_transfer_start_hits = 0
		try:
			file_server_widget = self._get_file_server_widget()
			file_server = getattr(file_server_widget, '_file_server', None)
			if file_server is not None:
				self._config_transfer_start_hits = file_server.path_hit_counters.get(key, 0)
		except Exception:
			logger.exception('Could not read file server hit counters')

		self._config_transfer_timer = QTimer(self)
		self._config_transfer_timer.setSingleShot(True)
		self._config_transfer_timer.timeout.connect(self._on_config_transfer_timeout)
		self._config_transfer_timer.start(CONFIG_FILE_TRANSFER_TIMEOUT * 1000)

	def _on_config_transfer_timeout(self):
		'''
		@brief    Handle config file transfer timeout.
		@return   None
		'''
		hits = self._config_transfer_start_hits
		try:
			file_server_widget = self._get_file_server_widget()
			file_server = getattr(file_server_widget, '_file_server', None)
			if file_server is not None and self._config_transfer_key:
				hits = file_server.path_hit_counters.get(self._config_transfer_key, hits)
		except Exception:
			logger.exception('Could not read file server hit counters')

		message = (
			f'Config file transfer did not complete within {CONFIG_FILE_TRANSFER_TIMEOUT} seconds.'
		)
		if hits <= self._config_transfer_start_hits:
			message += '\n\nNo file read activity was observed from the spool controller.'

		self._show_ok_dialog('Transfer Timeout', message)
		self._cleanup_config_transfer_timeout()

	def _cleanup_config_transfer_timeout(self):
		'''
		@brief    Stop the config file transfer timeout timer and reset state.
		@return   None
		'''
		if self._config_transfer_timer is not None:
			self._config_transfer_timer.stop()
			self._config_transfer_timer = None
		self._config_transfer_key = None
		self._config_transfer_start_hits = 0

	def _get_file_server_widget(self):
		'''
		@brief    Walk parent chain to find the main window file server widget.
		@return   FileServerWidget or None.
		'''
		parent = self.parent()
		while parent is not None:
			if hasattr(parent, '_file_server_widget'):
				return parent._file_server_widget
			parent = parent.parent()
		return None

	def _show_message(self, text, *fmt):
		'''
		@brief    Send a status message to the main window if available.
		@return   None
		'''
		parent = self.parent()
		while parent is not None:
			if hasattr(parent, 'show_message'):
				try:
					parent.show_message(text, *fmt)
					return
				except Exception:
					break
			parent = parent.parent()
		try:
			logger.info(text, *fmt)
		except Exception:
			pass

	def _cleanup_upload_handler(self):
		'''
		@brief    Remove the WriteConfigFile handler and stop the timeout timer.
		@return   None
		'''
		if self._upload_timeout_timer is not None:
			self._upload_timeout_timer.stop()
			self._upload_timeout_timer = None
		if self._upload_response_handle is not None:
			try:
				self._upload_response_handle.remove()
			except Exception:
				pass
			self._upload_response_handle = None
		if self._upload_button is not None:
			self._upload_button.setEnabled(True)

	def _cleanup_recall_handler(self):
		'''
		@brief    Remove the DesignConstantsSet handler and stop the timeout timer.
		@return   None
		'''
		if self._recall_constants_timeout_timer is not None:
			self._recall_constants_timeout_timer.stop()
			self._recall_constants_timeout_timer = None
		if self._recall_response_handle is not None:
			try:
				self._recall_response_handle.remove()
			except Exception:
				pass
			self._recall_response_handle = None
		if self._design_const_set_group is not None:
			self._design_const_set_group.setEnabled(True)

	def _cleanup_param_set_recall(self, param_set_id):
		'''
		@brief    Clean up a single pending ParamSet recall: stop its timer, re-enable
		          its groupbox, and remove the shared handler if no recalls remain.
		@param    param_set_id - The ParamSet ID whose recall is being cleaned up.
		@return   None
		'''
		timer = self._pending_param_set_recalls.pop(param_set_id, None)
		if timer is not None:
			timer.stop()
		self._pending_param_set_compare.pop(param_set_id, None)
		# Re-enable the groupbox
		groupbox = self._param_set_groupboxes.get(param_set_id)
		if groupbox is not None:
			groupbox.setEnabled(True)
		# Remove the shared handler if no more pending recalls
		if not self._pending_param_set_recalls and self._recall_param_set_handle is not None:
			try:
				self._recall_param_set_handle.remove()
			except Exception:
				pass
			self._recall_param_set_handle = None

	def _cleanup_param_set_recall_handler(self):
		'''
		@brief    Clean up all pending ParamSet recalls. Used during panel shutdown.
		@return   None
		'''
		for pid in list(self._pending_param_set_recalls):
			self._cleanup_param_set_recall(pid)

	def _on_design_constants_store(self):
		'''
		@brief    Handle Store button click: broadcast a flytrex.delcon.DesignConstantsSet message with OPERATION_STORE.
		@return   None
		'''
		if not self._field_inputs:
			show_error('Store Error', 'No design constant fields found.', '', parent=self, blocking=True)
			return

		try:
			msg = dronecan.flytrex.delcon.DesignConstantsSet()
		except Exception as ex:
			logger.exception('DesignConstantsSet DSDL type not available: %s', ex)
			show_error(
				'DSDL type not loaded',
				'Could not access dronecan.flytrex.delcon.DesignConstantsSet.',
				str(ex),
				parent=self,
				blocking=True,
			)
			return

		msg.operation = msg.OPERATION_STORE

		for field_name, textbox in self._field_inputs.items():
			raw_value = textbox.text().strip()
			if not hasattr(msg, field_name):
				logger.warning('Field "%s" not found on DesignConstantsSet message, skipping', field_name)
				continue
			field_type = self._design_constants_fields.get(field_name, {}).get('type', 'float32')
			try:
				value = self._parse_value(field_type, raw_value)
				setattr(msg, field_name, value)
			except Exception as ex:
				show_error(
					'Invalid field value',
					f'Could not parse field "{field_name}".',
					f'Type: {field_type}\nValue: {raw_value}\nError: {ex}',
					parent=self,
					blocking=True,
				)
				return

		try:
			self._node.broadcast(msg)
			logger.info('Broadcast DesignConstantsSet OPERATION_STORE')
		except Exception as ex:
			logger.exception('Failed to broadcast DesignConstantsSet: %s', ex)
			show_error('Broadcast failed', 'Could not broadcast DesignConstantsSet.', str(ex), parent=self, blocking=True)

	def _on_download_browse_clicked(self):
		'''
		@brief    Handle download browse button click to select a save destination.
		@return   None
		'''
		dialog = QFileDialog(self)
		dialog.setWindowTitle('Select file to download')
		dialog.setAcceptMode(QFileDialog.AcceptSave)
		dialog.setFileMode(QFileDialog.AnyFile)
		dialog.setNameFilter('All files (*.*)')

		initial_path = self._download_textbox.text().strip()
		if initial_path:
			dialog.selectFile(initial_path)

		if dialog.exec_():
			selected_files = dialog.selectedFiles()
			if selected_files:
				self._download_textbox.setText(selected_files[0])

	def _on_download_response(self, event):
		'''
		@brief    Handle an incoming ReadConfigFile response message.
		@param    event - DroneCAN transfer event containing the response message.
		@return   None
		'''
		self._cleanup_download_handler()

		msg = event.message
		logger.info('ReadConfigFile response received: status=%s', msg.status)

		try:
			if msg.status == msg.STATUS_OK:
				self._show_ok_dialog('Download Started', 'ReadConfigFile request accepted.')
			elif msg.status == msg.STATUS_BUSY:
				self._show_ok_dialog('Download Status', 'The spool controller is busy. Please try again later.')
			elif msg.status == msg.STATUS_LOW_MEM:
				self._show_ok_dialog('Download Status', 'The spool controller has low memory. Please try again later.')
			elif msg.status == msg.STATUS_UNKNOWN_ERR:
				self._show_ok_dialog('Download Error', 'An unknown error occurred during download.')
			else:
				self._show_ok_dialog('Download Error', f'Download failed with status code: {msg.status}')
		except Exception as ex:
			logger.exception('Error processing download response: %s', ex)
			self._show_ok_dialog('Download Error', f'Error processing download response: {ex}')

	def _cleanup_download_handler(self):
		'''
		@brief    Remove the ReadConfigFile handler and stop the timeout timer.
		@return   None
		'''
		if self._download_timeout_timer is not None:
			self._download_timeout_timer.stop()
			self._download_timeout_timer = None
		if self._download_response_handle is not None:
			try:
				self._download_response_handle.remove()
			except Exception:
				pass
			self._download_response_handle = None
		if self._download_button is not None:
			self._download_button.setEnabled(True)

	def _load_design_constants_fields(self):
		'''
		@brief    Load and parse the DesignConstantsSet.json file.
		@return   Dictionary containing field definitions.
		'''
		try:
			with open(self._design_const_set_path, 'r') as f:
				return json.load(f)
		except Exception as ex:
			logger.exception('Failed to load DesignConstantsSet.json: %s', ex)
			return {}

	def _load_param_set_fields(self):
		'''
		@brief    Load and parse the ParamSet.json file.
		@return   Dictionary containing field definitions.
		'''
		try:
			with open(self._param_set_path, 'r') as f:
				return json.load(f)
		except Exception as ex:
			logger.exception('Failed to load ParamSet.json: %s', ex)
			return {}

	def _parse_param_set_file(self, param_set_id, fields_container, fields_layout):
		'''
		@brief    Parse the ParamSet fields and populate the container with topic labels, sunken lines, and field widgets.
		@param    param_set_id - The ID of the ParamSet being edited.
		@param    fields_container - The parent widget for the field widgets.
		@param    fields_layout - The QGridLayout to add widgets to.
		@return   None
		'''
		font_topic = QFont()
		font_topic.setBold(True)

		field_inputs = {}  # field_name -> (QLineEdit, type_str)

		row = 0
		for topic_name, topic_data in self._param_set_fields.items():
			# Skip param_id
			if topic_name == 'param_id':
				continue

			# Main topic label (bold black)
			topic_label = QLabel(topic_name, fields_container)
			topic_label.setFixedHeight(20)
			topic_label.setFont(font_topic)
			topic_label.setStyleSheet('color: black;')
			fields_layout.addWidget(topic_label, row, 0, 1, 2)
			row += 1

			# Sunken line below topic label
			topic_line = QFrame(fields_container)
			topic_line.setFrameShape(QFrame.HLine)
			topic_line.setFrameShadow(QFrame.Sunken)
			fields_layout.addWidget(topic_line, row, 0, 1, 2)
			row += 1

			# Parse sub-fields
			for field_name, field_data in topic_data.items():
				# Label
				label = QLabel(field_name + ':', fields_container)
				label.setFixedHeight(20)
				comment = field_data.get('comment', '')
				if comment:
					label.setToolTip(comment)
				color = field_data.get('color', '')
				if color:
					label.setStyleSheet(f'color: {color};')
				fields_layout.addWidget(label, row, 0)

				# Textbox
				textbox = QLineEdit(fields_container)
				textbox.setFixedHeight(20)
				textbox.setStyleSheet("background-color: white;")
				default_value = field_data.get('default', '')
				textbox.setText(str(default_value))
				field_type = field_data.get('type', '')
				if field_type:
					textbox.setToolTip(f'{field_type} type')
				fields_layout.addWidget(textbox, row, 1)

				field_inputs[field_name] = (textbox, field_type)

				fields_layout.setRowMinimumHeight(row, 0)
				row += 1

		# Store field inputs for this ParamSet ID
		self._param_set_field_inputs[param_set_id] = field_inputs

		# Add stretch at the bottom to push fields to the top
		fields_layout.setRowStretch(row, 1)

	def _parse_design_constant_set_file(self, fields_container, fields_layout):
		'''
		@brief    Parse the design constants fields and populate the container with labels and textboxes.
		@param    fields_container - The parent widget for the field widgets.
		@param    fields_layout - The QGridLayout to add widgets to.
		@return   None
		'''
		self._field_inputs = {}
		row = 0
		for field_name, field_data in self._design_constants_fields.items():
			# Label
			label = QLabel(field_name + ':', fields_container)
			label.setFixedHeight(20)
			comment = field_data.get('comment', '')
			if comment:
				label.setToolTip(comment)
			color = field_data.get('color', '')
			if color:
				label.setStyleSheet(f'color: {color};')
			fields_layout.addWidget(label, row, 0)

			# Textbox
			textbox = QLineEdit(fields_container)
			textbox.setFixedHeight(20)
			textbox.setStyleSheet("background-color: white;")
			default_value = field_data.get('default', '')
			textbox.setText(str(default_value))
			field_type = field_data.get('type', '')
			if field_type:
				textbox.setToolTip(f'{field_type} type')
			fields_layout.addWidget(textbox, row, 1)

			self._field_inputs[field_name] = textbox
			fields_layout.setRowMinimumHeight(row, 0)
			row += 1

		# Add stretch at the bottom to push fields to the top
		fields_layout.setRowStretch(row, 1)

	def _create_design_constants_section(self, parent):
		'''
		@brief    Create the Design Constants section with groupbox.
		@param    parent - Parent widget.
		@return   QVBoxLayout containing the section.
		'''
		right_column = QVBoxLayout()
		right_column.setContentsMargins(0, 0, 0, 0)
		right_column.setSpacing(6)

		font_secondary = QFont()
		font_secondary.setBold(True)
		design_constants_label = QLabel(DESIGN_CONSTANTS_TUNE_NAME, parent)
		design_constants_label.setFont(font_secondary)
		design_constants_label.setFixedHeight(20)
		right_column.addWidget(design_constants_label, 0, Qt.AlignTop)

		# Horizontal line below design_constants_label
		design_line = QFrame(parent)
		design_line.setFrameShape(QFrame.HLine)
		design_line.setFrameShadow(QFrame.Sunken)
		right_column.addWidget(design_line)

		self._design_const_set_group = QGroupBox(DESIGN_CONSTANTS_SET_NAME, parent)
		design_const_set_group = self._design_const_set_group
		design_const_set_group.setMinimumHeight(200)
		design_const_set_group.setMaximumHeight(320)
		design_const_set_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
		design_const_set_group.setStyleSheet("""
			QGroupBox {
				border: 2px outset #b0b0b0;
				border-radius: 3px;
				margin-top: 0px;
				padding-top: 15px;
				background-color: palette(window);
			}
			QGroupBox::title {
				subcontrol-origin: margin;
				subcontrol-position: top left;
				padding: 2px 5px;
				background-color: palette(window);
				border: 2px inset #b0b0b0;
				font-weight: bold;
				top: 3px;
				left: 3px;
			}
		""")

		# Layout for design_const_set_group using grid for better alignment
		design_const_layout = QGridLayout(design_const_set_group)
		design_const_layout.setColumnStretch(0, 1)
		design_const_layout.setSpacing(10)
		design_const_layout.setContentsMargins(0, 10, 5, 5)
		design_const_layout.setRowMinimumHeight(0, 30)

		# Buttons row (fixed at top)
		buttons_layout = QHBoxLayout()
		buttons_layout.setSpacing(BUTTON_HORIZONTAL_SPACING)
		buttons_layout.addStretch(1)

		self._store_button = QPushButton('Store', design_const_set_group)
		self._store_button.clicked.connect(self._on_design_constants_store)
		buttons_layout.addWidget(self._store_button)

		self._recall_button = QPushButton('Recall', design_const_set_group)
		self._recall_button.clicked.connect(self._on_design_constants_recall)
		buttons_layout.addWidget(self._recall_button)

		design_const_layout.addLayout(buttons_layout, 0, 0)

		# Scrollable area for fields
		scroll_area = QScrollArea(design_const_set_group)
		scroll_area.setWidgetResizable(True)
		scroll_area.setFrameShape(QFrame.NoFrame)
		scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

		# Container widget for fields
		fields_container = QWidget()
		fields_layout = QGridLayout(fields_container)
		fields_layout.setColumnStretch(0, 0)
		fields_layout.setColumnStretch(1, 1)
		fields_layout.setSpacing(5)
		fields_layout.setContentsMargins(5, 0, 5, 0)

		self._parse_design_constant_set_file(fields_container, fields_layout)

		scroll_area.setWidget(fields_container)
		design_const_layout.addWidget(scroll_area, 1, 0)
		design_const_layout.setRowStretch(1, 1)

		right_column.addWidget(design_const_set_group, 0, Qt.AlignTop)

		right_column.addStretch(1)

		return right_column

	def __del__(self):
		'''
		@brief    Reset the singleton on destruction.
		@return   None
		'''

		global _singleton
		_singleton = None

	def closeEvent(self, event):
		'''
		@brief    Qt close event handler.
		@param    event - Qt close event.
		@return   None
		'''

		try:
			self._cleanup_upload_handler()
		except Exception:
			pass
		try:
			self._cleanup_config_transfer_timeout()
		except Exception:
			pass
		try:
			self._cleanup_download_handler()
		except Exception:
			pass
		try:
			self._cleanup_recall_handler()
		except Exception:
			pass
		try:
			self._cleanup_param_set_recall_handler()
		except Exception:
			pass

		try:
			super(SpoolControllerPanel, self).closeEvent(event)
		finally:
			# Ensure singleton reset/handler cleanup even if shutdown fails.
			try:
				self.__del__()
			except Exception:
				pass

def spawn(parent, node):
	'''
	@brief    Spawn (or show) the singleton Spool Controller panel.
	@param    parent - Parent Qt widget.
	@param    node - Local DroneCAN node instance.
	@return   SpoolControllerPanel singleton instance.
	'''

	global _singleton
	if _singleton is None:
		try:
			_singleton = SpoolControllerPanel(parent, node)
		except Exception as ex:
			logger.exception('Failed to spawn Spool Controller panel: %s', ex)
			raise

	_singleton.show()
	_singleton.raise_()
	_singleton.activateWindow()
	return _singleton


get_icon = partial(get_icon, 'fa6s.asterisk')