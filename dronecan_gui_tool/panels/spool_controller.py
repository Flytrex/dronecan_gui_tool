
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
import struct

from PyQt5.QtCore import Qt, QRect, QSize, QPoint, QTimer
from PyQt5.QtGui import QIntValidator, QColor, QFont
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView, \
	QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QComboBox, QGridLayout, QSizePolicy, QFrame, QScrollArea, QWidget, QLayout, QMessageBox, QProgressBar
import numpy as np

from ..widgets import get_icon, show_error
from ..widgets.file_server import FileServer_PathKey
from .utils import calculate_crc32

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
BROADCAST_PRIORITY = 16                             # DroneCAN message broadcast priority (lower number = higher priority)

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

class DesignConstantsSetPayload:
	'''
	@brief    Class representing the payload of a DesignConstantsSet message, responsible for parsing and storing field values.
	'''
	FORMAT = '<6f'
	SIZE = struct.calcsize(FORMAT)

	wire_diameter: float = 0.0                      # The diameter of the wire used in the delivery system. 4 bytes
	barrel_diameter: float = 0.0                    # The inner diameter of the barrel through which the payload is delivered. 4 bytes
	spool_width: float = 0.0                        # The width of the spool that holds the wire. 4 bytes
	gearbox_ratio: float = 0.0                      # The gear ratio of the spool controller's motor gearbox. 4 bytes
	wire_packing_efficiencies: float = 0.0          # The efficiency of wire packing on the spool. 4 bytes
	total_length_of_spooled_wire: float = 0.0       # The total length of wire currently spooled, used for calculating remaining wire and feed rate. 4 bytes

	def __init__(self):
		self.wire_diameter = 0.0
		self.barrel_diameter = 0.0
		self.spool_width = 0.0
		self.gearbox_ratio = 0.0
		self.wire_packing_efficiencies = 0.0
		self.total_length_of_spooled_wire = 0.0

	def set(self, **kwargs):
		'''
		@brief    Set multiple fields of the DesignConstantsSetPayload at once using keyword arguments.
		@param    kwargs - Field names and values to set (e.g. wire_diameter=0.5, spool_width=10.0).
		@return   None
		'''
		for key, value in kwargs.items():
			if hasattr(self, key):
				setattr(self, key, value)
			else:
				raise ValueError(f'Invalid field name for DesignConstantsSetPayload: {key}')

	def serialize(self) -> bytes:
		'''
		@brief    Serialize the DesignConstantsSetPayload to bytes.
		@return   Bytes array containing the serialized payload.
		'''
		return struct.pack(self.FORMAT,
			self.wire_diameter,
			self.barrel_diameter,
			self.spool_width,
			self.gearbox_ratio,
			self.wire_packing_efficiencies,
			self.total_length_of_spooled_wire
		)

	def deserialize(self, data, offset=0):
		'''
		@brief    Deserialize a DesignConstantsSetPayload from a bytes-like object.
		@param    data - Input bytes-like buffer.
		@param    offset - Starting index in the input buffer.
		@return   Next offset after parsing this payload.
		'''
		buffer = memoryview(data)
		end = offset + self.SIZE
		if end > len(buffer):
			raise ValueError(f'Not enough data to deserialize DesignConstantsSetPayload: need {self.SIZE} bytes from offset {offset}, got {len(buffer) - offset}')

		(
			wire_diameter,
			barrel_diameter,
			spool_width,
			gearbox_ratio,
			wire_packing_efficiencies,
			total_length_of_spooled_wire,
		) = struct.unpack_from(self.FORMAT, buffer, offset)

		self.wire_diameter = wire_diameter
		self.barrel_diameter = barrel_diameter
		self.spool_width = spool_width
		self.gearbox_ratio = gearbox_ratio
		self.wire_packing_efficiencies = wire_packing_efficiencies
		self.total_length_of_spooled_wire = total_length_of_spooled_wire
		return end

class ParamSetPayload:
	'''
	@brief    Class representing the payload of a ParamSet message, responsible for parsing and storing field values.
	'''
	FORMAT = '<H?9f'
	SIZE = struct.calcsize(FORMAT)

	param_set_id: int = 0                           # The ID of the ParamSet, used to identify which set of parameters is being edited or applied. 2 bytes
	load_not_shaft_control: bool = False            # Whether the spool controller should operate in load control mode (true) or shaft control mode (false). 1 byte
	shaft_pos_rad: float = 0.0                      # The target shaft position in radians, used when load_not_shaft_control is false. 4 bytes
	completion_time_s: float = 0.0                  # The desired time in seconds to complete the movement to the target position or load. 4 bytes
	min_torque_Nm: float = 0.0                      # The minimum torque in Newton-meters that the controller should apply during the movement. 4 bytes
	max_torque_Nm: float = 0.0                      # The maximum torque in Newton-meters that the controller should apply during the movement. 4 bytes
	obs_tension_detector_min_torque_Nm: float = 0.0  # The minimum torque threshold in Newton-meters for the obstacle tension detector, used to detect if the payload is snagged on an obstacle. 4 bytes
	obs_tension_detector_window_s: float = 0.0       # The time window in seconds for the obstacle tension detector to evaluate if the torque has been below the threshold for long enough to indicate a snag. 4 bytes
	obs_traj_deviation_pos_m: float = 0.0           # The position deviation threshold in meters for the obstacle trajectory deviation detector, used to detect if the payload is snagged on an obstacle based on unexpected deviations from the planned trajectory. 4 bytes
	obs_traj_deviation_neg_m: float = 0.0           # The position deviation threshold in meters for the obstacle trajectory deviation detector, used to detect if the payload is snagged on an obstacle based on unexpected deviations from the planned trajectory. 4 bytes
	obs_allowed_deviation_pos_window_s: float = 0.0  # The time window in seconds for the obstacle trajectory deviation detector to evaluate if the position has been above the positive deviation threshold for long enough to indicate a snag. 4 bytes

	def __init__(self):
		self.param_set_id = 0
		self.load_not_shaft_control = False
		self.shaft_pos_rad = 0.0
		self.completion_time_s = 0.0
		self.min_torque_Nm = 0.0
		self.max_torque_Nm = 0.0
		self.obs_tension_detector_min_torque_Nm = 0.0
		self.obs_tension_detector_window_s = 0.0
		self.obs_traj_deviation_pos_m = 0.0
		self.obs_traj_deviation_neg_m = 0.0
		self.obs_allowed_deviation_pos_window_s = 0.0

	def set(self, **kwargs):
		'''
		@brief    Set multiple fields of the ParamSetPayload at once using keyword arguments.
		@param    kwargs - Field names and values to set (e.g. param_set_id=1, shaft_pos_rad=0.5).
		@return   None
		'''
		for key, value in kwargs.items():
			if hasattr(self, key):
				setattr(self, key, value)
			else:
				raise ValueError(f'Invalid field name for ParamSetPayload: {key}')

	def serialize(self) -> bytes:
		'''
		@brief    Serialize the ParamSetPayload to bytes.
		@return   Bytes array containing the serialized payload.
		'''
		return struct.pack(self.FORMAT,
			self.param_set_id,
			self.load_not_shaft_control,
			self.shaft_pos_rad,
			self.completion_time_s,
			self.min_torque_Nm,
			self.max_torque_Nm,
			self.obs_tension_detector_min_torque_Nm,
			self.obs_tension_detector_window_s,
			self.obs_traj_deviation_pos_m,
			self.obs_traj_deviation_neg_m,
			self.obs_allowed_deviation_pos_window_s
		)

	def deserialize(self, data, offset=0):
		'''
		@brief    Deserialize a ParamSetPayload from a bytes-like object.
		@param    data - Input bytes-like buffer.
		@param    offset - Starting index in the input buffer.
		@return   Next offset after parsing this payload.
		'''
		buffer = memoryview(data)
		end = offset + self.SIZE
		if end > len(buffer):
			raise ValueError(f'Not enough data to deserialize ParamSetPayload: need {self.SIZE} bytes from offset {offset}, got {len(buffer) - offset}')

		(
			param_set_id,
			load_not_shaft_control,
			shaft_pos_rad,
			completion_time_s,
			min_torque_Nm,
			max_torque_Nm,
			obs_tension_detector_min_torque_Nm,
			obs_tension_detector_window_s,
			obs_traj_deviation_pos_m,
			obs_traj_deviation_neg_m,
			obs_allowed_deviation_pos_window_s,
		) = struct.unpack_from(self.FORMAT, buffer, offset)

		self.param_set_id = param_set_id
		self.load_not_shaft_control = load_not_shaft_control
		self.shaft_pos_rad = shaft_pos_rad
		self.completion_time_s = completion_time_s
		self.min_torque_Nm = min_torque_Nm
		self.max_torque_Nm = max_torque_Nm
		self.obs_tension_detector_min_torque_Nm = obs_tension_detector_min_torque_Nm
		self.obs_tension_detector_window_s = obs_tension_detector_window_s
		self.obs_traj_deviation_pos_m = obs_traj_deviation_pos_m
		self.obs_traj_deviation_neg_m = obs_traj_deviation_neg_m
		self.obs_allowed_deviation_pos_window_s = obs_allowed_deviation_pos_window_s
		return end

class ParamSetFile:
	'''
	@brief    Class representing a ParamSet definition file, responsible for parsing the file and providing field definitions.
	'''

	CRC_HEADER_INITIAL = 0x560D5450  # Initial CRC value for the header section, used to verify that the header CRC is calculated correctly (matches the C++ implementation)
	HEADER_FORMAT = '<BHII'
	HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
	VERSION = 1

	_version: int = VERSION                         # Version string from the ParamSet file (e.g. "1.0"). 1 byte
	_num_param_set: int = 0                         # Number of ParamSet definitions in the file. 2 bytes
	_hdr_crc: int = 0                               # CRC32 of the header section. 4 bytes
	_payload_crc: int = 0                           # CRC32 of the payload section (DesignConstantsSet + all ParamSetPayloads). 4 bytes
	_design_constants_set_payload: DesignConstantsSetPayload  # DesignConstantsSetPayload object parsed from the file
	_param_sets: list[ParamSetPayload] = []		    # List of ParamSetPayload objects parsed from the file, one for each ParamSet definition in the file

	def __init__(self):
		self._version = self.VERSION
		self._num_param_set = 0
		self._hdr_crc = 0
		self._payload_crc = 0
		self._design_constants_set_payload = DesignConstantsSetPayload()
		self._param_sets = []

	def serialize(self) -> bytes:
		'''
		@brief    Serialize the ParamSetFile to bytes.
		@return   Bytes array containing the serialized file.
		'''
		# Serialize header: version (1 byte), num_param_set (2 bytes), hdr_crc (4 bytes), payload_crc (4 bytes)
		result = struct.pack(self.HEADER_FORMAT,
			self._version,
			self._num_param_set,
			self._hdr_crc,
			self._payload_crc
		)

		# Serialize design constants payload
		result += self._design_constants_set_payload.serialize()

		# Serialize each param set
		for param_set in self._param_sets:
			result += param_set.serialize()

		return result

	def deserialize(self, data):
		'''
		@brief    Deserialize a ParamSetFile from a bytes-like object.
		@param    data - Input bytes-like buffer containing header and payload.
		@return   Self after parsing.
		'''
		buffer = memoryview(data)
		if len(buffer) < self.HEADER_SIZE:
			raise ValueError(f'Not enough data to deserialize ParamSetFile header: need {self.HEADER_SIZE} bytes, got {len(buffer)}')

		version, num_param_set, hdr_crc, payload_crc = struct.unpack_from(self.HEADER_FORMAT, buffer, 0)
		offset = self.HEADER_SIZE

		design_constants_payload = DesignConstantsSetPayload()
		offset = design_constants_payload.deserialize(buffer, offset)
		param_sets = []
		for _ in range(num_param_set):
			param_set_payload = ParamSetPayload()
			offset = param_set_payload.deserialize(buffer, offset)
			param_sets.append(param_set_payload)

		self._version = version
		self._num_param_set = num_param_set
		self._hdr_crc = hdr_crc
		self._payload_crc = payload_crc
		self._design_constants_set_payload = design_constants_payload
		self._param_sets = param_sets

		if offset != len(buffer):
			logger.warning('ParamSetFile deserialize: %d trailing bytes were not parsed', len(buffer) - offset)

		return self

	def calculate_header_crc(self) -> int:
		'''
		@brief    Calculate the CRC32 of the header section (version and num_param_set).
		@return   Calculated CRC32 value as an integer.
		'''
		self._hdr_crc = 0  # Set to 0 for CRC calculation
		header_bytes = struct.pack(self.HEADER_FORMAT,
			self._version,
			self._num_param_set,
			self._hdr_crc,
			self._payload_crc
		)
		self._hdr_crc = calculate_crc32(header_bytes, self.CRC_HEADER_INITIAL)

		return self._hdr_crc

	def calculate_payload_crc(self) -> int:
		'''
		@brief    Calculate the CRC32 of the payload section (DesignConstantsSet and all ParamSetPayloads).
		@return   Calculated CRC32 value as an integer.
		'''
		self._payload_crc = 0  # Set to 0 for CRC calculation
		payload_bytes = self._design_constants_set_payload.serialize()
		for param_set in self._param_sets:
			payload_bytes += param_set.serialize()
		self._payload_crc = calculate_crc32(payload_bytes, self.CRC_HEADER_INITIAL)

		return self._payload_crc

	def calculate_crc(self) -> int:
		'''
		@brief    Calculate the CRC32 of the entire ParamSetFile (header + payload).
		@         Payload CRC is calculated first, then header CRC is calculated.
		@return   Calculated CRC32 value as an integer.
		'''
		self._payload_crc = self.calculate_payload_crc()
		self._hdr_crc = self.calculate_header_crc()

		return calculate_crc32(self.serialize())

	def clear(self):
		'''
		@brief    Clear all fields and reset to default values.
		@return   None
		'''
		self._version = self.VERSION
		self._num_param_set = 0
		self._hdr_crc = 0
		self._payload_crc = 0
		self._design_constants_set_payload = DesignConstantsSetPayload()
		self._param_sets = []

	def set_design_constants(self, design_constants_set_payload: DesignConstantsSetPayload):
		'''
		@brief    Set the DesignConstantsSet payload for this ParamSetFile.
		@param    design_constants_set_payload - DesignConstantsSetPayload object containing the design constants values to set.
		@return   None
		'''
		self._design_constants_set_payload = design_constants_set_payload

	def add_param_set(self, param_set_payload: ParamSetPayload):
		'''
		@brief    Add a ParamSetPayload to the list of ParamSets in this file.
		@param    param_set_payload - ParamSetPayload object containing the parameter set values to add.
		@return   None
		'''
		self._param_sets.append(param_set_payload)
		self._num_param_set = len(self._param_sets)


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

		self._param_set_file: ParamSetFile = ParamSetFile()          # Currently loaded ParamSetFile object, used for editing and uploading

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
		self._pending_param_set_compare_notified = {}  # param_set_id -> bool indicating store success dialog shown
		self._pending_design_constants_compare = None  # {field_name: value} for store-then-recall compare
		self._param_set_store_response_handle = None   # Shared DroneCAN handler for ParamSet store responses
		self._pending_param_set_stores = {}            # param_set_id -> QTimer for each pending ParamSet store
		self._pending_param_set_store_snapshots = {}   # param_set_id -> {field_name: value} snapshot waiting for store response
		self._param_set_execute_response_handle = None  # Shared DroneCAN handler for ParamSet execute responses
		self._pending_param_set_executes = {}          # param_set_id -> QTimer for each pending ParamSet execute

		self._upload_response_handle = None            # DroneCAN handler handle for WriteConfigFile (active during upload)
		self._upload_timeout_timer = None              # QTimer for WriteConfigFile upload timeout
		self._download_response_handle = None          # DroneCAN handler handle for ReadConfigFile (active during download)
		self._download_timeout_timer = None            # QTimer for ReadConfigFile download timeout
		self._config_transfer_timer = None             # QTimer for config file transfer overall timeout
		self._config_transfer_inactivity_timer = None  # QTimer for polling file server hit counters (inactivity detection)
		self._config_transfer_key = None               # File server key used to track transfer activity
		self._config_transfer_start_hits = 0           # Hit count at transfer start
		self._config_transfer_last_hits = 0            # Hit count at last inactivity poll
		self._store_constants_response_handle = None   # DroneCAN handler handle for DesignConstantsSet store response
		self._store_constants_timeout_timer = None     # QTimer for DesignConstantsSet store timeout
		self._pending_store_constants_snapshot = None  # Snapshot of values sent during OPERATION_STORE

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

		self._create_config_file_button = QPushButton('Create Config File', header_group)
		self._create_config_file_button.clicked.connect(self._on_create_config_file_clicked)
		param_set_id_row.addWidget(self._create_config_file_button)

		self._read_config_file_button = QPushButton('Read Config File', header_group)
		self._read_config_file_button.clicked.connect(self._on_read_config_file_clicked)
		param_set_id_row.addWidget(self._read_config_file_button)

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

		self._param_file_manage_group = QGroupBox(PARAM_FILE_MANAGE_NAME, parent)
		param_file_manage_group = self._param_file_manage_group
		param_file_manage_group.setMinimumHeight(200)
		param_file_manage_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
		param_file_manage_group.setStyleSheet("""
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

		group_layout = QVBoxLayout(param_file_manage_group)
		group_layout.setSpacing(6)
		group_layout.setContentsMargins(5, 15, 5, 5)

		BUTTON_WIDTH = 110

		upload_row = QHBoxLayout()
		self._upload_button = QPushButton('Upload', param_file_manage_group)
		self._upload_button.setFixedWidth(BUTTON_WIDTH)
		self._upload_button.clicked.connect(self._on_upload_clicked)
		upload_row.addWidget(self._upload_button)

		self._upload_textbox = QLineEdit(param_file_manage_group)
		upload_row.addWidget(self._upload_textbox)

		self._upload_browse_button = QPushButton('Browse', param_file_manage_group)
		self._upload_browse_button.setFixedWidth(BUTTON_WIDTH)
		self._upload_browse_button.clicked.connect(self._on_upload_browse_clicked)
		upload_row.addWidget(self._upload_browse_button)
		group_layout.addLayout(upload_row)

		download_row = QHBoxLayout()
		self._download_button = QPushButton('Download', param_file_manage_group)
		self._download_button.setFixedWidth(BUTTON_WIDTH)
		self._download_button.clicked.connect(self._on_download_clicked)
		download_row.addWidget(self._download_button)

		self._download_textbox = QLineEdit(param_file_manage_group)
		download_row.addWidget(self._download_textbox)

		self._download_browse_button = QPushButton('Browse', param_file_manage_group)
		self._download_browse_button.setFixedWidth(BUTTON_WIDTH)
		self._download_browse_button.clicked.connect(self._on_download_browse_clicked)
		download_row.addWidget(self._download_browse_button)
		group_layout.addLayout(download_row)

		STATUS_LABEL_WIDTH = 60
		STATUS_TEXTBOX_WIDTH = 80

		status_row = QHBoxLayout()
		status_row.setSpacing(0)

		version_label = QLabel('Version:', param_file_manage_group)
		version_label.setFixedWidth(STATUS_LABEL_WIDTH)
		status_row.addWidget(version_label)

		self._version_textbox = QLineEdit(param_file_manage_group)
		self._version_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		self._version_textbox.setReadOnly(True)
		status_row.addWidget(self._version_textbox)

		status_row.addSpacing(23)

		crc32_label = QLabel('CRC32:', param_file_manage_group)
		crc32_label.setFixedWidth(STATUS_LABEL_WIDTH)
		status_row.addWidget(crc32_label)

		self._crc32_textbox = QLineEdit(param_file_manage_group)
		self._crc32_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		self._crc32_textbox.setReadOnly(True)
		status_row.addWidget(self._crc32_textbox)

		status_row.addSpacing(23)

		dirty_label = QLabel('Dirty:', param_file_manage_group)
		dirty_label.setFixedWidth(STATUS_LABEL_WIDTH)
		status_row.addWidget(dirty_label)

		self._dirty_textbox = QLineEdit(param_file_manage_group)
		self._dirty_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		self._dirty_textbox.setReadOnly(True)
		status_row.addWidget(self._dirty_textbox)

		status_row.addStretch(1)

		group_layout.addLayout(status_row)

		group_layout.addStretch(1)

		progress_row = QHBoxLayout()
		progress_row.setSpacing(0)
		self._config_transfer_progress = QProgressBar(param_file_manage_group)
		self._config_transfer_progress.setRange(0, 100)
		self._config_transfer_progress.setValue(0)
		self._config_transfer_progress.setAlignment(Qt.AlignCenter)
		self._config_transfer_progress.setFixedWidth(
			(STATUS_LABEL_WIDTH * 3) + (STATUS_TEXTBOX_WIDTH * 3) + 46
		)
		progress_row.addWidget(self._config_transfer_progress)
		progress_row.addStretch(1)
		group_layout.addLayout(progress_row)

		group_layout.addSpacing(2)

		left_column.addWidget(param_file_manage_group)

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

		upload_path = os.path.normcase(os.path.abspath(os.path.expanduser(upload_path)))

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

		# Disable the upload and browse buttons while waiting for response
		self._upload_button.setEnabled(False)
		self._upload_browse_button.setEnabled(False)
		self._download_browse_button.setEnabled(False)

		try:
			self._node.broadcast(msg, priority=BROADCAST_PRIORITY)
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

		# Disable the download and browse buttons while waiting for response
		self._download_button.setEnabled(False)
		self._upload_browse_button.setEnabled(False)
		self._download_browse_button.setEnabled(False)

		try:
			self._node.broadcast(msg, priority=BROADCAST_PRIORITY)
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

	def _on_create_config_file_clicked(self):
		'''
		@brief    Handle Create Config File button click.
		@return   None
		'''

		# Prompt user for save destination before doing any work
		dialog = QFileDialog(self)
		dialog.setWindowTitle('Save Config File')
		dialog.setAcceptMode(QFileDialog.AcceptSave)
		dialog.setFileMode(QFileDialog.AnyFile)
		dialog.setNameFilter('Binary config files (*.bin);;All files (*.*)')
		dialog.setDefaultSuffix('bin')

		if not dialog.exec_():
			return

		selected_files = dialog.selectedFiles()
		if not selected_files:
			return
		save_path = selected_files[0]

		# Clear any existing data in the ParamSetFile before populating with current field values
		self._param_set_file.clear()

		temp_design_constants = DesignConstantsSetPayload()
		# Extract the current values from the Design Constants fields
		for field_name, textbox in self._field_inputs.items():
			raw_value = textbox.text().strip()
			field_type = self._design_constants_fields.get(field_name, {}).get('type', 'float32')
			try:
				value = self._parse_value(field_type, raw_value)
				setattr(temp_design_constants, field_name, value)
			except Exception as ex:
				show_error(
					'Invalid field value',
					f'Could not parse field "{field_name}".',
					f'Type: {field_type}\nValue: {raw_value}\nError: {ex}',
					parent=self,
					blocking=True,
				)
				return
		self._param_set_file.set_design_constants(temp_design_constants)
		# Extract current field values from all open ParamSet groupboxes.
		extracted_param_sets = {}
		for param_set_id, _groupbox in self._param_set_groupboxes.items():
			field_inputs = self._param_set_field_inputs.get(param_set_id, {})
			fields = {}
			temp_param_set_payload = ParamSetPayload()

			# Set the param_set_id
			try:
				temp_param_set_payload.param_set_id = int(param_set_id)
			except ValueError:
				show_error(
					'Invalid ParamSet ID',
					f'ParamSet ID "{param_set_id}" is not a valid integer.',
					'',
					parent=self,
					blocking=True,
				)
				return

			# Extract and set field values
			for field_name, (textbox, field_type) in field_inputs.items():
				raw_value = textbox.text().strip()
				fields[field_name] = {
					'value': raw_value,
					'type': field_type,
				}
				try:
					value = self._parse_value(field_type, raw_value)
					setattr(temp_param_set_payload, field_name, value)
				except Exception as ex:
					show_error(
						'Invalid field value',
						f'Could not parse field "{field_name}" for ParamSet {param_set_id}.',
						f'Type: {field_type}\nValue: {raw_value}\nError: {ex}',
						parent=self,
						blocking=True,
					)
					return

			# Add the populated payload to the param set file
			self._param_set_file.add_param_set(temp_param_set_payload)
			extracted_param_sets[param_set_id] = fields

		logger.info('Extracted fields from %d ParamSet groupboxes', len(extracted_param_sets))
		# Calculate the CRC
		self._param_set_file.calculate_crc()

		# Serialize and write to file
		try:
			data = self._param_set_file.serialize()
			with open(save_path, 'wb') as f:
				f.write(data)
			logger.info('Config file written to %s (%d bytes)', save_path, len(data))
			self._show_ok_dialog('Create Config File', f'Config file saved to:\n{save_path}')
		except Exception as ex:
			logger.exception('Failed to write config file: %s', ex)
			show_error('Save Error', 'Could not write config file.', str(ex), parent=self, blocking=True)

	def _on_read_config_file_clicked(self):
		'''
		@brief    Handle Read Config File button click.
		@return   None
		'''
		filename, _ = QFileDialog.getOpenFileName(
			self,
			'Read Config File',
			'',
			'Binary config files (*.bin);;All files (*.*)'
		)
		if not filename:
			return

		try:
			with open(filename, 'rb') as file_handle:
				data = file_handle.read()
		except Exception as ex:
			logger.exception('Failed to read config file: %s', ex)
			show_error('Read Error', 'Could not read config file.', str(ex), parent=self, blocking=True)
			return

		try:
			self._param_set_file.clear()
			self._param_set_file.deserialize(data)
		except Exception as ex:
			logger.exception('Failed to deserialize config file: %s', ex)
			show_error('Read Error', 'Could not parse config file.', str(ex), parent=self, blocking=True)
			return

		self._clear_all_param_set_groupboxes()

		design_constants_payload = self._param_set_file._design_constants_set_payload
		for field_name, textbox in self._field_inputs.items():
			if not hasattr(design_constants_payload, field_name):
				continue
			textbox.blockSignals(True)
			textbox.setText(str(getattr(design_constants_payload, field_name)))
			textbox.blockSignals(False)

		for param_set_payload in self._param_set_file._param_sets:
			param_set_id = str(param_set_payload.param_set_id)
			self._add_param_set_editing_content(param_set_id)
			field_inputs = self._param_set_field_inputs.get(param_set_id, {})
			for field_name, (textbox, _field_type) in field_inputs.items():
				if not hasattr(param_set_payload, field_name):
					continue
				textbox.blockSignals(True)
				textbox.setText(str(getattr(param_set_payload, field_name)))
				textbox.blockSignals(False)
			self._param_set_dirty[param_set_id] = False

		self._version_textbox.setText(str(self._param_set_file._version))
		self._crc32_textbox.setText(f'{self._param_set_file.calculate_crc():08X}')
		self._dirty_textbox.setText('False')
		logger.info('Config file loaded from %s with %d ParamSet entries', filename, self._param_set_file._num_param_set)
		self._show_ok_dialog('Read Config File', f'Config file loaded from:\n{filename}')

	def _clear_all_param_set_groupboxes(self):
		'''
		@brief    Remove all ParamSet editing groupboxes without prompting the user.
		@return   None
		'''
		for param_set_id, groupbox in list(self._param_set_groupboxes.items()):
			self._param_set_dirty.pop(param_set_id, None)
			self._param_set_field_inputs.pop(param_set_id, None)
			self._param_set_groupboxes.pop(param_set_id, None)
			if param_set_id in self._param_set_id_list:
				self._param_set_id_list.remove(param_set_id)
			used_color = self._param_set_color_map.pop(param_set_id, None)
			if used_color and used_color in _PARAM_SET_LIGHT_COLORS and used_color not in self._available_colors:
				self._available_colors.append(used_color)
			self._param_set_container_layout.removeWidget(groupbox)
			groupbox.deleteLater()

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
			self._node.broadcast(msg, priority=BROADCAST_PRIORITY)
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

	@staticmethod
	def _parse_error_msg(error_obj, operation_context):
		'''
		@brief    Parse a DroneCAN error object and return appropriate user-facing messages.
		@param    error_obj - The error object from a DroneCAN message (e.g., msg.error).
		@param    operation_context - String describing the operation (e.g., 'upload', 'download', 'execute', 'store', 'recall').
		@return   Tuple of (dialog_title, dialog_message, log_message) or None if STATUS_OK.
		'''
		if not hasattr(error_obj, 'value'):
			return ('Error', f'{operation_context.capitalize()} failed with unknown error.', 'Unknown error object')

		error_value = error_obj.value

		# STATUS_OK = 0
		if error_value == 0:
			return None

		# Map error codes to messages
		if hasattr(error_obj, 'STATUS_FILE_NOT_FOUND') and error_value == error_obj.STATUS_FILE_NOT_FOUND:
			return (
				f'{operation_context.capitalize()} Error',
				'The spool controller could not find the file on the file server. Please check that the file was uploaded correctly and try again.',
				f'{operation_context} failed with file not found error'
			)
		elif hasattr(error_obj, 'STATUS_IO_ERROR') and error_value == error_obj.STATUS_IO_ERROR:
			return (
				f'{operation_context.capitalize()} Error',
				'An I/O error occurred while the spool controller was processing the request. Please try again.',
				f'{operation_context} failed with I/O error'
			)
		elif hasattr(error_obj, 'STATUS_ACCESS_DENIED') and error_value == error_obj.STATUS_ACCESS_DENIED:
			return (
				f'{operation_context.capitalize()} Error',
				'The spool controller was denied access. Please check permissions and try again.',
				f'{operation_context} failed with access denied error'
			)
		elif hasattr(error_obj, 'STATUS_IS_DIRECTORY') and error_value == error_obj.STATUS_IS_DIRECTORY:
			return (
				f'{operation_context.capitalize()} Error',
				'The specified path is a directory, not a file. Please check the path and try again.',
				f'{operation_context} failed with is directory error'
			)
		elif hasattr(error_obj, 'STATUS_INVALID_VALUE') and error_value == error_obj.STATUS_INVALID_VALUE:
			return (
				f'{operation_context.capitalize()} Error',
				'The request was invalid. Please check the parameters and try again.',
				f'{operation_context} failed with invalid value error'
			)
		elif hasattr(error_obj, 'STATUS_FILE_TOO_LARGE') and error_value == error_obj.STATUS_FILE_TOO_LARGE:
			return (
				f'{operation_context.capitalize()} Error',
				'The file is too large for the spool controller to handle. Please check the file size and try again.',
				f'{operation_context} failed with file too large error'
			)
		elif hasattr(error_obj, 'STATUS_OUT_OF_SPACE') and error_value == error_obj.STATUS_OUT_OF_SPACE:
			return (
				f'{operation_context.capitalize()} Error',
				'The spool controller does not have enough space. Please free up space and try again.',
				f'{operation_context} failed with out of space error'
			)
		elif hasattr(error_obj, 'STATUS_NOT_IMPLEMENTED') and error_value == error_obj.STATUS_NOT_IMPLEMENTED:
			return (
				f'{operation_context.capitalize()} Error',
				f'The spool controller does not support {operation_context} operations. Please check the controller capabilities and try again.',
				f'{operation_context} failed with not implemented error'
			)
		elif hasattr(error_obj, 'STATUS_IDX_OUT_OF_BOUNDS') and error_value == error_obj.STATUS_IDX_OUT_OF_BOUNDS:
			return (
				f'{operation_context.capitalize()} Error',
				'The request index was out of bounds. Please check the parameters and try again.',
				f'{operation_context} failed with index out of bounds error'
			)
		elif hasattr(error_obj, 'STATUS_BUSY') and error_value == error_obj.STATUS_BUSY:
			return (
				f'{operation_context.capitalize()} Status',
				'The spool controller is busy. Please try again later.',
				f'{operation_context} failed - controller busy'
			)
		elif hasattr(error_obj, 'STATUS_LOW_MEM') and error_value == error_obj.STATUS_LOW_MEM:
			return (
				f'{operation_context.capitalize()} Status',
				'The spool controller has low memory. Please try again later.',
				f'{operation_context} failed - low memory'
			)
		elif hasattr(error_obj, 'STATUS_UNKNOWN_ERROR') and error_value == error_obj.STATUS_UNKNOWN_ERROR:
			return (
				f'{operation_context.capitalize()} Error',
				f'An unknown error occurred during {operation_context}.',
				f'{operation_context} failed with unknown error'
			)
		else:
			return (
				f'{operation_context.capitalize()} Error',
				f'{operation_context.capitalize()} failed with error code: {error_value}',
				f'{operation_context} failed with error code {error_value}'
			)


	def _on_param_set_execute(self, param_set_id):
		'''
		@brief    Handle Execute button click: broadcast a flytrex.delcon.ParamSet message with OPERATION_EXECUTE.
		@param    param_set_id - The ParamSet ID to execute.
		@return   None
		'''
		# Clean up any previous execute handler for this param_set_id
		self._cleanup_param_set_execute(param_set_id)

		# Disable the groupbox while waiting for the response
		groupbox = self._param_set_groupboxes.get(param_set_id)
		if groupbox is not None:
			groupbox.setEnabled(False)

		if not self._send_param_set_msg(param_set_id, 'OPERATION_EXECUTE'):
			if groupbox is not None:
				groupbox.setEnabled(True)
			return

		# Register the shared handler if this is the first pending execute
		if not self._pending_param_set_executes and self._param_set_execute_response_handle is None:
			try:
				self._param_set_execute_response_handle = self._node.add_handler(
					dronecan.flytrex.delcon.ParamSet,
					self._on_param_set_execute_response,
				)
			except Exception as ex:
				logger.exception('Could not register ParamSet execute handler: %s', ex)
				if groupbox is not None:
					groupbox.setEnabled(True)
				return

		# Start a per-ID timeout timer
		timer = QTimer(self)
		timer.setSingleShot(True)
		timer.timeout.connect(
			lambda _id=param_set_id: (
				self._cleanup_param_set_execute(_id),
				self._on_recall_timeout(
					f'No ParamSet OPERATION_RESPONSE for ParamSet ID {_id} was received within {RESPONSE_TIMEOUT} seconds.\n\n'
					f'The spool controller may be offline or not responding.'
				),
			),
		)
		self._pending_param_set_executes[param_set_id] = timer
		timer.start(RESPONSE_TIMEOUT * 1000)

	def _on_param_set_store(self, param_set_id):
		'''
		@brief    Handle Store button click: broadcast a flytrex.delcon.ParamSet message with OPERATION_STORE.
		          Waits for a response to check error status before proceeding with recall.
		@param    param_set_id - The ParamSet ID to store.
		@return   None
		'''
		snapshot = self._capture_param_set_values(param_set_id)
		if snapshot is None:
			return

		# Clean up any previous store handler for this param_set_id
		self._cleanup_param_set_store(param_set_id)

		# Disable the groupbox while waiting for the response
		groupbox = self._param_set_groupboxes.get(param_set_id)
		if groupbox is not None:
			groupbox.setEnabled(False)

		if not self._send_param_set_msg(param_set_id, 'OPERATION_STORE'):
			if groupbox is not None:
				groupbox.setEnabled(True)
			return

		# Store snapshot for later comparison
		self._pending_param_set_store_snapshots[param_set_id] = snapshot

		# Register the shared handler if this is the first pending store
		if not self._pending_param_set_stores and self._param_set_store_response_handle is None:
			try:
				self._param_set_store_response_handle = self._node.add_handler(
					dronecan.flytrex.delcon.ParamSet,
					self._on_param_set_store_response,
				)
			except Exception as ex:
				logger.exception('Could not register ParamSet store handler: %s', ex)
				if groupbox is not None:
					groupbox.setEnabled(True)
				self._pending_param_set_store_snapshots.pop(param_set_id, None)
				return

		# Start a per-ID timeout timer
		timer = QTimer(self)
		timer.setSingleShot(True)
		timer.timeout.connect(
			lambda _id=param_set_id: (
				self._cleanup_param_set_store(_id),
				self._on_recall_timeout(
					f'No ParamSet OPERATION_STORE response for ParamSet ID {_id} was received within {RESPONSE_TIMEOUT} seconds.\n\n'
					f'The spool controller may be offline or not responding.'
				),
			)
		)
		self._pending_param_set_stores[param_set_id] = timer
		timer.start(RESPONSE_TIMEOUT * 1000)

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
			self._node.broadcast(msg, priority=BROADCAST_PRIORITY)
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
				self._clear_pending_design_constants_compare(),
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

		# Check for errors in the response
		if hasattr(msg, 'error') and hasattr(msg.error, 'value'):
			if msg.error.value != 0:  # STATUS_OK is 0
				error_info = self._parse_error_msg(msg.error, 'recall DesignConstantsSet')
				if error_info:
					title, message, log_msg = error_info
					self._show_ok_dialog(title, message)
					logger.warning(log_msg)
				return

		compare_snapshot = self._pending_design_constants_compare
		self._pending_design_constants_compare = None
		if compare_snapshot is not None:
			for field_name, textbox in self._field_inputs.items():
				if not hasattr(msg, field_name):
					continue
				recalled_value = getattr(msg, field_name, None)
				if recalled_value is None:
					continue
				stored_value = compare_snapshot.get(field_name)
				if not self._values_match(stored_value, recalled_value):
					self._show_ok_dialog('Constants', 'Constants send failed!')
					logger.warning('DesignConstants compare failed for field %s', field_name)
					return
			self._show_ok_dialog('Constants', 'Constants were set!')
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

		# Check for errors in the response
		if hasattr(msg, 'error') and hasattr(msg.error, 'value'):
			if msg.error.value != 0:  # STATUS_OK is 0
				error_info = self._parse_error_msg(msg.error, f'recall ParamSet {param_set_id}')
				if error_info:
					title, message, log_msg = error_info
					self._show_ok_dialog(title, message)
					logger.warning(log_msg)
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
					self._show_ok_dialog('ParamSet', f'ParamSet {param_set_id} store verification failed.')
					logger.warning('Store compare failed for ParamSet %s field %s', param_set_id, field_name)
					return
			if not self._pending_param_set_compare_notified.pop(param_set_id, False):
				self._show_ok_dialog('ParamSet', f'ParamSet {param_set_id} stored successfully.')
			logger.info('Store compare matched for ParamSet %s', param_set_id)
			self._param_set_dirty[param_set_id] = False
			return

		for field_name, (textbox, field_type) in field_inputs.items():
			value = getattr(msg, field_name, None)
			if value is not None:
				textbox.blockSignals(True)
				textbox.setText(str(value))
				textbox.blockSignals(False)
		self._param_set_dirty[param_set_id] = False
		logger.info('ParamSet OPERATION_RESPONSE received — ParamSet %s fields populated', param_set_id)

	def _on_param_set_store_response(self, event):
		'''
		@brief    Handle an incoming ParamSet response to OPERATION_STORE.
		@param    event - DroneCAN transfer event containing the response message.
		@return   None
		'''
		msg = event.message
		# Check that the 'operation' field is OPERATION_RESPONSE
		if msg.operation != msg.OPERATION_RESPONSE:
			return

		param_set_id = str(msg.param_id)
		if param_set_id not in self._pending_param_set_stores:
			logger.warning('ParamSet store response for param_id=%s but no pending store', param_set_id)
			return

		snapshot = self._pending_param_set_store_snapshots.get(param_set_id)
		self._cleanup_param_set_store(param_set_id)

		logger.info('ParamSet OPERATION_STORE response received for param_id=%s: error=%s', param_set_id, msg.error.value if hasattr(msg, 'error') else 'N/A')

		# Check for errors in the response
		if hasattr(msg, 'error') and hasattr(msg.error, 'value'):
			if msg.error.value != 0:  # STATUS_OK is 0
				error_info = self._parse_error_msg(msg.error, f'store ParamSet {param_set_id}')
				if error_info:
					title, message, log_msg = error_info
					self._show_ok_dialog(title, message)
					logger.warning(log_msg)
				return

		# Store successful - proceed with recall to verify
		if snapshot is not None:
			self._pending_param_set_compare[param_set_id] = snapshot
			self._pending_param_set_compare_notified[param_set_id] = True
			self._show_ok_dialog('ParamSet', f'ParamSet {param_set_id} stored successfully.')
			logger.info('ParamSet store successful for param_id=%s, proceeding with recall', param_set_id)
			self._on_param_set_recall(param_set_id)
		else:
			logger.warning('ParamSet store response received for param_id=%s but no snapshot found', param_set_id)

	def _on_param_set_execute_response(self, event):
		'''
		@brief    Handle an incoming ParamSet response to OPERATION_EXECUTE.
		@param    event - DroneCAN transfer event containing the response message.
		@return   None
		'''
		msg = event.message
		# Check that the 'operation' field is OPERATION_RESPONSE
		if msg.operation != msg.OPERATION_RESPONSE:
			return

		param_set_id = str(msg.param_id)
		if param_set_id not in self._pending_param_set_executes:
			return

		self._cleanup_param_set_execute(param_set_id)

		# Check for errors in the response
		if hasattr(msg, 'error') and hasattr(msg.error, 'value'):
			if msg.error.value != 0:  # STATUS_OK is 0
				error_info = self._parse_error_msg(msg.error, f'execute ParamSet {param_set_id}')
				if error_info:
					title, message, log_msg = error_info
					self._show_ok_dialog(title, message)
					logger.warning(log_msg)
				return

		self._show_ok_dialog('ParamSet', f'ParamSet {param_set_id} executed successfully.')

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
		logger.info('WriteConfigFile response received: error=%s', msg.error.value)

		try:
			if msg.error.value == msg.error.STATUS_OK:
				logger.info('Upload successful. Spool controller is reading the config file.')
				self._start_config_transfer_timeout()
			else:
				error_info = self._parse_error_msg(msg.error, 'upload')
				if error_info:
					title, message, log_msg = error_info
					self._show_ok_dialog(title, message)
					logger.warning(log_msg)
		except Exception as ex:
			logger.exception('Error processing upload response: %s', ex)
			self._show_ok_dialog('Upload Error', f'Error processing upload response: {ex}')

	def _start_config_transfer_timeout(self):
		'''
		@brief    Start timeouts for config file transfer activity.
		          Two timers are started:
		          1. An overall deadline of CONFIG_FILE_TRANSFER_TIMEOUT seconds.
		          2. A repeating inactivity poll every RESPONSE_TIMEOUT seconds that
		             fires if no new file-read hits are observed between polls.
		@return   None
		'''
		key = self._config_transfer_key
		self._cleanup_config_transfer_timeout()
		if not key:
			return

		self._config_transfer_key = key
		self._config_transfer_start_hits = 0
		self._config_transfer_last_hits = 0
		try:
			file_server_widget = self._get_file_server_widget()
			file_server = getattr(file_server_widget, '_file_server', None)
			if file_server is not None:
				current = file_server.path_hit_counters.get(key, 0)
				self._config_transfer_start_hits = current
				self._config_transfer_last_hits = current
		except Exception:
			logger.exception('Could not read file server hit counters')

		# Overall deadline timer
		self._config_transfer_timer = QTimer(self)
		self._config_transfer_timer.setSingleShot(True)
		self._config_transfer_timer.timeout.connect(self._on_config_transfer_timeout)
		self._config_transfer_timer.start(CONFIG_FILE_TRANSFER_TIMEOUT * 1000)

		# Inactivity poll timer
		self._config_transfer_inactivity_timer = QTimer(self)
		self._config_transfer_inactivity_timer.setSingleShot(False)
		self._config_transfer_inactivity_timer.timeout.connect(self._on_config_transfer_inactivity_check)
		self._config_transfer_inactivity_timer.start(RESPONSE_TIMEOUT * 1000)

	def _get_config_transfer_hits(self):
		'''
		@brief    Read the current file-server hit count for the active transfer key.
		@return   Current hit count, or self._config_transfer_last_hits on error.
		'''
		hits = self._config_transfer_last_hits
		try:
			file_server_widget = self._get_file_server_widget()
			file_server = getattr(file_server_widget, '_file_server', None)
			if file_server is not None and self._config_transfer_key:
				hits = file_server.path_hit_counters.get(self._config_transfer_key, hits)
		except Exception:
			logger.exception('Could not read file server hit counters')
		return hits

	def _on_config_transfer_inactivity_check(self):
		'''
		@brief    Periodic check for file-read inactivity during config transfer.
		          If the hit count has not increased since the last poll, the node
		          has stopped reading — show a timeout dialog.
		@return   None
		'''
		hits = self._get_config_transfer_hits()
		if hits > self._config_transfer_last_hits:
			# Activity detected — update baseline and keep waiting
			self._config_transfer_last_hits = hits
			return

		# No new reads since last poll
		if hits > self._config_transfer_start_hits:
			message = (
				f'The spool controller stopped reading the config file '
				f'(no activity for {RESPONSE_TIMEOUT} seconds).'
			)
		else:
			message = (
				f'No file read activity was observed from the spool controller '
				f'within {RESPONSE_TIMEOUT} seconds of the upload request.'
			)

		self._cleanup_config_transfer_timeout()
		self._show_ok_dialog('Transfer Timeout', message)

	def _on_config_transfer_timeout(self):
		'''
		@brief    Handle overall config file transfer timeout.
		@return   None
		'''
		message = (
			f'Config file transfer did not complete within {CONFIG_FILE_TRANSFER_TIMEOUT} seconds.'
		)

		self._cleanup_config_transfer_timeout()
		self._show_ok_dialog('Transfer Timeout', message)

	def _cleanup_config_transfer_timeout(self):
		'''
		@brief    Stop both config file transfer timers and reset state.
		@return   None
		'''
		if self._config_transfer_timer is not None:
			self._config_transfer_timer.stop()
			self._config_transfer_timer = None
		if self._config_transfer_inactivity_timer is not None:
			self._config_transfer_inactivity_timer.stop()
			self._config_transfer_inactivity_timer = None
		self._config_transfer_key = None
		self._config_transfer_start_hits = 0
		self._config_transfer_last_hits = 0

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
		if self._upload_browse_button is not None:
			self._upload_browse_button.setEnabled(True)
		if self._download_browse_button is not None:
			self._download_browse_button.setEnabled(True)

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

	def _cleanup_store_constants_handler(self):
		'''
		@brief    Remove the DesignConstantsSet store handler and stop the timeout timer.
		@return   None
		'''
		if self._store_constants_timeout_timer is not None:
			self._store_constants_timeout_timer.stop()
			self._store_constants_timeout_timer = None
		if self._store_constants_response_handle is not None:
			try:
				self._store_constants_response_handle.remove()
			except Exception:
				pass
			self._store_constants_response_handle = None
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
		self._pending_param_set_compare_notified.pop(param_set_id, None)
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

	def _cleanup_param_set_store(self, param_set_id):
		'''
		@brief    Clean up a single pending ParamSet store: stop its timer, re-enable
		          its groupbox, and remove the shared handler if no stores remain.
		@param    param_set_id - The ParamSet ID whose store is being cleaned up.
		@return   None
		'''
		timer = self._pending_param_set_stores.pop(param_set_id, None)
		if timer is not None:
			timer.stop()
		self._pending_param_set_store_snapshots.pop(param_set_id, None)
		# Re-enable the groupbox
		groupbox = self._param_set_groupboxes.get(param_set_id)
		if groupbox is not None:
			groupbox.setEnabled(True)
		# Remove the shared handler if no more pending stores
		if not self._pending_param_set_stores and self._param_set_store_response_handle is not None:
			try:
				self._param_set_store_response_handle.remove()
			except Exception:
				pass
			self._param_set_store_response_handle = None

	def _cleanup_param_set_execute(self, param_set_id):
		'''
		@brief    Clean up a single pending ParamSet execute: stop its timer, re-enable
		          its groupbox, and remove the shared handler if no executes remain.
		@param    param_set_id - The ParamSet ID whose execute is being cleaned up.
		@return   None
		'''
		timer = self._pending_param_set_executes.pop(param_set_id, None)
		if timer is not None:
			timer.stop()
		# Re-enable the groupbox
		groupbox = self._param_set_groupboxes.get(param_set_id)
		if groupbox is not None:
			groupbox.setEnabled(True)
		# Remove the shared handler if no more pending executes
		if not self._pending_param_set_executes and self._param_set_execute_response_handle is not None:
			try:
				self._param_set_execute_response_handle.remove()
			except Exception:
				pass
			self._param_set_execute_response_handle = None

	def _cleanup_param_set_store_handler(self):
		'''
		@brief    Clean up all pending ParamSet stores. Used during panel shutdown.
		@return   None
		'''
		for pid in list(self._pending_param_set_stores):
			self._cleanup_param_set_store(pid)

	def _cleanup_param_set_execute_handler(self):
		'''
		@brief    Clean up all pending ParamSet executes. Used during panel shutdown.
		@return   None
		'''
		for pid in list(self._pending_param_set_executes):
			self._cleanup_param_set_execute(pid)

	def _on_design_constants_store(self):
		'''
		@brief    Handle Store button click: broadcast a flytrex.delcon.DesignConstantsSet message with OPERATION_STORE.
		          Waits for a response to check error status before proceeding with recall.
		@return   None
		'''
		if not self._field_inputs:
			show_error('Store Error', 'No design constant fields found.', '', parent=self, blocking=True)
			return

		snapshot = {}
		for field_name, textbox in self._field_inputs.items():
			raw_value = textbox.text().strip()
			field_type = self._design_constants_fields.get(field_name, {}).get('type', 'float32')
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

		# Clean up any previous store handler
		self._cleanup_store_constants_handler()

		# Disable the groupbox while waiting for the response
		if self._design_const_set_group is not None:
			self._design_const_set_group.setEnabled(False)

		try:
			self._node.broadcast(msg, priority=BROADCAST_PRIORITY)
			logger.info('Broadcast DesignConstantsSet OPERATION_STORE')
		except Exception as ex:
			logger.exception('Failed to broadcast DesignConstantsSet: %s', ex)
			show_error('Broadcast failed', 'Could not broadcast DesignConstantsSet.', str(ex), parent=self, blocking=True)
			if self._design_const_set_group is not None:
				self._design_const_set_group.setEnabled(True)
			return

		# Store snapshot for later comparison
		self._pending_store_constants_snapshot = snapshot

		# Register handler for the response message
		try:
			self._store_constants_response_handle = self._node.add_handler(
				dronecan.flytrex.delcon.DesignConstantsSet,
				self._on_design_constants_store_response,
			)
		except Exception as ex:
			logger.exception('Could not register DesignConstantsSet store handler: %s', ex)
			if self._design_const_set_group is not None:
				self._design_const_set_group.setEnabled(True)
			self._pending_store_constants_snapshot = None
			return

		# Start timeout timer
		self._store_constants_timeout_timer = QTimer(self)
		self._store_constants_timeout_timer.setSingleShot(True)
		self._store_constants_timeout_timer.timeout.connect(
			lambda: (
				self._cleanup_store_constants_handler(),
				self._on_recall_timeout(
					f'No DesignConstantsSet OPERATION_STORE response was received within {RESPONSE_TIMEOUT} seconds.\n\n'
					f'The spool controller may be offline or not responding.'
				),
			)
		)
		self._store_constants_timeout_timer.start(RESPONSE_TIMEOUT * 1000)

	def _on_design_constants_store_response(self, event):
		'''
		@brief    Handle an incoming DesignConstantsSet response to OPERATION_STORE.
		@param    event - DroneCAN transfer event containing the response message.
		@return   None
		'''
		self._cleanup_store_constants_handler()

		msg = event.message
		# Check that the 'operation' field is OPERATION_RESPONSE
		if msg.operation != msg.OPERATION_RESPONSE:
			return

		logger.info('DesignConstantsSet OPERATION_STORE response received: error=%s', msg.error.value if hasattr(msg, 'error') else 'N/A')

		# Check for errors in the response
		if hasattr(msg, 'error') and hasattr(msg.error, 'value'):
			if msg.error.value != 0:  # STATUS_OK is 0
				error_info = self._parse_error_msg(msg.error, 'store DesignConstantsSet')
				if error_info:
					title, message, log_msg = error_info
					self._show_ok_dialog(title, message)
					logger.warning(log_msg)
				self._pending_store_constants_snapshot = None
				return

		# Store successful - proceed with recall to verify
		if self._pending_store_constants_snapshot is not None:
			self._pending_design_constants_compare = self._pending_store_constants_snapshot
			self._pending_store_constants_snapshot = None
			logger.info('DesignConstantsSet store successful, proceeding with recall')
			self._on_design_constants_recall()
		else:
			logger.warning('DesignConstantsSet store response received but no snapshot found')

	def _clear_pending_design_constants_compare(self):
		'''
		@brief    Clear pending DesignConstantsSet compare snapshot.
		@return   None
		'''
		self._pending_design_constants_compare = None

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
		logger.info('ReadConfigFile response received: error=%s', msg.error.value)

		try:
			if msg.error.value == msg.error.STATUS_OK:
				logger.info('Download request accepted. Spool controller is reading the config file.')
			else:
				error_info = self._parse_error_msg(msg.error, 'download')
				if error_info:
					title, message, log_msg = error_info
					self._show_ok_dialog(title, message)
					logger.warning(log_msg)
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
		if self._upload_browse_button is not None:
			self._upload_browse_button.setEnabled(True)
		if self._download_browse_button is not None:
			self._download_browse_button.setEnabled(True)

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
		design_const_set_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
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

		right_column.addWidget(design_const_set_group)

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
			self._cleanup_store_constants_handler()
		except Exception:
			pass
		try:
			self._cleanup_param_set_recall_handler()
		except Exception:
			pass
		try:
			self._cleanup_param_set_store_handler()
		except Exception:
			pass
		try:
			self._cleanup_param_set_execute_handler()
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