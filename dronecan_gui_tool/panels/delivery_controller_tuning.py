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
from dataclasses import dataclass
import threading
import os
import re
import json
import random
import struct
import ctypes
import tempfile
import atexit

from .delivery_controller import DeliveryControllerCommand, DeliveryControllerMode, NodeParametersHelper

from PyQt5 import QtCore
from PyQt5.QtCore import Qt, QRect, QSize, QPoint, QTimer, QLocale, pyqtSignal
from PyQt5.QtGui import QIntValidator, QColor, QFont, QKeySequence, QDoubleValidator
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView, \
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QComboBox, QGridLayout, QSizePolicy, QFrame, QScrollArea, \
    QWidget, QLayout, QMessageBox, QProgressBar, QShortcut, QCheckBox, QStyle
import numpy as np

from ..widgets import get_icon, show_error
from ..widgets.file_server import FileServer_PathKey
from .utils import crc32_stm32_batch

__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'Delivery Controller Tuning'           # Main panel window title
DESIGN_CONSTANTS_TUNE_NAME = 'Design Constants'  # Label for the design constants section header
PARAM_SET_EDIT_NAME = 'ParamSet Editing'            # Label for the ParamSet editing section
PARAM_SET_ID_NAME = 'ParamSet ID'                   # Label next to the ParamSet ID widget
PARAM_SET_NAME = 'ParamSet'                         # Prefix for individual ParamSet groupbox titles

BUTTON_HORIZONTAL_SPACING = 3                       # Horizontal spacing (px) between buttons in button rows
PARAM_SET_GROUPBOX_HEIGHT = 500                     # Fixed height (px) for each ParamSet editing groupbox
PARAM_SET_GROUPBOX_WIDTH = 240                      # Fixed width (px) for each ParamSet editing groupbox
LEFT_COLUMN_MAX_WIDTH = 260                         # Maximum width (px) of the narrow left-hand button column
RESPONSE_TIMEOUT = 3                                # Seconds to wait for a response to a sent message before showing a timeout error dialog
CONFIG_FILE_TRANSFER_TIMEOUT = 30                   # Number of seconds to wait for a param file upload/download to complete before showing a timeout error dialog
BROADCAST_PRIORITY = 16                             # DroneCAN message broadcast priority (lower number = higher priority)
DOWNLOAD_CONFIG_FILE_NAME = 'delcon_param_set'      # Remote file name requested via GetInfo after a successful ReadConfigFile response

BOOL_MIN = 0
BOOL_MAX = 1
FLOAT32_MIN = float(np.finfo(np.float32).min)
FLOAT32_MAX = float(np.finfo(np.float32).max)
FLOAT64_MIN = float(np.finfo(np.float64).min)
FLOAT64_MAX = float(np.finfo(np.float64).max)
FLOAT_DECIMALS = 5

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

class DesignConstantsSetPayload(ctypes.LittleEndianStructure):
    '''
    @brief    Class representing the payload of a DesignConstantsSet message, responsible for parsing and storing field values.
    @         Must match `design_constants_set_t` (see the The Delivery Controller firmware) field-for-field, including
    @         its explicit trailing pad bytes; `homing_window_ms` is FreeRTOS's `TickType_t`, assumed to be 32-bit here.
    '''
    _pack_ = 1

    SIZE: int
    _fields_ = [
        ('spool_width_m', ctypes.c_float),
        ('wire_diameter_m', ctypes.c_float),
        ('spool_barrel_diameter_m', ctypes.c_float),
        ('horizontal_packing', ctypes.c_float),
        ('radial_packing', ctypes.c_float),
        ('total_wire_length_m', ctypes.c_float),
        ('dead_wire_length_m', ctypes.c_float),
        ('gearbox_ratio', ctypes.c_float),

        ('homing_max_torque_Nm', ctypes.c_float),
        ('homing_window_ms', ctypes.c_uint32),
        ('torque_constant_Nm_A', ctypes.c_float),
        ('_pad', ctypes.c_uint8 * 3),
        ('reverse_phase_sequence', ctypes.c_bool),
    ]

    def set(self, **kwargs):
        '''
        @brief    Set multiple fields of the DesignConstantsSetPayload at once using keyword arguments.
        @param    kwargs - Field names and values to set (e.g. spool_width_m=0.5, gearbox_ratio=10.0).
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
        return bytes(self)

    def deserialize(self, data, offset=0):
        '''
        @brief    Deserialize a DesignConstantsSetPayload from a bytes-like object.
        @param    data - Input bytes-like buffer.
        @param    offset - Starting index in the input buffer.
        @return   Next offset after parsing this payload.
        '''
        buffer = memoryview(data)
        size = ctypes.sizeof(self)
        end = offset + size
        if end > len(buffer):
            raise ValueError(f'Not enough data to deserialize DesignConstantsSetPayload: need {size} bytes from offset {offset}, got {len(buffer) - offset}')

        ctypes.memmove(ctypes.addressof(self), bytes(buffer[offset:end]), size)
        return end


DesignConstantsSetPayload.SIZE = ctypes.sizeof(DesignConstantsSetPayload)

class ParamSetPayload(ctypes.LittleEndianStructure):
    '''
    @brief    Class representing the payload of a ParamSet message, responsible for parsing and storing field values.
    @         Must match paramset-manager's `param_set_t` (see the The Delivery Controller firmware) field-for-field.
    @		  TODO: automatically generate this based on the actual param_set_t definition?
    '''
    _pack_ = 1

    SIZE: int
    _fields_ = [
        ('param_set_id', ctypes.c_uint16),

        ('energize', ctypes.c_bool),
        ('load_not_shaft_control', ctypes.c_bool),
        ('execute_not_hold', ctypes.c_bool),
        ('ob_hold_on_event', ctypes.c_bool),
        ('ob_autostop', ctypes.c_bool),

        ('_pad', ctypes.c_uint8 * 1),

        ('min_effort_limit', ctypes.c_float),
        ('max_effort_limit', ctypes.c_float),

        ('ob_neg_departure_error', ctypes.c_float),
        ('ob_neg_departure_window_s', ctypes.c_float),
        ('ob_pos_departure_error', ctypes.c_float),
        ('ob_pos_departure_window_s', ctypes.c_float),
        ('ob_stall_min_speed', ctypes.c_float),
        ('ob_stall_window_s', ctypes.c_float),
        ('ob_target_margin', ctypes.c_float),
        ('ob_on_target_window_s', ctypes.c_float),
        ('ob_tension_min_effort', ctypes.c_float),
        ('ob_tension_max_effort', ctypes.c_float),
        ('ob_tension_window_s', ctypes.c_float),

        ('tg_position_setpoint', ctypes.c_float),
        ('tg_acceleration', ctypes.c_float),
        ('tg_deceleration', ctypes.c_float),
        ('tg_halt_deceleration', ctypes.c_float),
        ('tg_velocity', ctypes.c_float),
        ('tg_movement_time_s', ctypes.c_float)
    ]

    def set(self, **kwargs):
        '''
        @brief    Set multiple fields of the ParamSetPayload at once using keyword arguments.
        @param    kwargs - Field names and values to set (e.g. param_set_id=1, tg_position_setpoint=0.5).
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
        return bytes(self)

    def deserialize(self, data, offset=0):
        '''
        @brief    Deserialize a ParamSetPayload from a bytes-like object.
        @param    data - Input bytes-like buffer.
        @param    offset - Starting index in the input buffer.
        @return   Next offset after parsing this payload.
        '''
        buffer = memoryview(data)
        size = ctypes.sizeof(self)
        end = offset + size
        if end > len(buffer):
            raise ValueError(f'Not enough data to deserialize ParamSetPayload: need {size} bytes from offset {offset}, got {len(buffer) - offset}')

        ctypes.memmove(ctypes.addressof(self), bytes(buffer[offset:end]), size)
        return end


ParamSetPayload.SIZE = ctypes.sizeof(ParamSetPayload)


def _ctypes_struct_to_dict(struct_instance) -> dict:
    '''
    @brief    Convert a ctypes.Structure instance to a plain, JSON-serializable dict, field-for-field.
    @         Fields whose name starts with an underscore (e.g. `_pad`) are internal padding, not
    @         meaningful data, and are omitted so the JSON stays human-readable/editable.
    @param    struct_instance - The ctypes.Structure instance to convert.
    @return   Dict mapping field name to value.
    '''
    result = {}
    for field_name, _field_type in type(struct_instance)._fields_:
        if field_name.startswith('_'):
            continue
        result[field_name] = getattr(struct_instance, field_name)
    return result


def _ctypes_struct_from_dict(struct_instance, data: dict):
    '''
    @brief    Populate a ctypes.Structure instance from a dict produced by _ctypes_struct_to_dict, field-for-field.
    @         Padding fields (name starts with an underscore) are not expected in `data` and are left
    @         at their zero-initialized default.
    @param    struct_instance - The ctypes.Structure instance to populate.
    @param    data - Dict mapping field name to value.
    @return   The same struct_instance, populated.
    '''
    for field_name, _field_type in type(struct_instance)._fields_:
        if field_name.startswith('_'):
            continue
        if field_name not in data:
            raise ValueError(f'Missing field "{field_name}" for {type(struct_instance).__name__} in JSON data')
        setattr(struct_instance, field_name, data[field_name])
    return struct_instance


class ParamSetFile:
    '''
    @brief    Class representing a ParamSet definition file, responsible for parsing the file and providing field definitions.
    '''

    CRC_HEADER_INITIAL = 0x560D5450  # Initial CRC value for the header section, used to verify that the header CRC is calculated correctly (matches the C++ implementation)
    HEADER_FORMAT = '<HHII'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    VERSION = 1

    BIN_EXTENSION = '.delconparamb'
    JSON_EXTENSION = '.delconparama'

    BIN_FILTER = f'Binary Delivery Controller Parameters (*{BIN_EXTENSION})'
    JSON_FILTER = f'Text Delivery Controller Parameters (*{JSON_EXTENSION})'

    _version: int = VERSION                         # Version string from the ParamSet file (e.g. "1.0"). 2 bytes
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
        # Serialize header: version (2 bytes), num_param_set (2 bytes), hdr_crc (4 bytes), payload_crc (4 bytes)
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

    def toJSON(self) -> str:
        '''
        @brief    Serialize the ParamSetFile to a human-readable JSON string, intended to be
        @         hand-edited by a non-expert. Omits values that are meaningless to a human and
        @         are not needed to reconstruct a valid binary file: the header/payload CRCs
        @         and num_param_set (recalculated by fromJSON) and any struct padding bytes
        @         (always zeroed).
        @return   JSON string.
        '''
        data = {
            'version': self._version,
            'design_constants_set': _ctypes_struct_to_dict(self._design_constants_set_payload),
            'param_sets': [_ctypes_struct_to_dict(param_set) for param_set in self._param_sets],
        }
        return json.dumps(data, indent=2)

    @classmethod
    def fromJSON(cls, json_str: str) -> 'ParamSetFile':
        '''
        @brief    Deserialize a ParamSetFile from a JSON string produced by toJSON.
        @         The CRCs and num_param_set are not stored in the JSON (since a human editing it
        @         would have no way to keep them consistent); they are recalculated here instead.
        @param    json_str - JSON string to parse.
        @return   New ParamSetFile instance with freshly calculated CRCs.
        '''
        data = json.loads(json_str)

        file = cls()
        file._version = data['version']
        file._design_constants_set_payload = _ctypes_struct_from_dict(
            DesignConstantsSetPayload(), data['design_constants_set'])
        file._param_sets = [
            _ctypes_struct_from_dict(ParamSetPayload(), param_set) for param_set in data['param_sets']
        ]
        file._num_param_set = len(file._param_sets)
        file.calculate_crc()

        return file

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
        self._hdr_crc = crc32_stm32_batch(header_bytes, self.CRC_HEADER_INITIAL)

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
        self._payload_crc = crc32_stm32_batch(payload_bytes, self.CRC_HEADER_INITIAL)

        return self._payload_crc

    def calculate_crc(self) -> int:
        '''
        @brief    Calculate the CRC32 of the entire ParamSetFile (header + payload).
        @         Payload CRC is calculated first, then header CRC is calculated.
        @return   Calculated CRC32 value as an integer.
        '''
        self._payload_crc = self.calculate_payload_crc()
        self._hdr_crc = self.calculate_header_crc()

        return crc32_stm32_batch(self.serialize(), self.CRC_HEADER_INITIAL)

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
    @dataclass
    class NetLockCmd:
        net_up : bool = True
        lock_lock : bool = True

    
    TEXT_UPLOAD_BTN = '&Upload Params'
    TEXT_DOWNLOAD_BTN = '&Download Params'
    
    _file_download_finished_signal = pyqtSignal(bool, str)  # success, error_message
    _file_download_progress_signal = pyqtSignal(int)             # percent 0-100

    def __init__(self, parent, node):
        super().__init__(parent)
        self.setWindowTitle(PANEL_NAME)
        self.setWindowIcon(get_icon())
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.resize(900, 600)
        self.setMinimumSize(700, 400)

        self._node = node                      # Local DroneCAN node used for broadcasting messages and registering handlers
        self._node_param_helper = NodeParametersHelper(self._node)
        self._monitor = dronecan.app.node_monitor.NodeMonitor(node)
        self._param_set_id_list = []           # List of ParamSet IDs currently being edited
        self._param_set_color_map = {}         # param_set_id -> background color string assigned to its groupbox
        self._available_colors = list(_PARAM_SET_LIGHT_COLORS)  # Colors from the palette not currently assigned to any groupbox
        self._param_set_dirty = {}             # param_set_id -> bool indicating whether any field was edited since opening
        self._param_set_field_inputs = {}      # param_set_id -> {field_name: (QLineEdit, type_str, min_val, max_val)} for each ParamSet groupbox
        self._param_set_groupboxes = {}        # param_set_id -> QGroupBox widget for each ParamSet editing groupbox

        self._last_netlock_cmd = SpoolControllerPanel.NetLockCmd()

        self._param_set_file: ParamSetFile = ParamSetFile()          # Currently loaded ParamSetFile object, used for editing and uploading
        self._working_file_path = None          # The file under edit

        # Load the design constants definition file
        self._design_const_view_config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'DesignConstantsSet.json')
        self._design_constants_fields = self._load_design_constants_fields()  # Parsed DesignConstantsSet.json field definitions
        # Load the param set definition file
        self._param_set_view_config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'ParamSet.json')
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
        self._download_getinfo_timer = None            # QTimer for file.GetInfo timeout during download
        self._config_transfer_timer = None             # QTimer for param file transfer overall timeout
        self._config_transfer_inactivity_timer = None  # QTimer for polling file server hit counters (inactivity detection)
        self._config_transfer_progress_timer = None    # QTimer for fast progress bar updates during transfer
        self._config_transfer_key = None               # File server key used to track transfer activity
        self._config_transfer_start_hits = 0           # Hit count at transfer start
        self._config_transfer_last_hits = 0            # Hit count at last inactivity poll
        self._store_constants_response_handle = None   # DroneCAN handler handle for DesignConstantsSet store response
        self._store_constants_timeout_timer = None     # QTimer for DesignConstantsSet store timeout
        self._pending_store_constants_snapshot = None  # Snapshot of values sent during OPERATION_STORE

        self._file_download_thread = None              # Background thread for file.Read download
        self._file_download_stop = threading.Event()   # Event to signal the download thread to stop
        self._file_download_timeout_timer = None        # QTimer for overall file download deadline
        self._file_download_finished_signal.connect(self._on_file_download_finished)
        self._file_download_progress_signal.connect(lambda pct: self._config_transfer_progress.setValue(pct))

        self._temporary_file_bytes = None # This is to store the received ParamSetFile
        self._temporary_file = tempfile.NamedTemporaryFile(delete=False) # This for the uploaded ParamSetFile
        atexit.register(self._tempfile_cleanup)

        self._setup_ui()
        self._update_window_data()

    def _setup_ui(self):
        '''
        @brief    Main UI setup function that creates the window layout.
        @return   None
        '''

        self.setWindowFlag(Qt.WindowMaximizeButtonHint, True)

        layout = QVBoxLayout(self)

        # Create groupbox with header labels and sub-groupboxes
        header_group = QGroupBox(self)
        header_layout = QVBoxLayout(header_group)

        columns_row = QHBoxLayout()

        # Left area (narrow): Parameter File Management buttons; Design Constants Tuning
        # is opened via a button here rather than embedded in this layout.
        left_column = self._make_left_column(header_group)
        left_container = QWidget(header_group)
        left_container.setLayout(left_column)
        left_container.setMaximumWidth(LEFT_COLUMN_MAX_WIDTH)
        columns_row.addWidget(left_container, 0)

        # Right area: fully occupied by the ParamSet Editing section.
        right_column = QVBoxLayout()
        right_column.setContentsMargins(0, 0, 0, 0)
        right_column.setSpacing(6)

        # ParamSet Editing label
        param_set_edit_label = QLabel(PARAM_SET_EDIT_NAME, header_group)
        font_secondary = QFont()
        font_secondary.setBold(True)
        param_set_edit_label.setFont(font_secondary)
        right_column.addWidget(param_set_edit_label, 0, Qt.AlignLeft)

        # Sunken line below the label
        param_set_line = QFrame(header_group)
        param_set_line.setFrameShape(QFrame.HLine)
        param_set_line.setFrameShadow(QFrame.Sunken)
        right_column.addWidget(param_set_line)

        # ParamSet ID label, widget, and Edit/Delete buttons
        param_set_id_row = QHBoxLayout()
        param_set_id_label = QLabel(PARAM_SET_ID_NAME + ':', header_group)
        param_set_id_row.addWidget(param_set_id_label)

        self._text_box_param_set_id = QLineEdit(header_group)
        self._text_box_param_set_id.setFixedWidth(120)
        self._text_box_param_set_id.setToolTip('Enter a single ID (e.g., 5), range (e.g., 1-10), comma-separated IDs (e.g., 1, 5, 3), or any combination (e.g., 1, 3-6, 9, 15-17)')
        param_set_id_row.addWidget(self._text_box_param_set_id)

        self._edit_button = QPushButton('Edit', header_group)
        self._edit_button.clicked.connect(self._on_edit_clicked)
        param_set_id_row.addWidget(self._edit_button)

        self._delete_button = QPushButton('Delete', header_group)
        self._delete_button.clicked.connect(self._on_delete_clicked)
        param_set_id_row.addWidget(self._delete_button)

        param_set_id_row.addStretch(1)
        right_column.addLayout(param_set_id_row)

        # Scrollable area for ParamSet editing content, filling remaining vertical space
        self._param_set_scroll_area = QScrollArea(header_group)
        self._param_set_scroll_area.setWidgetResizable(True)
        self._param_set_scroll_area.setFrameShape(QFrame.StyledPanel)
        self._param_set_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._param_set_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self._param_set_container = _FlowContainer()
        self._param_set_container_layout = FlowLayout(self._param_set_container, margin=2, hSpacing=4, vSpacing=4)
        self._param_set_scroll_area.setWidget(self._param_set_container)

        right_column.addWidget(self._param_set_scroll_area, 1)

        columns_row.addLayout(right_column, 1)

        header_layout.addLayout(columns_row)

        layout.addWidget(header_group)

        # DesignConstantsSet Tuning lives in its own (non-modal) child window, opened via
        # a button in the Parameter File Management area.
        self._create_design_constants_window()

    def _make_left_column(self, parent):
        '''
        @brief    Create the Parameter File Management section.
        @param    parent - Parent widget.
        @return   QVBoxLayout containing the section.
        '''
        left_column = QVBoxLayout()
        left_column.setContentsMargins(0, 0, 0, 0)
        left_column.setSpacing(6)

        STATUS_LABEL_WIDTH = 60
        STATUS_TEXTBOX_WIDTH = 80

        self._param_file_groupbox = QGroupBox('Parameter File', parent)
        param_file_groupbox = self._param_file_groupbox

        # Buttons are stacked vertically to fit within the narrow left column.
        save_load_layout = QVBoxLayout(param_file_groupbox)

        version_row = QHBoxLayout()
        version_row.setSpacing(6)
        version_label = QLabel('Version:', param_file_groupbox)
        version_label.setFixedWidth(STATUS_LABEL_WIDTH)
        version_row.addWidget(version_label)
        self._version_textbox = QLabel(param_file_groupbox)
        self._version_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
        version_row.addWidget(self._version_textbox)
        version_row.addStretch(1)

        crc32_row = QHBoxLayout()
        crc32_row.setSpacing(6)
        crc32_label = QLabel('CRC32:', param_file_groupbox)
        crc32_label.setFixedWidth(STATUS_LABEL_WIDTH)
        crc32_row.addWidget(crc32_label)
        self._crc32_textbox = QLabel(param_file_groupbox)
        self._crc32_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
        crc32_row.addWidget(self._crc32_textbox)
        crc32_row.addStretch(1)

        save_load_layout.addLayout(version_row)
        save_load_layout.addLayout(crc32_row)

        self._open_button = QPushButton('&Open', param_file_groupbox)
        self._open_button.clicked.connect(self._on_open_clicked)
        save_load_layout.addWidget(self._open_button)

        self._reload_button = QPushButton('&Reload', param_file_groupbox)
        self._reload_button.clicked.connect(self._on_reload_clicked)
        self._reload_button.setEnabled(False)
        save_load_layout.addWidget(self._reload_button)

        self._save_button = QPushButton('Save', param_file_groupbox)
        self._save_button.clicked.connect(self._on_save_clicked)
        self._save_button.setEnabled(False)
        save_load_layout.addWidget(self._save_button)
        shortcut = QShortcut(QKeySequence('Ctrl+S'), self)
        shortcut.activated.connect(self._on_save_clicked)

        self._save_as_button = QPushButton('&Save As', param_file_groupbox)
        self._save_as_button.clicked.connect(self._on_save_as_clicked)
        save_load_layout.addWidget(self._save_as_button)

        self._design_constants_button = QPushButton('&Design Constants', parent)
        self._design_constants_button.clicked.connect(self._on_open_design_constants_clicked)
        save_load_layout.addWidget(self._design_constants_button)

        left_column.addWidget(self._param_file_groupbox)

        upload_download = QGroupBox(self)
        upload_download.setTitle('Upload/Download')
        upload_download_layout = QVBoxLayout(upload_download)

        # Upload/Download buttons
        self._upload_button = QPushButton('&Upload Params', upload_download)
        self._upload_button.clicked.connect(self._on_upload_clicked)

        self._download_button = QPushButton('&Download Params', upload_download)
        self._download_button.clicked.connect(self._on_download_clicked)

        self._config_transfer_progress = QProgressBar(param_file_groupbox)
        self._config_transfer_progress.setRange(0, 100)
        self._config_transfer_progress.setValue(0)
        self._config_transfer_progress.setAlignment(Qt.AlignCenter)

        upload_download_layout.addWidget(self._config_transfer_progress)
        upload_download_layout.addWidget(self._upload_button)
        upload_download_layout.addWidget(self._download_button)
        left_column.addWidget(upload_download)

        left_column.addWidget(self._make_device_ops_section(self))
        left_column.addStretch(1)

        return left_column


    def _make_device_ops_section(self, parent):
        # Device operations
        ops_groupbox = QGroupBox(self)
        ops_groupbox.setTitle('Device Operations')

        layout = QVBoxLayout(ops_groupbox)

        set_override_button = QPushButton('Override', ops_groupbox)
        set_override_button.clicked.connect(lambda _: self._send_mode_command(DeliveryControllerMode.DIRECT_OVERRIDE))

        emergency_release_button = QPushButton('&Emergency Release', ops_groupbox)
        emergency_release_button.clicked.connect(lambda _: self._send_mode_command(DeliveryControllerMode.RELEASE_WIRE))
        emergency_release_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_MessageBoxCritical))

        homing_button = QPushButton('&Homing', ops_groupbox)
        homing_button.clicked.connect(lambda _: self._send_mode_command(DeliveryControllerMode.HOMING))

        align_encoder_button = QPushButton('Ali&gn Encoder', ops_groupbox)
        align_encoder_button.clicked.connect(lambda _: self._send_mode_command(DeliveryControllerMode.ALIGN_ENCODER))

        layout.addWidget(emergency_release_button)
        layout.addWidget(set_override_button)
        layout.addWidget(align_encoder_button)
        layout.addWidget(homing_button)

        net = QGroupBox(self)
        net.setTitle('Net')

        net_up = QPushButton('Up', net)
        net_up.clicked.connect(lambda _: self._send_netlock_command(net_up=True))
        net_down = QPushButton('Down', net)
        net_down.clicked.connect(lambda _: self._send_netlock_command(net_up=False))
        net_layout = QHBoxLayout(net)
        net_layout.addWidget(net_up)
        net_layout.addWidget(net_down)

        layout.addWidget(net)

        lock = QGroupBox(self)
        lock.setTitle('Lock')

        lock_lock = QPushButton('Lock', net)
        lock_lock.clicked.connect(lambda _: self._send_netlock_command(lock_lock=True))
        lock_unlock = QPushButton('Unlock', net)
        lock_unlock.clicked.connect(lambda _: self._send_netlock_command(lock_lock=False))
        lock_layout = QHBoxLayout(lock)
        lock_layout.addWidget(lock_lock)
        lock_layout.addWidget(lock_unlock)
        layout.addWidget(lock)

        return ops_groupbox

    def _find_first_delcon(self):
        first_delcon = None
        for node in self._monitor.find_all(lambda node_:
                                           True if node_.info and str(node_.info.name).startswith('com.flytrex.delcon')
                                           else False):
            first_delcon = node
            break

        return first_delcon.node_id if first_delcon else None

    def _send_netlock_command(self, net_up = None, lock_lock = None):
        if net_up is not None:
            self._last_netlock_cmd.net_up = net_up
        if lock_lock is not None:
            self._last_netlock_cmd.lock_lock = lock_lock

        msg = dronecan.flytrex.delcon.NetLockCommand(net_up = self._last_netlock_cmd.net_up,
                                                     lock_lock = self._last_netlock_cmd.lock_lock)
        self._node.broadcast(msg)

    def _send_mode_command(self, mode : DeliveryControllerMode):
        cmd = DeliveryControllerCommand(mode)
        try:
            self._node_param_helper.delcon_mode_command(self._find_first_delcon(), cmd)
        except Exception as e:
            show_error(title='Failed to send command', text='Mode command not sent',
                       informative_text=str(e), blocking=False, parent=self)
            return

    def _tempfile_cleanup(self):
        if self._temporary_file is not None:
            try:
                self._temporary_file.close()
                os.unlink(self._temporary_file.name)
                self._temporary_file = None
            except FileNotFoundError:
                pass

    def _paramsetfile_to_tempfile(self):
        self._tempfile_cleanup()
        self._temporary_file = tempfile.NamedTemporaryFile(delete=False)
        self._temporary_file.write(self._param_set_file.serialize())
        self._temporary_file.close()

    def _on_upload_clicked(self):
        '''
        @brief    Handle upload button click: validate local file, configure file server, and send WriteConfigFile.
                  If an upload is already in progress (button shows 'Cancel'), cancel the transfer.
        @return   None
        '''
        if self._upload_button.text() == 'Cancel':
            self._cancel_upload()
            return

        self._config_transfer_progress.setValue(0)
        self._cleanup_config_transfer_timeout()

        self._extract_param_set_file_from_ui()
        self._populate_ui_from_param_set_file()
        self._clean_all_dirty()
        self._update_window_data()

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

        try:
            if self._temporary_file:
                file_server_widget.remove_path(self._temporary_file.name)
        except Exception as ex:
            pass

        self._paramsetfile_to_tempfile()

        # Add the file to the file server
        try:
            file_server_widget.add_path(self._temporary_file.name)
            file_server_widget.force_start()
            logger.info('File server configured for: %s', self._temporary_file.name)
        except Exception as ex:
            logger.exception('Could not configure file server: %s', ex)
            show_error('File Server Error', 'Could not configure file server.', str(ex), parent=self, blocking=True)
            return

        # Get the remote path that the file server will use
        remote_config_file = FileServer_PathKey(os.path.normcase(self._temporary_file.name))
        logger.info('Remote param file path: %r', remote_config_file)
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

        # Switch button to Cancel mode and disable browse buttons while waiting for response
        self._upload_button.setText('Cancel')

        try:
            self._node.broadcast(msg, priority=BROADCAST_PRIORITY)
            logger.info('Broadcast WriteConfigFile for %s', remote_config_file)
        except Exception as ex:
            logger.exception('Failed to broadcast WriteConfigFile: %s', ex)
            show_error('Broadcast failed', 'Could not broadcast WriteConfigFile.', str(ex), parent=self, blocking=True)
            self._upload_button.setText(SpoolControllerPanel.TEXT_UPLOAD_BTN)
            return

        # Register handler for the response message
        try:
            self._upload_response_handle = self._node.add_handler(
                dronecan.flytrex.delcon.WriteConfigFile,
                self._on_upload_response,
            )
        except Exception as ex:
            logger.exception('Could not register WriteConfigFile handler: %s', ex)
            self._upload_button.setText(SpoolControllerPanel.TEXT_UPLOAD_BTN)
            self._cleanup_upload_handler()
            return

        # Start timeout timer
        self._upload_timeout_timer = QTimer(self)
        self._upload_timeout_timer.setSingleShot(True)
        self._upload_timeout_timer.timeout.connect(
            lambda: (
                self._cleanup_upload_handler(),
                self._config_transfer_progress.setValue(0),
                self._on_recall_timeout(
                    f'No WriteConfigFile response was received within {RESPONSE_TIMEOUT} seconds.\n\n'
                    f'The Delivery Controller may be offline or not responding.'
                ),
            )
        )
        self._upload_timeout_timer.start(RESPONSE_TIMEOUT * 1000)

        self._show_message('Upload request sent. Waiting for Delivery Controller response...')

    def _on_download_clicked(self):
        '''
        @brief    Handle download button click: validate destination path and send ReadConfigFile.
        @return   None
        '''

        if self._temporary_file_bytes is not None:
            self._show_ok_dialog('Another operation is in progress.', QMessageBox.Warning)

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
            self._node.broadcast(msg, priority=BROADCAST_PRIORITY)
        except Exception as ex:
            logger.exception('Failed to broadcast ReadConfigFile: %s', ex)
            show_error('Broadcast failed', 'Could not broadcast ReadConfigFile.', str(ex), parent=self, blocking=True)
            self._download_button.setEnabled(True)
            return

        self._temporary_file = tempfile.TemporaryFile()

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
                    f'The Delivery Controller may be offline or not responding.'
                ),
            )
        )
        self._download_timeout_timer.start(RESPONSE_TIMEOUT * 1000)

    @staticmethod
    def _parse_param_set_id_input(input_str: str) -> list[str]:
        '''
        @brief    Parse ParamSet ID input that can be a single ID, range, comma-separated IDs, or any combination.
        @param    input_str - User input string (e.g., '5' or '1-10' or '1, 5, 3' or '1, 3-6, 9, 15-17').
        @return   List of ID strings to process, or empty list on error.
        '''
        input_str = input_str.strip()
        if not input_str:
            return []

        # Split by comma and process each part (single ID or range)
        ids = []
        for part in input_str.split(','):
            part = part.strip()
            if not part:
                logger.warning('Empty part in input')
                return []

            # Check if this part is a range (contains hyphen)
            if '-' in part:
                parts = part.split('-')
                if len(parts) != 2:
                    logger.warning('Invalid range format: %s', part)
                    return []
                try:
                    start = int(parts[0].strip())
                    end = int(parts[1].strip())
                    if start < 0 or end < 0 or start > end:
                        logger.warning('Invalid range values: start=%d, end=%d', start, end)
                        return []
                    ids.extend([str(i) for i in range(start, end + 1)])
                except ValueError:
                    logger.warning('Could not parse range: %s', part)
                    return []
            else:
                # Single ID
                try:
                    id_val = int(part)
                    if id_val < 0:
                        logger.warning('Invalid ParamSet ID: %d', id_val)
                        return []
                    ids.append(str(id_val))
                except ValueError:
                    logger.warning('Could not parse ParamSet ID: %s', part)
                    return []

        return ids

    def _on_edit_clicked(self):
        '''
        @brief    Handle Edit button click for ParamSet editing.
        @return   None
        '''
        input_str = self._text_box_param_set_id.text().strip()
        if not input_str:
            show_error('Edit Error', 'Please enter a ParamSet ID, range, or combination.', '', parent=self, blocking=True)
            return

        param_set_ids = self._parse_param_set_id_input(input_str)
        if not param_set_ids:
            show_error(
                'Edit Error',
                'Invalid ParamSet ID format.',
                'Enter a single ID (e.g., 5), range (e.g., 1-10), comma-separated IDs (e.g., 1, 5, 3), or combination (e.g., 1, 3-6, 9, 15-17).',
                parent=self,
                blocking=True
            )
            return

        # Check for duplicate IDs (already being edited) before adding any
        duplicate_ids = [pid for pid in param_set_ids if pid in self._param_set_id_list]
        if duplicate_ids:
            show_error(
                'Duplicate ParamSet ID',
                f'ParamSet ID(s) already being edited: {", ".join(duplicate_ids)}',
                'Aborting operation.',
                parent=self,
                blocking=True
            )
            return

        logger.info('Edit clicked for ParamSet IDs: %s', param_set_ids)
        for param_set_id in param_set_ids:
            self._add_param_set_editing_content(param_set_id)

    def _on_delete_clicked(self):
        '''
        @brief    Handle Delete button click for ParamSet deletion.
        @return   None
        '''
        input_str = self._text_box_param_set_id.text().strip()
        if not input_str:
            show_error('Delete Error', 'Please enter a ParamSet ID, range, or combination.', '', parent=self, blocking=True)
            return

        param_set_ids = self._parse_param_set_id_input(input_str)
        if not param_set_ids:
            show_error(
                'Delete Error',
                'Invalid ParamSet ID format.',
                'Enter a single ID (e.g., 5), range (e.g., 1-10), comma-separated IDs (e.g., 1, 5, 3), or combination (e.g., 1, 3-6, 9, 15-17).',
                parent=self,
                blocking=True
            )
            return

        logger.info('Delete clicked for ParamSet IDs: %s', param_set_ids)
        deleted_count = 0
        for param_set_id in param_set_ids:
            # Only delete if the ID exists (no error if it doesn't)
            if param_set_id in self._param_set_groupboxes:
                groupbox = self._param_set_groupboxes[param_set_id]
                self._delete_param_set_groupbox(param_set_id, groupbox)
                deleted_count += 1

        if deleted_count > 0:
            logger.info('Deleted %d ParamSet groupbox(es)', deleted_count)

    def _extract_param_set_file_from_ui(self) -> bool:
        '''
        @brief    Clear self._param_set_file and repopulate it (Design Constants fields plus all
                  open ParamSet groupboxes) from the current UI field values, then recalculate its CRC.
        @return   True on success; False if a field failed to parse (an error dialog was already shown).
        '''

        self._param_set_file.clear()

        temp_design_constants = DesignConstantsSetPayload()
        # Extract the current values from the Design Constants fields
        for field_name, widget in self._field_inputs.items():
            raw_value = None
            if isinstance(widget, QCheckBox):
                raw_value = widget.isChecked()
                setattr(temp_design_constants, field_name, bool(raw_value))
            elif isinstance(widget, QLineEdit):
                raw_value = widget.text().strip()
                if isinstance(widget.validator(), QIntValidator):
                    setattr(temp_design_constants, field_name, int(raw_value))
                else:
                    setattr(temp_design_constants, field_name, float(raw_value))
            assert raw_value is not None

        self._param_set_file.set_design_constants(temp_design_constants)

        # Extract current field values from all open ParamSet groupboxes.
        for param_set_id, _groupbox in self._param_set_groupboxes.items():
            field_inputs = self._param_set_field_inputs.get(param_set_id, {})
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
                return False

            # Extract and set field values
            for field_name, (widget, field_type, *_) in field_inputs.items():
                raw_value = None
                if isinstance(widget, QCheckBox):
                    raw_value = widget.isChecked()
                    setattr(temp_param_set_payload, field_name, bool(raw_value))
                elif isinstance(widget, QLineEdit):
                    raw_value = widget.text().strip()
                    if isinstance(widget.validator(), QIntValidator):
                        setattr(temp_param_set_payload, field_name, int(raw_value))
                    else:
                        setattr(temp_param_set_payload, field_name, float(raw_value))
                assert raw_value is not None

            # Add the populated payload to the param set file
            self._param_set_file.add_param_set(temp_param_set_payload)

        logger.info('Extracted fields from %d ParamSet groupboxes', len(self._param_set_groupboxes))
        self._param_set_file.calculate_crc()
        return True

    def _populate_ui_from_param_set_file(self) -> None:
        '''
        @brief    Populate the Design Constants fields, ParamSet groupboxes, and version/CRC/dirty
                  textboxes from the currently loaded self._param_set_file.
        @return   None
        '''

        self._clear_all_param_set_groupboxes()

        design_constants_payload = self._param_set_file._design_constants_set_payload
        for field_name, textbox in self._field_inputs.items():
            if not hasattr(design_constants_payload, field_name):
                continue
            value = getattr(design_constants_payload, field_name)
            self._set_param_edit_value_guarded(textbox, value)


        for param_set_payload in self._param_set_file._param_sets:
            param_set_id = str(param_set_payload.param_set_id)
            self._add_param_set_editing_content(param_set_id)
            field_inputs = self._param_set_field_inputs.get(param_set_id, {})
            for field_name, (textbox, _field_type, *_) in field_inputs.items():
                if not hasattr(param_set_payload, field_name):
                    continue
                value = getattr(param_set_payload, field_name)
                self._set_param_edit_value_guarded(textbox, value)
            self._param_set_dirty[param_set_id] = False

        self._version_textbox.setText(str(self._param_set_file._version))
        self._crc32_textbox.setText(f'{self._param_set_file.calculate_crc():08X}')

    def _params_load_text(self, file):
        try:
            with open(file, 'r') as f:
                data = f.read()

            new = ParamSetFile.fromJSON(data)
            self._param_set_file = new
        except Exception as ex:
            logger.exception('Failed to open param file: %s', ex)
            show_error('Read Error', 'Could not parse param file.', str(ex), parent=self, blocking=True)
            return

    def _params_load_binary(self, file):
        try:
            with open(file, 'rb') as f:
                data = f.read()

            new = ParamSetFile()
            new.deserialize(data)
            self._param_set_file = new
        except Exception as ex:
            logger.exception('Failed to open param file: %s', ex)
            show_error('Read Error', 'Could not parse param file.', str(ex), parent=self, blocking=True)
            return

    def _on_reload_clicked(self):
        if not self._working_file_path:
            return

        if self._working_file_path.endswith(ParamSetFile.JSON_EXTENSION):
            self._params_load_text(self._working_file_path)
        else:
            self._params_load_binary(self._working_file_path)

        self._populate_ui_from_param_set_file()
        self._clean_all_dirty()
        self._update_window_data()

    def _on_open_clicked(self):
        directory = os.path.dirname(self._working_file_path) if self._working_file_path is not None else QtCore.QDir.homePath()

        file_path, file_filter = QFileDialog.getOpenFileName(
            parent=self,
            caption='Open Parameter File',
            directory=directory,
            filter=ParamSetFile.JSON_FILTER + ';;' + ParamSetFile.BIN_FILTER,
            initialFilter=ParamSetFile.JSON_FILTER,
        )

        if not file_path or not file_filter:
            return

        if file_filter == ParamSetFile.JSON_FILTER:
            self._params_load_text(file_path)
        else:
            self._params_load_binary(file_path)

        self._working_file_path = file_path

        self._populate_ui_from_param_set_file()
        self._clean_all_dirty()
        self._update_window_data()

    def _clean_all_dirty(self):
        for k in self._param_set_dirty:
            self._param_set_dirty[k] = False

    def _any_dirty(self):
        if any(self._param_set_dirty.values()):
            return True
        return False

    def _update_window_data(self):
        self._version_textbox.setText(str(self._param_set_file._version))
        self._crc32_textbox.setText(f'{self._param_set_file.calculate_crc():08X}')

        file_path = self._working_file_path if self._working_file_path else '(new file)'
        if self._any_dirty():
            file_path += '*'

        self.setWindowTitle('Delivery Controller Tuning: ' + file_path)

    def _params_save_binary(self, path):
        if not self._extract_param_set_file_from_ui():
            return
        try:
            data = self._param_set_file.serialize()
            with open(path, 'wb') as f:
                f.write(data)
            logger.info('Binary param file written to %s (%d bytes)', path, len(data))

        except Exception as ex:
            logger.exception('Failed to write param file: %s', ex)
            show_error('Save Error', 'Could not write param file.', str(ex), parent=self, blocking=True)

    def _params_save_text(self, path):
        if not self._extract_param_set_file_from_ui():
            return
        try:
            data = self._param_set_file.toJSON()
            with open(path, 'w') as f:
                f.write(data)
            logger.info('Config file written to %s (%d lines)', path, len(data.splitlines()))
        except Exception as ex:
            logger.exception('Failed to write param file: %s', ex)
            show_error('Save Error', 'Could not write param file.', str(ex), parent=self, blocking=True)

    def _on_save_clicked(self):
        if not self._working_file_path:
            return

        if ParamSetFile.BIN_EXTENSION in self._working_file_path:
            self._params_save_binary(self._working_file_path)
        elif ParamSetFile.JSON_EXTENSION in self._working_file_path:
            self._params_save_text(self._working_file_path)
        else:
            return

        self._populate_ui_from_param_set_file()
        self._clean_all_dirty()
        self._update_window_data()

    def _on_save_as_clicked(self):
        # Prompt user for save destination before doing any work
        directory = os.path.dirname(self._working_file_path) if self._working_file_path is not None else QtCore.QDir.homePath()

        file_path, file_filter = QFileDialog.getSaveFileName(
            parent=self,
            caption='Save Parameter File',
            directory=directory,
            filter=ParamSetFile.JSON_FILTER + ';;' + ParamSetFile.BIN_FILTER,
            initialFilter=ParamSetFile.JSON_FILTER,
        )

        if not file_path or not file_filter:
            return

        if file_filter == ParamSetFile.JSON_FILTER:
            self._params_save_text(file_path)
        else:
            self._params_save_binary(file_path)

        self._working_file_path = file_path
        self._save_button.setEnabled(True)
        self._reload_button.setEnabled(True)
        self._clean_all_dirty()
        self._populate_ui_from_param_set_file()
        self._update_window_data()

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

        clear_button = QPushButton('Clear', groupbox)
        clear_button.clicked.connect(lambda: self._on_param_set_clear(param_set_id))
        buttons_layout.addWidget(clear_button)

        execute_button = QPushButton('Execute', groupbox)
        execute_button.clicked.connect(lambda: self._on_param_set_execute(param_set_id))
        buttons_layout.addWidget(execute_button)

        store_button = QPushButton('Store', groupbox)
        store_button.clicked.connect(lambda: self._on_param_set_store(param_set_id))
        buttons_layout.addWidget(store_button)

        recall_button = QPushButton('Recall', groupbox)
        recall_button.clicked.connect(lambda: self._on_param_set_recall(param_set_id))
        buttons_layout.addWidget(recall_button)

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
        param_set_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        param_set_fields_container = QWidget()
        param_set_fields_container.setStyleSheet(f"background-color: {color};")
        param_set_fields_layout = QGridLayout(param_set_fields_container)
        param_set_fields_layout.setColumnStretch(0, 0)
        param_set_fields_layout.setColumnStretch(1, 1)
        param_set_fields_layout.setSpacing(5)
        param_set_fields_layout.setContentsMargins(5, 0, 5, 0)

        self._parse_param_set_file(param_set_id, param_set_fields_container, param_set_fields_layout)

        # Mark groupbox dirty when any field widget is edited
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

        self._delete_param_set_groupbox(param_set_id, groupbox)

    def _delete_param_set_groupbox(self, param_set_id, groupbox):
        '''
        @brief    Delete and deregister a ParamSet editing groupbox without prompting.
        @param    param_set_id - The ID of the ParamSet to delete.
        @param    groupbox - The QGroupBox widget to remove.
        @return   None
        '''
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
        @param    raw_value - The raw string from the widget.
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

    @staticmethod
    def _get_type_range(field_type):
        '''
        @brief    Return the (min_val, max_val) bounds for a given field type.
        @param    field_type - Type string (e.g. 'float32', 'uint16', 'bool').
        @return   Tuple of (min_val, max_val).
        '''
        ft = (field_type or '').strip().lower()
        if ft == 'bool':
            return (BOOL_MIN, BOOL_MAX)
        if ft == 'float32' or ft == 'float':
            return (FLOAT32_MIN, FLOAT32_MAX)
        if ft in ('float64', 'double'):
            return (FLOAT64_MIN, FLOAT64_MAX)
        match = re.fullmatch(r'(u?int)(\d+)', ft) or re.fullmatch(r'(u?int)(\d+)_t', ft)
        if match:
            is_unsigned = match.group(1).startswith('u')
            bit_width = int(match.group(2))
            if is_unsigned:
                return (0, (1 << bit_width) - 1)
            else:
                return (-(1 << (bit_width - 1)), (1 << (bit_width - 1)) - 1)
        return (None, None)

    @staticmethod
    def _clear_value_for_type(field_type):
        '''
        @brief    Return the default clear value text for a field type.
        @param    field_type - Type string (e.g. 'float32', 'uint16', 'bool').
        @return   String value to place in a widget when clearing.
        '''
        ft = (field_type or '').strip().lower()

        if ft == 'bool':
            return 'False'

        if ft.startswith('float') or ft in ('float', 'double'):
            return '0.0'

        if re.fullmatch(r'(u?int)(\d+)', ft) or re.fullmatch(r'(u?int)(\d+)_t', ft):
            return '0'

        return '0'

    def _on_param_set_clear(self, param_set_id):
        '''
        @brief    Clear all fields in the specified ParamSet groupbox.
        @param    param_set_id - The ParamSet ID to clear.
        @return   None
        '''
        field_inputs = self._param_set_field_inputs.get(param_set_id)
        if not field_inputs:
            show_error('Clear Error', 'No fields found for this ParamSet.', '', parent=self, blocking=True)
            return

        for _field_name, (textbox, field_type, *_) in field_inputs.items():
            textbox.setText(self._clear_value_for_type(field_type))

        self._param_set_dirty[param_set_id] = True
        self._update_window_data()

    def _on_design_constants_clear(self):
        '''
        @brief    Clear all DesignConstantsSet fields in the groupbox.
        @return   None
        '''
        if not self._field_inputs:
            show_error('Clear Error', 'No design constant fields found.', '', parent=self, blocking=True)
            return

        for field_name, textbox in self._field_inputs.items():
            field_type = self._design_constants_fields.get(field_name, {}).get('type', '')
            textbox.setText(self._clear_value_for_type(field_type))


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
            for field_name, (widget, field_type, *_) in field_inputs.items():
                if not hasattr(msg, field_name):
                    logger.warning('Field "%s" not found on ParamSet message, skipping', field_name)
                    continue
                msg.param_values.append(0)  # placeholder value for recall
                msg.param_value_types.append(field_type)

        else:
            for field_name, (widget, field_type, *_) in field_inputs.items():
                raw_value = None
                if isinstance(widget, QCheckBox):
                    raw_value = widget.isChecked()
                elif isinstance(widget, QLineEdit):
                    raw_value = widget.text().strip()

                assert raw_value is not None

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
        for field_name, (textbox, field_type, *_) in field_inputs.items():
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

    def _show_ok_dialog(self, title, message, icon : QMessageBox.Icon = QMessageBox.Warning):
        '''
        @brief    Show a warning dialog with a title, message, and a single OK button.
        @param    title - Dialog window title.
        @param    message - Dialog body text.
        @return   None
        '''
        dlg = QMessageBox(self)
        if icon:
            dlg.setIcon(icon)
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
                'The Delivery Controller could not find the file on the file server. Please check that the file was uploaded correctly and try again.',
                f'{operation_context} failed with file not found error'
            )
        elif hasattr(error_obj, 'STATUS_IO_ERROR') and error_value == error_obj.STATUS_IO_ERROR:
            return (
                f'{operation_context.capitalize()} Error',
                'An I/O error occurred while The Delivery Controller was processing the request. Please try again.',
                f'{operation_context} failed with I/O error'
            )
        elif hasattr(error_obj, 'STATUS_ACCESS_DENIED') and error_value == error_obj.STATUS_ACCESS_DENIED:
            return (
                f'{operation_context.capitalize()} Error',
                'The Delivery Controller was denied access. Please check permissions and try again.',
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
                'The file is too large for The Delivery Controller to handle. Please check the file size and try again.',
                f'{operation_context} failed with file too large error'
            )
        elif hasattr(error_obj, 'STATUS_OUT_OF_SPACE') and error_value == error_obj.STATUS_OUT_OF_SPACE:
            return (
                f'{operation_context.capitalize()} Error',
                'The Delivery Controller does not have enough space. Please free up space and try again.',
                f'{operation_context} failed with out of space error'
            )
        elif hasattr(error_obj, 'STATUS_NOT_IMPLEMENTED') and error_value == error_obj.STATUS_NOT_IMPLEMENTED:
            return (
                f'{operation_context.capitalize()} Error',
                f'The Delivery Controller does not support {operation_context} operations. Please check the controller capabilities and try again.',
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
                'The Delivery Controller is busy. Please try again later.',
                f'{operation_context} failed - controller busy'
            )
        elif hasattr(error_obj, 'STATUS_LOW_MEM') and error_value == error_obj.STATUS_LOW_MEM:
            return (
                f'{operation_context.capitalize()} Status',
                'The Delivery Controller has low memory. Please try again later.',
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
                    f'The Delivery Controller may be offline or not responding.'
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
                    f'The Delivery Controller may be offline or not responding.'
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
                    f'The Delivery Controller may be offline or not responding.'
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
                    f'The Delivery Controller may be offline or not responding.'
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
                self._set_param_edit_value_guarded(textbox, value)
        logger.info('DesignConstantsSet received — fields populated')

    @staticmethod
    def _set_param_edit_value_guarded(widget, value):
        widget.blockSignals(True)
        if type(value) is float or type(value) is int:
            assert isinstance(widget, QLineEdit)
            widget.setText(str(round(value, FLOAT_DECIMALS)))
        elif type(value) is bool:
            assert isinstance(widget, QCheckBox)
            widget.setChecked(value)
        else:
            assert isinstance(widget, QLineEdit)
            widget.setText(str(value))
        widget.blockSignals(False)

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
            for field_name, (textbox, field_type, *_) in field_inputs.items():
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

        for field_name, (textbox, field_type, *_) in field_inputs.items():
            value = getattr(msg, field_name, None)
            if value is not None:
                self._set_param_edit_value_guarded(textbox, value)
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
                  The Delivery Controller will automatically request the file from the file server.
        @param    event - DroneCAN transfer event containing the response message.
        @return   None
        '''
        self._cleanup_upload_handler()

        msg = event.message
        logger.info('WriteConfigFile response received: error=%s', msg.error.value)

        try:
            if msg.error.value == msg.error.STATUS_OK:
                logger.info('The configuration file is being uploaded...')
                self._upload_button.setText('Cancel')
                self._start_config_transfer_timeout()
            else:
                error_info = self._parse_error_msg(msg.error, 'upload')
                if error_info:
                    title, message, log_msg = error_info
                    self._config_transfer_progress.setValue(0)
                    self._show_ok_dialog(title, message)
                    logger.warning(log_msg)
        except Exception as ex:
            logger.exception('Error processing upload response: %s', ex)
            self._config_transfer_progress.setValue(0)
            self._show_ok_dialog('Upload Error', f'Error processing upload response: {ex}')

    def _start_config_transfer_timeout(self):
        '''
        @brief    Start timeouts for param file transfer activity.
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

        # Fast progress update timer
        self._config_transfer_progress_timer = QTimer(self)
        self._config_transfer_progress_timer.setSingleShot(False)
        self._config_transfer_progress_timer.timeout.connect(self._on_config_transfer_progress_tick)
        self._config_transfer_progress_timer.start(100)

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
                hits = file_server.key_hit_counters.get(self._config_transfer_key, hits)
        except Exception:
            logger.exception('Could not read file server hit counters')
        return hits

    def _is_config_transfer_complete(self):
        '''
        @brief    Check whether the file server has finished serving the transfer key.
        @return   True if the entire file has been read by the remote node.
        '''
        try:
            file_server_widget = self._get_file_server_widget()
            file_server = getattr(file_server_widget, '_file_server', None)
            if file_server is not None and self._config_transfer_key:
                return file_server.is_key_complete(self._config_transfer_key)
        except Exception:
            logger.exception('Could not check file server transfer completion')
        return False

    def _on_config_transfer_inactivity_check(self):
        '''
        @brief    Periodic check for file-read inactivity during config transfer.
                  If the transfer completed (EOF reached), show success.
                  If the hit count has not increased since the last poll, the node
                  has stopped reading — show a timeout dialog.
        @return   None
        '''
        # Check if the entire file has been served
        if self._is_config_transfer_complete():
            # Completion handled by the fast progress timer
            return

        hits = self._get_config_transfer_hits()
        if hits > self._config_transfer_last_hits:
            # Activity detected — update baseline and keep waiting
            self._config_transfer_last_hits = hits
            return

        # No new reads since last poll
        if hits > self._config_transfer_start_hits:
            message = (
                f'The Delivery Controller stopped reading the param file '
                f'(no activity for {RESPONSE_TIMEOUT} seconds).'
            )
        else:
            message = (
                f'No file read activity was observed from The Delivery Controller '
                f'within {RESPONSE_TIMEOUT} seconds of the upload request.'
            )

        self._cleanup_config_transfer_timeout()
        self._cleanup_upload_handler()
        self._config_transfer_progress.setValue(0)
        self._show_ok_dialog('Transfer Timeout', message)

    def _update_config_transfer_progress(self):
        '''
        @brief    Update the progress bar based on bytes served by the file server.
        @return   None
        '''
        try:
            file_server_widget = self._get_file_server_widget()
            file_server = getattr(file_server_widget, '_file_server', None)
            if file_server is not None and self._config_transfer_key:
                sent, total = file_server.get_key_progress(self._config_transfer_key)
                if total > 0:
                    percent = int(sent * 100 / total)
                    self._config_transfer_progress.setValue(min(percent, 100))
        except Exception:
            logger.exception('Could not update transfer progress')

    def _on_config_transfer_timeout(self):
        '''
        @brief    Handle overall param file transfer timeout.
        @return   None
        '''
        message = (
            f'Config file transfer did not complete within {CONFIG_FILE_TRANSFER_TIMEOUT} seconds.'
        )

        self._cleanup_config_transfer_timeout()
        self._cleanup_upload_handler()
        self._config_transfer_progress.setValue(0)
        self._show_ok_dialog('Transfer Timeout', message)

    def _on_config_transfer_progress_tick(self):
        '''
        @brief    Fast periodic callback to update the progress bar and detect completion.
        @return   None
        '''
        if self._is_config_transfer_complete():
            self._config_transfer_progress.setValue(100)
            self._cleanup_config_transfer_timeout()
            self._cleanup_upload_handler()
            # self._show_ok_dialog('Upload Complete', 'Upload succesful.', icon=QMessageBox.Information)
            return
        self._update_config_transfer_progress()

    def _cleanup_config_transfer_timeout(self):
        '''
        @brief    Stop both param file transfer timers and reset state.
        @return   None
        '''
        if self._config_transfer_timer is not None:
            self._config_transfer_timer.stop()
            self._config_transfer_timer = None
        if self._config_transfer_inactivity_timer is not None:
            self._config_transfer_inactivity_timer.stop()
            self._config_transfer_inactivity_timer = None
        if self._config_transfer_progress_timer is not None:
            self._config_transfer_progress_timer.stop()
            self._config_transfer_progress_timer = None
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

    def _cancel_upload(self):
        '''
        @brief    Cancel an in-progress upload: stop all timers, reset UI.
        @return   None
        '''
        logger.info('Upload cancelled by user')
        self._cleanup_upload_handler()
        self._cleanup_config_transfer_timeout()
        self._config_transfer_progress.setValue(0)
        self._show_ok_dialog('Upload Cancelled', 'The upload was cancelled.')

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
            self._upload_button.setText(SpoolControllerPanel.TEXT_UPLOAD_BTN)


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
                    f'The Delivery Controller may be offline or not responding.'
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


    def _on_download_response(self, event):
        '''
        @brief    Handle an incoming ReadConfigFile response message.
                  On success, sends a file.GetInfo request to the responding node.
        @param    event - DroneCAN transfer event containing the response message.
        @return   None
        '''
        # Stop the ReadConfigFile timeout but keep buttons disabled
        if self._download_timeout_timer is not None:
            self._download_timeout_timer.stop()
            self._download_timeout_timer = None
        if self._download_response_handle is not None:
            try:
                self._download_response_handle.remove()
            except Exception:
                pass
            self._download_response_handle = None

        msg = event.message
        logger.info('ReadConfigFile response received: error=%s', msg.error.value)

        try:
            if msg.error.value == msg.error.STATUS_OK:
                logger.info('Download request accepted. Sending GetInfo for %s.', DOWNLOAD_CONFIG_FILE_NAME)
                self._send_download_getinfo(event.transfer.source_node_id)
            else:
                self._cleanup_download_handler()
                error_info = self._parse_error_msg(msg.error, 'download')
                if error_info:
                    title, message, log_msg = error_info
                    self._show_ok_dialog(title, message)
                    logger.warning(log_msg)
        except Exception as ex:
            logger.exception('Error processing download response: %s', ex)
            self._cleanup_download_handler()
            self._show_ok_dialog('Download Error', f'Error processing download response: {ex}')

    def _send_download_getinfo(self, target_node_id):
        '''
        @brief    Send a uavcan.protocol.file.GetInfo request to the target node
                  for DOWNLOAD_CONFIG_FILE_NAME.
        @param    target_node_id - Node ID to send the request to.
        @return   None
        '''
        try:
            req = dronecan.uavcan.protocol.file.GetInfo.Request()
            req.path.path = DOWNLOAD_CONFIG_FILE_NAME
        except Exception as ex:
            logger.exception('Could not create GetInfo request: %s', ex)
            self._cleanup_download_handler()
            show_error('DSDL Error', 'Could not create file.GetInfo request.', str(ex), parent=self, blocking=True)
            return

        # Start a timeout timer for the GetInfo response
        self._download_getinfo_timer = QTimer(self)
        self._download_getinfo_timer.setSingleShot(True)
        self._download_getinfo_timer.timeout.connect(self._on_download_getinfo_timeout)
        self._download_getinfo_timer.start(RESPONSE_TIMEOUT * 1000)

        try:
            self._node.request(req, target_node_id, self._on_download_getinfo_response)
            logger.info('Sent GetInfo request to node %d for %s', target_node_id, DOWNLOAD_CONFIG_FILE_NAME)
        except Exception as ex:
            logger.exception('Failed to send GetInfo request: %s', ex)
            self._cleanup_download_getinfo()
            self._cleanup_download_handler()
            show_error('Request Failed', 'Could not send file.GetInfo request.', str(ex), parent=self, blocking=True)

    def _on_download_getinfo_response(self, event):
        '''
        @brief    Handle the file.GetInfo response during download.
        @param    event - DroneCAN transfer event, or None on timeout.
        @return   None
        '''
        self._cleanup_download_getinfo()

        if event is None:
            self._cleanup_download_handler()
            self._show_ok_dialog(
                'Download Timeout',
                f'No GetInfo response for "{DOWNLOAD_CONFIG_FILE_NAME}" was received.\n\n'
                f'The Delivery Controller may be offline or not responding.'
            )
            return

        resp = event.response
        logger.info('GetInfo response: error=%s, size=%s', resp.error.value, resp.size)

        if resp.error.value != resp.error.OK:
            self._cleanup_download_handler()
            self._show_ok_dialog(
                'Download Error',
                f'GetInfo for "{DOWNLOAD_CONFIG_FILE_NAME}" failed with error code {resp.error.value}.'
            )
            return

        # GetInfo succeeded — file exists on the remote node
        file_size = resp.size
        target_node_id = event.transfer.source_node_id
        logger.info(f'GetInfo OK: file size = {file_size} bytes, starting download from node {target_node_id}')

        if self._temporary_file_bytes is not None:
            self._show_ok_dialog(title='Error', message='Another operation is in progress', icon=QMessageBox.Warning)

        # Start the file download thread
        self._file_download_stop.clear()
        self._file_download_thread = threading.Thread(
            target=self._file_download_thread_func,
            args=(target_node_id, file_size),
            daemon=True,
        )
        self._file_download_thread.start()

        # Start overall download deadline timer on the main thread
        self._file_download_timeout_timer = QTimer(self)
        self._file_download_timeout_timer.setSingleShot(True)
        self._file_download_timeout_timer.timeout.connect(self._on_file_download_timeout)
        self._file_download_timeout_timer.start(CONFIG_FILE_TRANSFER_TIMEOUT * 1000)

    def _file_download_thread_func(self, target_node_id, file_size):
        '''
        @brief    Background thread that reads a file from a remote node using
                  uavcan.protocol.file.Read requests.
        @param    target_node_id - Node ID to read from.
        @param    file_size - Expected file size in bytes (from GetInfo).
        @param    save_path - Local file path to save the downloaded data.
        @return   None
        '''

        READ_DATA_CAPACITY = 256
        offset = 0
        bytes_written = 0
        error_message = None

        if self._temporary_file_bytes is not None:
            raise FileExistsError('Another operation is in progress')

        self._temporary_file_bytes = bytes(0)

        try:
            while not self._file_download_stop.is_set():
                read_event = threading.Event()
                read_result = [None]  # [event_or_None]

                def _on_read_response(evt, _result=read_result, _flag=read_event):
                    _result[0] = evt
                    _flag.set()

                try:
                    req = dronecan.uavcan.protocol.file.Read.Request()
                    req.offset = offset
                    req.path.path = DOWNLOAD_CONFIG_FILE_NAME
                except Exception as ex:
                    error_message = f'Could not create file.Read request: {ex}'
                    break

                try:
                    self._node.request(req, target_node_id, _on_read_response, timeout=RESPONSE_TIMEOUT)
                except Exception as ex:
                    error_message = f'Failed to send file.Read request: {ex}'
                    break

                # Wait for the response callback. If neither the actual response nor
                # the dronecan internal timeout fires within RESPONSE_TIMEOUT seconds,
                # this wait itself acts as the per-packet timeout.
                if not read_event.wait(timeout=RESPONSE_TIMEOUT):
                    logger.warning('file.Read per-packet timeout at offset %d '
                        '(no response within %d seconds)', offset, RESPONSE_TIMEOUT)
                    error_message = (
                        f'file.Read request timed out at offset {offset}.\n\n'
                        f'No response was received within {RESPONSE_TIMEOUT} seconds.\n'
                        f'The Delivery Controller may be offline or not responding.'
                    )
                    break

                evt = read_result[0]
                if evt is None:
                    # dronecan internal timeout fired — callback was called with None
                    logger.warning('file.Read dronecan timeout at offset %d', offset)
                    error_message = (
                        f'file.Read request timed out at offset {offset}.\n\n'
                        f'No response was received within {RESPONSE_TIMEOUT} seconds.\n'
                        f'The Delivery Controller may be offline or not responding.'
                    )
                    break

                resp = evt.response
                if resp.error.value != 0:
                    error_message = f'file.Read error at offset {offset}: error code {resp.error.value}'
                    break

                chunk = bytes(resp.data)
                self._temporary_file_bytes += chunk
                bytes_written += len(chunk)
                offset += len(chunk)
                logger.debug('file.Read offset=%d, received=%d bytes, total=%d/%d',
                    offset - len(chunk), len(chunk), bytes_written, file_size)

                # Update progress bar on the main thread
                if file_size > 0:
                    percent = min(int(bytes_written * 100 / file_size), 100)
                    self._file_download_progress_signal.emit(percent)

                # End of file: data shorter than capacity
                if len(chunk) < READ_DATA_CAPACITY:
                    break

        except Exception as ex:
            logger.exception('Unexpected error in file download thread: %s', ex)
            error_message = f'Unexpected error during download: {ex}'

        # Handle cancellation
        if error_message is None and self._file_download_stop.is_set():
            error_message = 'Download was cancelled.'

        if error_message is None:
            logger.info('File downloaded successfully: %d bytes', bytes_written)

        # Signal the main thread to handle completion
        success = error_message is None
        self._file_download_finished_signal.emit(success, error_message or '')

    def _on_file_download_finished(self, success, error_message):
        '''
        @brief    Called on the main thread when the file download thread finishes.
        @param    success - True if the download succeeded.
        @param    error_message - Error description string, or empty string on success.
        @param    save_path - Local file path where data was saved.
        @return   None
        '''
        self._cleanup_file_download_timeout()
        self._cleanup_download_handler()
        self._file_download_thread = None

        if success:
            try:
                param_set_file = ParamSetFile()
                param_set_file.deserialize(self._temporary_file_bytes)

                # Save original CRCs from the file before recalculation
                orig_hdr_crc = param_set_file._hdr_crc
                orig_payload_crc = param_set_file._payload_crc

                # Recalculate CRCs
                calc_payload_crc = param_set_file.calculate_payload_crc()
                calc_hdr_crc = param_set_file.calculate_header_crc()

                if orig_payload_crc != calc_payload_crc or orig_hdr_crc != calc_hdr_crc:
                    logger.warning('Downloaded file CRC mismatch: '
                        'hdr_crc file=0x%08X calc=0x%08X, payload_crc file=0x%08X calc=0x%08X',
                        orig_hdr_crc, calc_hdr_crc, orig_payload_crc, calc_payload_crc)
                    self._version_textbox.clear()
                    self._crc32_textbox.clear()
                    self._show_ok_dialog(
                        'Download CRC Error',
                        f'CRC verification failed.\n'
                        f'Header CRC: file=0x{orig_hdr_crc:08X}, calculated=0x{calc_hdr_crc:08X}\n'
                        f'Payload CRC: file=0x{orig_payload_crc:08X}, calculated=0x{calc_payload_crc:08X}')
                else:
                    self._param_set_file = param_set_file
                    self._populate_ui_from_param_set_file()
                    self._version_textbox.setText(str(param_set_file._version))
                    self._crc32_textbox.setText(f'{orig_hdr_crc:08X}')
                    # self._show_ok_dialog('Download Complete', f'Downloaded successfully', icon = QMessageBox.Information)
            except Exception as ex:
                logger.exception('Failed to verify downloaded file: %s', ex)
                self._version_textbox.clear()
                self._crc32_textbox.clear()
                self._show_ok_dialog(
                    'Download Verification Error',
                    f'Verification failed: {ex}'
                )

            self._populate_ui_from_param_set_file()
            self._clean_all_dirty()
            self._update_window_data()
        else:
            self._show_ok_dialog('Download Failed', error_message)

        self._temporary_file_bytes = None


    def _on_file_download_timeout(self):
        '''
        @brief    Handle overall file download timeout. Signals the download thread
                  to stop and shows a timeout dialog.
        @return   None
        '''
        logger.warning('File download timed out after %d seconds', CONFIG_FILE_TRANSFER_TIMEOUT)
        self._file_download_stop.set()
        self._cleanup_file_download_timeout()
        self._cleanup_download_handler()
        self._file_download_thread = None
        self._config_transfer_progress.setValue(0)
        self._show_ok_dialog(
            'Download Timeout',
            f'File download did not complete within {CONFIG_FILE_TRANSFER_TIMEOUT} seconds.\n\n'
            f'The Delivery Controller may be offline or not responding.'
        )
        self._temporary_file_bytes = None

    def _cleanup_file_download_timeout(self):
        '''
        @brief    Stop the file download deadline timer.
        @return   None
        '''
        if self._file_download_timeout_timer is not None:
            self._file_download_timeout_timer.stop()
            self._file_download_timeout_timer = None

    def _stop_file_download_thread(self):
        '''
        @brief    Signal the file download thread to stop and wait for it to finish.
        @return   None
        '''
        self._cleanup_file_download_timeout()
        self._file_download_stop.set()
        if self._file_download_thread is not None:
            self._file_download_thread.join(timeout=5)
            self._file_download_thread = None

    def _on_download_getinfo_timeout(self):
        '''
        @brief    Handle timeout waiting for file.GetInfo response.
        @return   None
        '''
        self._cleanup_download_getinfo()
        self._cleanup_download_handler()
        self._show_ok_dialog(
            'Download Timeout',
            f'No GetInfo response for "{DOWNLOAD_CONFIG_FILE_NAME}" was received '
            f'within {RESPONSE_TIMEOUT} seconds.\n\n'
            f'The Delivery Controller may be offline or not responding.'
        )

    def _cleanup_download_getinfo(self):
        '''
        @brief    Stop the GetInfo timeout timer.
        @return   None
        '''
        if self._download_getinfo_timer is not None:
            self._download_getinfo_timer.stop()
            self._download_getinfo_timer = None

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
            with open(self._design_const_view_config_path, 'r') as f:
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
            with open(self._param_set_view_config_path, 'r') as f:
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

                field_type = field_data.get('type', '')
                type_min, type_max = self._get_type_range(field_type)
                min_val = field_data.get('min_val', type_min)
                max_val = field_data.get('max_val', type_max)


                default_value = field_data.get('default', '')
                if 'float' in field_type:
                    widget = QLineEdit(fields_container)
                    widget.setFixedHeight(20)
                    widget.setStyleSheet("background-color: white;")
                    widget.setText(str(default_value))
                    validator = QDoubleValidator(min_val, max_val, FLOAT_DECIMALS, self)
                    validator.setNotation(QDoubleValidator.Notation.StandardNotation)
                    validator.setLocale(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))
                    widget.setValidator(validator)
                elif 'bool' in field_type:
                    widget = QCheckBox(fields_container)
                    widget.setChecked(bool(default_value))
                else:
                    raise AssertionError(f'invalid type {field_type}')

                tip_parts = []
                if field_type:
                    tip_parts.append(f'{field_type} type')
                if min_val is not None and max_val is not None:
                    tip_parts.append(f'Range: [{min_val}, {max_val}]')
                if tip_parts:
                    widget.setToolTip('\n'.join(tip_parts))
                fields_layout.addWidget(widget, row, 1)

                field_inputs[field_name] = (widget, field_type, min_val, max_val)

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

            default_value = field_data.get('default', '')
            field_type = field_data.get('type', '')
            type_min, type_max = self._get_type_range(field_type)
            min_val = field_data.get('min_val', type_min)
            max_val = field_data.get('max_val', type_max)

            # Textbox
            if 'float' in field_type or 'int' in field_type:
                validator = None
                widget = QLineEdit(fields_container)
                widget.setFixedHeight(20)
                widget.setStyleSheet("background-color: white;")
                widget.setText(str(default_value))
                if 'float' in field_type:
                    validator = QDoubleValidator(min_val, max_val, FLOAT_DECIMALS, self)
                    validator.setNotation(QDoubleValidator.Notation.StandardNotation)
                    validator.setLocale(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))
                    widget.setValidator(validator)
                elif 'int' in field_type:
                    validator = QIntValidator(min_val, max_val, self)
                widget.setValidator(validator)
            elif 'bool' in field_type:
                widget = QCheckBox(fields_container)
                widget.setChecked(bool(default_value))
            else:
                raise AssertionError(f'invalid type {field_type}')

            tip_parts = []
            if field_type:
                tip_parts.append(f'{field_type} type')
            if min_val is not None and max_val is not None:
                tip_parts.append(f'Range: [{min_val}, {max_val}]')
            if tip_parts:
                widget.setToolTip('\n'.join(tip_parts))
            fields_layout.addWidget(widget, row, 1)

            self._field_inputs[field_name] = widget
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

        self._design_const_set_group = QGroupBox(parent)
        design_const_set_group = self._design_const_set_group
        design_const_set_group.setMinimumHeight(200)
        design_const_set_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Layout for design_const_set_group using grid for better alignment
        design_const_layout = QGridLayout(design_const_set_group)
        design_const_layout.setColumnStretch(0, 1)
        design_const_layout.setSpacing(10)
        design_const_layout.setContentsMargins(10, 10, 10, 10)
        design_const_layout.setRowMinimumHeight(0, 30)

        # Buttons row (fixed at top)
        buttons_layout = QHBoxLayout()
        buttons_layout.setSpacing(BUTTON_HORIZONTAL_SPACING)
        buttons_layout.addStretch(1)

        clear_button = QPushButton('Clear', design_const_set_group)
        clear_button.clicked.connect(self._on_design_constants_clear)
        buttons_layout.addWidget(clear_button)

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

    def _create_design_constants_window(self):
        '''
        @brief    Create the DesignConstantsSet Tuning window: a separate, non-modal child
                  window (rather than an embedded section), opened via a button in the
                  Parameter File Management area.
        @return   None
        '''
        self._design_const_window = QDialog(self)
        self._design_const_window.setWindowTitle(DESIGN_CONSTANTS_TUNE_NAME)
        self._design_const_window.resize(420, 520)

        window_layout = self._create_design_constants_section(self._design_const_window)
        self._design_const_window.setLayout(window_layout)

    def _on_open_design_constants_clicked(self):
        '''
        @brief    Handle the "Design Constants..." button click: show (or bring to front)
                  the DesignConstantsSet Tuning window.
        @return   None
        '''
        self._design_const_window.show()
        self._design_const_window.raise_()
        self._design_const_window.activateWindow()

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
            self._cleanup_config_transfer_timeout()
            self._cleanup_download_handler()
            self._cleanup_download_getinfo()
            self._cleanup_recall_handler()
            self._cleanup_store_constants_handler()
            self._cleanup_param_set_recall_handler()
            self._cleanup_param_set_store_handler()
            self._cleanup_param_set_execute_handler()
            self._stop_file_download_thread()
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