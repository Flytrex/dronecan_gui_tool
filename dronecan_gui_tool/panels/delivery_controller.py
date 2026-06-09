
#
# Copyright (C) 2026  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ilan Graidy
# Date:   2026-01-22
#

import dronecan
from functools import partial
from logging import getLogger
import threading
import os
import re
import xml.etree.ElementTree as ET

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIntValidator, QColor
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView, \
	QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QComboBox, QGridLayout, QSizePolicy
import numpy as np

from ..widgets import get_icon, show_error

__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'Delivery Controller'

logger = getLogger(__name__)

_singleton = None


class DeliveryControllerPanel(QDialog):
	REQUEST_PRIORITY = 30

	SET_MODE = 'Set Mode'
	SET_WIRE_LENGTH_LOWER = 'Set Wire Length Lower'
	SET_WIRE_LENGTH_LIFT = 'Set Wire Length Lift'

	def show_message(self, text, *fmt) -> None:
		"""Best-effort status reporting (main window status bar if available)."""
		try:
			# Unlike many widgets, this panel is a top-level QDialog, so self.window() is usually self.
			# Prefer sending status messages to the parent/main window if it exposes show_message().
			parent = self.parent()
			if parent is not None and hasattr(parent, 'show_message'):
				parent.show_message(text, *fmt)
				return
		except Exception:
			pass
		try:
			logger.info(text % fmt)
		except Exception:
			logger.info('%s %s', text, fmt)

	@staticmethod
	def _encode_param_name(name: str) -> bytes:
		# UAVCAN v0 param name is uint8[<=92] (bytes)
		return name.encode('utf-8')

	def __init__(self, parent, node):
		'''
		@brief    Create the Delivery Controller panel window.
		@param    parent - Parent Qt widget.
		@param    node - Local DroneCAN node instance.
		@return   None
		'''

		super(DeliveryControllerPanel, self).__init__(parent)
		self._handlers = []
		self._field_rows = {}
		self._field_types = {}

		self.setWindowTitle(PANEL_NAME)
		self.setAttribute(Qt.WA_DeleteOnClose)
		self.resize(900, 600)
		self.setMinimumSize(700, 400)

		self._node = node
		# Load initial fields from XML in the config directory. This is just a starting point; the user can edit and save to other files.
		self._xml_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'delivery_controller_fields.xml')
		self._live_param_read_thread: threading.Thread | None = None
		self._live_param_read_stop_event: threading.Event | None = None
		self._live_param_read_node_id: int | None = None
		self._auto_node_id_timer: QTimer | None = None

		layout = QVBoxLayout(self)

		# Node selector
		node_row = QHBoxLayout()
		node_row.addWidget(QLabel('Node ID:', self))
		self._node_id_edit = QLineEdit(self)
		self._node_id_edit.setPlaceholderText('e.g. 42')
		self._node_id_edit.setValidator(QIntValidator(1, 127, self))
		node_row.addWidget(self._node_id_edit)
		node_row.addStretch(1)
		layout.addLayout(node_row)

		fields_group = QGroupBox('Fields', self)
		group_layout = QVBoxLayout(fields_group)

		# Set up the fields table
		self._table = QTableWidget(fields_group)
		self._table.setColumnCount(5)
		self._table.setHorizontalHeaderLabels(['Field Name', 'Value', 'Type', 'Num Bits', 'Comment'])
		self._table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
		self._table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
		self._table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
		self._table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
		self._table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
		self._table.verticalHeader().setVisible(False)
		self._table.setAlternatingRowColors(True)
		self._table.setSortingEnabled(False)
		self._table.setWordWrap(True)
		self._table.setSelectionBehavior(QTableWidget.SelectRows)
		self._table.setSelectionMode(QTableWidget.SingleSelection)

		# Add the tables's buttons
		self._load_from_file_btn = QPushButton('Load From File', self)
		self._save_to_file_btn = QPushButton('Save To File', self)
		group_layout.addWidget(self._table)
		file_buttons_row = QHBoxLayout()
		file_buttons_row.addWidget(self._load_from_file_btn)
		file_buttons_row.addWidget(self._save_to_file_btn)
		file_buttons_row.addStretch(1)
		group_layout.addLayout(file_buttons_row)

		layout.addWidget(fields_group)

		# Controls (below the fields table group)
		controls_group = QGroupBox('Controls', self)
		# Keep this group compact; let the fields table take extra vertical space.
		controls_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
		controls_layout = QGridLayout(controls_group)
		controls_layout.setColumnStretch(0, 1)
		controls_layout.setColumnStretch(1, 1)
		controls_layout.setRowStretch(1, 1)

		# Mode controls
		mode_group = QGroupBox('Mode', controls_group)
		mode_layout = QGridLayout(mode_group)
		mode_layout.setColumnStretch(1, 1)

		self._active_mode_combo = QComboBox(mode_group)
		self._active_mode_combo.addItem('ENCODER_ALIGNMENT', 1)
		self._active_mode_combo.addItem('WIRE_HOMING', 2)
		self._active_mode_combo.addItem('DIRECT_OVERRIDE', 4)
		self._active_mode_combo.addItem('HALT', 5)
		self._active_mode_combo.addItem('GROUND_UNLOAD', 6)
		self._active_mode_combo.addItem('RESET', 7)
		self._active_mode_combo.addItem('PREPARE_FOR_PICKUP', 8)
		self._active_mode_combo.addItem('LIFT_PACKAGE', 9)
		self._active_mode_combo.addItem('LANDING', 11)
		self._active_mode_combo.addItem('PREPARE_FOR_DELIVERY', 12)
		self._active_mode_combo.addItem('DELIVERY', 13)
		self._active_mode_combo.addItem('RELEASE_WIRE', 14)
		self._active_mode_combo.setCurrentIndex(0)
		self._set_mode_btn = QPushButton('Set Mode', mode_group)
		self._set_wire_length_lower_btn = QPushButton('Set Wire Length Lower', mode_group)
		self._wire_length_lower_edit = QLineEdit(mode_group)
		self._wire_length_lower_edit.setPlaceholderText('Length')
		self._set_wire_length_lift_btn = QPushButton('Set Wire Length Lift', mode_group)
		self._wire_length_lift_edit = QLineEdit(mode_group)
		self._wire_length_lift_edit.setPlaceholderText('Length')

		# Buttons aligned in column 0, inputs in column 1
		mode_layout.addWidget(self._set_mode_btn, 0, 0)
		mode_layout.addWidget(self._active_mode_combo, 0, 1)
		mode_layout.addWidget(self._set_wire_length_lower_btn, 1, 0)
		mode_layout.addWidget(self._wire_length_lower_edit, 1, 1)
		mode_layout.addWidget(self._set_wire_length_lift_btn, 2, 0)
		mode_layout.addWidget(self._wire_length_lift_edit, 2, 1)

		# Params controls
		params_group = QGroupBox('Params', controls_group)
		params_layout = QHBoxLayout(params_group)
		self._set_params_btn = QPushButton('Set Params', params_group)
		self._get_params_btn = QPushButton('Get Params', params_group)
		self._live_param_read_btn = QPushButton('Live Param Read', params_group)
		self._live_param_read_btn.setCheckable(True)
		self._live_param_read_btn.setStyleSheet('QPushButton { background-color: #FFFACD; }')
		params_layout.addWidget(self._set_params_btn)
		params_layout.addWidget(self._get_params_btn)
		params_layout.addWidget(self._live_param_read_btn)

		# Layout: Mode spans both rows; Params buttons sit on the top row.
		controls_layout.addWidget(mode_group, 0, 0, 2, 1)
		controls_layout.addWidget(params_group, 0, 1, 1, 1, Qt.AlignTop)

		layout.addWidget(controls_group)

		# Connect Fields groupbox buttons signals
		self._load_from_file_btn.clicked.connect(self._on_load_from_file_clicked)
		self._save_to_file_btn.clicked.connect(self._on_save_to_file_clicked)
		# Connect Controls groupbox buttons signals
		self._set_params_btn.clicked.connect(self._on_set_params_clicked)
		self._get_params_btn.clicked.connect(self._on_get_params_clicked)
		self._live_param_read_btn.toggled.connect(self._on_live_param_read_toggled)
		self._set_mode_btn.clicked.connect(self._on_set_mode_clicked)
		self._set_wire_length_lower_btn.clicked.connect(self._on_set_wire_length_lower_clicked)
		self._set_wire_length_lift_btn.clicked.connect(self._on_set_wire_length_lift_clicked)

		self._load_fields_into_table(self._xml_path)
		self._start_auto_node_id_lookup()

	def _stop_live_param_read_thread(self) -> None:
		"""Stop any background work related to live param read.

		Live Param Read currently uses message handlers rather than a background thread,
		but older versions used a worker thread. Keep this method so closeEvent and __del__
		can safely stop either implementation.
		"""
		try:
			if self._live_param_read_stop_event is not None:
				self._live_param_read_stop_event.set()
		except Exception:
			pass

		try:
			t = self._live_param_read_thread
			if t is not None and t.is_alive():
				t.join(timeout=2.0)
		except Exception:
			pass
		finally:
			self._live_param_read_thread = None
			self._live_param_read_stop_event = None
			self._live_param_read_node_id = None

	def _find_delcon_node_id(self) -> int | None:
		'''
		@brief    Search online nodes for a node whose name contains "delcon".
		@return   Matching node ID or None.
		'''
		parent = self.parent()
		while parent is not None:
			node_monitor_widget = getattr(parent, '_node_monitor_widget', None)
			if node_monitor_widget is not None and hasattr(node_monitor_widget, 'monitor'):
				try:
					entries = list(node_monitor_widget.monitor.find_all(lambda _: True))
					for entry in sorted(entries, key=lambda e: int(e.node_id)):
						name = ''
						if getattr(entry, 'info', None) is not None:
							name = getattr(entry.info, 'name', '')
						if isinstance(name, bytes):
							name = name.decode('utf-8', errors='ignore')
						name = str(name).strip()
						if 'delcon' in name.lower():
							return int(entry.node_id)
				except Exception:
					logger.exception('Failed to scan node monitor entries for delcon node')
				return None
			parent = parent.parent()
		return None

	def _try_auto_fill_node_id(self) -> None:
		'''
		@brief    Auto-fill Node ID when a delcon node is detected online.
		@return   None
		'''
		if self._node_id_edit.text().strip():
			if self._auto_node_id_timer is not None and self._auto_node_id_timer.isActive():
				self._auto_node_id_timer.stop()
			return

		node_id = self._find_delcon_node_id()
		if node_id is None:
			return

		self._node_id_edit.setText(str(node_id))
		self.show_message('Auto-selected Delcon node ID %d', node_id)
		if self._auto_node_id_timer is not None and self._auto_node_id_timer.isActive():
			self._auto_node_id_timer.stop()

	def _start_auto_node_id_lookup(self) -> None:
		'''
		@brief    Start periodic lookup for a delcon node and auto-fill Node ID.
		@return   None
		'''
		if self._auto_node_id_timer is not None:
			self._auto_node_id_timer.stop()

		self._auto_node_id_timer = QTimer(self)
		self._auto_node_id_timer.setSingleShot(False)
		self._auto_node_id_timer.timeout.connect(self._try_auto_fill_node_id)
		self._auto_node_id_timer.start(500)

		# Attempt immediately so users don't wait for the first timer tick.
		self._try_auto_fill_node_id()

	@staticmethod
	def _type_to_num_bits(field_type: str) -> str:
		'''
        @brief    Convert a DSDL field type string to number of bits string.
        @param    field_type - The field type string.
        @return   The number of bits as a string, or empty string if unrecognized.
        '''

		ft = (field_type or '').strip()
		if not ft:
			return ''

		# UAVCAN/DroneCAN DSDL primitive types (recommended):
		#   bool
		#   int8/int16/int32/int64
		#   uint8/uint16/uint32/uint64
		#   float16/float32/float64
		# Arrays:
		#   uint8[16]      (fixed length)
		#   uint8[<=16]    (variable length, max 16)
		m = re.fullmatch(r'(?P<base>[A-Za-z_][A-Za-z0-9_\.]*)\[(?P<var><=)?(?P<len>\d+)\]', ft)
		if m:
			base = m.group('base')
			is_var = m.group('var') is not None
			length = int(m.group('len'))
			elem_bits = DeliveryControllerPanel._type_to_num_bits(base)
			if elem_bits.isdigit():
				total = int(elem_bits) * length
				return f'<= {total}' if is_var else str(total)
			return ''

		dsdl_bits = {
			'bool': 1,
			'uint2': 2,
            'uint3': 3,
			'uint4': 4,
            'uint5': 5,
			'uint6': 6,
            'uint7': 7,
			'uint8': 8,
			'uint9': 9,
            'uint10': 10,
			'uint11': 11,
            'uint12': 12,
			'uint13': 13,
            'uint14': 14,
			'uint15': 15,
			'uint16': 16,
            'uint17': 17,
            'uint18': 18,
			'uint19': 19,
            'uint20': 20,
			'uint21': 21,
            'uint22': 22,
			'uint23': 23,
            'uint24': 24,
			'uint25': 25,
            'uint26': 26,
            'uint27': 27,
			'uint28': 28,
            'uint29': 29,
            'uint30': 30,
			'uint31': 31,
			'uint32': 32,
			'uint64': 64,
			'int2': 2,
            'int3': 3,
			'int4': 4,
            'int5': 5,
			'int6': 6,
            'int7': 7,
			'int8': 8,
			'int9': 9,
            'int10': 10,
			'int11': 11,
            'int12': 12,
			'int13': 13,
            'int14': 14,
			'int15': 15,
			'int16': 16,
            'int17': 17,
            'int18': 18,
			'int19': 19,
            'int20': 20,
			'int21': 21,
            'int22': 22,
            'int23': 23,
            'int24': 24,
            'int25': 25,
            'int26': 26,
            'int27': 27,
            'int28': 28,
            'int29': 29,
            'int30': 30,
            'int31': 31,
			'int32': 32,
			'int64': 64,
			'float16': 16,
			'float32': 32,
			'float64': 64,
		}
		if ft in dsdl_bits:
			return str(dsdl_bits[ft])

		m = re.fullmatch(r'(u?int)(\d+)_t', ft)
		if m:
			return m.group(2)

		if ft == 'float':
			return '32'
		if ft == 'double':
			return '64'
		if ft == 'bool':
			return '1'

		m = re.fullmatch(r'char\[(\d+)\]', ft)
		if m:
			# Total bits for the array
			return str(int(m.group(1)) * 8)

		return ''

	@staticmethod
	def _parse_bool_attr(value: str | None, default: bool = True) -> bool:
		'''
		@brief    Parse an XML boolean attribute to a Python bool.
		@param    value - The raw XML attribute value (e.g., "true", "false", "1", "0").
		@param    default - Fallback value if the attribute is missing or unrecognized.
		@return   Parsed boolean.
		'''

		if value is None:
			return bool(default)
		v = str(value).strip().lower()
		if v in ('1', 'true', 'yes', 'y', 'on'):
			return True
		if v in ('0', 'false', 'no', 'n', 'off'):
			return False
		return bool(default)

	@staticmethod
	def parse_value(field_type: str, raw_value: object):
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

		# Integers like uint8/uint16/... or int8/int16/... and legacy *_t
		if re.fullmatch(r'(u?int)(\d+)', ft) or re.fullmatch(r'(u?int)(\d+)_t', ft):
			return 0 if raw == '' else int(raw, 0)

		# If type is missing/unknown, try int then float
		if raw == '':
			return 0
		try:
			return int(raw, 0)
		except Exception:
			return float(raw)

	def _gather_fields_from_table(self) -> list[dict[str, object]]:
		'''
		@brief    Collect all fields from the UI table.
		@return   List of dictionaries with keys: name, type, default, comment, edit.
		'''

		fields: list[dict[str, object]] = []
		for row in range(self._table.rowCount()):
			name_item = self._table.item(row, 0)
			value_item = self._table.item(row, 1)
			type_item = self._table.item(row, 2)
			comment_item = self._table.item(row, 4)

			name = '' if name_item is None else name_item.text().strip()
			value = '' if value_item is None else value_item.text()
			field_type = '' if type_item is None else type_item.text().strip()
			comment = '' if comment_item is None else comment_item.text()
			editable = True
			if value_item is not None:
				editable = bool(value_item.flags() & Qt.ItemIsEditable)

			if not name:
				continue
			fields.append({
				'name': name,
				'type': field_type,
				'default': value,
				'comment': comment,
				'edit': editable,
			})
		return fields

	def _write_fields_xml(self, out_path: str) -> None:
		'''
		@brief    Write current table fields to an XML file.
		@param    out_path - Output XML file path.
		@return   None
		'''

		root = ET.Element('fields')
		for f in self._gather_fields_from_table():
			attrib = {
				'name': str(f.get('name', '')),
				'type': str(f.get('type', '')),
				'default': str(f.get('default', '')),
				'comment': str(f.get('comment', '')),
				'edit': 'true' if bool(f.get('edit', True)) else 'false',
			}
			# Avoid writing empty attributes except name (required)
			attrib = {k: v for (k, v) in attrib.items() if (k == 'name') or (v != '')}
			ET.SubElement(root, 'field', attrib=attrib)

		tree = ET.ElementTree(root)
		try:
			ET.indent(tree, space='  ', level=0)
		except Exception:
			pass
		tree.write(out_path, encoding='utf-8', xml_declaration=True)

	def _on_save_to_file_clicked(self) -> None:
		'''
		@brief    Handle Save To File button click.
		@return   None
		'''

		try:
			initial_dir = os.path.dirname(self._xml_path) if self._xml_path else os.path.dirname(__file__)
			file_path, _ = QFileDialog.getSaveFileName(
				self,
				'Save Delivery Controller Fields',
				initial_dir,
				'XML files (*.xml);;All files (*.*)'
			)
			if not file_path:
				return
			self._write_fields_xml(file_path)
			self._xml_path = file_path
		except Exception as ex:
			logger.exception('Failed to save fields XML: %s', ex)
			show_error('Save error', 'Could not save XML file.', ex, parent=self)

	def _on_load_from_file_clicked(self) -> None:
		'''
		@brief    Handle Load From File button click.
		@return   None
		'''

		initial_dir = os.path.dirname(self._xml_path) if self._xml_path else os.path.dirname(__file__)
		file_path, _ = QFileDialog.getOpenFileName(
			self,
			'Load Delivery Controller Fields',
			initial_dir,
			'XML files (*.xml);;All files (*.*)'
		)
		if not file_path:
			return
		self._xml_path = file_path
		self._load_fields_into_table(self._xml_path)

	def _get_target_node_id(self) -> int | None:
		'''
		@brief    Read and validate the target node-ID from the UI.
		@return   Node ID (1..127) or None if invalid.
		'''

		text = self._node_id_edit.text().strip()
		if not text:
			show_error('Missing Node ID', 'Please enter a target Node ID.', '', parent=self)
			return None
		try:
			node_id = int(text, 10)
		except Exception:
			show_error('Invalid Node ID', 'Node ID must be an integer.', text, parent=self)
			return None
		if not (1 <= node_id <= 127):
			show_error('Invalid Node ID', 'Node ID must be in range 1..127.', str(node_id), parent=self)
			return None
		return node_id

	def _on_set_mode_clicked(self) -> None:
		'''
		@brief    Handle Set Mode button click.
		@return   None
		'''

		def _on_response(e):
			if e is None:
				self.show_message('Request timed out')
			else:
				logger.info('Param get/set response: %s', e.response)
				self.show_message('Response received')

		node_id = self._get_target_node_id()
		if node_id is None:
			return

		try:
			mode = int(self._active_mode_combo.currentData())
			msg = dronecan.uavcan.protocol.param.GetSet.Request(
				name=self._encode_param_name(self.SET_MODE),
				value=dronecan.uavcan.protocol.param.Value(integer_value=mode),
			)
		except Exception as ex:
			logger.exception('SetActiveMode type not available: %s', ex)
			return

		try:
			self._node.request(msg, node_id, _on_response, priority=self.REQUEST_PRIORITY, canfd=True)
			logger.info('Send %s for target node %s', self.SET_MODE, node_id)
		except Exception as ex:
			logger.exception('Failed to broadcast %s: %s', self.SET_MODE, ex)
			show_error('Send failed', f'Could not broadcast {self.SET_MODE}.', ex, parent=self)

	def _on_set_wire_length_lower_clicked(self) -> None:
		'''
		@brief    Handle Set Wire Length Lower button click.
		@return   None
		'''

		def _on_response(e):
			if e is None:
				self.show_message('Request timed out')
			else:
				logger.info('Param get/set response: %s', e.response)
				self.show_message('Response received')

		node_id = self._get_target_node_id()
		if node_id is None:
			return

		wire_len = 0.0

		try:
			text = self._wire_length_lower_edit.text().strip()
			# Convert to float16 (half precision) then back to Python float.
			wire_len = float(np.float16(text))
		except Exception as ex:
			logger.exception('Invalid wire length lower value: %s', ex)
			show_error('Invalid Value', 'Wire Length Lower must be a floating point number.', text, parent=self)
			return

		try:
			msg = dronecan.uavcan.protocol.param.GetSet.Request(
				name=self._encode_param_name(self.SET_WIRE_LENGTH_LOWER),
				value=dronecan.uavcan.protocol.param.Value(real_value=float(wire_len)),
			)
		except Exception as ex:
			logger.exception('GetSet type not available: %s', ex)
			return

		try:
			self._node.request(msg, node_id, _on_response, priority=self.REQUEST_PRIORITY, canfd=True)
			logger.info('Send %s for target node %s', self.SET_WIRE_LENGTH_LOWER, node_id)
		except Exception as ex:
			logger.exception('Failed to send %s: %s', self.SET_WIRE_LENGTH_LOWER, ex)
			show_error('Send failed', f'Could not send {self.SET_WIRE_LENGTH_LOWER}.', ex, parent=self)

	def _on_set_wire_length_lift_clicked(self) -> None:
		'''
		@brief    Handle Set Wire Length Lift button click.
		@return   None
		'''

		def _on_response(e):
			if e is None:
				self.show_message('Request timed out')
			else:
				logger.info('Param get/set response: %s', e.response)
				self.show_message('Response received')

		node_id = self._get_target_node_id()
		if node_id is None:
			return

		wire_len = 0.0

		try:
			text = self._wire_length_lift_edit.text().strip()
			# Convert to float16 (half precision) then back to Python float.
			wire_len = float(np.float16(text))
		except Exception as ex:
			logger.exception('Invalid wire length lift value: %s', ex)
			show_error('Invalid Value', 'Wire Length Lift must be a floating point number.', text, parent=self)
			return

		try:
			msg = dronecan.uavcan.protocol.param.GetSet.Request(
				name=self._encode_param_name(self.SET_WIRE_LENGTH_LIFT),
				value=dronecan.uavcan.protocol.param.Value(real_value=float(wire_len)),
			)
		except Exception as ex:
			logger.exception('GetSet type not available: %s', ex)
			return

		try:
			self._node.request(msg, node_id, _on_response, priority=self.REQUEST_PRIORITY, canfd=True)
			logger.info('Send %s for target node %s', self.SET_WIRE_LENGTH_LIFT, node_id)
		except Exception as ex:
			logger.exception('Failed to send %s: %s', self.SET_WIRE_LENGTH_LIFT, ex)
			show_error('Send failed', f'Could not send {self.SET_WIRE_LENGTH_LIFT}.', ex, parent=self)

	def _on_set_params_clicked(self) -> None:
		'''
		@brief    Handle Set Params button click.
		@return   None
		'''

		node_id = self._get_target_node_id()
		if node_id is None:
			return

		try:
			msg = dronecan.flytrex.delcon.DirectOverride()
		except Exception as ex:
			logger.exception('DirectOverride type not available: %s', ex)
			show_error(
				'DSDL type not loaded',
				'Could not access dronecan.flytrex.delcon.DirectOverride. Make sure your custom DSDL directory loaded successfully.',
				ex,
				parent=self,
			)
			return

		# Add the node ID to the message header
		setattr(msg, 'node_id', node_id)
		set_count = 0
		for f in self._gather_fields_from_table():
			name = str(f.get('name', '')).strip()
			if not name:
				continue
			if not bool(f.get('edit', True)):
				continue
			if not hasattr(msg, name):
				continue
			try:
				value = self.parse_value(str(f.get('type', '')), f.get('default', ''))
				setattr(msg, name, value)
				set_count += 1
			except Exception as ex:
				show_error(
					'Invalid field value',
					f'Could not set {name} on DirectOverride.',
					f'Type: {f.get("type", "")}\nValue: {f.get("default", "")}\nError: {ex}',
					parent=self,
				)
				return

		if set_count == 0:
			show_error(
				'Nothing to send',
				'No editable fields matched DirectOverride message fields.',
				'Check your XML field names against the DSDL definition.',
				parent=self,
			)
			return

		try:
			self._node.broadcast(msg, canfd=True)
			logger.info('Broadcast DirectOverride (%d fields set) for target node %s', set_count, node_id)
		except Exception as ex:
			logger.exception('Failed to broadcast DirectOverride: %s', ex)
			show_error('Send failed', 'Could not broadcast DirectOverride.', ex, parent=self)

	def _update_fields_table_from_state_report(self, report: dronecan.flytrex.delcon.StateReport) -> None:
		'''
		@brief    Update the fields table from a StateReport message.
		@param    report - The StateReport message instance.
		@return   None
		'''

		if report is None:
			return

		for row in range(self._table.rowCount()):
			name_item = self._table.item(row, 0)
			value_item = self._table.item(row, 1)
			if name_item is None or value_item is None:
				continue
			field_name = name_item.text().strip()
			if not field_name:
				continue
			if not hasattr(report, field_name):
				continue
			try:
				value = getattr(report, field_name)
				value_item.setText(str(value))
			except Exception:
				logger.exception('Failed to update field %s from StateReport', field_name)

	def _on_get_params_clicked(self) -> None:
		'''
		@brief    Handle Get Params button click.
		@return   None
		'''

		node_id = self._get_target_node_id()
		if node_id is None:
			return

		holder: dict[str, object] = {'handler': None}

		def on_state_report(e):
			# Ignore messages not coming from the selected node.
			if e.transfer.source_node_id != node_id:
				return

			# print(dronecan.to_yaml(e.message))

			# Update the fields table from the received StateReport.
			self._update_fields_table_from_state_report(e.message)

			# One-shot: remove handler after first matching message.
			h = holder.get('handler')
			try:
				if h is not None:
					h.try_remove()
			except Exception:
				logger.exception('Failed to remove one-shot StateReport handler')
			finally:
				try:
					if h is not None and h in self._handlers:
						self._handlers.remove(h)
				except Exception:
					pass

		# Install one-shot handler; keep it in _handlers so close/cleanup can remove it if needed.
		h = self._node.add_handler(dronecan.flytrex.delcon.StateReport, on_state_report)
		holder['handler'] = h
		self._handlers.append(h)
		logger.info('Waiting for one StateReport from node %s', node_id)

	def _on_live_param_read_toggled(self, checked: bool) -> None:
		'''
		@brief    Handle Live Param Read toggle button.
		@param    checked - True when enabled (pressed), False when disabled (released).
		@return   None
		'''

		def on_state_report(e):
			node_id = self._get_target_node_id()
			if node_id is None:
				return

			if e.transfer.source_node_id != node_id:
				return
			print('StateReport from', e.transfer.source_node_id)
			# print(dronecan.to_yaml(e.message))

			# Update the fields table from the received StateReport.
			self._update_fields_table_from_state_report(e.message)

		if checked:
			node_id = self._get_target_node_id()
			if node_id is None:
				# Revert toggle state if we can't start.
				self._live_param_read_btn.blockSignals(True)
				self._live_param_read_btn.setChecked(False)
				self._live_param_read_btn.blockSignals(False)
				return
			# Add an handler to process incoming parameters here.
			h = self._node.add_handler(dronecan.flytrex.delcon.StateReport, on_state_report)
			self._handlers.append(h)
		else:
			for h in list(self._handlers):
				try:
					h.try_remove()
				except Exception:
					logger.exception('Failed to remove handler')
			self._handlers = []

	def _load_fields_into_table(self, xml_path: str) -> None:
		'''
		@brief    Load fields XML and recreate the table.
		@param    xml_path - XML file path to load.
		@return   None
		'''

		try:
			if not os.path.exists(xml_path):
				show_error('Missing XML file',
						   'Could not find Delivery Controller fields XML.',
						   xml_path,
						   parent=self)
				return

			tree = ET.parse(xml_path)
			root = tree.getroot()
			if root.tag != 'fields':
				show_error('Invalid XML file',
						   'Unexpected root element; expected <fields>.',
						   f'Found <{root.tag}> in {xml_path}',
						   parent=self)
				return

			fields = []
			for field_elem in root.findall('field'):
				name = (field_elem.get('name') or '').strip()
				field_type = (field_elem.get('type') or '').strip()
				default_value = field_elem.get('default')
				default_value = '' if default_value is None else str(default_value)
				comment = field_elem.get('comment')
				comment = '' if comment is None else str(comment)
				editable = self._parse_bool_attr(field_elem.get('edit'), default=True)

				if not name:
					continue

				fields.append((name, field_type, default_value, comment, editable))

			self._table.setRowCount(len(fields))
			self._field_rows.clear()
			self._field_types.clear()

			unparsed_types: list[tuple[str, str]] = []
			for row_index, (name, field_type, default_value, comment, editable) in enumerate(fields):
				num_bits = self._type_to_num_bits(field_type)
				if field_type and not num_bits:
					unparsed_types.append((name, field_type))

				name_item = QTableWidgetItem(name)
				name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
				name_item.setTextAlignment(Qt.AlignCenter)
				if field_type:
					name_item.setToolTip(field_type)
				self._table.setItem(row_index, 0, name_item)

				value_item = QTableWidgetItem(default_value)
				value_item.setTextAlignment(Qt.AlignCenter)
				tooltip_lines: list[str] = []
				if field_type:
					tooltip_lines.append(field_type)
				if not editable:
					value_item.setFlags(value_item.flags() & ~Qt.ItemIsEditable)
					value_item.setBackground(QColor(Qt.lightGray).lighter(120))
					tooltip_lines.append('Read-only')
				if tooltip_lines:
					value_item.setToolTip('\n'.join(tooltip_lines))
				self._table.setItem(row_index, 1, value_item)

				type_item = QTableWidgetItem(field_type)
				type_item.setFlags(type_item.flags() & ~Qt.ItemIsEditable)
				type_item.setTextAlignment(Qt.AlignCenter)
				self._table.setItem(row_index, 2, type_item)

				num_bits_item = QTableWidgetItem(num_bits)
				num_bits_item.setFlags(num_bits_item.flags() & ~Qt.ItemIsEditable)
				num_bits_item.setTextAlignment(Qt.AlignCenter)
				self._table.setItem(row_index, 3, num_bits_item)

				comment_item = QTableWidgetItem(comment)
				comment_item.setFlags(comment_item.flags() & ~Qt.ItemIsEditable)
				self._table.setItem(row_index, 4, comment_item)

				self._field_rows[name] = row_index
				self._field_types[name] = field_type

			if unparsed_types:
				details = '\n'.join([f'- {n}: {t}' for (n, t) in unparsed_types])
				show_error(
					'Invalid field type',
					'Could not derive Num Bits from one or more field types.',
					details,
					parent=self,
				)

		except Exception as ex:
			logger.exception('Failed to load Delivery Controller fields XML: %s', ex)
			show_error('XML load error',
					   'Could not load Delivery Controller fields XML.',
					   ex,
					   parent=self)

	def __del__(self):
		'''
		@brief    Reset the singleton on destruction.
		@return   None
		'''

		try:
			self._stop_live_param_read_thread()
		except Exception:
			pass

		try:
			if self._auto_node_id_timer is not None:
				self._auto_node_id_timer.stop()
				self._auto_node_id_timer = None
		except Exception:
			pass

		# Remove any remaining DroneCAN handlers registered by this panel.
		try:
			for h in list(getattr(self, '_handlers', []) or []):
				try:
					h.try_remove()
				except Exception:
					pass
			self._handlers = []
		except Exception:
			pass

		global _singleton
		_singleton = None

	def closeEvent(self, event):
		'''
		@brief    Qt close event handler.
		@param    event - Qt close event.
		@return   None
		'''

		try:
			self._stop_live_param_read_thread()
		except Exception:
			logger.exception('Failed to stop live param read')
		try:
			if self._auto_node_id_timer is not None:
				self._auto_node_id_timer.stop()
		except Exception:
			logger.exception('Failed to stop auto node ID timer')
		try:
			super(DeliveryControllerPanel, self).closeEvent(event)
		finally:
			# Ensure singleton reset/handler cleanup even if shutdown fails.
			try:
				self.__del__()
			except Exception:
				pass


def spawn(parent, node):
	'''
	@brief    Spawn (or show) the singleton Delivery Controller panel.
	@param    parent - Parent Qt widget.
	@param    node - Local DroneCAN node instance.
	@return   DeliveryControllerPanel singleton instance.
	'''

	global _singleton
	if _singleton is None:
		try:
			_singleton = DeliveryControllerPanel(parent, node)
		except Exception as ex:
			logger.exception('Failed to spawn Delivery Controller panel: %s', ex)
			raise

	_singleton.show()
	_singleton.raise_()
	_singleton.activateWindow()
	return _singleton


get_icon = partial(get_icon, 'fa6s.asterisk')

