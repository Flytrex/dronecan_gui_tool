
#
# Copyright (C) 2026  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ilan Graidy
#

from functools import partial
from logging import getLogger
import threading
import os
import re
import xml.etree.ElementTree as ET

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIntValidator
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView, \
	QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog

from ..widgets import get_icon, show_error

__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'Delivery Controller'

logger = getLogger(__name__)

_singleton = None


class DeliveryControllerPanel(QDialog):
	def __init__(self, parent, node):
		'''
		@brief    Create the Delivery Controller panel window.
		@param    parent - Parent Qt widget.
		@param    node - Local DroneCAN node instance.
		@return   None
		'''

		super(DeliveryControllerPanel, self).__init__(parent)
		self.setWindowTitle(PANEL_NAME)
		self.setAttribute(Qt.WA_DeleteOnClose)
		self.resize(900, 600)
		self.setMinimumSize(700, 400)

		self._node = node
		self._field_rows = {}
		self._field_types = {}
		self._xml_path = os.path.join(os.path.dirname(__file__), 'delivery_controller_fields.xml')
		self._live_param_read_thread: threading.Thread | None = None
		self._live_param_read_stop_event: threading.Event | None = None
		self._live_param_read_node_id: int | None = None

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
		group_layout.addWidget(self._table)

		layout.addWidget(fields_group)

		buttons_row = QHBoxLayout()
		self._load_from_file_btn = QPushButton('Load From File', self)
		self._save_to_file_btn = QPushButton('Save To File', self)
		self._set_params_btn = QPushButton('Set Params', self)
		self._get_params_btn = QPushButton('Get Params', self)
		self._live_param_read_btn = QPushButton('Live Param Read', self)
		self._live_param_read_btn.setCheckable(True)
		self._live_param_read_btn.setStyleSheet('QPushButton { background-color: #FFFACD; }')
		self._load_from_file_btn.clicked.connect(self._on_load_from_file_clicked)
		self._save_to_file_btn.clicked.connect(self._on_save_to_file_clicked)
		self._set_params_btn.clicked.connect(self._on_set_params_clicked)
		self._get_params_btn.clicked.connect(self._on_get_params_clicked)
		self._live_param_read_btn.toggled.connect(self._on_live_param_read_toggled)
		buttons_row.addWidget(self._load_from_file_btn)
		buttons_row.addWidget(self._save_to_file_btn)
		buttons_row.addStretch(1)
		buttons_row.addWidget(self._set_params_btn)
		buttons_row.addWidget(self._get_params_btn)
		buttons_row.addWidget(self._live_param_read_btn)
		layout.addLayout(buttons_row)

		self._load_fields_into_table(self._xml_path)

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

	def _on_set_params_clicked(self) -> None:
		'''
		@brief    Handle Set Params button click.
		@return   None
		'''

		node_id = self._get_target_node_id()
		if node_id is None:
			return
		logger.info('Set Params clicked for node %s (not implemented yet)', node_id)

	def _on_get_params_clicked(self) -> None:
		'''
		@brief    Handle Get Params button click.
		@return   None
		'''

		node_id = self._get_target_node_id()
		if node_id is None:
			return
		logger.info('Get Params clicked for node %s (not implemented yet)', node_id)

	def _start_live_param_read_thread(self, node_id: int) -> None:
		'''
		@brief    Start the Live Param Read background thread.
		@param    node_id - Target node ID.
		@return   None
		'''

		self._stop_live_param_read_thread()
		stop_event = threading.Event()
		thread = threading.Thread(
			target=self._live_param_read_loop,
			args=(node_id, stop_event),
			name='DeliveryControllerLiveParamRead',
			daemon=True,
		)
		self._live_param_read_stop_event = stop_event
		self._live_param_read_thread = thread
		self._live_param_read_node_id = node_id
		thread.start()

	def _stop_live_param_read_thread(self) -> None:
		'''
		@brief    Request the Live Param Read thread to stop.
		@return   None
		'''

		evt = self._live_param_read_stop_event
		thr = self._live_param_read_thread
		self._live_param_read_stop_event = None
		self._live_param_read_thread = None
		self._live_param_read_node_id = None

		if evt is not None:
			evt.set()
		if thr is not None and thr.is_alive():
			# Keep the UI responsive: wait briefly only.
			thr.join(timeout=0.25)

	def _live_param_read_loop(self, node_id: int, stop_event: threading.Event) -> None:
		'''
		@brief    Background worker loop for Live Param Read.
		@param    node_id - Target node ID.
		@param    stop_event - Event that signals worker shutdown.
		@return   None
		'''

		logger.info('Live Param Read thread started for node %s', node_id)
		# NOTE: This is currently a stub. Replace the body with actual DroneCAN parameter reads.
		period_s = 1.0
		while not stop_event.is_set():
			logger.debug('Live Param Read tick (node %s)', node_id)
			# Wait returns early when stop_event is set.
			stop_event.wait(timeout=period_s)
		logger.info('Live Param Read thread stopped for node %s', node_id)

	def _on_live_param_read_toggled(self, checked: bool) -> None:
		'''
		@brief    Handle Live Param Read toggle button.
		@param    checked - True when enabled (pressed), False when disabled (released).
		@return   None
		'''

		if checked:
			node_id = self._get_target_node_id()
			if node_id is None:
				# Revert toggle state if we can't start.
				self._live_param_read_btn.blockSignals(True)
				self._live_param_read_btn.setChecked(False)
				self._live_param_read_btn.blockSignals(False)
				return
			self._start_live_param_read_thread(node_id)
		else:
			self._stop_live_param_read_thread()

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
					value_item.setBackground(Qt.lightGray)
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

		global _singleton
		_singleton = None

	def closeEvent(self, event):
		'''
		@brief    Qt close event handler.
		@param    event - Qt close event.
		@return   None
		'''

		self._stop_live_param_read_thread()

		super(DeliveryControllerPanel, self).closeEvent(event)
		self.__del__()


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

