
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
import xml.etree.ElementTree as ET

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIntValidator, QColor, QFont
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView, \
	QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QComboBox, QGridLayout, QSizePolicy, QFrame, QScrollArea, QWidget
import numpy as np

from ..widgets import get_icon, show_error

__all__ = 'PANEL_NAME', 'spawn', 'get_icon'

PANEL_NAME = 'Spool Controller'
SPOOL_CONTROLLER_TUNE_NAME = 'Spool Controller Tuning'
PARAM_FILE_MANAGE_NAME = 'Parameter File Management'
DESIGN_CONSTANTS_TUNE_NAME = 'DesignConstantsSet Tuning'
DESIGN_CONSTANTS_SET_NAME = 'DesignConstantsSet'
PARAM_SET_EDIT_NAME = 'ParamSet Editing'
PARAM_SET_ID_NAME = 'ParamSet ID'
PARAM_SET_NAME = 'ParamSet'

BUTTON_HORIZONTAL_SPACING = 3
PARAM_SET_GROUPBOX_HEIGHT = 200

logger = getLogger(__name__)

_singleton = None


class SpoolControllerPanel(QDialog):
	def __init__(self, parent, node):
		super().__init__(parent)
		self.setWindowTitle(PANEL_NAME)
		self.setWindowIcon(get_icon())
		self.setAttribute(Qt.WA_DeleteOnClose)
		self.resize(900, 600)
		self.setMinimumSize(700, 400)

		self._node = node
		self._param_set_id_list = []

		# Load the design constants definition file
		self._design_const_set_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'DesignConstantsSet.json')
		self._design_constants_fields = self._load_design_constants_fields()
		# Load the param set definition file
		self._param_set_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'ParamSet.json')
		self._param_set_fields = self._load_param_set_fields()

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
		self._param_set_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)

		self._param_set_container = QWidget()
		self._param_set_container_layout = QVBoxLayout(self._param_set_container)
		self._param_set_container_layout.setContentsMargins(5, 5, 5, 5)
		self._param_set_container_layout.setAlignment(Qt.AlignTop)
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
		self._upload_button.setStyleSheet("background-color: lightblue;")
		self._upload_button.setFixedWidth(BUTTON_WIDTH)
		upload_row.addWidget(self._upload_button)

		self._upload_textbox = QLineEdit(parent)
		upload_row.addWidget(self._upload_textbox)

		self._upload_browse_button = QPushButton('Browse', parent)
		self._upload_browse_button.setStyleSheet("background-color: lightblue;")
		self._upload_browse_button.setFixedWidth(BUTTON_WIDTH)
		self._upload_browse_button.clicked.connect(self._on_upload_browse_clicked)
		upload_row.addWidget(self._upload_browse_button)
		left_column.addLayout(upload_row)

		download_row = QHBoxLayout()
		self._download_button = QPushButton('Download', parent)
		self._download_button.setStyleSheet("background-color: lightblue;")
		self._download_button.setFixedWidth(BUTTON_WIDTH)
		self._download_button.clicked.connect(self._on_download_clicked)
		download_row.addWidget(self._download_button)

		self._download_textbox = QLineEdit(parent)
		download_row.addWidget(self._download_textbox)

		self._download_browse_button = QPushButton('Browse', parent)
		self._download_browse_button.setStyleSheet("background-color: lightblue;")
		self._download_browse_button.setFixedWidth(BUTTON_WIDTH)
		self._download_browse_button.clicked.connect(self._on_download_clicked)
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
		self._add_param_set_editing_groupbox(param_set_id)

	def _add_param_set_editing_groupbox(self, param_set_id):
		'''
		@brief    Create and add a groupbox for editing a specific ParamSet ID.
		@param    param_set_id - The ID of the ParamSet to edit.
		@return   None
		'''

		groupbox = QGroupBox(f'{PARAM_SET_NAME} {param_set_id}', self._param_set_container)
		groupbox.setFixedHeight(PARAM_SET_GROUPBOX_HEIGHT)
		# Use scroll area width minus scrollbar width for consistent sizing
		scrollbar_width = self._param_set_scroll_area.verticalScrollBar().sizeHint().width()
		available_width = self._param_set_scroll_area.width() - scrollbar_width - self._param_set_container_layout.contentsMargins().left() - self._param_set_container_layout.contentsMargins().right()
		groupbox.setFixedWidth(available_width // 2)
		groupbox.setStyleSheet("""
			QGroupBox {
				border: 1px solid gray;
				border-radius: 3px;
				margin-top: 0px;
				padding-top: 15px;
				background-color: lightyellow;
			}
			QGroupBox::title {
				subcontrol-origin: margin;
				subcontrol-position: top left;
				padding: 2px 5px;
				background-color: palette(window);
				border: 1px solid gray;
				top: 0px;
				left: 0px;
			}
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
		buttons_layout.addWidget(execute_button)

		store_button = QPushButton('Store', groupbox)
		buttons_layout.addWidget(store_button)

		recall_button = QPushButton('Recall', groupbox)
		buttons_layout.addWidget(recall_button)

		close_button = QPushButton('Close', groupbox)
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
		param_set_fields_container.setStyleSheet("background-color: lightyellow;")
		param_set_fields_layout = QGridLayout(param_set_fields_container)
		param_set_fields_layout.setColumnStretch(0, 0)
		param_set_fields_layout.setColumnStretch(1, 1)
		param_set_fields_layout.setSpacing(5)
		param_set_fields_layout.setContentsMargins(5, 0, 5, 0)

		self._parse_param_set_file(param_set_id, param_set_fields_container, param_set_fields_layout)

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
		if param_set_id in self._param_set_id_list:
			self._param_set_id_list.remove(param_set_id)
		self._param_set_container_layout.removeWidget(groupbox)
		groupbox.deleteLater()

	def _on_download_clicked(self):
		'''
		@brief    Handle download button click to select a save destination.
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

				fields_layout.setRowMinimumHeight(row, 0)
				row += 1

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

		design_const_set_group = QGroupBox(DESIGN_CONSTANTS_SET_NAME, parent)
		design_const_set_group.setMinimumHeight(200)
		design_const_set_group.setMaximumHeight(320)
		design_const_set_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
		design_const_set_group.setStyleSheet("""
			QGroupBox {
				border: 1px solid gray;
				border-radius: 3px;
				margin-top: 0px;
				padding-top: 15px;
				background-color: lightblue;
			}
			QGroupBox::title {
				subcontrol-origin: margin;
				subcontrol-position: top left;
				padding: 2px 5px;
				background-color: palette(window);
				border: 1px solid gray;
				top: 0px;
				left: 0px;
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
		buttons_layout.addWidget(self._store_button)

		self._recall_button = QPushButton('Recall', design_const_set_group)
		buttons_layout.addWidget(self._recall_button)

		design_const_layout.addLayout(buttons_layout, 0, 0)

		# Scrollable area for fields
		scroll_area = QScrollArea(design_const_set_group)
		scroll_area.setWidgetResizable(True)
		scroll_area.setFrameShape(QFrame.NoFrame)
		scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

		# Container widget for fields
		fields_container = QWidget()
		fields_container.setStyleSheet("background-color: lightblue;")
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