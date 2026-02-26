
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

		# Load the design constants definition file
		self._design_const_set_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'DesignConstantsSet.json')
		self._design_constants_fields = self._load_design_constants_fields()

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

		# Second row: Two columns with labels and groupbox
		columns_layout = QHBoxLayout()

		# Left column: Parameter File Management section
		left_column = self._create_param_file_manage_section(header_group)
		columns_layout.addLayout(left_column, 1)

		# Right column: Design Constants section
		right_column = self._create_design_constants_section(header_group)
		columns_layout.addLayout(right_column, 1)

		header_layout.addLayout(columns_layout)

		layout.addWidget(header_group)

	def _create_param_file_manage_section(self, parent):
		'''
		@brief    Create the Parameter File Management section.
		@param    parent - Parent widget.
		@return   QVBoxLayout containing the section.
		'''
		left_column = QVBoxLayout()

		font_secondary = QFont()
		font_secondary.setBold(True)
		param_file_manage_label = QLabel(PARAM_FILE_MANAGE_NAME, parent)
		param_file_manage_label.setFont(font_secondary)
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
		upload_row.addWidget(self._upload_browse_button)
		left_column.addLayout(upload_row)

		download_row = QHBoxLayout()
		self._download_button = QPushButton('Download', parent)
		self._download_button.setStyleSheet("background-color: lightblue;")
		self._download_button.setFixedWidth(BUTTON_WIDTH)
		download_row.addWidget(self._download_button)

		self._download_textbox = QLineEdit(parent)
		download_row.addWidget(self._download_textbox)

		self._download_browse_button = QPushButton('Browse', parent)
		self._download_browse_button.setStyleSheet("background-color: lightblue;")
		self._download_browse_button.setFixedWidth(BUTTON_WIDTH)
		download_row.addWidget(self._download_browse_button)
		left_column.addLayout(download_row)

		STATUS_LABEL_WIDTH = 80
		STATUS_TEXTBOX_WIDTH = 150

		version_row = QHBoxLayout()
		version_label = QLabel('Version', parent)
		version_label.setFixedWidth(STATUS_LABEL_WIDTH)
		version_row.addWidget(version_label)

		self._version_textbox = QLineEdit(parent)
		self._version_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		version_row.addWidget(self._version_textbox)
		version_row.addStretch(1)

		left_column.addLayout(version_row)

		crc32_row = QHBoxLayout()
		crc32_label = QLabel('CRC32', parent)
		crc32_label.setFixedWidth(STATUS_LABEL_WIDTH)
		crc32_row.addWidget(crc32_label)

		self._crc32_textbox = QLineEdit(parent)
		self._crc32_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		crc32_row.addWidget(self._crc32_textbox)
		crc32_row.addStretch(1)

		left_column.addLayout(crc32_row)

		dirty_row = QHBoxLayout()
		dirty_label = QLabel('Dirty', parent)
		dirty_label.setFixedWidth(STATUS_LABEL_WIDTH)
		dirty_row.addWidget(dirty_label)

		self._dirty_textbox = QLineEdit(parent)
		self._dirty_textbox.setFixedWidth(STATUS_TEXTBOX_WIDTH)
		dirty_row.addWidget(self._dirty_textbox)
		dirty_row.addStretch(1)

		left_column.addLayout(dirty_row)

		left_column.addStretch(1)

		return left_column

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

	def _create_design_constants_section(self, parent):
		'''
		@brief    Create the Design Constants section with groupbox.
		@param    parent - Parent widget.
		@return   QVBoxLayout containing the section.
		'''
		right_column = QVBoxLayout()

		font_secondary = QFont()
		font_secondary.setBold(True)
		design_constants_label = QLabel(DESIGN_CONSTANTS_TUNE_NAME, parent)
		design_constants_label.setFont(font_secondary)
		right_column.addWidget(design_constants_label, 0, Qt.AlignTop)

		# Horizontal line below design_constants_label
		design_line = QFrame(parent)
		design_line.setFrameShape(QFrame.HLine)
		design_line.setFrameShadow(QFrame.Sunken)
		right_column.addWidget(design_line)

		design_const_set_group = QGroupBox(DESIGN_CONSTANTS_SET_NAME, parent)
		design_const_set_group.setMinimumHeight(200)
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

		# Create label and textbox for each field
		self._field_inputs = {}
		row = 0
		for field_name, field_data in self._design_constants_fields.items():
			# Label
			label = QLabel(field_name + ':', fields_container)
			label.setFixedHeight(20)
			comment = field_data.get('comment', '')
			if comment:
				label.setToolTip(comment)
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

		scroll_area.setWidget(fields_container)
		design_const_layout.addWidget(scroll_area, 1, 0)
		design_const_layout.setRowStretch(1, 1)

		right_column.addWidget(design_const_set_group, 1)

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