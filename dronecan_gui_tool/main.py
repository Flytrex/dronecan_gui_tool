#
# Copyright (C) 2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import logging
import multiprocessing
import os
import sys
import time
import tempfile
import re
import glob
import io
import shutil
import zipfile
import xml.etree.ElementTree as ET

assert sys.version[0] == '3'

from argparse import ArgumentParser
parser = ArgumentParser(description='DroneCAN GUI tool')

parser.add_argument("--debug", action='store_true', help="enable debugging")
parser.add_argument("--dsdl", help="path to custom DSDL")
parser.add_argument("--signing-passphrase", help="MAVLink2 signing passphrase", default=None)
parser.add_argument("--interface", help="skip the setup dialog by setting the device to connect to")
parser.add_argument("--baudrate", help="set the baudrate", type=int, default=115200)
parser.add_argument("--bitrate", help="set the bitrate of the CAN Bus", type=int, default=1000000)
parser.add_argument("--bus", help="set the CAN Bus number", type=int, default=1)
parser.add_argument("--filtered", action='store_true', help="enable filtering of DroneCAN traffic")
parser.add_argument("--target-system", help="set the targetted system", type=int, default=0)

args = parser.parse_args()

#
# Configuring logging before other packages are imported
#
if args.debug:
    logging_level = logging.DEBUG
else:
    logging_level = logging.INFO

logging.basicConfig(stream=sys.stderr, level=logging_level,
                    format='%(asctime)s %(levelname)s %(name)s %(message)s')

log_file = tempfile.NamedTemporaryFile(mode='w', prefix='dronecan_gui_tool-', suffix='.log', delete=False)
file_handler = logging.FileHandler(log_file.name)
file_handler.setLevel(logging_level)
file_handler.setFormatter(logging.Formatter('%(asctime)s [%(process)d] %(levelname)-8s %(name)-25s %(message)s'))
logging.root.addHandler(file_handler)

logger = logging.getLogger(__name__.replace('__', ''))
logger.info('Spawned')

#
# Applying Windows-specific hacks
#
os.environ['PATH'] = os.environ['PATH'] + ';' + os.path.dirname(sys.executable)  # Otherwise it fails to load on Win 10

#
# Configuring multiprocessing.
# Start method must be configured globally, and only once. Using 'spawn' ensures full compatibility with Windoze.
# We need to check first if the start mode is already configured, because this code will be re-run for every child.
#
if multiprocessing.get_start_method(True) != 'spawn':
    multiprocessing.set_start_method('spawn')

#
# Importing other stuff once the logging has been configured
#
from serial import SerialException

import dronecan

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QSplitter, QAction, QActionGroup, QMessageBox
from PyQt5.QtGui import QKeySequence, QDesktopServices
from PyQt5.QtCore import QTimer, Qt, QUrl, QThread, QObject, pyqtSignal, pyqtSlot

from .version import __version__, __flytrex_version__
from .setup_window import run_setup_window
from .active_data_type_detector import ActiveDataTypeDetector

from .widgets import show_error, get_icon, get_app_icon
from .widgets.node_monitor import NodeMonitorWidget
from .widgets.local_node import LocalNodeWidget
from .widgets.local_node import AdapterSettingsWidget
from .widgets.local_node import setup_filtering
from .widgets.log_message_display import LogMessageDisplayWidget
from .widgets.bus_monitor import BusMonitorManager
from .widgets.dynamic_node_id_allocator import DynamicNodeIDAllocatorWidget
from .widgets.file_server import FileServerWidget
from .widgets.node_properties import NodePropertiesWindow
from .widgets.console import ConsoleManager, InternalObjectDescriptor
from .widgets.subscriber import SubscriberWindow
from .widgets.plotter import PlotterManager
from .widgets.about_window import AboutWindow
from .widgets.can_adapter_control_panel import spawn_window as spawn_can_adapter_control_panel

from urllib.request import Request, urlopen

from .panels import PANELS


NODE_NAME = 'org.dronecan.gui_tool'

# DSDL update source: public Flytrex fork of public_regulated_data_types
DSDL_REPO = 'Flytrex/public_regulated_data_types'
DEFAULT_DSDL_REPO_BRANCH = 'Flyhawk-5.0'
DSDL_LOAD_NAMESPACES = ('uavcan', 'dronecan', 'ardupilot', 'com', 'cuav', 'flytrex')
DSDL_MANIFEST = '.flytrex_dsdl_manifest'
# Top-level archive entries that are not DSDL trees and must not be synced.
DSDL_SYNC_EXCLUDES = {'.github', '.gitignore', 'tests', 'LICENSE', 'README.md',
                     'test.py', '.flytrex_dsdl_version'}


def _bundled_config_file_path():
    if getattr(sys, 'frozen', False):
        return os.path.join(os.path.dirname(sys.executable), 'config.xml')
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config.xml')

def _user_config_file_path():
    app_name = NODE_NAME.rsplit('.', 1)[-1]
    if os.name == 'nt':
        config_root = os.environ.get('APPDATA') or os.path.expanduser('~')
    else:
        config_root = os.environ.get('XDG_CONFIG_HOME') or os.path.join(os.path.expanduser('~'), '.config')
    return os.path.join(config_root, app_name, 'config.xml')

def _read_config():
    for config_path in (_user_config_file_path(), _bundled_config_file_path()):
        try:
            tree = ET.parse(config_path)
            root = tree.getroot()
            dsdl_branch = root.findtext('dsdl_branch')
            return {'dsdl_branch': dsdl_branch.strip()} if dsdl_branch and dsdl_branch.strip() else {}
        except FileNotFoundError:
            continue
        except Exception:
            logger.warning('Could not read config file: %s', config_path, exc_info=True)
            return {}
    return {}

def _write_config(config):
    root = ET.Element('config')
    dsdl_branch = config.get('dsdl_branch')
    if dsdl_branch:
        ET.SubElement(root, 'dsdl_branch').text = dsdl_branch
    config_path = _user_config_file_path()
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    ET.ElementTree(root).write(config_path, encoding='utf-8', xml_declaration=True)


class _DsdlBranchListWorker(QObject):
    finished = pyqtSignal(dict)

    def __init__(self, repo):
        super().__init__()
        self._repo = repo

    @pyqtSlot()
    def run(self):
        result = {'branches': [], 'error': None}
        try:
            import json
            api_url = 'https://api.github.com/repos/{}/branches?per_page=100'.format(self._repo)
            req = Request(api_url,
                          headers={'Accept': 'application/vnd.github+json',
                                   'User-Agent': 'dronecan_gui_tool'})
            with urlopen(req, timeout=10) as resp:
                data = json.load(resp)
            result['branches'] = [item['name'] for item in data]
        except Exception as ex:
            result['error'] = str(ex)

        self.finished.emit(result)


class _UpdateCheckWorker(QObject):
    """
    Performs the slow I/O for the update check (SMB share scan and GitHub
    API request) on a background thread. Emits a single ``finished`` signal
    carrying the result dict; the GUI thread does the dialog work.
    """

    finished = pyqtSignal(dict)

    def __init__(self, updates_dir, dsdl_target, dsdl_skip_marker, repo, branch):
        super().__init__()
        self._updates_dir = updates_dir
        self._dsdl_target = dsdl_target
        # If non-None and exists at run-time, the DSDL check is skipped.
        # Used to avoid touching a real git working tree in dev.
        self._dsdl_skip_marker = dsdl_skip_marker
        self._repo = repo
        self._branch = branch

    @pyqtSlot()
    def run(self):
        result = {
            'msi': {'unreachable': False, 'error': None, 'latest': None},
            'dsdl': {'skipped': False, 'reason': None, 'error': None,
                     'remote_sha': None, 'branch': self._branch,
                     'git_working_tree': False},
        }

        # --- MSI scan on the shared drive (can hang on disconnected SMB) ---
        try:
            if not os.path.isdir(self._updates_dir):
                result['msi']['unreachable'] = True
            else:
                pattern = re.compile(
                    r'^dronecan_gui_tool-(\d+(?:\.\d+)*)-win64-flytrex-(\d+(?:\.\d+)*)\.msi$',
                    re.IGNORECASE)
                latest = None
                for name in os.listdir(self._updates_dir):
                    m = pattern.match(name)
                    if not m:
                        continue
                    v = tuple(int(x) for x in m.group(1).split('.'))
                    fv = tuple(int(x) for x in m.group(2).split('.'))
                    key = (v, fv)
                    if latest is None or key > latest[0]:
                        latest = (key, m.group(1), m.group(2), name)
                result['msi']['latest'] = latest
        except OSError as ex:
            result['msi']['error'] = str(ex)

        # --- DSDL: GitHub API request (up to 10 s) -------------------------
        try:
            if not self._branch:
                result['dsdl']['skipped'] = True
                result['dsdl']['reason'] = 'no_branch'
            elif not os.path.isdir(self._dsdl_target):
                result['dsdl']['skipped'] = True
                result['dsdl']['reason'] = 'no_target'
            elif self._dsdl_skip_marker and os.path.exists(self._dsdl_skip_marker):
                result['dsdl']['git_working_tree'] = True

            if not result['dsdl']['skipped']:
                import json
                api_url = 'https://api.github.com/repos/{}/branches/{}'.format(
                    self._repo, self._branch)
                req = Request(api_url,
                              headers={'Accept': 'application/vnd.github+json',
                                       'User-Agent': 'dronecan_gui_tool'})
                with urlopen(req, timeout=10) as resp:
                    data = json.load(resp)
                result['dsdl']['remote_sha'] = data['commit']['sha']
        except Exception as ex:
            result['dsdl']['error'] = str(ex)

        self.finished.emit(result)


class MainWindow(QMainWindow):
    MAX_SUCCESSIVE_NODE_ERRORS = 1000

    # noinspection PyTypeChecker,PyCallByClass,PyUnresolvedReferences
    def __init__(self, node, iface_name, iface_kwargs):
        # Parent
        super(MainWindow, self).__init__()
        self.setWindowTitle('DroneCAN GUI Tool v{} (Flytrex v{})'.format(
            '.'.join(map(str, __version__)),
            '.'.join(map(str, __flytrex_version__))))
        self.setWindowIcon(get_app_icon())

        self._node = node
        self._successive_node_errors = 0
        self._iface_name = iface_name

        self._active_data_type_detector = ActiveDataTypeDetector(self._node)

        self._node_spin_timer = QTimer(self)
        self._node_spin_timer.timeout.connect(self._spin_node)
        self._node_spin_timer.setSingleShot(False)
        self._node_spin_timer.start(10)

        self._node_windows = {}  # node ID : window object

        # Background worker for the (slow, network-heavy) update check.
        # See _check_for_updates / _UpdateCheckWorker.
        self._update_check_thread = None
        self._update_check_worker = None
        self._dsdl_branch_thread = None
        self._dsdl_branch_worker = None
        self._config = _read_config()
        self._selected_dsdl_branch = self._config.get('dsdl_branch') or DEFAULT_DSDL_REPO_BRANCH
        if not self._config.get('dsdl_branch'):
            self._config['dsdl_branch'] = self._selected_dsdl_branch
            try:
                _write_config(self._config)
            except Exception:
                logger.warning('Could not write default config file: %s', _user_config_file_path(), exc_info=True)

        self._node_monitor_widget = NodeMonitorWidget(self, node)
        self._node_monitor_widget.on_info_window_requested = self._show_node_window

        self._local_node_widget = LocalNodeWidget(self, node)
        self._adapter_settings_widget = AdapterSettingsWidget(self, node)
        self._log_message_widget = LogMessageDisplayWidget(self, node)
        self._dynamic_node_id_allocation_widget = DynamicNodeIDAllocatorWidget(self, node,
                                                                               self._node_monitor_widget.monitor)
        self._file_server_widget = FileServerWidget(self, node)

        self._plotter_manager = PlotterManager(self._node)
        self._bus_monitor_manager = BusMonitorManager(self._node, iface_name)
        # Console manager depends on other stuff via context, initialize it last
        self._console_manager = ConsoleManager(self._make_console_context)

        if args.signing_passphrase is not None:
            self._node.can_driver.set_signing_passphrase(args.signing_passphrase)
        elif iface_kwargs['mavlink_signing_key']:
            self._node.can_driver.set_signing_passphrase(iface_kwargs['mavlink_signing_key'])

        #
        # File menu
        #
        quit_action = QAction(get_icon('fa6s.right-from-bracket'), '&Quit', self)
        quit_action.setShortcut(QKeySequence('Ctrl+Shift+Q'))
        quit_action.triggered.connect(self.close)

        file_menu = self.menuBar().addMenu('&File')
        file_menu.addAction(quit_action)

        #
        # Tools menu
        #
        show_bus_monitor_action = QAction(get_icon('fa6s.bus'), '&Bus Monitor', self)
        show_bus_monitor_action.setShortcut(QKeySequence('Ctrl+Shift+B'))
        show_bus_monitor_action.setStatusTip('Open bus monitor window')
        show_bus_monitor_action.triggered.connect(self._bus_monitor_manager.spawn_monitor)

        show_console_action = QAction(get_icon('fa6s.terminal'), 'Interactive &Console', self)
        show_console_action.setShortcut(QKeySequence('Ctrl+Shift+T'))
        show_console_action.setStatusTip('Open interactive console window')
        show_console_action.triggered.connect(self._show_console_window)

        new_subscriber_action = QAction(get_icon('fa6.newspaper'), '&Subscriber', self)
        new_subscriber_action.setShortcut(QKeySequence('Ctrl+Shift+S'))
        new_subscriber_action.setStatusTip('Open subscription tool')
        new_subscriber_action.triggered.connect(
            lambda: SubscriberWindow.spawn(self, self._node, self._active_data_type_detector))

        new_plotter_action = QAction(get_icon('fa6s.chart-area'), '&Plotter', self)
        new_plotter_action.setShortcut(QKeySequence('Ctrl+Shift+P'))
        new_plotter_action.setStatusTip('Open new graph plotter window')
        new_plotter_action.triggered.connect(self._plotter_manager.spawn_plotter)

        show_can_adapter_controls_action = QAction(get_icon('fa6s.plug'), 'CAN &Adapter Control Panel', self)
        show_can_adapter_controls_action.setShortcut(QKeySequence('Ctrl+Shift+A'))
        show_can_adapter_controls_action.setStatusTip('Open CAN adapter control panel (if supported by the adapter)')
        show_can_adapter_controls_action.triggered.connect(self._try_spawn_can_adapter_control_panel)

        tools_menu = self.menuBar().addMenu('&Tools')
        tools_menu.addAction(show_bus_monitor_action)
        tools_menu.addAction(show_console_action)
        tools_menu.addAction(new_subscriber_action)
        tools_menu.addAction(new_plotter_action)
        tools_menu.addAction(show_can_adapter_controls_action)

        #
        # Panels menu
        #
        panels_menu = self.menuBar().addMenu('&Panels')

        for idx, panel in enumerate(PANELS):
            action = QAction(panel.name, self)
            icon = panel.get_icon()
            if icon:
                action.setIcon(icon)
            if idx < 9:
                action.setShortcut(QKeySequence('Ctrl+Shift+%d' % (idx + 1)))
            action.triggered.connect(lambda state, panel=panel: panel.safe_spawn(self, self._node))
            panels_menu.addAction(action)

        #
        # Configurations menu
        #
        configurations_menu = self.menuBar().addMenu('&Configurations')
        self._set_dsdl_branch_menu = configurations_menu.addMenu('Set &DSDL Branch')
        self._dsdl_branch_action_group = QActionGroup(self)
        self._dsdl_branch_action_group.setExclusive(True)
        self._dsdl_branch_action_group.triggered.connect(
            lambda action: self._set_dsdl_branch(action.data()))
        self._populate_dsdl_branch_menu(
            [self._selected_dsdl_branch] if self._selected_dsdl_branch else [])
        self._load_dsdl_branches()

        #
        # Help menu
        #
        dronecan_website_action = QAction(get_icon('fa6s.globe'), 'Open DroneCAN &Website', self)
        dronecan_website_action.triggered.connect(lambda: QDesktopServices.openUrl(QUrl('http://dronecan.org')))

        show_log_directory_action = QAction(get_icon('fa6.pen-to-square'), 'Open &Log Directory', self)
        show_log_directory_action.triggered.connect(
            lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(os.path.dirname(log_file.name))))

        check_for_updates_action = QAction(get_icon('fa6s.cloud-arrow-down'), 'Check for &Updates', self)
        check_for_updates_action.setStatusTip('Check the shared drive for a newer MSI release of the DroneCAN GUI Tool')
        check_for_updates_action.triggered.connect(self._check_for_updates)

        about_action = QAction(get_icon('fa6s.info'), '&About', self)
        about_action.triggered.connect(lambda: AboutWindow(self).show())

        help_menu = self.menuBar().addMenu('&Help')
        help_menu.addAction(dronecan_website_action)
        help_menu.addAction(show_log_directory_action)
        help_menu.addAction(check_for_updates_action)
        help_menu.addAction(about_action)

        #
        # Window layout
        #
        self.statusBar().show()

        def make_vbox(*widgets, stretch_index=None):
            box = QVBoxLayout(self)
            for idx, w in enumerate(widgets):
                box.addWidget(w, 1 if idx == stretch_index else 0)
            container = QWidget(self)
            container.setLayout(box)
            container.setContentsMargins(0, 0, 0, 0)
            return container

        def make_splitter(orientation, *widgets):
            spl = QSplitter(orientation, self)
            for w in widgets:
                spl.addWidget(w)
            return spl

        self.setCentralWidget(make_splitter(Qt.Horizontal,
                                            make_vbox(self._local_node_widget,
                                                      self._adapter_settings_widget,
                                                      self._node_monitor_widget,
                                                      self._file_server_widget),
                                            make_splitter(Qt.Vertical,
                                                          make_vbox(self._log_message_widget),
                                                          make_vbox(self._dynamic_node_id_allocation_widget,
                                                                    stretch_index=1))))

        # Run an update check shortly after the window is shown.
        QTimer.singleShot(2000, lambda: self._check_for_updates(silent=True))

    def _try_spawn_can_adapter_control_panel(self):
        try:
            spawn_can_adapter_control_panel(self, self._node, self._iface_name)
        except Exception as ex:
            show_error('CAN Adapter Control Panel error', 'Could not spawn CAN Adapter Control Panel', ex, self)

    def _populate_dsdl_branch_menu(self, branches):
        self._set_dsdl_branch_menu.clear()
        for action in self._dsdl_branch_action_group.actions():
            self._dsdl_branch_action_group.removeAction(action)

        branches = [branch for branch in branches if branch]
        if self._selected_dsdl_branch and self._selected_dsdl_branch not in branches:
            branches.insert(0, self._selected_dsdl_branch)

        if not branches:
            action = QAction('No branches available', self)
            action.setEnabled(False)
            self._set_dsdl_branch_menu.addAction(action)
            return

        for branch in branches:
            action = QAction(branch, self)
            action.setCheckable(True)
            action.setData(branch)
            action.setChecked(branch == self._selected_dsdl_branch)
            self._dsdl_branch_action_group.addAction(action)
            self._set_dsdl_branch_menu.addAction(action)

    def _load_dsdl_branches(self):
        if self._dsdl_branch_thread is not None:
            return

        thread = QThread(self)
        worker = _DsdlBranchListWorker(DSDL_REPO)
        worker.moveToThread(thread)

        thread.started.connect(worker.run)
        worker.finished.connect(self._handle_dsdl_branch_results)
        worker.finished.connect(thread.quit)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._on_dsdl_branch_finished)

        self._dsdl_branch_thread = thread
        self._dsdl_branch_worker = worker
        thread.start()

    def _handle_dsdl_branch_results(self, result):
        if result['error']:
            logger.warning('Could not load DSDL repo branches: %s', result['error'])
            return
        self._populate_dsdl_branch_menu(result['branches'])

    def _on_dsdl_branch_finished(self):
        self._dsdl_branch_thread = None
        self._dsdl_branch_worker = None

    def _set_dsdl_branch(self, dsdl_branch):
        self._selected_dsdl_branch = dsdl_branch
        self._config['dsdl_branch'] = dsdl_branch
        try:
            _write_config(self._config)
        except Exception as ex:
            logger.warning('Could not write config file: %s', _user_config_file_path(), exc_info=True)
            QMessageBox.warning(self, 'Configuration Error',
                                'Could not save configuration to:\n{}\n\n{}'.format(
                                    _user_config_file_path(), ex))
            return
        self.statusBar().showMessage('DSDL branch set to {}'.format(dsdl_branch), 3000)

    def _check_for_updates(self, silent=False):
        """
        Schedule an update check on a background thread.

        Both the SMB share listing and the GitHub API request can block for
        several seconds (or much longer if the network/share is unreachable),
        so we never run them on the GUI thread. Results are handled in
        ``_handle_update_result`` once the worker emits ``finished``.
        """
        if self._update_check_thread is not None:
            if not silent:
                self.statusBar().showMessage('Update check already in progress...', 3000)
            return

        updates_dir = r'G:\Shared drives\Engineering\Lab Tools\DroneCAN GUI Tool, Flytrex Version'
        dsdl_target = self._dsdl_specs_dir()

        # Refuse to touch a real git working tree (e.g. the source submodule
        # in dev). The installed MSI may have a stray .git file/folder copied
        # in from the source tree -- that's a packaging artifact, not a real
        # checkout, so we still want to update it. Only skip when running
        # under the workspace layout (dronecan imported from a "pydronecan"
        # sibling, not from the frozen `lib/`).
        dronecan_dir = os.path.dirname(os.path.abspath(dronecan.__file__))
        is_dev = os.path.basename(os.path.dirname(dronecan_dir)).lower() == 'pydronecan'
        dsdl_skip_marker = os.path.join(dsdl_target, '.git') if is_dev else None

        thread = QThread(self)
        worker = _UpdateCheckWorker(updates_dir, dsdl_target, dsdl_skip_marker,
                                    DSDL_REPO, self._selected_dsdl_branch)
        worker.moveToThread(thread)

        thread.started.connect(worker.run)
        worker.finished.connect(
            lambda result, silent=silent: self._handle_update_result(result, silent))
        worker.finished.connect(thread.quit)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._on_update_check_finished)

        self._update_check_thread = thread
        self._update_check_worker = worker
        logger.info('Update check started (silent=%s)', silent)
        thread.start()

    def _on_update_check_finished(self):
        self._update_check_thread = None
        self._update_check_worker = None

    def _handle_update_result(self, result, silent):
        """Slot called on the GUI thread after the worker finishes."""
        updates_dir = r'G:\Shared drives\Engineering\Lab Tools\DroneCAN GUI Tool, Flytrex Version'
        current_version = '.'.join(map(str, __version__))
        current_flytrex = '.'.join(map(str, __flytrex_version__))
        current_combo = (tuple(__version__), tuple(__flytrex_version__))

        # --- MSI part -------------------------------------------------------
        msi = result['msi']
        if msi['unreachable']:
            if silent:
                logger.info('Update check skipped: %s not accessible', updates_dir)
            else:
                QMessageBox.warning(self, 'Check for Updates',
                                    'Could not access the updates directory:\n{}\n\n'
                                    'Make sure the shared drive is mounted.'.format(updates_dir))
        elif msi['error']:
            logger.warning('Update check failed: %s', msi['error'])
            if not silent:
                QMessageBox.warning(self, 'Check for Updates',
                                    'Could not list the updates directory:\n{}'.format(msi['error']))
        else:
            latest = msi['latest']
            if latest is None:
                if not silent:
                    QMessageBox.warning(self, 'Check for Updates',
                                        'No installation files were found in:\n{}'.format(updates_dir))
            elif latest[0] > current_combo:
                installer_path = os.path.join(updates_dir, latest[3])
                msg = QMessageBox(self)
                msg.setIcon(QMessageBox.Information)
                msg.setWindowTitle('Check for Updates')
                msg.setTextFormat(Qt.RichText)
                msg.setText(
                    'A new version is available.<br><br>'
                    'Installed: <b>{cur} (flytrex {curf})</b><br>'
                    'Latest: <b>{latest} (flytrex {latestf})</b><br><br>'
                    'File: <code>{name}</code><br><br>'
                    'Press <b>Install Now</b> to close the application and run the installer, '
                    'or open the <a href="file:///{url}">updates folder</a> to install manually.'.format(
                        cur=current_version, curf=current_flytrex,
                        latest=latest[1], latestf=latest[2],
                        name=latest[3],
                        url=updates_dir.replace('\\', '/')))
                msg.setTextInteractionFlags(Qt.TextBrowserInteraction)
                install_btn = msg.addButton('Install Now', QMessageBox.AcceptRole)
                msg.addButton('Later', QMessageBox.RejectRole)
                msg.setDefaultButton(install_btn)
                msg.exec_()
                if msg.clickedButton() is install_btn:
                    self._launch_installer_and_quit(installer_path)
                    return  # app is quitting; don't bother with DSDL dialog
            elif not silent:
                QMessageBox.information(
                    self, 'Check for Updates',
                    'You are running the latest version ({} flytrex {}).'.format(
                        current_version, current_flytrex))

        # --- DSDL part ------------------------------------------------------
        dsdl = result['dsdl']
        dsdl_branch = dsdl.get('branch')
        if dsdl['skipped']:
            logger.info('DSDL update check skipped (%s)', dsdl['reason'])
            return
        if dsdl['error']:
            logger.warning('DSDL update check failed: %s', dsdl['error'])
            if not silent:
                QMessageBox.warning(self, 'Check for DSDL Updates',
                                    'Could not check for DSDL updates:\n{}'.format(dsdl['error']))
            return

        remote_sha = dsdl['remote_sha']
        if not remote_sha:
            return

        local_sha = self._read_local_dsdl_sha()
        if local_sha == remote_sha:
            if not silent:
                QMessageBox.information(
                    self, 'Check for DSDL Updates',
                    'DSDL definitions are up to date.\n'
                    'Branch: {}\nCommit: {}'.format(dsdl_branch, remote_sha[:7]))
            return

        target = self._dsdl_specs_dir()
        short_local = local_sha[:7] if local_sha else '(unknown)'
        if dsdl.get('git_working_tree'):
            logger.info('DSDL update available for %s, but target is a git working tree: %s',
                        dsdl_branch, target)
            if not silent:
                QMessageBox.information(
                    self, 'DSDL Update Available',
                    'Newer DSDL definitions are available on branch {}.\n\n'
                    'Local commit: {}\n'
                    'Remote commit: {}\n\n'
                    'The DSDL directory is a git working tree, so it will not be overwritten:\n{}\n\n'
                    'Use git to switch or update this checkout.'.format(
                        dsdl_branch, short_local, remote_sha[:7], target))
            return

        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle('DSDL Update Available')
        msg.setTextFormat(Qt.RichText)
        msg.setText(
            'Newer DSDL definitions are available on branch <b>{branch}</b> of '
            '<a href="https://github.com/{repo}/tree/{branch}">{repo}</a>.<br><br>'
            'Local commit: <b>{local}</b><br>'
            'Remote commit: <b>{remote}</b><br><br>'
            'Update the DSDL files in:<br><code>{path}</code>?'.format(
                repo=DSDL_REPO, branch=dsdl_branch,
                local=short_local, remote=remote_sha[:7],
                path=target))
        msg.setTextInteractionFlags(Qt.TextBrowserInteraction)
        update_btn = msg.addButton('Update', QMessageBox.AcceptRole)
        msg.addButton('Later', QMessageBox.RejectRole)
        msg.setDefaultButton(update_btn)
        msg.exec_()
        if msg.clickedButton() is update_btn:
            self._apply_dsdl_update(remote_sha)

    def _dsdl_specs_dir(self):
        # Installed (MSI) layout: dsdl files live next to the dronecan package.
        installed = os.path.join(os.path.dirname(dronecan.__file__), 'dsdl_specs')
        if os.path.isdir(installed):
            return installed
        # Dev layout: dronecan loads DSDL from <repo>/public_regulated_data_types
        # (see pydronecan/dronecan/__init__.py).
        dev = os.path.normpath(os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(dronecan.__file__))),
            '..', 'public_regulated_data_types'))
        if os.path.isdir(dev):
            return dev
        return installed  # report the canonical path even if missing

    def _read_local_dsdl_sha(self):
        marker = os.path.join(self._dsdl_specs_dir(), '.flytrex_dsdl_version')
        try:
            with open(marker, 'r', encoding='utf-8') as f:
                return f.read().strip() or None
        except OSError:
            return None

    def _write_local_dsdl_sha(self, sha):
        marker = os.path.join(self._dsdl_specs_dir(), '.flytrex_dsdl_version')
        try:
            with open(marker, 'w', encoding='utf-8') as f:
                f.write(sha)
        except OSError:
            logger.warning('Could not write DSDL version marker', exc_info=True)

    def _write_dsdl_manifest(self):
        dsdl_dir = self._dsdl_specs_dir()
        manifest = os.path.join(dsdl_dir, DSDL_MANIFEST)
        try:
            entries = []
            for dirpath, _dirnames, filenames in os.walk(dsdl_dir):
                for filename in filenames:
                    full = os.path.join(dirpath, filename)
                    rel = os.path.relpath(full, dsdl_dir).replace(os.sep, '/')
                    if rel != DSDL_MANIFEST:
                        entries.append(rel)
            entries.sort()
            entries.append(DSDL_MANIFEST)
            with open(manifest, 'w', encoding='utf-8') as f:
                f.write('\n'.join(entries))
                f.write('\n')
        except OSError:
            logger.warning('Could not write DSDL manifest', exc_info=True)

    def _validate_dsdl_dir(self, dsdl_dir):
        paths = [os.path.join(dsdl_dir, name) for name in DSDL_LOAD_NAMESPACES
                 if os.path.isdir(os.path.join(dsdl_dir, name))]
        if not paths:
            raise RuntimeError('Archive contains no supported DSDL namespaces')
        dronecan.dsdl.parse_namespaces(paths)

    def _apply_dsdl_update(self, remote_sha):

        target = self._dsdl_specs_dir()
        archive_url = 'https://github.com/{}/archive/{}.zip'.format(DSDL_REPO, remote_sha)
        staging_dir = None

        try:
            req = Request(archive_url, headers={'User-Agent': 'dronecan_gui_tool'})
            with urlopen(req, timeout=60) as resp:
                blob = resp.read()
            zf = zipfile.ZipFile(io.BytesIO(blob))
            names = zf.namelist()
            if not names:
                raise RuntimeError('Archive is empty')
            # Archive root is e.g. 'public_regulated_data_types-<sha>/'
            root = names[0].split('/', 1)[0] + '/'

            staging_dir = tempfile.mkdtemp(prefix='dronecan-dsdl-update-')

            # Extract and validate before touching the installed DSDL tree.
            extracted = 0
            for n in names:
                if not n.startswith(root):
                    continue
                rel = n[len(root):]
                if not rel:
                    continue
                top = rel.split('/', 1)[0]
                if top in DSDL_SYNC_EXCLUDES:
                    continue
                dest = os.path.join(staging_dir, rel)
                if n.endswith('/'):
                    os.makedirs(dest, exist_ok=True)
                    continue
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                with zf.open(n) as src, open(dest, 'wb') as out:
                    shutil.copyfileobj(src, out)
                extracted += 1

            self._validate_dsdl_dir(staging_dir)

            # Remove existing top-level entries (other than our marker / excluded
            # files), so stale folders disappear and renames are handled.
            os.makedirs(target, exist_ok=True)
            for entry in os.listdir(target):
                if entry in DSDL_SYNC_EXCLUDES:
                    continue
                full = os.path.join(target, entry)
                try:
                    if os.path.isdir(full):
                        shutil.rmtree(full)
                    else:
                        os.remove(full)
                except OSError:
                    logger.warning('Failed to remove %s', full, exc_info=True)

            for entry in os.listdir(staging_dir):
                src = os.path.join(staging_dir, entry)
                dest = os.path.join(target, entry)
                if os.path.isdir(src):
                    shutil.copytree(src, dest)
                else:
                    shutil.copy2(src, dest)
            logger.info('DSDL update: %d files written to %s', extracted, target)
            self._write_local_dsdl_sha(remote_sha)
            self._write_dsdl_manifest()
        except PermissionError as ex:
            logger.error('DSDL update failed (permissions)', exc_info=True)
            QMessageBox.critical(
                self, 'DSDL Update Failed',
                'Permission denied while writing to:\n{}\n\n'
                'Try running the application as Administrator, or '
                'check folder permissions.\n\nDetails: {}'.format(target, ex))
            return
        except Exception as ex:
            logger.error('DSDL update failed', exc_info=True)
            QMessageBox.critical(self, 'DSDL Update Failed',
                                 'Failed to update DSDL files:\n{}'.format(ex))
            return
        finally:
            if staging_dir:
                shutil.rmtree(staging_dir, ignore_errors=True)

        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle('DSDL Updated')
        msg.setText(
            'DSDL files updated successfully to commit {}.\n\n'
            'The application must restart for the changes to take effect.'
            .format(remote_sha[:7]))
        restart_btn = msg.addButton('Restart Now', QMessageBox.AcceptRole)
        msg.addButton('Later', QMessageBox.RejectRole)
        msg.setDefaultButton(restart_btn)
        msg.exec_()
        if msg.clickedButton() is restart_btn:
            self._restart_application()

    def _restart_application(self):
        import subprocess
        try:
            # sys.argv[0] is the launcher script in dev, or the frozen exe path
            # when packaged with cx_Freeze. sys.executable points at the
            # interpreter (dev) or the same frozen exe (frozen).
            if getattr(sys, 'frozen', False):
                args = [sys.executable] + sys.argv[1:]
            else:
                args = [sys.executable] + sys.argv
            DETACHED_PROCESS = 0x00000008
            CREATE_NEW_PROCESS_GROUP = 0x00000200
            subprocess.Popen(args, close_fds=True,
                             creationflags=DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP)
        except Exception as ex:
            logger.warning('Failed to restart application', exc_info=True)
            QMessageBox.critical(self, 'Restart Failed',
                                 'Failed to restart the application:\n{}'.format(ex))
            return
        logger.info('Restart requested, closing current instance')
        QApplication.quit()

    def _launch_installer_and_quit(self, installer_path):
        import subprocess
        try:
            # Launch MSI via msiexec, fully detached so it survives our exit.
            # The Windows Installer will handle uninstalling the previous version
            # (provided the MSI was authored with a matching UpgradeCode).
            DETACHED_PROCESS = 0x00000008
            CREATE_NEW_PROCESS_GROUP = 0x00000200
            subprocess.Popen(
                ['msiexec', '/i', installer_path],
                close_fds=True,
                creationflags=DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP)
        except Exception as ex:
            logger.warning('Failed to launch installer', exc_info=True)
            QMessageBox.critical(self, 'Check for Updates',
                                 'Failed to launch the installer:\n{}'.format(ex))
            return
        logger.info('Installer launched, closing application: %s', installer_path)
        QApplication.quit()

    def _make_console_context(self):
        default_transfer_priority = 30

        active_handles = []

        def print_yaml(obj):
            """
            Formats the argument as YAML structure using dronecan.to_yaml(), and prints the result into stdout.
            Use this function to print received DroneCAN structures.
            """
            if obj is None:
                return

            print(dronecan.to_yaml(obj))

        def throw_if_anonymous():
            if self._node.is_anonymous:
                raise RuntimeError('Local node is configured in anonymous mode. '
                                   'You need to set the local node ID (see the main window) in order to be able '
                                   'to send transfers.')

        def request(payload, server_node_id, callback=None, priority=None, timeout=None):
            """
            Sends a service request to the specified node. This is a convenient wrapper over node.request().
            Args:
                payload:        Request payload of type CompoundValue, e.g. dronecan.uavcan.protocol.GetNodeInfo.Request()
                server_node_id: Node ID of the node that will receive the request.
                callback:       Response callback. Default handler will print the response to stdout in YAML format.
                priority:       Transfer priority; defaults to a very low priority.
                timeout:        Response timeout, default is set according to the DroneCAN specification.
            """
            if isinstance(payload, dronecan.dsdl.CompoundType):
                print('Interpreting the first argument as:', payload.full_name + '.Request()')
                payload = dronecan.TYPENAMES[payload.full_name].Request()
            throw_if_anonymous()
            priority = priority or default_transfer_priority
            callback = callback or print_yaml
            return self._node.request(payload, server_node_id, callback, priority=priority, timeout=timeout)

        def serve(dronecan_type, callback):
            """
            Registers a service server. The callback will be invoked every time the local node receives a
            service request of the specified type. The callback accepts an dronecan.Event object
            (refer to the PyDroneCAN documentation for more info), and returns the response object.
            Example:
                >>> def serve_acs(e):
                >>>     print_yaml(e.request)
                >>>     return dronecan.uavcan.protocol.AccessCommandShell.Response()
                >>> serve(dronecan.uavcan.protocol.AccessCommandShell, serve_acs)
            Args:
                dronecan_type:    DroneCAN service type to serve requests of.
                callback:       Service callback with the business logic, see above.
            """
            if dronecan_type.kind != dronecan_type.KIND_SERVICE:
                raise RuntimeError('Expected a service type, got a different kind')

            def process_callback(e):
                try:
                    return callback(e)
                except Exception:
                    logger.error('Unhandled exception in server callback for %r, server terminated',
                                 dronecan_type, exc_info=True)
                    sub_handle.remove()

            sub_handle = self._node.add_handler(dronecan_type, process_callback)
            active_handles.append(sub_handle)
            return sub_handle

        def broadcast(payload, priority=None, interval=None, count=None, duration=None):
            """
            Broadcasts messages, either once or periodically in the background.
            Periodic broadcasting can be configured with one or multiple termination conditions; see the arguments for
            more info. Multiple termination conditions will be joined with logical OR operation.
            Example:
                # Send one message:
                >>> broadcast(dronecan.uavcan.protocol.debug.KeyValue(key='key', value=123))
                # Repeat message every 100 milliseconds for 10 seconds:
                >>> broadcast(dronecan.uavcan.protocol.NodeStatus(), interval=0.1, duration=10)
                # Send 100 messages with 10 millisecond interval:
                >>> broadcast(dronecan.uavcan.protocol.Panic(reason_text='42!'), interval=0.01, count=100)
            Args:
                payload:    DroneCAN message structure, e.g. dronecan.uavcan.protocol.debug.KeyValue(key='key', value=123)
                priority:   Transfer priority; defaults to a very low priority.
                interval:   Broadcasting interval in seconds.
                            If specified, the message will be re-published in the background with this interval.
                            If not specified (which is default), the message will be published only once.
                count:      Stop background broadcasting when this number of messages has been broadcasted.
                            By default it is not set, meaning that the periodic broadcasting will continue indefinitely,
                            unless other termination conditions are configured.
                            Setting this value without interval is not allowed.
                duration:   Stop background broadcasting after this amount of time, in seconds.
                            By default it is not set, meaning that the periodic broadcasting will continue indefinitely,
                            unless other termination conditions are configured.
                            Setting this value without interval is not allowed.
            Returns:    If periodic broadcasting is configured, this function returns a handle that implements a method
                        'remove()', which can be called to stop the background job.
                        If no periodic broadcasting is configured, this function returns nothing.
            """
            # Validating inputs
            if isinstance(payload, dronecan.dsdl.CompoundType):
                print('Interpreting the first argument as:', payload.full_name + '()')
                payload = dronecan.TYPENAMES[payload.full_name]()

            if (interval is None) and (duration is not None or count is not None):
                raise RuntimeError('Cannot setup background broadcaster: interval is not set')

            throw_if_anonymous()

            # Business end is here
            def do_broadcast():
                self._node.broadcast(payload, priority or default_transfer_priority)

            do_broadcast()

            if interval is not None:
                num_broadcasted = 1         # The first was broadcasted before the job was launched
                if duration is None:
                    duration = 3600 * 24 * 365 * 1000       # See you in 1000 years
                deadline = time.monotonic() + duration

                def process_next():
                    nonlocal num_broadcasted
                    try:
                        do_broadcast()
                    except Exception:
                        logger.error('Automatic broadcast failed, job cancelled', exc_info=True)
                        timer_handle.remove()
                    else:
                        num_broadcasted += 1
                        if (count is not None and num_broadcasted >= count) or (time.monotonic() >= deadline):
                            logger.info('Background publisher for %r has stopped',
                                        dronecan.get_dronecan_data_type(payload).full_name)
                            timer_handle.remove()

                timer_handle = self._node.periodic(interval, process_next)
                active_handles.append(timer_handle)
                return timer_handle

        def subscribe(dronecan_type, callback=None, count=None, duration=None, on_end=None):
            """
            Receives specified DroneCAN messages from the bus and delivers them to the callback.
            Args:
                dronecan_type:    DroneCAN message type to listen for.
                callback:       Callback will be invoked for every received message.
                                Default callback will print the response to stdout in YAML format.
                count:          Number of messages to receive before terminating the subscription.
                                Unlimited by default.
                duration:       Amount of time, in seconds, to listen for messages before terminating the subscription.
                                Unlimited by default.
                on_end:         Callable that will be invoked when the subscription is terminated.
            Returns:    Handler with method .remove(). Calling this method will terminate the subscription.
            """
            if (count is None and duration is None) and on_end is not None:
                raise RuntimeError('on_end is set, but it will never be called because the subscription has '
                                   'no termination condition')

            if dronecan_type.kind != dronecan_type.KIND_MESSAGE:
                raise RuntimeError('Expected a message type, got a different kind')

            callback = callback or print_yaml

            def process_callback(e):
                nonlocal count
                stop_now = False
                try:
                    callback(e)
                except Exception:
                    logger.error('Unhandled exception in subscription callback for %r, subscription terminated',
                                 dronecan_type, exc_info=True)
                    stop_now = True
                else:
                    if count is not None:
                        count -= 1
                        if count <= 0:
                            stop_now = True
                if stop_now:
                    sub_handle.remove()
                    try:
                        timer_handle.remove()
                    except Exception:
                        pass
                    if on_end is not None:
                        on_end()

            def cancel_callback():
                try:
                    sub_handle.remove()
                except Exception:
                    pass
                else:
                    if on_end is not None:
                        on_end()

            sub_handle = self._node.add_handler(dronecan_type, process_callback)
            timer_handle = None
            if duration is not None:
                timer_handle = self._node.defer(duration, cancel_callback)
            active_handles.append(sub_handle)
            return sub_handle

        def periodic(period_sec, callback):
            """
            Calls the specified callback with the specified time interval.
            """
            handle = self._node.periodic(period_sec, callback)
            active_handles.append(handle)
            return handle

        def defer(delay_sec, callback):
            """
            Calls the specified callback after the specified amount of time.
            """
            handle = self._node.defer(delay_sec, callback)
            active_handles.append(handle)
            return handle

        def stop():
            """
            Stops all periodic broadcasts (see broadcast()), terminates all subscriptions (see subscribe()),
            and cancels all deferred and periodic calls (see defer(), periodic()).
            """
            for h in active_handles:
                try:
                    logger.debug('Removing handle %r', h)
                    h.remove()
                except Exception:
                    pass
            active_handles.clear()

        def can_send(can_id, data, extended=False):
            """
            Args:
                can_id:     CAN ID of the frame
                data:       Payload as bytes()
                extended:   True to send a 29-bit frame; False to send an 11-bit frame
            """
            self._node.can_driver.send(can_id, data, extended=extended)

        return [
            InternalObjectDescriptor('can_iface_name', self._iface_name,
                                     'Name of the CAN bus interface'),
            InternalObjectDescriptor('node', self._node,
                                     'DroneCAN node instance'),
            InternalObjectDescriptor('node_monitor', self._node_monitor_widget.monitor,
                                     'Object that stores information about nodes currently available on the bus'),
            InternalObjectDescriptor('request', request,
                                     'Sends DroneCAN request transfers to other nodes'),
            InternalObjectDescriptor('serve', serve,
                                     'Serves DroneCAN service requests'),
            InternalObjectDescriptor('broadcast', broadcast,
                                     'Broadcasts DroneCAN messages, once or periodically'),
            InternalObjectDescriptor('subscribe', subscribe,
                                     'Receives DroneCAN messages'),
            InternalObjectDescriptor('periodic', periodic,
                                     'Invokes a callback from the node thread with the specified time interval'),
            InternalObjectDescriptor('defer', defer,
                                     'Invokes a callback from the node thread once after the specified timeout'),
            InternalObjectDescriptor('stop', stop,
                                     'Stops all ongoing tasks of broadcast(), subscribe(), defer(), periodic()'),
            InternalObjectDescriptor('print_yaml', print_yaml,
                                     'Prints DroneCAN entities in YAML format'),
            InternalObjectDescriptor('dronecan', dronecan,
                                     'The main Pydronecan module'),
            InternalObjectDescriptor('main_window', self,
                                     'Main window object, holds references to all business logic objects'),
            InternalObjectDescriptor('can_send', can_send,
                                     'Sends a raw CAN frame'),
        ]

    def _show_console_window(self):
        try:
            self._console_manager.show_console_window(self)
        except Exception as ex:
            logger.error('Could not spawn console', exc_info=True)
            show_error('Console error', 'Could not spawn console window', ex, self)
            return

    def _show_node_window(self, node_id):
        if node_id in self._node_windows:
            # noinspection PyBroadException
            try:
                self._node_windows[node_id].close()
                self._node_windows[node_id].setParent(None)
                self._node_windows[node_id].deleteLater()
            except Exception:
                pass    # Sometimes fails with "wrapped C/C++ object of type NodePropertiesWindow has been deleted"
            del self._node_windows[node_id]

        w = NodePropertiesWindow(self, self._node, node_id, self._file_server_widget,
                                 self._node_monitor_widget.monitor, self._dynamic_node_id_allocation_widget)
        w.show()
        self._node_windows[node_id] = w

    def _spin_node(self):
        # We're running the node in the GUI thread.
        # This is not great, but at the moment seems like other options are even worse.
        try:
            self._node.spin(0)
            self._successive_node_errors = 0
        except Exception as ex:
            self._successive_node_errors += 1

            msg = 'Node spin error [%d of %d]: %r' % (self._successive_node_errors, self.MAX_SUCCESSIVE_NODE_ERRORS, ex)

            if self._successive_node_errors >= self.MAX_SUCCESSIVE_NODE_ERRORS:
                show_error('Node failure',
                           'Local DroneCAN node has generated too many errors and will be terminated.\n'
                           'Please restart the application.',
                           msg, self)
                self._node_spin_timer.stop()
                self._node.close()

            logger.error(msg, exc_info=True)
            self.statusBar().showMessage(msg, 3000)

    def closeEvent(self, qcloseevent):
        self._plotter_manager.close()
        self._console_manager.close()
        self._active_data_type_detector.close()
        super(MainWindow, self).closeEvent(qcloseevent)

def main():
    logger.info('Starting the application')
    app = QApplication(sys.argv)

    while True:
        # Asking the user to specify which interface to work with
        try:
            if args.interface is not None:
                iface = args.interface
                iface_kwargs = {}
                iface_kwargs['baudrate'] = int(args.baudrate)
                iface_kwargs['bitrate'] = int(args.bitrate)
                iface_kwargs['bus_number'] = int(args.bus)
                iface_kwargs['filtered'] = bool(args.filtered)
                iface_kwargs['mavlink_target_system'] = int(args.target_system)
                iface_kwargs['mavlink_signing_key'] = str(args.signing_passphrase if args.signing_passphrase is not None else '')
                dsdl_directory = args.dsdl
            else:
                iface, iface_kwargs, dsdl_directory = run_setup_window(get_app_icon(), args.dsdl, args.baudrate, args.bitrate, args.bus, args.filtered, args.target_system, args.signing_passphrase)
            if not iface:
                sys.exit(0)
        except Exception as ex:
            show_error('Fatal error', 'Could not list available interfaces', ex, blocking=True)
            sys.exit(1)

        if not dsdl_directory:
            dsdl_directory = args.dsdl

        try:
            if dsdl_directory:
                logger.info('Loading custom DSDL from %r', dsdl_directory)
                dronecan.load_dsdl(dsdl_directory)
                logger.info('Custom DSDL loaded successfully')

                # setup an environment variable for sub-processes to know where to load custom DSDL from
                os.environ['DroneCAN_CUSTOM_DSDL_PATH'] = dsdl_directory
        except Exception as ex:
            logger.exception('No DSDL loaded from %r, only standard messages will be supported', dsdl_directory)
            show_error('DSDL not loaded',
                       'Could not load DSDL definitions from %r.\n'
                       'The application will continue to work without the custom DSDL definitions.' % dsdl_directory,
                       ex, blocking=True)

        # Trying to start the node on the specified interface
        try:
            node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
            node_info.name = NODE_NAME
            node_info.software_version.major = __version__[0]
            node_info.software_version.minor = __version__[1]

            node = dronecan.make_node(iface,
                                    node_info=node_info,
                                    mode=dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL,
                                    **iface_kwargs)

            if iface_kwargs["filtered"]:
                setup_filtering(node)

            # Making sure the interface is alright
            node.spin(0.1)
        except dronecan.transport.TransferError:
            # allow unrecognized messages on startup:
            logger.warning('DroneCAN Transfer Error occurred on startup', exc_info=True)
            break
        except SerialException as ex:
            logger.error('DroneCAN node init failed', exc_info=True)
            show_error('Error', 'Could not find serial port', ex, blocking=True)

            # The serial port had a fault, so reset args and the interface and return to the setup window
            args.interface = None
            iface = None
        except Exception as ex:
            logger.error('DroneCAN node init failed', exc_info=True)
            show_error('Fatal error', 'Could not initialize DroneCAN node', ex, blocking=True)
        else:
            break

    logger.info('Creating main window; iface %r', iface)
    window = MainWindow(node, iface, iface_kwargs)
    window.show()

    logger.info('Init complete, invoking the Qt event loop')
    exit_code = app.exec_()

    node.close()

    sys.exit(exit_code)
