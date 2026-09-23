#
# Copyright (C) 2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import dronecan
from dronecan import uavcan
import os
import json
import zlib
import base64
import struct
import threading
from time import monotonic
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QWidget, QDirModel, QCompleter, QFileDialog, QLabel
from PyQt5.QtCore import QTimer, QObject
from logging import getLogger
from . import make_icon_button, CommitableComboBoxWithHistory, get_icon, flash, LabelWithIcon


logger = getLogger(__name__)


READ_ACTIVITY_WINDOW_SEC = 0.250
READ_CACHE_REVALIDATE_INTERVAL_SEC = 1.000

def FileServer_PathKey(path):
    '''
    return key used in file read request for a path. This is kept to 7 bytes
    to keep the read request in 2 frames
    '''
    ret = base64.b64encode(struct.pack("<I",zlib.crc32(bytearray(path,'utf-8'))))[:7].decode('utf-8')
    # avoid path separators
    ret = ret.replace('/','_').replace('\\','_')
    return ret

class PathItem(QWidget):
    def __init__(self, parent, default=None):
        super(PathItem, self).__init__(parent)

        self.on_remove = lambda _: None
        self.on_path_changed = lambda *_: None

        self._remove_button = make_icon_button('fa6s.xmark', 'Remove this path', self,
                                               on_clicked=lambda: self.on_remove(self))

        completer = QCompleter(self)
        completer.setModel(QDirModel(completer))

        self._path_bar = CommitableComboBoxWithHistory(self)
        if default:
            self._path_bar.setCurrentText(default)
        self._path_bar.setCompleter(completer)
        self._path_bar.setAcceptDrops(True)
        self._path_bar.setToolTip('Lookup path for file services; should point either to a file or to a directory')
        self._path_bar.currentTextChanged.connect(self._on_path_changed)

        self._select_file_button = make_icon_button('fa6.file', 'Specify file path', self,
                                                    on_clicked=self._on_select_path_file)

        self._select_dir_button = make_icon_button('fa6.folder-open', 'Specify directory path', self,
                                                   on_clicked=self._on_select_path_directory)

        self._hit_count_label = LabelWithIcon(get_icon('fa6s.upload'), '0', self)
        self._hit_count_label.setToolTip('Hit count')

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._remove_button)
        layout.addWidget(self._path_bar, 1)
        layout.addWidget(self._select_file_button)
        layout.addWidget(self._select_dir_button)
        layout.addWidget(self._hit_count_label)
        self.setLayout(layout)

    def _on_path_changed(self):
        self.reset_hit_counts()
        self.on_path_changed()

    def _on_select_path_file(self):
        path = QFileDialog().getOpenFileName(self, 'Add file path to be served by the file server',
                                             os.path.expanduser('~'))
        self._path_bar.setCurrentText(path[0])

    def _on_select_path_directory(self):
        path = QFileDialog().getExistingDirectory(self, 'Add directory lookup path for the file server',
                                                  os.path.expanduser('~'))
        self._path_bar.setCurrentText(path)

    @property
    def path(self):
        p = self._path_bar.currentText()
        return os.path.normcase(os.path.abspath(os.path.expanduser(p))) if p else None

    def update_hit_count(self, _path, hit_count):
        self._hit_count_label.setText(str(hit_count))

    def reset_hit_counts(self):
        self._hit_count_label.setText('0')

def hex2bin(heximage):
    '''
    Convert an Intel HEX format bytes array to binary format
    '''
    import intelhex
    import io

    hex_stream = io.StringIO(heximage.decode('utf-8'))
    bin_stream = io.BytesIO()
    intelhex.hex2bin(hex_stream, bin_stream)
    bin_stream.seek(0)
    return bin_stream.read()

class FileServerJson(dronecan.app.file_server.FileServer):
    def __init__(self, node):
        super(FileServerJson, self).__init__(node)
        self._node = node
        self._images = {}
        self._image_views = {}
        self._image_timestamps = {}
        self._image_last_checked_at = {}
        self._key_to_path = {}
        self._key_hit_counters = {}
        self._key_complete = set()
        self._key_max_offset = {}
        self._cache_lock = threading.RLock()

    def _is_firmware_update_mode(self):
        return bool(getattr(self._node, '_firmware_update_mode', False))

    def _is_active_firmware_read_window(self):
        active_until = float(getattr(self._node, '_firmware_read_active_until', 0.0) or 0.0)
        return active_until > monotonic()

    def _resolve_path(self, relative):
        rel = relative.path.decode().replace(chr(relative.SEPARATOR), os.path.sep)
        if rel in self._key_to_path:
            return self._key_to_path[rel]
        return super(FileServerJson, self)._resolve_path(relative)

    def _load_image(self, path):
        if path.lower().endswith('.apj') or path.lower().endswith('.px4'):
            # load JSON image
            j = json.load(open(path,'r'))
            if not 'image' in j:
                print("Missing image in %s" % path)
                return None
            return bytearray(zlib.decompress(base64.b64decode(j['image'].encode('utf-8'))))
        if path.lower().endswith('.amj'):
            # load JSON image as hex image
            j = json.load(open(path,'r'))
            if not 'hex' in j:
                print("Missing hex image in %s" % path)
                return None
            return hex2bin(base64.b64decode(j['hex']))
        if path.lower().endswith('.hex'):
            # intel hex image
            h = open(path,'rb').read()
            return hex2bin(h)
        return open(path,'rb').read()

    def _check_path_change(self, path):
        with self._cache_lock:
            key = FileServer_PathKey(path)
            mtime = os.path.getmtime(path)
            if path not in self._images or key not in self._key_to_path or mtime != self._image_timestamps.get(path):
                self._image_timestamps[path] = mtime
                self._images[path] = self._load_image(path)
                self._image_views[path] = memoryview(self._images[path])
                self._key_to_path[key] = path
                # transfer progress of the previous image must not leak into the new one
                self._key_complete.discard(key)
                self._key_max_offset.pop(key, None)

    def purge_path(self, path):
        """Remove all cached data for a path so file.Read requests for it will fail."""
        with self._cache_lock:
            key = FileServer_PathKey(path)
            self._key_to_path.pop(key, None)
            self._images.pop(path, None)
            self._image_views.pop(path, None)
            self._image_timestamps.pop(path, None)
            self._key_complete.discard(key)
            self._key_max_offset.pop(key, None)
            self._key_hit_counters.pop(key, None)

    def set_lookup_paths(self, paths):
        with self._cache_lock:
            self.lookup_paths = list(paths)
            for cached_path in list(self._images.keys()):
                if cached_path not in self.lookup_paths:
                    self.purge_path(cached_path)

    @property
    def key_hit_counters(self):
        with self._cache_lock:
            return dict(self._key_hit_counters)

    def is_key_complete(self, key):
        with self._cache_lock:
            return key in self._key_complete

    def get_key_progress(self, key):
        with self._cache_lock:
            if key not in self._key_to_path:
                return (0, 0)
            path = self._key_to_path[key]
            total = len(self._images.get(path, b''))
            sent = min(self._key_max_offset.get(key, 0), total)
            return (sent, total)

    def _read(self, e):
        with self._cache_lock:
            if not self._is_firmware_update_mode():
                logger.debug("[#{0:03d}:uavcan.protocol.file.Read] {1!r} @ offset {2:d}"
                             .format(e.transfer.source_node_id, e.request.path.path.decode(), e.request.offset))
            if self._is_firmware_update_mode():
                setattr(self._node, '_firmware_read_active_until', monotonic() + READ_ACTIVITY_WINDOW_SEC)
            key = e.request.path.path.decode()
            try:
                if key in self._key_to_path:
                    path = self._key_to_path[key]
                    self._key_hit_counters[key] = self._key_hit_counters.get(key, 0) + 1
                else:
                    path = self._resolve_path(e.request.path)

                self._check_path_change(path)

                resp = uavcan.protocol.file.Read.Response()
                read_size = dronecan.get_dronecan_data_type(dronecan.get_fields(resp)['data']).max_size

                image_view = self._image_views[path]
                end_offset = min(e.request.offset + read_size, len(image_view))
                payload_size = max(0, end_offset - e.request.offset)
                resp.data = image_view[e.request.offset:end_offset]
                resp.error.value = resp.error.OK

                if key in self._key_to_path:
                    end_offset = e.request.offset + payload_size
                    prev = self._key_max_offset.get(key, 0)
                    if end_offset > prev:
                        self._key_max_offset[key] = end_offset
                    if payload_size < read_size:
                        self._key_complete.add(key)
            except Exception:
                logger.exception("[#{0:03d}:uavcan.protocol.file.Read] error")
                resp = uavcan.protocol.file.Read.Response()
                resp.error.value = resp.error.UNKNOWN_ERROR

            return resp


class FileServerController(QObject):
    def __init__(self, node, parent=None):
        super(FileServerController, self).__init__(parent)
        self._node = node
        self._file_server = None
        self._paths = []

    @property
    def node(self):
        return self._node

    @property
    def file_server(self):
        return self._file_server

    @property
    def is_running(self):
        return self._file_server is not None

    @property
    def path_hit_counters(self):
        return {} if self._file_server is None else self._file_server.path_hit_counters

    def set_paths(self, paths):
        self._paths = [os.path.normcase(os.path.abspath(os.path.expanduser(path))) for path in paths if path]

        if self._file_server:
            logger.info('Updating lookup paths: %r', self._paths)
            self._file_server.set_lookup_paths(self._paths)

    def add_path(self, path):
        path = os.path.normcase(os.path.abspath(os.path.expanduser(path)))
        if path not in self._paths:
            self.set_paths(self._paths + [path])

    def remove_path(self, path):
        path = os.path.normcase(os.path.abspath(os.path.expanduser(path)))
        if path in self._paths:
            self.set_paths([existing for existing in self._paths if existing != path])

    def force_start(self):
        if not self._file_server:
            self._file_server = FileServerJson(self._node)
            self.set_paths(self._paths)

    def stop(self):
        if self._file_server:
            try:
                self._file_server.close()
            except Exception:
                logger.error('Could not stop file server', exc_info=True)
            self._file_server = None
            logger.info('File server stopped')

    def close(self):
        self.stop()

    def is_key_complete(self, key):
        return self._file_server.is_key_complete(key) if self._file_server else False

    def get_key_progress(self, key):
        return self._file_server.get_key_progress(key) if self._file_server else (0, 0)


class FileServerWidget(QGroupBox):
    def __init__(self, parent, node):
        super(FileServerWidget, self).__init__(parent)
        self.setTitle('File server (dronecan.uavcan.protocol.file.*)')

        if isinstance(node, FileServerController):
            self._controller = node
            self._owns_controller = False
        else:
            self._controller = FileServerController(node, self)
            self._owns_controller = True

        self._path_widgets = []

        self._start_button = make_icon_button('fa6s.rocket', 'Launch/stop the file server', self,
                                              checkable=True,
                                              on_clicked=self._on_start_stop)
        self._start_button.setEnabled(False)

        self._tmr = QTimer(self)
        self._tmr.setSingleShot(False)
        self._tmr.timeout.connect(self._update_on_timer)
        self._tmr.start(500)

        self._add_path_button = \
            make_icon_button('fa6s.plus', 'Add lookup path (lookup paths can be modified while the server is running)',
                             self, on_clicked=self._on_add_path)

        layout = QVBoxLayout(self)

        controls_layout = QHBoxLayout(self)
        controls_layout.addWidget(self._start_button)
        controls_layout.addWidget(self._add_path_button)
        controls_layout.addStretch(1)

        layout.addLayout(controls_layout)
        self.setLayout(layout)

    def _update_on_timer(self):
        self._start_button.setEnabled(not self._controller.node.is_anonymous)
        self._start_button.setChecked(self._controller.is_running)
        if self._controller.is_running:
            for path, count in self._controller.path_hit_counters.items():
                for w in self._path_widgets:
                    if w.path and path.startswith(w.path):
                        w.update_hit_count(path, count)
        else:
            for w in self._path_widgets:
                w.reset_hit_counts()

    def _get_paths(self):
        return [x.path for x in self._path_widgets if x.path]

    def _sync_paths(self):
        paths = self._get_paths()
        self._controller.set_paths(paths)
        if self._controller.is_running:
            flash(self, 'File server lookup paths: %r', paths, duration=3)

    def _on_start_stop(self):
        if self._controller.is_running:
            self._controller.stop()
        else:
            self._controller.force_start()
        self._sync_paths()

    def _on_remove_path(self, path):
        orig_len = len(self._path_widgets)
        self._path_widgets.remove(path)
        assert orig_len - 1 == len(self._path_widgets)

        self.layout().removeWidget(path)
        path.setParent(None)
        path.deleteLater()

        self._sync_paths()

    def _on_add_path(self, default=None):
        new = PathItem(self, default)
        new.on_path_changed = self._sync_paths
        new.on_remove = self._on_remove_path

        self._path_widgets.append(new)
        self.layout().addWidget(new)

        self._sync_paths()

    def add_path(self, path):
        path = os.path.normcase(os.path.abspath(os.path.expanduser(path)))

        for it in self._path_widgets:
            if it.path == path:
                self._sync_paths()      # Already listed; make sure it is (re)loaded and served
                return

        self._on_add_path(path)

    def serve_path(self, path):
        """Add the path, make sure the server runs, and load it. Raises if the file cannot be served."""
        path = os.path.normcase(os.path.abspath(os.path.expanduser(path)))
        self.add_path(path)
        self.force_start()
        self._file_server._check_path_change(path)

    def force_start(self):
        if not self._controller.is_running:
            self._on_start_stop()

    def remove_path(self, path):
        path = os.path.normcase(os.path.abspath(os.path.expanduser(path)))

        for it in list(self._path_widgets):
            if it.path == path:
                self._on_remove_path(it)
                return

    @property
    def _file_server(self):
        return self._controller.file_server

