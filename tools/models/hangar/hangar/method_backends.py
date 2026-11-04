"""Byte closure of the actual imported normal-production method backends.

No environment-wide discovery, installs or model writes. Timings belong
in the caller's separate operational receipt, never this deterministic record.
"""
import ctypes
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import stat
import sys

SCHEMA = 'fsim.hangar.method-backends.v1'
MAX_FILES = 50000
MAX_ENTRIES = 100000
MAX_TOTAL_BYTES = 2 * 1024**3
MAX_FILE_BYTES = 512 * 1024**2
MAX_DEPTH = 32
_PRUNED = frozenset(('.codex', '.git', '__pycache__'))
_EPHEMERAL = frozenset(('.pyc', '.pyo'))
_NATIVE_NAMES = (
    'python3.dll', 'vcruntime140.dll', 'vcruntime140_1.dll', 'msvcp140.dll',
    'msvcp140_1.dll', 'msvcp140_2.dll', 'concrt140.dll', 'vcomp140.dll',
    'libstdc++-6.dll', 'libgcc_s_seh-1.dll', 'libwinpthread-1.dll', 'libgomp-1.dll',
    'libgfortran-5.dll', 'libquadmath-0.dll',
    'libopenblas.dll', 'libblas.dll', 'liblapack.dll', 'tbb12.dll', 'tbbmalloc.dll',
    'libtbb12.dll', 'zlib1.dll', 'libpng16-16.dll', 'jpeg62.dll', 'libtiff-6.dll',
    'libwebp-7.dll', 'ucrtbase.dll', 'kernel32.dll', 'ntdll.dll',
)


class BackendClosureError(RuntimeError):
    """The current actual method cannot be completely admitted at this seam."""


def _path(value):
    path = Path(value)
    if any(part.casefold() in ('.codex', '.git') or part == '..' for part in path.parts):
        raise BackendClosureError('Excluded or traversing backend path')
    return Path(os.path.abspath(path))


def _lstat(path, optional=False):
    """Reject links/reparse points in every ancestor before following a path."""
    chain = list(reversed(path.parents)) + [path]
    result = None
    for item in chain:
        try:
            result = os.lstat(item)
        except FileNotFoundError:
            if optional and item == path:
                return None
            raise BackendClosureError('Backend path is absent: ' + str(item)) from None
        if stat.S_ISLNK(result.st_mode) or getattr(result, 'st_file_attributes', 0) & 0x400:
            raise BackendClosureError('Linked or reparse backend input: ' + str(item))
    return result


def _distribution_metadata(name, container):
    """Optional metadata must belong to the actual imported package container.

    RECORD identifies metadata only. Every payload byte is independently read.
    An absent dist-info is explicit, never permission to invent a module origin.
    """
    canonical = lambda value: re.sub(r'[-_.]+', '-', value).casefold()
    matches = []
    # Global distribution(name) may find an unrelated site-packages record
    # while the actual imported overlay intentionally has no dist-info.
    for index, candidate in enumerate(importlib.metadata.distributions(path=[str(container)])):
        if index >= MAX_ENTRIES:
            raise BackendClosureError('Backend metadata discovery exceeds its entry bound')
        location = getattr(candidate, '_path', None)
        if isinstance(location, (str, os.PathLike)) and any(part.casefold() in ('.codex', '.git') for part in Path(location).parts):
            continue  # Prune before reading metadata or resolving its path.
        if canonical(candidate.metadata.get('Name', '')) == canonical(name):
            matches.append(candidate)
    if not matches:
        return None, []
    if len(matches) != 1:
        raise BackendClosureError('Duplicate actual-container distribution metadata: ' + name)
    distribution = matches[0]
    base = _path(distribution.locate_file(''))
    if os.path.normcase(str(base)) != os.path.normcase(str(container)):
        raise BackendClosureError('Distribution metadata redirects imported backend: ' + name)
    metadata = set()
    actual_metadata = getattr(distribution, '_path', None)
    if isinstance(actual_metadata, (str, os.PathLike)):
        actual_metadata = _path(actual_metadata)
        if os.path.normcase(str(actual_metadata.parent)) != os.path.normcase(str(container)) or not actual_metadata.name.casefold().endswith(('.dist-info', '.egg-info')):
            raise BackendClosureError('Distribution metadata has an unsupported actual origin: ' + name)
        metadata.add(actual_metadata)
    for entry in distribution.files or ():
        parts = Path(entry).parts
        if not parts or any(part.casefold() in ('.codex', '.git') for part in parts):
            continue
        if parts[0].casefold().endswith(('.dist-info', '.egg-info')):
            metadata.add(_path(distribution.locate_file(parts[0])))
    if not metadata:
        raise BackendClosureError('Present distribution metadata has no byte-bound filesystem root: ' + name)
    return str(distribution.version), sorted(metadata, key=str)


def _backend_roots():
    """Actual-origin provider, patchable without importing native test inputs."""
    if os.name != 'nt':
        raise BackendClosureError('Actual production backend closure requires Windows loaded-module binding')
    # Importing plotting code can initialize its cache. The caller owns these
    # directories; refuse before importing rather than create profile caches.
    for variable in ('MPLCONFIGDIR', 'XDG_CACHE_HOME'):
        configured = os.environ.get(variable)
        if not configured or not _path(configured).is_relative_to(_path('D:/ENV')):
            raise BackendClosureError(variable + ' must select a caller-owned directory under D:/ENV')
    origins, versions, roots = {}, {}, []
    for name, distribution_name in (('numpy', 'numpy'), ('scipy', 'scipy'),
            ('PIL', 'Pillow'), ('manifold3d', 'manifold3d'), ('matplotlib', 'matplotlib'),
            ('contourpy', 'contourpy'), ('kiwisolver', 'kiwisolver'), ('fontTools', 'fonttools'),
            ('packaging', 'packaging'), ('pyparsing', 'pyparsing'), ('cycler', 'cycler'),
            ('dateutil', 'python-dateutil'), ('six', 'six')):
        module = importlib.import_module(name)
        origin = getattr(module, '__file__', None)
        if not origin:
            raise BackendClosureError('Imported backend has no actual file: ' + name)
        origin = _path(origin)
        is_package = bool(getattr(module, '__path__', None))
        root = origin.parent if is_package else origin
        container = root.parent
        metadata_version, metadata = _distribution_metadata(distribution_name, container)
        version = getattr(module, '__version__', None)
        versions[name] = dict(value=str(version) if version is not None else metadata_version,
                              basis='imported-module' if version is not None else
                              'actual-origin-matched-metadata' if metadata_version else 'unavailable')
        origins[name] = str(origin)
        roots.append(dict(role='package:' + name, path=str(root), kind='tree' if is_package else 'file', required=True))
        for extra in metadata:
            roots.append(dict(role='metadata:' + name + ':' + extra.name, path=str(extra), kind='tree', required=True))
        # Exact adjacent namespaces only; never search the environment prefix.
        adjacent_names = sorted(set((name.lower(), distribution_name.lower().replace('-', '_'))))
        for adjacent_name in adjacent_names:
            for suffix in ('.libs', '.data'):
                extra = container / (adjacent_name + suffix)
                roots.append(dict(role='adjacent:' + name + ':' + extra.name, path=str(extra), kind='tree', required=False))
    # Force the methods' numerical extensions to load before observing their
    # runtime DLL names; package trees also bind lazy package payloads.
    for name in ('numpy.linalg', 'numpy._core._multiarray_umath', 'numpy.linalg._umath_linalg',
                 'scipy.linalg', 'scipy.linalg._fblas', 'scipy.linalg._flapack',
                 'PIL.Image', 'PIL._imaging', 'matplotlib.pyplot', 'matplotlib._path',
                 'matplotlib.ft2font', 'matplotlib.backends.backend_agg', 'matplotlib.backends._backend_agg'):
        module = importlib.import_module(name)
        origin = getattr(module, '__file__', None)
        if not origin:
            raise BackendClosureError('Fixed numerical/render backend has no actual file: ' + name)
        origins[name] = str(_path(origin))
    from .shape import meshkit
    library = meshkit.library()
    if library is None:
        raise BackendClosureError('Actual normal-model meshkit backend is unavailable')
    mesh = _path(library._name)
    origins['meshkit'] = str(mesh)
    roots.append(dict(role='native:meshkit', path=str(mesh), kind='file', required=True))
    executable = _path(sys.executable)
    origins['python'] = str(executable)
    roots.append(dict(role='runtime:python-executable', path=str(executable), kind='file', required=True))
    runtime = 'python' + str(sys.version_info.major) + str(sys.version_info.minor) + '.dll'
    return dict(origins=origins, versions=versions, roots=roots,
                platform_basis=dict(system=platform.system(), release=platform.release(),
                    version=platform.version(), machine=platform.machine(), python=sys.version,
                    implementation=platform.python_implementation(), cache_tag=sys.implementation.cache_tag),
                native_names=sorted(set(_NATIVE_NAMES + (runtime,))), required_python_runtime=runtime)


def _native_dependencies(names):
    if os.name != 'nt':
        raise BackendClosureError('Windows module provenance is unavailable')
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
    kernel.GetModuleHandleW.restype = ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint]
    kernel.GetModuleFileNameW.restype = ctypes.c_uint
    kernel.GetWindowsDirectoryW.argtypes = [ctypes.c_wchar_p, ctypes.c_uint]
    kernel.GetWindowsDirectoryW.restype = ctypes.c_uint
    buffer = ctypes.create_unicode_buffer(32768)
    count = kernel.GetWindowsDirectoryW(buffer, len(buffer))
    if not count or count >= len(buffer):
        raise BackendClosureError('Windows system dependency basis is unavailable')
    windows = _path(buffer.value)
    rows = []
    for name in sorted(set(names), key=lambda value: (value.casefold(), value)):
        handle = kernel.GetModuleHandleW(name)
        if not handle:
            rows.append(dict(name=name, state='not-loaded'))
            continue
        count = kernel.GetModuleFileNameW(handle, buffer, len(buffer))
        if not count or count >= len(buffer) - 1:
            raise BackendClosureError('Loaded backend dependency path is unavailable: ' + name)
        path = _path(buffer.value)
        system = os.path.normcase(str(path)).startswith(os.path.normcase(str(windows)) + os.sep)
        rows.append(dict(name=name, state='os-system' if system else 'loaded-file', path=str(path)))
    return rows


class _Inventory:
    def __init__(self):
        self.entries = 0
        self.files = {}
        self.signatures = {}
        self.total_bytes = 0

    def file(self, path):
        key = str(path)
        before = _lstat(path)
        if not stat.S_ISREG(before.st_mode):
            raise BackendClosureError('Backend payload is not a regular file: ' + key)
        if before.st_size > MAX_FILE_BYTES:
            raise BackendClosureError('Backend file exceeds its byte bound: ' + key)
        signature = (before.st_size, before.st_mtime_ns, before.st_ino)
        if key in self.files and self.signatures[key] != signature:
            raise BackendClosureError('Backend file changed between inventory references: ' + key)
        if key not in self.files:
            if len(self.files) >= MAX_FILES or self.total_bytes + before.st_size > MAX_TOTAL_BYTES:
                raise BackendClosureError('Backend closure exceeds its file/byte bound')
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
            after = _lstat(path)
            if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
                raise BackendClosureError('Backend file changed while hashing: ' + key)
            self.files[key] = dict(size_bytes=before.st_size, sha256=digest.hexdigest())
            self.signatures[key] = signature
            self.total_bytes += before.st_size
        return self.files[key]

    def tree(self, root, directory=None, depth=0):
        directory = root if directory is None else directory
        if depth > MAX_DEPTH:
            raise BackendClosureError('Backend tree exceeds its depth bound')
        before = _lstat(directory)
        if not stat.S_ISDIR(before.st_mode):
            raise BackendClosureError('Backend tree is not a directory: ' + str(directory))
        rows = []
        with os.scandir(directory) as stream:
            # Prune by name BEFORE is_dir/stat/path resolution or descent.
            entries = []
            for entry in stream:
                if entry.name.casefold() in _PRUNED:
                    continue
                self.entries += 1
                if self.entries > MAX_ENTRIES:
                    raise BackendClosureError('Backend tree exceeds its entry bound')
                if Path(entry.name).suffix.casefold() not in _EPHEMERAL:
                    entries.append(entry)
            entries.sort(key=lambda entry: (entry.name.casefold(), entry.name))
        for entry in entries:
            path = directory / entry.name
            info = _lstat(path)
            if stat.S_ISDIR(info.st_mode):
                rows.extend(self.tree(root, path, depth + 1))
            elif stat.S_ISREG(info.st_mode):
                rows.append(dict(relative=path.relative_to(root).as_posix(), **self.file(path)))
            else:
                raise BackendClosureError('Unsupported backend payload: ' + str(path))
        after = _lstat(directory)
        if before.st_mtime_ns != after.st_mtime_ns:
            raise BackendClosureError('Backend directory changed while hashing: ' + str(directory))
        return sorted(rows, key=lambda row: row['relative'])


def snapshot():
    """Return deterministic actual-origin/file bytes, or refuse explicitly."""
    source = _backend_roots()
    inventory = _Inventory()
    roots = []
    names = set(source.get('native_names', ()))
    for descriptor in sorted(source['roots'], key=lambda item: (item['role'], item['path'])):
        path = _path(descriptor['path'])
        info = _lstat(path, optional=not descriptor['required'])
        row = dict(role=descriptor['role'], path=str(path), kind=descriptor['kind'], present=info is not None)
        if info is None:
            row['files'] = []
        elif descriptor['kind'] == 'tree':
            row['files'] = inventory.tree(path)
            if descriptor['required'] and not row['files']:
                raise BackendClosureError('Required backend package is empty: ' + str(path))
        elif descriptor['kind'] == 'file':
            row['files'] = [dict(relative='', **inventory.file(path))]
        else:
            raise BackendClosureError('Unknown backend root kind')
        for file in row['files']:
            name = Path(file['relative']).name if file['relative'] else path.name
            if name.lower().endswith('.dll'):
                names.add(name)
        roots.append(row)
    for name, origin in source['origins'].items():
        path = _path(origin)
        if str(path) not in inventory.files:
            raise BackendClosureError('Actual imported origin is outside its inventoried payload: ' + name)
    native = _native_dependencies(names)
    required_runtime = source.get('required_python_runtime')
    if required_runtime and not any(row['name'].casefold() == required_runtime.casefold()
                                    and row['state'] != 'not-loaded' for row in native):
        raise BackendClosureError('Actual Python runtime DLL is not loaded')
    for row in native:
        if row['state'] == 'loaded-file' or (row['state'] == 'os-system' and row['name'] == required_runtime):
            row['file'] = inventory.file(_path(row['path']))
    result = dict(schema=SCHEMA, platform_basis=source['platform_basis'], origins=source['origins'],
        versions=source['versions'], roots=roots, native_dependencies=native,
        system_dependency_basis='Loaded Windows-system DLL paths and OS version are recorded; OS bytes are not a portable package closure.',
        lazy_dependency_limit='Loaded selected-method dependency roster only; no claim about future plugins. Selected package payloads include their lazy files.',
        limits=dict(max_files=MAX_FILES, max_entries=MAX_ENTRIES, max_total_bytes=MAX_TOTAL_BYTES,
                    max_file_bytes=MAX_FILE_BYTES, max_depth=MAX_DEPTH),
        file_count=len(inventory.files), total_bytes=inventory.total_bytes)
    result['sha256'] = hashlib.sha256(json.dumps(result, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    return result


def verify(record):
    """Reuse only exact current bytes AND actual origins; refusal means False."""
    try:
        return isinstance(record, dict) and record == snapshot()
    except (BackendClosureError, OSError, ImportError, ValueError, TypeError):
        return False
