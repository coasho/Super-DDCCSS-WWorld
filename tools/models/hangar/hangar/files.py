"""Writing the files hangar makes, patiently.

On Windows a file another program has mapped cannot be opened for writing: the open fails "Invalid argument"
(ERROR_USER_MAPPED_FILE), or "Permission denied" while it is being scanned. Whatever watches a repository does that to
a file just written - a version-control client refreshing its status, an editor indexing, a virus scanner - for some
tens of milliseconds, and a stage that writes its aircraft's file again then stopped a run of several minutes (one
design in ten, with seven running side by side). open_write() tries again for a few seconds before it gives up.
"""
import errno
from contextlib import contextmanager
import os
from pathlib import Path
import tempfile
import time

PATIENCE_S = 20.0
PAUSE_S = 0.1


def replace(source, destination):
    """Retry bounded Windows sharing conflicts; preserve other I/O failures."""
    return _publish_closed(source, destination, _preserve_refusal=True)


def atomic_write_text(path, text):
    """Publish a complete UTF-8 file from the same directory or retain its old bytes."""
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', newline='\n',
                                         dir=path.parent, prefix=path.name+'.',
                                         suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _windows_rename_closed(source, destination):
    """Rename a closed staged file without truncating a mapped destination.

    Check ordinary DELETE sharing before the POSIX-style rename. A mapped
    reader retains its original bytes; a reader denying delete sharing refuses
    publication. No readonly/ACL-ignore flags are used.
    """
    import ctypes
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                  ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    kernel.CreateFileW.restype = wintypes.HANDLE
    kernel.SetFileInformationByHandle.argtypes = [wintypes.HANDLE, ctypes.c_int,
                                                ctypes.c_void_p, wintypes.DWORD]
    kernel.SetFileInformationByHandle.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    invalid = ctypes.c_void_p(-1).value
    gate = staged = invalid
    try:
        # OPEN_EXISTING, FILE_FLAG_OPEN_REPARSE_POINT: inspect the named entry,
        # not a symlink's target. DELETE access enforces existing sharing rules.
        gate = kernel.CreateFileW(str(destination), 0x10000, 7, None, 3, 0x00200000, None)
        if gate == invalid and ctypes.get_last_error() not in (2, 3):
            raise ctypes.WinError(ctypes.get_last_error())
        staged = kernel.CreateFileW(str(source), 0x10000, 7, None, 3, 0x00200000, None)
        if staged == invalid:
            raise ctypes.WinError(ctypes.get_last_error())
        name = os.path.abspath(destination).encode('utf-16-le')
        class RenameInfo(ctypes.Structure):
            _fields_ = [('flags', wintypes.DWORD), ('root', wintypes.HANDLE),
                        ('length', wintypes.DWORD), ('name', wintypes.WCHAR*(len(name)//2+1))]
        info = RenameInfo()
        info.flags = 3  # FILE_RENAME_REPLACE_IF_EXISTS | FILE_RENAME_POSIX_SEMANTICS
        info.length = len(name)
        ctypes.memmove(ctypes.addressof(info)+RenameInfo.name.offset, name, len(name))
        if not kernel.SetFileInformationByHandle(staged, 22, ctypes.byref(info), ctypes.sizeof(info)):
            raise ctypes.WinError(ctypes.get_last_error())
    finally:
        if staged != invalid: kernel.CloseHandle(staged)
        if gate != invalid: kernel.CloseHandle(gate)


def _publish_closed(source, destination, *, _preserve_refusal=False):
    """Bounded atomic replacement, including Windows delete-sharing mappings."""
    deadline = time.monotonic()+PATIENCE_S
    while True:
        try:
            try:
                return os.replace(source, destination)
            except OSError as refused:
                if os.name != 'nt' or getattr(refused, 'winerror', None) not in (5, 32, 33, 1224):
                    raise
                try:
                    return _windows_rename_closed(source, destination)
                except OSError as native_refused:
                    if _preserve_refusal:
                        raise refused from native_refused
                    raise
        except OSError as refused:
            if getattr(refused, 'winerror', None) not in (5, 32, 33, 1224) or time.monotonic() >= deadline:
                raise
            time.sleep(PAUSE_S)


@contextmanager
def atomic_write_stream(path, mode='wb', **kwargs):
    """Stage a whole binary/text file, close it, then publish with bounded replace.

    Failure before replacement preserves the destination. Each destination is
    one atomic publication; multiple calls are not a cross-file transaction.
    """
    if mode not in ('w', 'wb'):
        raise ValueError('Atomic publication requires whole-file w or wb mode')
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode=mode, dir=path.parent,
                                         prefix=path.name+'.', suffix='.tmp',
                                         delete=False, **kwargs) as stream:
            temporary = Path(stream.name)
            yield stream
            stream.flush()
            os.fsync(stream.fileno())
        _publish_closed(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def open_write(path, mode="w", **kwargs):
    """open(path, mode, ...) for a mode that writes; a refusal that passes (another program holding the file for a
    moment) is waited out, anything else - and a refusal that does not pass - raised as it came."""
    deadline = time.monotonic() + PATIENCE_S
    while True:
        try:
            return open(path, mode, **kwargs)
        except OSError as refused:
            if refused.errno not in (errno.EINVAL, errno.EACCES) or time.monotonic() >= deadline:
                raise
            time.sleep(PAUSE_S)
