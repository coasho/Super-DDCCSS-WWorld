"""Writing the files hangar makes, patiently.

On Windows a file another program has mapped cannot be opened for writing: the open fails "Invalid argument"
(ERROR_USER_MAPPED_FILE), or "Permission denied" while it is being scanned. Whatever watches a repository does that to
a file just written - a version-control client refreshing its status, an editor indexing, a virus scanner - for some
tens of milliseconds, and a stage that writes its aircraft's file again then stopped a run of several minutes (one
design in ten, with seven running side by side). open_write() tries again for a few seconds before it gives up.
"""
import errno
import time

PATIENCE_S = 20.0
PAUSE_S = 0.1


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
