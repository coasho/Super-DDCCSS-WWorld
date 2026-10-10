"""hangar's tests. Before 3.13, CPython's platform module answers what machine it is on (numpy.testing's import
asks) with a WMI query on a thread of its own that, when the query outwaits it - 1 s, which 16 test processes at once
on a small machine overrun - goes on through a pointer into the frame its caller has left, and faults at random in
no Python thread (PC/_wmimodule.cpp; 3.13 gives the thread a copy). Here platform answers from the registry and the
environment instead, as it does where WMI is not there."""
import sys

if sys.platform == "win32" and sys.version_info < (3, 13):
    import platform

    platform._wmi = None
