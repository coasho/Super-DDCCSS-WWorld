"""Bound nested native work within a fleet model's CPU reservation."""
import os
from contextlib import contextmanager
from contextvars import ContextVar

_RESERVATION=ContextVar('hangar_native_thread_reservation',default=None)


@contextmanager
def thread_reservation(cap):
    """An explicit phase reservation; nested scopes may only reduce it."""
    if type(cap) is not int or not 1<=cap<=128:raise ValueError('Native thread reservation must be an integer from 1 to 128')
    previous=_RESERVATION.get()
    token=_RESERVATION.set(min(previous,cap) if previous is not None else cap)
    try:yield
    finally:_RESERVATION.reset(token)


def threads(requested=0):
    value=os.environ.get('FSIM_HANGAR_THREADS')
    if value is not None and (not value.isdecimal() or not 1<=int(value)<=128):
        raise ValueError('FSIM_HANGAR_THREADS must be an integer from 1 to 128')
    reservation=_RESERVATION.get()
    if reservation is not None:return min(requested,reservation) if requested>0 else reservation
    if value is None:return requested
    cap=int(value)
    return min(requested,cap) if requested>0 else cap
