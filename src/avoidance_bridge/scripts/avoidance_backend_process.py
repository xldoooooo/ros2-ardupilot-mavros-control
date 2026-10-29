#!/usr/bin/env python3
"""Bind the optional Linux backend's lifetime to its manager before executing the module."""

import ctypes
import os
import signal
import sys

# Linux prctl operation: deliver a signal when the creating parent thread exits.
PR_SET_PDEATHSIG = 1


def main():
    """Set the death signal outside threaded Popen, close its race, and retain the PID."""
    parent_pid = int(sys.argv[1])
    libc = ctypes.CDLL(None, use_errno=True)
    libc.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong,
                          ctypes.c_ulong, ctypes.c_ulong]
    libc.prctl.restype = ctypes.c_int
    if libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    # The manager may have died before prctl was installed; never start an orphan probe.
    if os.getppid() != parent_pid:
        raise SystemExit('FPGA manager exited before backend startup')
    os.execv(sys.argv[2], sys.argv[2:])


if __name__ == '__main__':
    main()
