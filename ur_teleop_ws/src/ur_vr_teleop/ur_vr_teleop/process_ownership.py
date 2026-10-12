"""Linux process identities and inherited ownership for the desktop launcher."""
import os
from pathlib import Path
import signal

BOOT_ID = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
SESSION_KEY = 'UR_TELEOP_MANAGED_SESSION'


def identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {'pid': int(pid), 'start': int(fields[19]), 'boot': BOOT_ID}
    except (OSError, ValueError, IndexError):
        return None


def is_same(saved):
    return bool(saved) and identity(saved['pid']) == saved


def tagged_processes(token):
    if not token:
        return []
    tag = (SESSION_KEY+'='+token).encode()
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if tag in (entry/'environ').read_bytes().split(b'\0'):
                saved = identity(int(entry.name))
                if saved:
                    found.append(saved)
        except OSError:
            pass
    return found


def send_signal(saved, sig):
    try:
        fd = os.pidfd_open(saved['pid'])
    except ProcessLookupError:
        return
    try:
        if is_same(saved):
            signal.pidfd_send_signal(fd, sig)
    except ProcessLookupError:
        pass
    finally:
        os.close(fd)


def main():
    import ctypes
    import sys
    parent = int(sys.argv[1])
    command = sys.argv[2:]
    libc = ctypes.CDLL(None, use_errno=True)
    # PR_SET_PDEATHSIG: interrupt this launch process if the desktop disappears.
    if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'Cannot set parent-death signal')
    if os.getppid() != parent:
        return
    os.execvpe(command[0], command, os.environ)


if __name__ == '__main__':
    main()
