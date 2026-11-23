"""No-op stand-in for the Unix-only fcntl stdlib module (imported transitively on Windows)."""
F_GETFL = 3
F_SETFL = 4
F_GETLK = 5
F_SETLK = 6
F_SETLKW = 7
LOCK_SH = 1
LOCK_EX = 2
LOCK_NB = 4
LOCK_UN = 8


def fcntl(*a, **k):
    return 0


def ioctl(*a, **k):
    return 0


def flock(*a, **k):
    return None


def lockf(*a, **k):
    return None
