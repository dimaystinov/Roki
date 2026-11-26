"""Process-local monotonic IDs for internal work, never UUID strings."""
from itertools import count

_ids = count(1)


def next_id():
    return next(_ids)
