import collections
import collections.abc


def ensure_dronekit_compat() -> None:
    """Patch deprecated collections aliases required by dronekit on Python 3.10+."""
    aliases = (
        "Mapping",
        "MutableMapping",
        "Sequence",
        "MutableSequence",
        "Set",
        "MutableSet",
        "Iterable",
    )

    for alias in aliases:
        if not hasattr(collections, alias) and hasattr(collections.abc, alias):
            setattr(collections, alias, getattr(collections.abc, alias))
