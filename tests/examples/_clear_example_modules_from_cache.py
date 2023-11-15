import sys

from ..conftest import EXAMPLES_DIR


def clear_example_modules_from_cache() -> None:
    """Clear any module from the sys.modules cache that is part of the examples directory."""
    modules_to_delete = [
        name
        for name, mod in sys.modules.items()
        if mod is not None
        and hasattr(mod, "__file__")
        and getattr(mod, "__file__") is not None
        and getattr(mod, "__file__", "").startswith(EXAMPLES_DIR)
    ]
    for mod in modules_to_delete:
        del sys.modules[mod]
