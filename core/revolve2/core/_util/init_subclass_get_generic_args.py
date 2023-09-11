from typing import Any, Type, TypeVar, get_args, get_origin

TChild = TypeVar("TChild")
TParent = TypeVar("TParent")


def init_subclass_get_generic_args(
    child: Type[TChild], parent: Type[TParent]
) -> tuple[Any, ...]:
    """
    Get the generic arguments from a class within the __init_subclass__ function.

    :param child: The type passed to the __init_subclass__ function.
    :param parent: The type of the parent class, which is the class __init_subclass__ is implemented for.
    :returns: The types. Keep in mind these can be `ForwardRef`.
    """
    # find parent and its type annotations in the list of base classes of child
    orig_bases: list[Type[TParent]] = [orig_base for orig_base in child.__orig_bases__ if get_origin(orig_base) is parent]  # type: ignore[attr-defined]
    assert (
        len(orig_bases) == 1
    ), "Implementer thinks this should be impossible. Expected that user can only inherit from parent class once."
    orig_base = orig_bases[0]
    # get the generic types from the type annotations
    return get_args(orig_base)
