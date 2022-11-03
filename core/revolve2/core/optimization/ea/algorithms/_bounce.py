from revolve2.core.optimization.ea.population import Parameters


def bounce(val: float) -> float:
    """
    Bounce a value between 0 and 1, inclusive.

    :param val: The value to bounce.
    :returns: The bounced result.
    """
    val = abs(val) % 2
    if val > 1:
        return 2 - val
    else:
        return val


def bounce_parameters(params: Parameters) -> Parameters:
    """
    Bounce all parameters according to 'bounce'.

    :param params: Parameters to bounce.
    :returns: Bounced parameters.
    """
    return Parameters([bounce(p) for p in params])
