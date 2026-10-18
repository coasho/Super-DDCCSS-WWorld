"""Three-grid Richardson/GCI arithmetic, never automatic physical admission.

Method and independent scalar example:
https://www.grc.nasa.gov/www/wind/valid/tutorial/spatconv.html

Inputs must be systematically refined, iteratively converged solutions of the
same problem. Three points fit a leading error term; that fit alone does not
prove an asymptotic regime, geometry fidelity or agreement with physical data.
The caller supplies an explicit positive scale, particularly near zero loads.
"""
import math

from aero_mesh import finite


def richardson(spacings, values, *, normalizer):
    """Fit f(h)=f(0)+C*h**p for positive p and unequal refinement ratios.

    Finest first. Oscillation, equal loads, a nonpositive observed order or
    unsupported extreme order fails explicitly rather than assuming p=2.
    """
    if len(spacings) != 3 or len(values) != 3:
        raise ValueError("Richardson estimation requires exactly three grids")
    h = [finite(v, "grid spacing") for v in spacings]
    f = [finite(v, "grid functional") for v in values]
    scale = finite(normalizer, "uncertainty normalizer")
    if not 0 < h[0] < h[1] < h[2] or scale <= 0:
        raise ValueError("grid spacings must increase and normalization must be positive")
    d12, d23 = f[1]-f[0], f[2]-f[1]
    if d12 == 0 or d23 == 0 or (d12 > 0) != (d23 > 0):
        raise ValueError("grid sequence is constant or oscillatory; no positive-order estimate")
    ratio = finite(d23/d12, "grid difference ratio")
    a, b = math.log(h[1]/h[0]), math.log(h[2]/h[1])

    def predicted(order):
        return math.expm1(order*b) / -math.expm1(-order*a)

    low, high = 1e-8, 10.0
    try:
        if not predicted(low) < ratio < predicted(high):
            raise ValueError("grid sequence has no supported positive observed order")
        for _ in range(100):
            middle = (low+high)/2
            if predicted(middle) < ratio:
                low = middle
            else:
                high = middle
    except OverflowError as error:
        raise ValueError("unsupported extreme refinement ratios") from error
    order = (low+high)/2
    correction = -d12/math.expm1(order*a)
    extrapolated = finite(f[0]+correction, "extrapolated functional")
    uncertainty = finite(1.25*abs(correction), "absolute grid uncertainty")
    return {"observed_order": order, "extrapolated": extrapolated, "safety_factor": 1.25,
            "absolute_uncertainty": uncertainty, "normalizer": scale,
            "normalized_uncertainty": finite(uncertainty/scale, "normalized uncertainty"),
            "refinement_ratios": [h[1]/h[0], h[2]/h[1]],
            "asymptotic_regime_established": False, "model_accepted": False}
