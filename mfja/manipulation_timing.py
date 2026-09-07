"""Time-parameterize manipulation paths with stops at geometric junctions."""

from itertools import pairwise

from pyhpp.core import InterpolatedPath
from pyhpp.core.path import Vector


def toppra_with_stops(path, toppra):
    """Apply a configured TOPPRA optimizer to each smooth geometric path portion.

    Preserve graph constraints and interpolation points. Each portion starts
    and ends at rest
    """
    flat = Vector(path.outputSize(), path.outputDerivativeSize())
    path.flatten(flat)
    timed = Vector(path.outputSize(), path.outputDerivativeSize())
    for rank in range(flat.numberPaths()):
        leaf = flat.pathAtRank(rank)
        if isinstance(leaf, InterpolatedPath):
            params = [t for t, _ in leaf.interpolationPoints()]
        else:
            params = [leaf.timeRange().first, leaf.timeRange().second]
        for start, end in pairwise(params):
            if end - start <= 1e-9:
                continue
            if end - start < 1e-6:
                raise ValueError("Path portion is too short for TOPPRA timing")
            portion = Vector(path.outputSize(), path.outputDerivativeSize())
            portion.appendPath(leaf.extract(start, end))
            timed.concatenate(toppra.optimize(portion))
    return timed
