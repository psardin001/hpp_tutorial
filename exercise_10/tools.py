"""Time-parameterize manipulation paths with stops at geometric junctions."""

from pyhpp.manipulation import ManipulationSpline
from pyhpp_toppra import Toppra as _Toppra


class Toppra(_Toppra):
    """Time-parameterize paths with stops at geometric junctions."""

    def __init__(self, problem):
        super().__init__(problem)
        self.stopMethod = "junctions"


class SplineToppra(_Toppra):
    """Smooth transitions, join them within manipulation states, then time."""

    def __init__(self, problem, graph):
        super().__init__(problem)
        self.stopMethod = "subpaths"
        self.spline = ManipulationSpline(problem)
        # Penalize changes of tangent before applying acceleration-limited timing.
        self.spline.costOrder = 2
        self.spline.maxIterations(100)

    @property
    def singleSplineTransitions(self):
        return self.spline.singleSplineTransitions

    @singleSplineTransitions.setter
    def singleSplineTransitions(self, transitions):
        self.spline.singleSplineTransitions = transitions

    def optimize(self, path):
        return super().optimize(self.spline.optimize(path))
