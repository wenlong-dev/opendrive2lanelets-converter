
from functools import lru_cache
import numpy as np

from opendriveparser.elements.roadPlanView import PlanView

class Border(object):
    """
    A lane border defines a path along a whole lane section
    - a lane always used an inner and outer lane border
    - the reference can be another lane border or a plan view
    """

    def __init__(self):

        self._refOffset = 0.0

        self._coeffsOffsets = []
        self._coeffs = []

        self._reference = None

    def __str__(self):
        return str(self._refOffset)

    @property
    def reference(self):
        return self._reference

    @reference.setter
    def reference(self, value):
        if not isinstance(value, Border) and not isinstance(value, PlanView):
            raise TypeError("Value must be instance of Border or PlanView")

        self._reference = value

    @property
    def refOffset(self):
        """ The reference offset  """
        return self._refOffset

    @refOffset.setter
    def refOffset(self, value):
        self._refOffset = float(value)

    @property
    def coeffs(self):
        """ It is assumed the coeffs are added in ascending order! ([0] + [1] * x + [2] * x**2 + ...) """
        return self._coeffs

    @property
    def coeffsOffsets(self):
        """ Offsets for coeffs """
        return self._coeffsOffsets

    @lru_cache(maxsize=200000)
    def calc(self, sPos):
        """ Calculate the border  """

        if isinstance(self._reference, PlanView):
            # Last reference has to be a reference geometry
            refPos, refTang = self._reference.calc(self._refOffset + sPos)

        elif isinstance(self._reference, Border):
            # Offset of all inner lanes
            refPos, refTang = self._reference.calc(self._refOffset + sPos)

        else:
            raise Exception("Reference must be plan view or other lane border.")

        if not self._coeffs or not self._coeffsOffsets:
            raise Exception("No entries for width definitions.")

        # Find correct coefficients
        widthIdx = next((self._coeffsOffsets.index(n) for n in self._coeffsOffsets[::-1] if n <= sPos), len(self._coeffsOffsets))

        # Calculate width at sPos
        distance = np.polynomial.polynomial.polyval(sPos - self._coeffsOffsets[widthIdx], self._coeffs[widthIdx])

        # New point is in orthogonal direction
        ortho = refTang + np.pi / 2
        newPos = refPos + np.array([distance * np.cos(ortho), distance * np.sin(ortho)])

        return newPos, refTang
