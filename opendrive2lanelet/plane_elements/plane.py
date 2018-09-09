
import numpy as np

from opendrive2lanelet.plane_elements.border import Border
from opendrive2lanelet.commonroad import Lanelet

class PLane(object):
    """ A lane defines a part of a road along a reference trajectory (plan view), using lane borders and start/stop positions (parametric) """

    def __init__(self, id, type):
        """ Each lane is defined with a starting point, an offset to the reference trajectory and a lane width """

        self._id = id
        self._type = type
        self._length = None
        self._innerBorder = None
        self._innerBorderOffset = None
        self._outerBorder = None
        self._outerBorderOffset = None

        self._isNotExistent = False
        self._innerNeighbours = []
        self._outerNeighbours = []

        self._successors = []
        self._predecessors = []

    @property
    def id(self):
        return self._id

    @property
    def type(self):
        return self._type

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, value):
        self._length = float(value)

    @property
    def innerBorder(self):
        return self._innerBorder

    @innerBorder.setter
    def innerBorder(self, value):
        if not isinstance(value, Border):
            raise TypeError("Value must be instance of _LaneBorder.")

        self._innerBorder = value

    def calcInnerBorder(self, sPos, addOffset=0.0):
        return self._innerBorder.calc(self._innerBorderOffset + sPos, addOffset=addOffset)

    @property
    def outerBorder(self):
        return self._outerBorder

    @property
    def innerBorderOffset(self):
        return self._innerBorderOffset

    @innerBorderOffset.setter
    def innerBorderOffset(self, value):
        self._innerBorderOffset = float(value)

    @outerBorder.setter
    def outerBorder(self, value):
        if not isinstance(value, Border):
            raise TypeError("Value must be instance of Border.")

        self._outerBorder = value

    def calcOuterBorder(self, sPos, addOffset=0.0):
        return self._outerBorder.calc(self._outerBorderOffset + sPos, addOffset=addOffset)

    @property
    def outerBorderOffset(self):
        return self._outerBorderOffset

    @outerBorderOffset.setter
    def outerBorderOffset(self, value):
        self._outerBorderOffset = float(value)

    def calcWidth(self, sPos):
        innerCoords = self.calcInnerBorder(sPos)
        outerCoords = self.calcOuterBorder(sPos)

        return np.linalg.norm(innerCoords[0] - outerCoords[0])

    def convertToLanelet(self, precision=0.5, ref=None, refDistance=[0.0, 0.0], refMinDistance=3.0):
        # Define calculation points
        # TODO dependent on max error
        numSteps = max(2, np.ceil(self._length / float(precision)))
        poses = np.linspace(0, self._length, numSteps)

        left_vertices = []
        right_vertices = []

        for pos in poses:
            if ref is None:
                left_vertices.append(self.calcInnerBorder(pos)[0])
                right_vertices.append(self.calcOuterBorder(pos)[0])
            else:
                x = pos
                m = (refDistance[1] - refDistance[0]) / self._length
                t = refDistance[0]

                d = m*x + t

                if ref == "left":
                    left_vertices.append(self.calcInnerBorder(pos)[0])
                    right_vertices.append(self.calcOuterBorder(pos, d)[0])
                elif ref == "right":
                    left_vertices.append(self.calcInnerBorder(pos, d)[0])
                    right_vertices.append(self.calcOuterBorder(pos)[0])

        center_vertices = [(l + r) / 2 for (l, r) in zip(left_vertices, right_vertices)]

        return Lanelet(
            left_vertices=np.array([np.array([x, y]) for x, y in left_vertices]),
            center_vertices=np.array([np.array([x, y]) for x, y in center_vertices]),
            right_vertices=np.array([np.array([x, y]) for x, y in right_vertices]),
            lanelet_id=self._id
        )

    @property
    def isNotExistent(self):
        return self._isNotExistent

    @isNotExistent.setter
    def isNotExistent(self, value):
        self._isNotExistent = bool(value)

    @property
    def innerNeighbours(self):
        return self._innerNeighbours

    @innerNeighbours.setter
    def innerNeighbours(self, value):
        self._innerNeighbours = value

    @property
    def outerNeighbours(self):
        return self._outerNeighbours

    @outerNeighbours.setter
    def outerNeighbours(self, value):
        self._outerNeighbours = value

    @property
    def successors(self):
        return self._successors

    @property
    def predecessors(self):
        return self._predecessors
