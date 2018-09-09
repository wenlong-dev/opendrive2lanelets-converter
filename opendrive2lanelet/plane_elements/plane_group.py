
from opendrive2lanelet.commonroad import Lanelet

class PLaneGroup(object):
    """ A group of pLanes can be converted to a lanelet just like a single pLane """

    def __init__(self, id=None, pLanes=None, innerNeighbour=None, innerNeighbourSameDirection=True, outerNeighbour=None, reverse = False):

        self._pLanes = []
        self._id = id
        self._innerNeighbour = innerNeighbour
        self._innerNeighbourSameDirection = innerNeighbourSameDirection
        self._outerNeighbour = outerNeighbour

        self._reverse = reverse

        if pLanes is not None:

            if isinstance(pLanes, list):
                self._pLanes.extend(pLanes)
            else:
                self._pLanes.append(pLanes)

    def append(self, pLane):
        self._pLanes.append(pLane)

    @property
    def id(self):
        if self._id is not None:
            return self._id

        raise Exception()

    @property
    def type(self):
        return self._pLanes[0]._type

    @property
    def length(self):
        return sum([x.length for x in self._pLanes])

    def convertToLanelet(self, precision=0.5, ref=None, refDistance=[0.0, 0.0]):

        lanelet = None
        y1 = refDistance[0]
        x = 0

        for pLane in self._pLanes:

            x += pLane.length
            y2 = (refDistance[1] - refDistance[0]) / self.length * x + refDistance[0]

            # First lanelet
            if lanelet is None:
                lanelet = pLane.convertToLanelet(precision=precision, ref=ref, refDistance=[y1, y2])
                lanelet.lanelet_id = self.id
                continue

            # Append all following lanelets
            newLanelet = pLane.convertToLanelet(precision=precision, ref=ref, refDistance=[y1, y2])

            lanelet = lanelet.concatenate(newLanelet, self.id)

            y1 = y2

            if lanelet is None:
                raise Exception("Lanelet concatenation problem")

        # Adjacent lanes
        if self.innerNeighbour is not None:
            lanelet.adj_left = self.innerNeighbour
            lanelet.adj_left_same_direction = self.innerNeighbourSameDirection

        if self.outerNeighbour is not None:
            lanelet.adj_right = self.outerNeighbour
            lanelet.adj_right_same_direction = True

        if self._reverse:
            newLanelet = Lanelet(
                left_vertices=lanelet.right_vertices[::-1],
                center_vertices=lanelet.center_vertices[::-1],
                right_vertices=lanelet.left_vertices[::-1],
                lanelet_id=lanelet.lanelet_id
            )

            newLanelet.adj_left = lanelet.adj_left
            newLanelet.adj_left_same_direction = lanelet.adj_left_same_direction
            newLanelet.adj_right = lanelet.adj_right
            newLanelet.adj_right_same_direction = lanelet.adj_right_same_direction

            lanelet = newLanelet

        return lanelet

    @property
    def innerNeighbour(self):
        return self._innerNeighbour

    @innerNeighbour.setter
    def innerNeighbour(self, value):
        self._innerNeighbour = value

    @property
    def innerNeighbourSameDirection(self):
        return self._innerNeighbourSameDirection

    @innerNeighbourSameDirection.setter
    def innerNeighbourSameDirection(self, value):
        self._innerNeighbourSameDirection = value

    @property
    def outerNeighbour(self):
        """ Outer neighbour has always the same driving direction """
        return self._outerNeighbour

    @outerNeighbour.setter
    def outerNeighbour(self, value):
        self._outerNeighbour = value
