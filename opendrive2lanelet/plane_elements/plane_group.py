
class PLaneGroup(object):
    """ A group of pLanes can be converted to a lanelet just like a single pLane """

    def __init__(self, id=None, pLanes=None, innerNeighbour=None, innerNeighbourSameDirection=True, outerNeighbour=None):

        self._pLanes = []
        self._id = id
        self._innerNeighbour = innerNeighbour
        self._innerNeighbourSameDirection = innerNeighbourSameDirection
        self._outerNeighbour = outerNeighbour

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

    def convertToLanelet(self, precision=0.5):

        lanelet = None

        for pLane in self._pLanes:

            # First lanelet
            if lanelet is None:
                lanelet = pLane.convertToLanelet(precision=precision)
                lanelet.lanelet_id = self.id
                continue

            # Append all following lanelets
            lanelet = lanelet.concatenate(pLane.convertToLanelet(), self.id)

            if lanelet is None:
                raise Exception("Lanelet concatenation problem")

        # Adjacent lanes
        if self.innerNeighbour is not None:
            lanelet.adj_left = self.innerNeighbour
            lanelet.adj_left_same_direction = self.innerNeighbourSameDirection

        if self.outerNeighbour is not None:
            lanelet.adj_right = self.outerNeighbour
            lanelet.adj_right_same_direction = True

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
