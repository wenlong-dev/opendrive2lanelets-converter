
import time

import numpy as np

from lxml import etree
from lxml.builder import E

class Scenario(object):

    def __init__(self, dt, benchmark_id=None):
        self.dt = dt
        self.lanelet_network = LaneletNetwork()
        self.benchmark_id = benchmark_id

    def add_objects(self, o):
        if type(o) == list:
            for oo in o:
                self.add_objects(oo)
        elif type(o) == LaneletNetwork:
            for l in o.lanelets:
                self.lanelet_network.add_lanelet(l)
        elif type(o) == Lanelet:
            self.lanelet_network.add_lanelet(o)
        else:
            raise ScenarioError

    def export_to_string(self):

        rootElement = E("commonRoad", commonRoadVersion="2017a", date=time.strftime("%d-%b-%Y"), timeStepSize="0.1")

        for lanelet in self.lanelet_network.lanelets:

            # Bounds
            leftPointsElements = E("leftBound")

            for (x, y) in lanelet.left_vertices:
                leftPointsElements.append(E("point", E("x", str(x)), E("y", str(y))))

            rightPointsElements = E("rightBound")

            for (x, y) in lanelet.right_vertices:
                rightPointsElements.append(E("point", E("x", str(x)), E("y", str(y))))

            laneletElement = E("lanelet", leftPointsElements, rightPointsElements)
            laneletElement.set("id", str(lanelet.lanelet_id))

            rootElement.append(laneletElement)

        return etree.tostring(rootElement, pretty_print=True)

class ScenarioError(Exception):
    """Base class for exceptions in this module."""
    pass

class LaneletNetwork(object):

    def __init__(self):
        self.lanelets = []

    def find_lanelet_by_id(self, lanelet_id):
        for l in self.lanelets:
            if l.lanelet_id == lanelet_id:
                return l
        raise ScenarioError

    def add_lanelet(self, lanelet):
        if type(lanelet) == list:
            for l in lanelet:
                self.add_lanelet(l)
        else:
            try:
                self.find_lanelet_by_id(lanelet.lanelet_id)
            except ScenarioError:
                self.lanelets.append(lanelet)
            else:
                raise ScenarioError

class Lanelet(object):

    def __init__(self, left_vertices, center_vertices, right_vertices,
                 lanelet_id,
                 predecessor=None, successor=None,
                 adjacent_left=None, adjacent_left_same_direction=None,
                 adjacent_right=None, adjacent_right_same_direction=None,
                 speed_limit=None):
        if (len(left_vertices) != len(center_vertices) and
                    len(center_vertices) != len(right_vertices)):
            raise ScenarioError
        self.left_vertices = left_vertices
        self.center_vertices = center_vertices
        self.right_vertices = right_vertices
        if predecessor is None:
            self.predecessor = []
        else:
            self.predecessor = predecessor
        if successor is None:
            self.successor = []
        else:
            self.successor = successor
        self.adj_left = adjacent_left
        self.adj_left_same_direction = adjacent_left_same_direction
        self.adj_right = adjacent_right
        self.adj_right_same_direction = adjacent_right_same_direction

        self.lanelet_id = lanelet_id
        self.speed_limit = speed_limit
        self.distance = [0]
        for i in range(1, len(self.center_vertices)):
            self.distance.append(self.distance[i-1] +
                                 np.linalg.norm(self.center_vertices[i] -
                                                self.center_vertices[i-1]))
        self.distance = np.array(self.distance)

    def concatenate(self, lanelet):
        float_tolerance = 1e-6
        if (np.linalg.norm(self.center_vertices[-1] - lanelet.center_vertices[0]) > float_tolerance or
            np.linalg.norm(self.left_vertices[-1] - lanelet.left_vertices[0]) > float_tolerance or
            np.linalg.norm(self.right_vertices[-1] - lanelet.right_vertices[0]) > float_tolerance):
            return None
        left_vertices = np.vstack((self.left_vertices,
                                   lanelet.left_vertices[1:]))
        center_vertices = np.vstack((self.center_vertices,
                                     lanelet.center_vertices[1:]))
        right_vertices = np.vstack((self.right_vertices,
                                    lanelet.right_vertices[1:]))
        return Lanelet(center_vertices=center_vertices,
                       left_vertices=left_vertices,
                       right_vertices=right_vertices,
                       predecessor=self.predecessor.copy(),
                       successor=lanelet.successor.copy(),
                       adjacent_left=None,
                       adjacent_left_same_direction=None,
                       adjacent_right=None,
                       adjacent_right_same_direction=None,
                       lanelet_id=-1,
                       speed_limit=None)
