
import os
import time

import numpy as np
from lxml import etree, objectify

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

        rootElement = E("commonRoad", benchmarkID="a", commonRoadVersion="2017a", date=time.strftime("%Y-%m-%d"), timeStepSize="0.1")

        # Road network
        for lanelet in self.lanelet_network.lanelets:

            # Bounds
            leftPointsElements = E("leftBound")

            for (x, y) in lanelet.left_vertices:
                leftPointsElements.append(E("point", E("x", str(x)), E("y", str(y))))

            rightPointsElements = E("rightBound")

            for (x, y) in lanelet.right_vertices:
                rightPointsElements.append(E("point", E("x", str(x)), E("y", str(y))))

            laneletElement = E("lanelet", leftPointsElements, rightPointsElements)
            laneletElement.set("id", str(int(lanelet.lanelet_id)))

            for predecessor in lanelet.predecessor:
                laneletElement.append(E("predecessor", ref=str(predecessor)))

            for successor in lanelet.successor:
                laneletElement.append(E("successor", ref=str(successor)))

            if lanelet.adj_left is not None:
                laneletElement.append(E("adjacentLeft", ref=str(lanelet.adj_left), drivingDirection=str("same" if lanelet.adj_left_same_direction else "opposite")))

            if lanelet.adj_right is not None:
                laneletElement.append(E("adjacentRight", ref=str(lanelet.adj_right), drivingDirection=str("same" if lanelet.adj_right_same_direction else "opposite")))

            rootElement.append(laneletElement)

        # Dummy planning problem
        planningProblem = E("planningProblem", id=str(0))

        initialState = E("initialState")
        initialState.append(E('position', E('point', E('x', str(0.0)), E('y', str(0.0)))))
        initialState.append(E('velocity', E('exact', str(0.0))))
        initialState.append(E('orientation', E('exact', str(0.0))))
        initialState.append(E('yawRate', E('exact', str(0.0))))
        initialState.append(E('slipAngle', E('exact', str(0.0))))
        initialState.append(E('time', E('exact', str(0.0))))

        planningProblem.append(initialState)

        goalRegion = E("goalRegion")

        goalState = E("state")
        goalState.append(E('position', E('point', E('x', str(0.0)), E('y', str(0.0)))))
        goalState.append(E('orientation', E('exact', str(0.0))))
        goalState.append(E('time', E('exact', str(0.0))))
        goalState.append(E('velocity', E('exact', str(0.0))))
        goalState.append(E('acceleration', E('exact', str(0.0))))

        goalRegion.append(goalState)

        planningProblem.append(goalRegion)

        rootElement.append(planningProblem)

        return etree.tostring(rootElement, pretty_print=True, xml_declaration=True, encoding='utf-8')

    @staticmethod
    def read_from_string(input_string, dt=0.1):

        # Parse XML using CommonRoad schema
        schema = etree.XMLSchema(file=open(os.path.dirname(os.path.abspath(__file__)) + "/XML_commonRoad_XSD.xsd", "rb"))
        parser = objectify.makeparser(schema=schema)

        root = objectify.fromstring(input_string, parser=parser)

        # Create scenario
        scenario = Scenario(
            dt=dt,
            benchmark_id=root.attrib['benchmarkID']
        )

        for lanelet in root.iterchildren('lanelet'):

            left_vertices = []
            right_vertices = []

            for pt in lanelet.leftBound.getchildren():
                left_vertices.append(np.array([float(pt.x), float(pt.y)]))

            for pt in lanelet.rightBound.getchildren():
                right_vertices.append(np.array([float(pt.x), float(pt.y)]))

            center_vertices = [(l + r) / 2 for (l, r) in zip(left_vertices, right_vertices)]

            scenario.lanelet_network.add_lanelet(Lanelet(
                left_vertices=np.array([np.array([x, y]) for x, y in left_vertices]),
                center_vertices=np.array([np.array([x, y]) for x, y in center_vertices]),
                right_vertices=np.array([np.array([x, y]) for x, y in right_vertices]),
                lanelet_id=int(lanelet.attrib['id']),
                predecessor=[int(el.attrib['ref']) for el in lanelet.iterchildren(tag='predecessor')],
                successor=[int(el.attrib['ref']) for el in lanelet.iterchildren(tag='successor')]
            ))

        return scenario


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
                raise Exception("Lanelet with id {} already in network.".format(lanelet.lanelet_id))

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
        self.description = ""

    def calc_width_at_start(self):
        return np.linalg.norm(self.left_vertices[0], self.right_vertices[0])

    def calc_width_at_end(self):
        return np.linalg.norm(np.array(self.left_vertices[-1]) - np.array(self.right_vertices[-1]))

    def concatenate(self, lanelet, lanelet_id=-1):
        float_tolerance = 1e-6
        if (np.linalg.norm(self.center_vertices[-1] - lanelet.center_vertices[0]) > float_tolerance or
            np.linalg.norm(self.left_vertices[-1] - lanelet.left_vertices[0]) > float_tolerance or
            np.linalg.norm(self.right_vertices[-1] - lanelet.right_vertices[0]) > float_tolerance):
            pass
            #return None
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
                       lanelet_id=lanelet_id,
                       speed_limit=None)
