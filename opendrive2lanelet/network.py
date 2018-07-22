
import copy

import numpy as np

from opendriveparser.elements.openDrive import OpenDrive

from opendrive2lanelet.plane_elements.plane import PLane
from opendrive2lanelet.plane_elements.plane_group import PLaneGroup
from opendrive2lanelet.plane_elements.border import Border
from opendrive2lanelet.utils import encode_road_section_lane_width_id, decode_road_section_lane_width_id, allCloseToZero

from opendrive2lanelet.commonroad import LaneletNetwork, Scenario, ScenarioError


class Network(object):
    """ Represents a network of parametric lanes """

    def __init__(self):
        self._planes = []
        self._linkIndex = None

    def loadOpenDrive(self, openDrive):
        """ Load all elements of an OpenDRIVE network to a parametric lane representation """

        if not isinstance(openDrive, OpenDrive):
            raise TypeError()

        self._linkIndex = self.createLinkIndex(openDrive)

        # Convert all parts of a road to parametric lanes (planes)
        for road in openDrive.roads:

            # The reference border is the base line for the whole road
            referenceBorder = Network.createReferenceBorder(road.planView, road.lanes.laneOffsets)

            # A lane section is the smallest part that can be converted at once
            for laneSection in road.lanes.laneSections:

                pLanes = Network.laneSectionToPLanes(laneSection, referenceBorder)

                self._planes.extend(pLanes)

    def addPLane(self, pLane):
        if not isinstance(pLane, PLane):
            raise TypeError()
        self._planes.append(pLane)

    def exportLaneletNetwork(self, filterTypes=None):
        """ Export lanelet as lanelet network """

        # Convert groups to lanelets
        laneletNetwork = LaneletNetwork()

        for pLane in self._planes:
            if filterTypes is not None and pLane.type not in filterTypes:
                continue

            lanelet = pLane.convertToLanelet()

            lanelet.predecessor = self._linkIndex.getPredecessors(pLane.id)
            lanelet.successor = self._linkIndex.getSuccessors(pLane.id)

            lanelet.refPLane = pLane

            laneletNetwork.add_lanelet(lanelet)

        # Prune all not existing references
        lanelet_ids = [x.lanelet_id for x in laneletNetwork.lanelets]

        for lanelet in laneletNetwork.lanelets:
            for predecessor in lanelet.predecessor:
                if predecessor not in lanelet_ids:
                    lanelet.predecessor.remove(predecessor)
            for successor in lanelet.successor:
                if successor not in lanelet_ids:
                    lanelet.successor.remove(successor)
            if lanelet.adj_left not in lanelet_ids:
                lanelet.adj_left = None
            if lanelet.adj_right not in lanelet_ids:
                lanelet.adj_right = None

        # Perform lane merges
        # Condition for lane merge:
        # - Lanelet ends (has no successor or predecessor)
        # - Has an adjacent (left or right) with same type
        for lanelet in laneletNetwork.lanelets:

            if len(lanelet.successor) == 0:

                if lanelet.adj_left is not None:
                    adj_left_lanelet = laneletNetwork.find_lanelet_by_id(lanelet.adj_left)

                    newLanelet = lanelet.refPLane.convertToLanelet(
                        ref="right",
                        refDistance=[adj_left_lanelet.calc_width_at_end(), 0.0]
                    )

                    lanelet.left_vertices = newLanelet.left_vertices
                    lanelet.center_vertices = newLanelet.center_vertices
                    lanelet.right_vertices = newLanelet.right_vertices

                    lanelet.successor.extend(adj_left_lanelet.successor)

                if lanelet.adj_right is not None:
                    adj_right_lanelet = laneletNetwork.find_lanelet_by_id(lanelet.adj_right)

                    newLanelet = lanelet.refPLane.convertToLanelet(
                        ref="left",
                        refDistance=[-1 * adj_right_lanelet.calc_width_at_end(), 0.0]
                    )

                    lanelet.left_vertices = newLanelet.left_vertices
                    lanelet.center_vertices = newLanelet.center_vertices
                    lanelet.right_vertices = newLanelet.right_vertices

                    lanelet.successor.extend(adj_right_lanelet.successor)

            if len(lanelet.predecessor) == 0:

                if lanelet.adj_left is not None:
                    adj_left_lanelet = laneletNetwork.find_lanelet_by_id(lanelet.adj_left)

                    newLanelet = lanelet.refPLane.convertToLanelet(
                        ref="right",
                        refDistance=[0.0, -1 * adj_left_lanelet.calc_width_at_end()]
                    )

                    lanelet.left_vertices = newLanelet.left_vertices
                    lanelet.center_vertices = newLanelet.center_vertices
                    lanelet.right_vertices = newLanelet.right_vertices

                    lanelet.predecessor.extend(adj_left_lanelet.predecessor)

                if lanelet.adj_right is not None:
                    adj_right_lanelet = laneletNetwork.find_lanelet_by_id(lanelet.adj_right)

                    newLanelet = lanelet.refPLane.convertToLanelet(
                        ref="left",
                        refDistance=[0.0, adj_right_lanelet.calc_width_at_end()]
                    )

                    lanelet.left_vertices = newLanelet.left_vertices
                    lanelet.center_vertices = newLanelet.center_vertices
                    lanelet.right_vertices = newLanelet.right_vertices

                    lanelet.predecessor.extend(adj_right_lanelet.predecessor)


        # Assign an integer id to each lanelet
        def convert_to_new_id(old_lanelet_id):
            if old_lanelet_id in convert_to_new_id.id_assign:
                new_lanelet_id = convert_to_new_id.id_assign[old_lanelet_id]
            else:
                new_lanelet_id = convert_to_new_id.lanelet_id
                convert_to_new_id.id_assign[old_lanelet_id] = new_lanelet_id

            convert_to_new_id.lanelet_id += 1
            return new_lanelet_id

        convert_to_new_id.id_assign = {}
        convert_to_new_id.lanelet_id = 100

        for lanelet in laneletNetwork.lanelets:
            lanelet.description = lanelet.lanelet_id
            lanelet.lanelet_id = convert_to_new_id(lanelet.lanelet_id)

            lanelet.predecessor = [convert_to_new_id(x) for x in lanelet.predecessor]
            lanelet.successor = [convert_to_new_id(x) for x in lanelet.successor]
            lanelet.adj_left = None if lanelet.adj_left is None else convert_to_new_id(lanelet.adj_left)
            lanelet.adj_right = None if lanelet.adj_right is None else convert_to_new_id(lanelet.adj_right)

        return laneletNetwork

    def exportCommonRoadScenario(self, dt=0.1, benchmark_id=None, filterTypes=None):
        """ Export a full CommonRoad scenario """

        scenario = Scenario(
            dt=dt,
            benchmark_id=benchmark_id if benchmark_id is not None else "none"
        )

        scenario.add_objects(self.exportLaneletNetwork(
            filterTypes=filterTypes if isinstance(filterTypes, list) else ['driving', 'onRamp', 'offRamp', 'exit', 'entry']
        ))

        return scenario

    ##############################################################################################
    ## Helper functions

    @staticmethod
    def createReferenceBorder(planView, laneOffsets):
        """ Create the first (most inner) border line for a road, includes the lane Offsets """

        firstLaneBorder = Border()

        # Set reference to plan view
        firstLaneBorder.reference = planView
        firstLaneBorder.refOffset = 0.0

        # Lane offfsets will be coeffs
        if any(laneOffsets):
            for laneOffset in laneOffsets:
                firstLaneBorder.coeffsOffsets.append(laneOffset.sPos)
                firstLaneBorder.coeffs.append(laneOffset.coeffs)
        else:
            firstLaneBorder.coeffsOffsets.append(0.0)
            firstLaneBorder.coeffs.append([0.0])

        return firstLaneBorder

    @staticmethod
    def laneSectionToPLanes(laneSection, referenceBorder):
        """ Convert a whole lane section into a list of planes """

        newPLanes = []

        # Length of this lane section
        laneSectionStart = laneSection.sPos

        for side in ["right", "left"]:

            # lanes loaded by opendriveparser are aleady sorted by id
            # coeffsFactor decides if border is left or right of the reference line
            if side == "right":
                lanes = laneSection.rightLanes
                coeffsFactor = -1.0

            else:
                lanes = laneSection.leftLanes
                coeffsFactor = 1.0

            # Most inner border
            laneBorders = [referenceBorder]
            prevInnerNeighbours = []

            for lane in lanes:

                if abs(lane.id) > 1:

                    if lane.id > 0:
                        innerLaneId = lane.id - 1
                        outerLaneId = lane.id + 1
                    else:
                        innerLaneId = lane.id + 1
                        outerLaneId = lane.id - 1

                    innerNeighbourId = encode_road_section_lane_width_id(laneSection.parentRoad.id, laneSection.idx, innerLaneId, -1)
                    innerNeighbourSameDirection = True

                    outerNeighbourId = encode_road_section_lane_width_id(laneSection.parentRoad.id, laneSection.idx, outerLaneId, -1)
                else:
                    # Skip lane id 0

                    if lane.id == 1:
                        innerLaneId = -1
                        outerLaneId = 2
                    else:
                        innerLaneId = 1
                        outerLaneId = -2

                    innerNeighbourId = encode_road_section_lane_width_id(laneSection.parentRoad.id, laneSection.idx, innerLaneId, -1)
                    innerNeighbourSameDirection = False

                    outerNeighbourId = encode_road_section_lane_width_id(laneSection.parentRoad.id, laneSection.idx, outerLaneId, -1)

                newPLaneGroup = PLaneGroup(
                    id=encode_road_section_lane_width_id(laneSection.parentRoad.id, laneSection.idx, lane.id, -1),
                    innerNeighbour=innerNeighbourId,
                    innerNeighbourSameDirection=innerNeighbourSameDirection,
                    outerNeighbour=outerNeighbourId
                )

                innerNeighbours = []

                # Create outer lane border
                newPLaneBorder = Border()
                newPLaneBorder.reference = laneBorders[-1]

                if len(laneBorders) == 1:
                    newPLaneBorder.refOffset = laneSectionStart
                else:
                    # Offset from reference border is already included in first created outer lane border
                    newPLaneBorder.refOffset = 0.0

                for width in lane.widths:
                    newPLaneBorder.coeffsOffsets.append(width.sOffset)
                    newPLaneBorder.coeffs.append([x * coeffsFactor for x in width.coeffs])

                laneBorders.append(newPLaneBorder)

                # Create new lane for each width segment
                for width in lane.widths:

                    newPLane = PLane(
                        id=encode_road_section_lane_width_id(laneSection.parentRoad.id, laneSection.idx, lane.id, width.idx),
                        type=lane.type
                    )

                    if allCloseToZero(width.coeffs):
                        newPLane.isNotExistent = True

                    newPLane.innerNeighbours = prevInnerNeighbours

                    newPLane.length = width.length

                    newPLane.innerBorder = laneBorders[-2]
                    newPLane.innerBorderOffset = width.sOffset + laneBorders[-1].refOffset

                    newPLane.outerBorder = laneBorders[-1]
                    newPLane.outerBorderOffset = width.sOffset

                    newPLaneGroup.append(newPLane)
                    innerNeighbours.append(newPLane)

                newPLanes.append(newPLaneGroup)

                prevInnerNeighbours = innerNeighbours

        return newPLanes

    @staticmethod
    def createLinkIndex(openDrive):
        """ Step through all junctions and each single lane to build up a index """

        def add_to_index(linkIndex, pLaneId, successorId, reverse=False):
            if reverse:
                linkIndex.addLink(successorId, pLaneId)
            else:
                linkIndex.addLink(pLaneId, successorId)

        linkIndex = LinkIndex()

        # Extract link information from road lanes
        for road in openDrive.roads:
            for laneSection in road.lanes.laneSections:
                for lane in laneSection.allLanes:
                    pLaneId = encode_road_section_lane_width_id(road.id, laneSection.idx, lane.id, -1)

                    # Not the last lane section? > Next lane section in same road
                    if laneSection.idx < road.lanes.getLastLaneSectionIdx():

                        successorId = encode_road_section_lane_width_id(road.id, laneSection.idx + 1, lane.link.successorId, -1)

                        add_to_index(linkIndex, pLaneId, successorId, lane.id >= 0)

                    # Last lane section! > Next road in first lane section
                    else:

                         # Try to get next road
                         if road.link.successor is not None and road.link.successor.elementType != "junction":

                            nextRoad = openDrive.getRoad(road.link.successor.elementId)

                            if nextRoad is not None:

                                if road.link.successor.contactPoint == "start":
                                    successorId = encode_road_section_lane_width_id(nextRoad.id, 0, lane.link.successorId, -1)
                                    add_to_index(linkIndex, pLaneId, successorId, lane.id >= 0)

                                else: # contact point = end
                                    successorId = encode_road_section_lane_width_id(nextRoad.id, nextRoad.lanes.getLastLaneSectionIdx(), lane.link.successorId, -1)
                                    add_to_index(linkIndex, pLaneId, successorId, lane.id >= 0)


                    # Not first lane section? > Previous lane section in same road
                    if laneSection.idx > 0:
                        predecessorId = encode_road_section_lane_width_id(road.id, laneSection.idx - 1, lane.link.predecessorId, -1)

                        add_to_index(linkIndex, predecessorId, pLaneId, lane.id >= 0)

                    # First lane section! > Previous road
                    else:

                        # Try to get previous road
                        if road.link.predecessor is not None and road.link.predecessor.elementType != "junction":

                            prevRoad = openDrive.getRoad(road.link.predecessor.elementId)

                            if prevRoad is not None:

                                if road.link.predecessor.contactPoint == "start":
                                    predecessorId = encode_road_section_lane_width_id(prevRoad.id, 0, lane.link.predecessorId, -1)
                                    add_to_index(linkIndex, predecessorId, pLaneId, lane.id >= 0)

                                else: # contact point = end
                                    predecessorId = encode_road_section_lane_width_id(prevRoad.id, prevRoad.lanes.getLastLaneSectionIdx(), lane.link.predecessorId, -1)
                                    add_to_index(linkIndex, predecessorId, pLaneId, lane.id >= 0)

        # Add junctions
        for road in openDrive.roads:

            # Add junction links to end of road
            if road.link.successor is not None and road.link.successor.elementType == "junction":

                junction = openDrive.getJunction(road.link.successor.elementId)

                if junction is not None:

                    for connection in junction.connections:

                        roadA = openDrive.getRoad(connection.incomingRoad)
                        roadAcp = "end"
                        roadB = openDrive.getRoad(connection.connectingRoad)
                        roadBcp = connection.contactPoint

                        if roadA.id != road.id:
                            roadA, roadB = [roadB, roadA]

                        for laneLink in connection.laneLinks:

                            if roadAcp == "start":
                                pLaneId = encode_road_section_lane_width_id(roadA.id, 0, laneLink.fromId, -1)
                            else:
                                successorId = encode_road_section_lane_width_id(roadA.id, roadA.lanes.getLastLaneSectionIdx(), laneLink.fromId, -1)

                            if roadBcp == "start":
                                pLaneId = encode_road_section_lane_width_id(roadB.id, 0, laneLink.toId, -1)
                            else:
                                successorId = encode_road_section_lane_width_id(roadB.id, roadB.lanes.getLastLaneSectionIdx(), laneLink.toId, -1)

                            add_to_index(linkIndex, pLaneId, successorId, laneLink.fromId < 0)

            # Add junction links to start of road
            if road.link.predecessor is not None and road.link.predecessor.elementType == "junction":

                junction = openDrive.getJunction(road.link.predecessor.elementId)

                if junction is not None:

                    for connection in junction.connections:

                        roadA = openDrive.getRoad(connection.incomingRoad)
                        roadAcp = "start"
                        roadB = openDrive.getRoad(connection.connectingRoad)
                        roadBcp = connection.contactPoint

                        if roadA.id != road.id:
                            roadA, roadB = [roadB, roadA]

                        for laneLink in connection.laneLinks:

                            if roadAcp == "start":
                                pLaneId = encode_road_section_lane_width_id(roadA.id, 0, laneLink.fromId, -1)
                            else:
                                predecessorId = encode_road_section_lane_width_id(roadA.id, roadA.lanes.getLastLaneSectionIdx(), laneLink.fromId, -1)

                            if roadBcp == "start":
                                pLaneId = encode_road_section_lane_width_id(roadB.id, 0, laneLink.toId, -1)
                            else:
                                predecessorId = encode_road_section_lane_width_id(roadB.id, roadB.lanes.getLastLaneSectionIdx(), laneLink.toId, -1)

                            add_to_index(linkIndex, predecessorId, pLaneId, laneLink.fromId < 0)

        # for junction in openDrive.junctions:
        #     for connection in junction.connections:

        #         incomingRoad = openDrive.getRoad(connection.incomingRoad)
        #         connectingRoad = openDrive.getRoad(connection.connectingRoad)

        #         if incomingRoad is not None and connectingRoad is not None:

        #             for laneLink in connection.laneLinks:

        #                 pLaneId = encode_road_section_lane_width_id(incomingRoad.id, incomingRoad.lanes.getLastLaneSectionIdx(), laneLink.fromId, -1)

        #                 if connection.contactPoint == "end":
        #                     successorId = encode_road_section_lane_width_id(connectingRoad.id, 0, laneLink.toId, -1)
        #                 else:
        #                     successorId = encode_road_section_lane_width_id(connectingRoad.id, connectingRoad.lanes.getLastLaneSectionIdx(), laneLink.toId, -1)

        #                 add_to_index(linkIndex, pLaneId, successorId, lane.id >= 0)


        return linkIndex


class LinkIndex(object):
    """ Overall index of all links in the file, save everything as successors, predecessors can be found via a reverse search """

    def __init__(self):
        self._successors = {}

    def addLink(self, pLaneId, successor):
        if pLaneId not in self._successors:
            self._successors[pLaneId] = []

        if successor not in self._successors[pLaneId]:
            self._successors[pLaneId].append(successor)

    def remove(self, pLaneId):
        # Delete key
        if pLaneId in self._successors:
            del self._successors[pLaneId]

        # Delete all occurances in successor lists
        for successorsId, successors in self._successors.items():
            if pLaneId in successors:
                self._successors[successorsId].remove(pLaneId)

    def getSuccessors(self, pLaneId):
        if pLaneId not in self._successors:
            return []

        return self._successors[pLaneId]

    def getPredecessors(self, pLaneId):
        predecessors = []

        for successorsPLaneId, successors in self._successors.items():
            if pLaneId not in successors:
                continue

            if successorsPLaneId in predecessors:
                continue

            predecessors.append(successorsPLaneId)

        return predecessors

    def __str__(self):
        retstr = "Link Index:\n"

        for pre, succs in self._successors.items():
            retstr += "\t{:15} > {}\n".format(pre, ", ".join(succs))

        return retstr
