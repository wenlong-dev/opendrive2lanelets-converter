
import copy

import numpy as np

from opendriveparser.elements.openDrive import OpenDrive

from opendrive2lanelet.plane_elements.plane import PLane
from opendrive2lanelet.plane_elements.border import Border
from opendrive2lanelet.utils import encode_road_section_lane_width_id, decode_road_section_lane_width_id, allCloseToZero

from opendrive2lanelet.commonroad import LaneletNetwork, Scenario


class Network(object):
    """ Represents a network of parametric lanes """

    def __init__(self):
        self._planes = []

    def loadOpenDrive(self, openDrive):
        """ Load all elements of an OpenDRIVE network to a parametric lane representation """

        if not isinstance(openDrive, OpenDrive):
            raise TypeError()

        linkIndex = self.createLinkIndex(openDrive)

        # Convert all parts of a road to parametric lanes (planes)
        for road in openDrive.roads:

            # The reference border is the base line for the whole road
            referenceBorder = Network.createReferenceBorder(road.planView, road.lanes.laneOffsets)

            # A lane section is the smallest part that can be converted at once
            for laneSection in road.lanes.laneSections:

                pLanes = self.laneSectionToPLanes(laneSection, referenceBorder, road.id, linkIndex)

                # Now adjust the pLanes at the merging and split points
                self.handleMergingLanes(pLanes, linkIndex)

                # The road id is used to create a traceable id value to each plane
                self._planes.extend(pLanes)

    def addPLane(self, pLane):
        if not isinstance(pLane, PLane):
            raise TypeError()
        self._planes.append(pLane)

    def exportLaneletNetwork(self, filterTypes=None):
        """ Export lanelet as lanelet network """

        # Group all width in the same lane
        laneletGroups = {}

        for pLane in self._planes:
            roadId, sectionId, laneId, _ = decode_road_section_lane_width_id(pLane.id)
            groupId = ".".join([str(roadId), str(sectionId), str(laneId)])

            if filterTypes is not None and pLane.type not in filterTypes:
                continue

            if groupId not in laneletGroups:
                laneletGroups[groupId] = []

            laneletGroups[groupId].append(pLane)

        # Convert groups to lanelets
        laneletNetwork = LaneletNetwork()

        for groupId, pLanes in laneletGroups.items():

            pLanes.sort(key=lambda x: decode_road_section_lane_width_id(pLane.id)[3], reverse=False)

            prevLanelet = None

            for pLane in pLanes:
                lanelet = pLane.converToLanelet()

                if prevLanelet is None:
                    prevLanelet = lanelet
                    prevLanelet.lanelet_id = groupId
                    continue

                prevLanelet.left_vertices = np.vstack((prevLanelet.left_vertices, lanelet.left_vertices[1:]))
                prevLanelet.center_vertices = np.vstack((prevLanelet.center_vertices, lanelet.center_vertices[1:]))
                prevLanelet.right_vertices = np.vstack((prevLanelet.right_vertices, lanelet.right_vertices[1:]))

            laneletNetwork.add_lanelet(prevLanelet)

        # Assign an integer id to each lanelet
        id_assign = {}
        lanelet_id = 100

        for lanelet in laneletNetwork.lanelets:

            if lanelet.lanelet_id in id_assign:
                new_lanelet_id = id_assign[lanelet.lanelet_id]
            else:
                new_lanelet_id = lanelet_id
                id_assign[lanelet.lanelet_id] = new_lanelet_id

                lanelet_id += 1

            lanelet.lanelet_id = new_lanelet_id

        return laneletNetwork

    def exportCommonRoadScenario(self, dt=0.5, benchmark_id=None, filterTypes=['driving', 'sidewalk']):
        """ Export a full CommonRoad scenario """

        scenario = Scenario(
            dt=dt,
            benchmark_id=benchmark_id if benchmark_id is not None else "none"
        )

        scenario.add_objects(self.exportLaneletNetwork(
            filterTypes=filterTypes
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
    def laneSectionToPLanes(laneSection, referenceBorder, roadId, linkIndex):
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
                        id=encode_road_section_lane_width_id(roadId, laneSection.idx, lane.id, width.idx),
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

                    newPLanes.append(newPLane)
                    innerNeighbours.append(newPLane)

                prevInnerNeighbours = innerNeighbours

        return newPLanes

    @staticmethod
    def createLinkIndex(openDrive):
        """ Step through all junctions and each single lane to build up a index """

        linkIndex = LinkIndex()

        # 1. Convert junction definitions
        for junction in openDrive.junctions:

            for connection in junction.connections:

                for laneLink in connection.laneLinks:

                    # Predecessor
                    predecessorRoad = openDrive.getRoad(connection.incomingRoad)

                    if predecessorRoad is None:
                        continue

                    # Check if predecessor road has junction as predecessor or successor
                    if predecessorRoad.link.predecessor is not None and \
                        predecessorRoad.link.predecessor.elementType == "junction" and \
                        predecessorRoad.link.predecessor.elementId == junction.id:

                        predecessorLaneSection = predecessorRoad.lanes.getLaneSection(0)

                    else:
                        predecessorLaneSection = predecessorRoad.lanes.getLaneSection(predecessorRoad.lanes.getLastLaneSectionIdx())

                    predecessorLane = predecessorLaneSection.getLane(laneLink.fromId)
                    predecessorWidth = predecessorLane.getWidth(predecessorLane.getLastLaneWidthIdx())

                    predecessor = encode_road_section_lane_width_id(
                        predecessorRoad.id,
                        predecessorLaneSection.idx,
                        predecessorLane.id,
                        predecessorWidth.idx
                    )

                    # Successor
                    successorRoad = openDrive.getRoad(connection.connectingRoad)

                    if successorRoad is None:
                        continue

                    successorLaneSection = successorRoad.lanes.getLaneSection(successorRoad.lanes.getLastLaneSectionIdx() if connection.contactPoint == "end" else 0)
                    successorLane = successorLaneSection.getLane(laneLink.toId)
                    successorWidth = successorLane.getWidth(successorLane.getLastLaneWidthIdx() if connection.contactPoint == "end" else 0)

                    successor = encode_road_section_lane_width_id(
                        successorRoad.id,
                        successorLaneSection.idx,
                        successorLane.id,
                        successorWidth.idx
                    )

                    linkIndex.addLink(predecessor, successor)

        # 2. Get links from the roads
        for road in openDrive.roads:

            for laneSection in road.lanes.laneSections:

                for lane in laneSection.allLanes:

                    for width in lane.widths:

                        pLaneId = encode_road_section_lane_width_id(road.id, laneSection.idx, lane.id, width.idx)

                        # TODO HANDLE predecessors of first width and laneSection

                        # Not the last width entry? > Next width entry in same lane section
                        if width.idx < lane.getLastLaneWidthIdx():
                            successor = encode_road_section_lane_width_id(road.id, laneSection.idx, lane.id, width.idx + 1)

                            linkIndex.addLink(pLaneId, successor)

                            continue

                        # Not the last lane section? > Next lane section in same road
                        if laneSection.idx < road.lanes.getLastLaneSectionIdx():

                            # If lane does not provide link, we do not have enough information, skip
                            if lane.link.successorId is None:
                                #print(str(pLaneId) + " skipped")
                                continue

                            successor = encode_road_section_lane_width_id(road.id, laneSection.idx + 1, lane.link.successorId, 0)

                            linkIndex.addLink(pLaneId, successor)
                            continue

                        # Connecting to another road
                        if road.link.successor is not None:

                            if road.link.successor.elementType == "road":

                                successorRoad = openDrive.getRoad(road.link.successor.elementId)
                                successorLaneSection = successorRoad.lanes.getLaneSection(successorRoad.lanes.getLastLaneSectionIdx() if road.link.successor.contactPoint == "end" else 0)

                                # If lane does not provide link, we do not have enough information, skip
                                if lane.link.successorId is None:
                                    #print(str(pLaneId) + " skipped")
                                    continue

                                successorLane = successorLaneSection.getLane(lane.link.successorId)
                                successorWidth = successorLane.getWidth(successorLane.getLastLaneWidthIdx() if road.link.successor.contactPoint == "end" else 0)


                                successor = encode_road_section_lane_width_id(
                                    successorRoad.id,
                                    successorLaneSection.idx,
                                    successorLane.id,
                                    successorWidth.idx
                                )

                                linkIndex.addLink(pLaneId, successor)
                                continue

                        if road.link.predecessor is not None:

                            if road.link.predecessor.elementType == "road":

                                predecessorRoad = openDrive.getRoad(road.link.predecessor.elementId)
                                predecessorLaneSection = predecessorRoad.lanes.getLaneSection(predecessorRoad.lanes.getLastLaneSectionIdx() if road.link.predecessor.contactPoint == "end" else 0)

                                # If lane does not provide link, we do not have enough information, skip
                                if lane.link.predecessorId is None:
                                    #print(str(pLaneId) + " skipped")
                                    continue

                                predecessorLane = predecessorLaneSection.getLane(lane.link.predecessorId)
                                predecessorWidth = predecessorLane.getWidth(predecessorLane.getLastLaneWidthIdx() if road.link.predecessor.contactPoint == "end" else 0)

                                predecessor = encode_road_section_lane_width_id(
                                    predecessorRoad.id,
                                    predecessorLaneSection.idx,
                                    predecessorLane.id,
                                    predecessorWidth.idx
                                )

                                linkIndex.addLink(predecessor, pLaneId)
                                continue

        return linkIndex

    @staticmethod
    def handleMergingLanes(pLanes, linkIndex):

        def purgeNullPLanes(pLanes, linkIndex):
            """ Delete all zero wide lanes """

            for pLane in pLanes:
                if pLane.isNotExistent:
                    # Delete from link index
                    linkIndex.remove(pLane.id)

                    # Delete from neighbours
                    # TODO!!!

                    # And delete from list itself
                    pLanes.remove(pLane)

        def isMergingIntoInnerPLane(pLane, linkIndex):
            if len(linkIndex.getSuccessors(pLane.id)) == 0 and pLane.innerNeighbours and pLane.innerNeighbours[0].type == pLane.type:
                return True
            return False

        def isMergingIntoOuterPLane(pLane, linkIndex):
            if len(linkIndex.getSuccessors(pLane.id)) == 0 and pLane.outerNeighbours and pLane.outerNeighbours[0].type == pLane.type:
                return True
            return False

        def makeOuterNeighbourConnections(pLanes):
            for pLane in pLanes:
                # No neighbours
                if pLane.innerNeighbours is None:
                    continue

                for neighbouringPLane in pLane.innerNeighbours:
                    # Do not add twice
                    if pLane in neighbouringPLane.outerNeighbours:
                        continue
                    neighbouringPLane.outerNeighbours.append(pLane)

        # Delete all pLanes with width = 0
        purgeNullPLanes(pLanes, linkIndex)
        makeOuterNeighbourConnections(pLanes)

        mergingPLanes = []

        for pLane in pLanes:

            if isMergingIntoInnerPLane(pLane, linkIndex):
                sPos = pLane.innerBorder.refOffset + pLane.length

                # TODO add findNeighbourAt method to pLanes

                # Lane can have more than one neighbour, find right one
                for neighbour in pLane.innerNeighbours:

                    # End of lane is actually in the neighbours domain
                    if neighbour.outerBorderOffset + neighbour.outerBorder.refOffset <= sPos <= neighbour.outerBorderOffset + neighbour.outerBorder.refOffset + neighbour.length:

                        newMergingLane = copy.copy(pLane)

                        # End width
                        width = neighbour.calcWidth(sPos)

                        # Define new outer border based on previous one
                        newMergingLaneBorder = Border()
                        newMergingLaneBorder.reference = newMergingLane.outerBorder

                        # TODO make function that transitions from the first width to the next one
                        newMergingLaneBorder.coeffsOffsets.append(0)
                        newMergingLaneBorder.coeffs.append([width])

                        newMergingLane.innerBorder = newMergingLaneBorder
                        newMergingLane.innerBorderOffset = newMergingLane.outerBorderOffset

                        mergingPLanes.append(newMergingLane)

                        break

            if isMergingIntoOuterPLane(pLane, linkIndex):
                sPos = pLane.outerBorder.refOffset + pLane.length

                # Find corresponding neighbour for position
                for neighbour in pLane.outerNeighbours:

                    if neighbour.innerBorderOffset + neighbour.innerBorder.refOffset <= sPos <= neighbour.innerBorderOffset + neighbour.innerBorder.refOffset + neighbour.length:

                        newMergingLane = copy.copy(pLane)

                        width = neighbour.calcWidth(sPos)

                        # Define new outer border based on previous one
                        newMergingLaneBorder = Border()
                        newMergingLaneBorder.reference = newMergingLane.innerBorder

                        newMergingLaneBorder.coeffsOffsets.append(0)
                        newMergingLaneBorder.coeffs.append([width])

                        newMergingLane.outerBorder = newMergingLaneBorder
                        newMergingLane.outerBorderOffset = newMergingLane.innerBorderOffset

                        mergingPLanes.append(newMergingLane)

                        break

        # Remove the pLanes that get merged
        for mergingPLane in mergingPLanes:
            for pLane in pLanes:
                if pLane.id == mergingPLane.id:
                    pLanes.remove(pLane)

        pLanes.extend(mergingPLanes)


class LinkIndex(object):
    """ Overall index of all links in the file, save everything as successors, predecessors can be found via a reverse search """

    def __init__(self):
        self._successors = {}

    def addLink(self, predecessor, successor):
        if predecessor not in self._successors:
            self._successors[predecessor] = []

        if successor not in self._successors[predecessor]:
            self._successors[predecessor].append(successor)

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

    def __str__(self):
        retstr = "Link Index:\n"

        for pre, succs in self._successors.items():
            retstr += "\t{:15} > {}\n".format(pre, ", ".join(succs))

        return retstr
