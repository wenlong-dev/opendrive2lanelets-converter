
import copy

import numpy as np

from opendriveparser.elements.openDrive import OpenDrive

from opendrive2lanelet.plane_elements.plane import PLane
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

        for pLane in self._planes:
            if filterTypes is not None and pLane.type not in filterTypes:
                continue

            lanelet = pLane.converToLanelet()

            lanelet.predecessor = self._linkIndex.getPredecessors(pLane.id)
            lanelet.successor = self._linkIndex.getSuccessors(pLane.id)

            laneletNetwork.add_lanelet(lanelet)

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

                    newPLanes.append(newPLane)
                    innerNeighbours.append(newPLane)

                prevInnerNeighbours = innerNeighbours

        return newPLanes


    @staticmethod
    def createLinkIndex(openDrive):
        """ Step through all junctions and each single lane to build up a index """

        linkIndex = LinkIndex()

        # 2. Get links from the roads
        for road in openDrive.roads:
            for laneSection in road.lanes.laneSections:
                for lane in laneSection.allLanes:
                    for width in lane.widths:
                        pLaneId = encode_road_section_lane_width_id(road.id, laneSection.idx, lane.id, width.idx)

                        # Not the last width entry? > Next width entry in same lane section
                        if width.idx < lane.getLastLaneWidthIdx():
                            successor = encode_road_section_lane_width_id(road.id, laneSection.idx, lane.id, width.idx + 1)

                            if lane.id >= 0:
                                linkIndex.addLink(successor, pLaneId)
                            else:
                                linkIndex.addLink(pLaneId, successor)

                            continue

                        # Not the last lane section? > Next lane section in same road
                        if laneSection.idx < road.lanes.getLastLaneSectionIdx():

                            # If lane does not provide link, we do not have enough information, skip
                            if lane.link.successorId is None:
                                print(str(pLaneId) + " skipped because lane does not provide link information")
                                continue

                            successor = encode_road_section_lane_width_id(road.id, laneSection.idx + 1, lane.link.successorId, 0)

                            if lane.id >= 0:
                                linkIndex.addLink(successor, pLaneId)
                            else:
                                linkIndex.addLink(pLaneId, successor)

                            continue

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
