
def encode_road_section_lane_width_id(roadId, sectionId, laneId, widthId):
    return str(roadId) + "." + str(sectionId) + "." + str(laneId) + "." + str(widthId)

def decode_road_section_lane_width_id(encodedString):

    parts = encodedString.split(".")

    if parts != 4:
        raise Exception()

    return (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))

def allCloseToZero(a):
    """ Tests if all elements of a are close to zero. """

    import numpy
    return numpy.allclose(a, numpy.zeros(numpy.shape(a)))

def getLanePredecessor(opendriveLane):
    """ Takes OpenDRIVE lane and determines if it has a defined predecessor """
    pass
