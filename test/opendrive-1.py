
import pickle
import os
import unittest

from lxml import etree
from opendriveparser import parse_opendrive
from opendrive2lanelet import Network

class OpenDrive1Test(unittest.TestCase):

    def setUp(self):

        # Write CommonRoad scenario to file
        fh = open(os.path.dirname(os.path.realpath(__file__)) + "/opendrive-1.xodr", 'r')
        openDrive = parse_opendrive(etree.parse(fh).getroot())
        fh.close()

        self.roadNetwork = Network()
        self.roadNetwork.loadOpenDrive(openDrive)

        self.scenario = self.roadNetwork.exportCommonRoadScenario()

        #pickle.dump(self.roadNetwork, open(os.path.dirname(os.path.realpath(__file__)) + "opendrive-1-rn.dump", "wb"))
        #pickle.dump(self.scenario, open(os.path.dirname(os.path.realpath(__file__)) + "opendrive-1-sc.dump", "wb"))


    def test_planes(self):

        with open(os.path.dirname(os.path.realpath(__file__)) + "/opendrive-1-rn.dump", "rb") as fh:
            self.assertEqual(self.roadNetwork, pickle.load(fh))

    def test_lanelets(self):

        with open(os.path.dirname(os.path.realpath(__file__)) + "/opendrive-1-sc.dump", "rb") as fh:
            self.assertEqual(self.scenario, pickle.load(fh))



if __name__ == '__main__':
    unittest.main()
