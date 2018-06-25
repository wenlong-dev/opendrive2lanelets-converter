from lxml import etree
from opendriveparser import parse_opendrive
from opendrive2lanelet import Network


fh = open("test/opendrive-1.xodr", 'r')
openDrive = parse_opendrive(etree.parse(fh).getroot())
fh.close()

roadNetwork = Network()
roadNetwork.loadOpenDrive(openDrive)

scenario = roadNetwork.exportCommonRoadScenario()

fh = open("test/opendrive-1.xml", "wb")
fh.write(scenario.export_to_string())
fh.close()
