# OpenDRIVE 2 Lanelets - Converter

This repository provides the code for an OpenDRIVE ([www.opendrive.org](http://www.opendrive.org)) to lanelet ([www.mrt.kit.edu/software/liblanelet](https://www.mrt.kit.edu/software/libLanelet/libLanelet.html)) converter.

## Requirements

- Python 3.x
- numpy
- scipy
- lxml

## Installation

If needed, the converter can be installed using ```pip```:

```bash
git clone https://gitlab.lrz.de/koschi/converter.git
cd converter && pip install .
```

## Usage

```python
from lxml import etree
from opendriveparser import parse_opendrive
from opendrive2lanelet import Network

fh = open("input_opendrive.xodr", 'r')
openDrive = parse_opendrive(etree.parse(fh).getroot())
fh.close()

roadNetwork = Network()
roadNetwork.loadOpenDrive(openDrive)

scenario = roadNetwork.exportCommonRoadScenario()

# TODO extract common road writer from fvks
writer = CommonRoadFileWriter(scenario, None)
writer.write_scenario_to_file("output_commonroad_file.xml")
```
