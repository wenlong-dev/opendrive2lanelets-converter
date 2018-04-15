# OpenDRIVE 2 Lanelets - Converter

This repository provides the code for an OpenDRIVE ([www.opendrive.org](http://www.opendrive.org)) to lanelet ([www.mrt.kit.edu/software/liblanelet](https://www.mrt.kit.edu/software/libLanelet/libLanelet.html)) converter.

## Requirements

- Python 3.x
- numpy
- scipy
- lxml

## Installation

If needed, the converter libraries can be installed using ```pip```:

```bash
git clone https://gitlab.lrz.de/koschi/converter.git
cd converter && pip install .
```

## Example Files

Download example files from: http://opendrive.org/download.html

## Usage

### Using our provided GUI

Additional requirement: PyQt5. Start the GUI with ```python gui.py```

![GUI screenshot](gui_screenshot.png "Screenshot of converter GUI")

### Using the library in your own scripts

```python
from lxml import etree
from opendriveparser import parse_opendrive
from opendrive2lanelet import Network

# Import, parse and convert OpenDRIVE file
fh = open("input_opendrive.xodr", 'r')
openDrive = parse_opendrive(etree.parse(fh).getroot())
fh.close()

roadNetwork = Network()
roadNetwork.loadOpenDrive(openDrive)

scenario = roadNetwork.exportCommonRoadScenario()

# Write CommonRoad scenario to file
fh = open("output_commonroad_file.xml", "wb")
fh.write(scenario.export_to_string())
fh.close()
```
