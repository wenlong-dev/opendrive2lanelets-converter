# OpenDRIVE 2 Lanelets - Converter

This repository provides the code for an OpenDRIVE ([www.opendrive.org](http://www.opendrive.org)) to lanelets ([www.mrt.kit.edu/software/liblanelet](https://www.mrt.kit.edu/software/libLanelet/libLanelet.html)) converter.

## Installation

The following python packages have to be available:
- numpy
- scipy
- lxml
- PyQt5 only for GUI

If needed, the converter libraries can be installed using ```pip```:

```bash
git clone https://gitlab.lrz.de/koschi/converter.git
cd converter && pip install .
```

## Example OpenDRIVE Files

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


## Known Problems

- When trying to use the gui.py under Wayland, the following error occurs:
  ```
  This application failed to start because it could not find or load the Qt platform plugin "wayland" in "".
  Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
  Reinstalling the application may fix this problem.
  ```
  Set the platform to *xcb* using this command: ```export QT_QPA_PLATFORM="xcb"```

## ToDo

- CommonRoad does not support lane types right now. When this is implemented, the OpenDRIVE types have to be mapped onto the CommonRoad ones and added to the lanelet output.

- Adjacent lanes are not working properly yet, support is not included in software right now.

- Lane merges may cause problems with predecessor or successor definition.

- Unit-Tests

## License (MIT)

Copyright (c) 2018 Stefan Urban

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
